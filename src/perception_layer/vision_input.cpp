// =============================================================================
// vision_input.cpp — Smart Spectacles Vision & YOLO Object Detection Node
// =============================================================================
// Camera capture + YOLO ONNX inference for environmental object detection.
// Target: Raspberry Pi 4 (4GB) — optimized for low-latency edge inference.
// Output: /vision/detections (JSON), /vision/raw_image (for SLAM)
// =============================================================================

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>

#ifdef HAS_ONNXRUNTIME
#include <onnxruntime_cxx_api.h>
#endif

#include <chrono>
#include <vector>
#include <string>
#include <sstream>
#include <iomanip>
#include <memory>
#include <algorithm>
#include <fstream>

using namespace std::chrono_literals;

struct Detection {
    int class_id;
    std::string label;
    float confidence;
    cv::Rect bbox;
    float distance_estimate;
};

static const std::vector<std::string> YOLO_CLASSES = {
    "person","bicycle","car","motorcycle","airplane","bus","train","truck",
    "boat","traffic light","fire hydrant","stop sign","parking meter","bench",
    "bird","cat","dog","horse","sheep","cow","elephant","bear","zebra",
    "giraffe","backpack","umbrella","handbag","tie","suitcase","frisbee",
    "skis","snowboard","sports ball","kite","baseball bat","baseball glove",
    "skateboard","surfboard","tennis racket","bottle","wine glass","cup",
    "fork","knife","spoon","bowl","banana","apple","sandwich","orange",
    "broccoli","carrot","hot dog","pizza","donut","cake","chair","couch",
    "potted plant","bed","dining table","toilet","tv","laptop","mouse",
    "remote","keyboard","cell phone","microwave","oven","toaster","sink",
    "refrigerator","book","clock","vase","scissors","teddy bear",
    "hair drier","toothbrush"
};

static const std::vector<std::string> HAZARD_CLASSES = {
    "car","motorcycle","bus","truck","train","bicycle","dog","fire hydrant","stop sign"
};

class VisionInputNode : public rclcpp::Node {
public:
    VisionInputNode() : Node("vision_input_node"), frame_count_(0) {
        this->declare_parameter<std::string>("model_path", "src/models/best.onnx");
        this->declare_parameter<int>("camera_id", 0);
        this->declare_parameter<int>("frame_width", 640);
        this->declare_parameter<int>("frame_height", 480);
        this->declare_parameter<double>("fps_target", 15.0);
        this->declare_parameter<float>("confidence_threshold", 0.45f);
        this->declare_parameter<float>("nms_threshold", 0.50f);
        this->declare_parameter<int>("inference_size", 416);
        this->declare_parameter<int>("skip_frames", 2);

        model_path_ = this->get_parameter("model_path").as_string();
        camera_id_ = this->get_parameter("camera_id").as_int();
        frame_w_ = this->get_parameter("frame_width").as_int();
        frame_h_ = this->get_parameter("frame_height").as_int();
        fps_ = this->get_parameter("fps_target").as_double();
        conf_thresh_ = static_cast<float>(this->get_parameter("confidence_threshold").as_double());
        nms_thresh_ = static_cast<float>(this->get_parameter("nms_threshold").as_double());
        infer_sz_ = this->get_parameter("inference_size").as_int();
        skip_ = this->get_parameter("skip_frames").as_int();

        det_pub_ = this->create_publisher<std_msgs::msg::String>("/vision/detections", 10);
        img_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/vision/raw_image", 5);

        if (!initCamera()) { RCLCPP_FATAL(get_logger(), "Camera open failed"); rclcpp::shutdown(); return; }
        initModel();

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / fps_)),
            std::bind(&VisionInputNode::processFrame, this));

        RCLCPP_INFO(get_logger(), "Vision node started: cam %d, %dx%d, %.0f fps", camera_id_, frame_w_, frame_h_, fps_);
    }

    ~VisionInputNode() { if (cap_.isOpened()) cap_.release(); }

private:
    bool initCamera() {
        cap_.open(camera_id_, cv::CAP_V4L2);
        if (!cap_.isOpened()) cap_.open(camera_id_);
        if (!cap_.isOpened()) return false;
        cap_.set(cv::CAP_PROP_FRAME_WIDTH, frame_w_);
        cap_.set(cv::CAP_PROP_FRAME_HEIGHT, frame_h_);
        cap_.set(cv::CAP_PROP_BUFFERSIZE, 1);
        return true;
    }

    bool initModel() {
        std::ifstream f(model_path_);
        if (!f.good()) { RCLCPP_WARN(get_logger(), "Model not found: %s", model_path_.c_str()); return false; }
        f.close();
        try {
            net_ = cv::dnn::readNetFromONNX(model_path_);
            net_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
            net_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
            model_loaded_ = true;
            RCLCPP_INFO(get_logger(), "YOLO model loaded: %s", model_path_.c_str());
            return true;
        } catch (const cv::Exception& e) {
            RCLCPP_ERROR(get_logger(), "Model load failed: %s", e.what());
            return false;
        }
    }

    void processFrame() {
        cv::Mat frame;
        if (!cap_.read(frame) || frame.empty()) return;
        frame_count_++;

        // Publish raw image for SLAM
        auto img_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
        img_msg->header.stamp = this->now();
        img_msg->header.frame_id = "camera_frame";
        img_pub_->publish(*img_msg);

        // Run inference on every Nth frame
        if (frame_count_ % skip_ != 0) {
            if (!last_json_.empty()) { auto m = std_msgs::msg::String(); m.data = last_json_; det_pub_->publish(m); }
            return;
        }

        auto t0 = std::chrono::steady_clock::now();
        auto dets = runInference(frame);
        double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();

        last_json_ = encodeDetections(dets, ms);
        auto m = std_msgs::msg::String(); m.data = last_json_; det_pub_->publish(m);
    }

    std::vector<Detection> runInference(const cv::Mat& frame) {
        std::vector<Detection> dets;
        if (!model_loaded_) return dets;

        cv::Mat blob;
        cv::dnn::blobFromImage(frame, blob, 1.0/255.0, cv::Size(infer_sz_, infer_sz_), cv::Scalar(), true, false);
        net_.setInput(blob);

        std::vector<cv::Mat> outs;
        try { net_.forward(outs, net_.getUnconnectedOutLayersNames()); }
        catch (...) { return dets; }
        if (outs.empty()) return dets;

        cv::Mat output = outs[0];
        if (output.dims == 3) output = output.reshape(1, output.size[1]);

        int rows = output.rows, cols = output.cols;
        int nc = cols - 5; // YOLOv5: x,y,w,h,obj + classes
        bool v8 = (nc <= 0);
        if (v8) nc = cols - 4;

        std::vector<int> cids; std::vector<float> confs; std::vector<cv::Rect> boxes;
        float sx = (float)frame.cols / infer_sz_, sy = (float)frame.rows / infer_sz_;

        for (int i = 0; i < rows; i++) {
            const float* r = output.ptr<float>(i);
            float best = 0; int bcls = 0;
            if (v8) {
                for (int c = 0; c < nc; c++) { if (r[4+c] > best) { best = r[4+c]; bcls = c; } }
            } else {
                float obj = r[4]; if (obj < conf_thresh_) continue;
                for (int c = 0; c < nc; c++) { float s = r[5+c]*obj; if (s > best) { best = s; bcls = c; } }
            }
            if (best < conf_thresh_) continue;

            float cx = r[0], cy = r[1], w = r[2], h = r[3];
            int x1 = std::max(0, (int)((cx-w/2)*sx)), y1 = std::max(0, (int)((cy-h/2)*sy));
            int bw = std::min((int)(w*sx), frame.cols-x1), bh = std::min((int)(h*sy), frame.rows-y1);
            boxes.emplace_back(x1, y1, bw, bh);
            confs.push_back(best);
            cids.push_back(bcls);
        }

        std::vector<int> nms_idx;
        cv::dnn::NMSBoxes(boxes, confs, conf_thresh_, nms_thresh_, nms_idx);

        for (int idx : nms_idx) {
            Detection d;
            d.class_id = cids[idx]; d.confidence = confs[idx]; d.bbox = boxes[idx];
            d.label = (d.class_id < (int)YOLO_CLASSES.size()) ? YOLO_CLASSES[d.class_id] : "unknown";
            float area_r = (float)d.bbox.area() / (frame.cols * frame.rows);
            d.distance_estimate = (area_r > 0.3f) ? 0.5f : (area_r > 0.1f) ? 1.5f : (area_r > 0.03f) ? 3.0f : 5.0f;
            dets.push_back(d);
        }

        std::sort(dets.begin(), dets.end(), [](auto& a, auto& b){ return a.distance_estimate < b.distance_estimate; });
        return dets;
    }

    bool isHazard(const std::string& l) const {
        return std::find(HAZARD_CLASSES.begin(), HAZARD_CLASSES.end(), l) != HAZARD_CLASSES.end();
    }

    std::string encodeDetections(const std::vector<Detection>& dets, double ms) {
        std::ostringstream j;
        j << "{\"timestamp\":" << this->now().nanoseconds()
          << ",\"inference_ms\":" << std::fixed << std::setprecision(1) << ms
          << ",\"count\":" << dets.size() << ",\"detections\":[";
        for (size_t i = 0; i < dets.size(); i++) {
            auto& d = dets[i];
            if (i) j << ",";
            float cx = d.bbox.x + d.bbox.width / 2.0f;
            std::string pos = (cx / frame_w_ < 0.33f) ? "left" : (cx / frame_w_ > 0.67f) ? "right" : "center";
            j << "{\"label\":\"" << d.label << "\",\"confidence\":" << std::setprecision(2) << d.confidence
              << ",\"distance\":" << std::setprecision(1) << d.distance_estimate
              << ",\"is_hazard\":" << (isHazard(d.label) ? "true" : "false")
              << ",\"bbox\":{\"x\":" << d.bbox.x << ",\"y\":" << d.bbox.y
              << ",\"w\":" << d.bbox.width << ",\"h\":" << d.bbox.height << "}"
              << ",\"position\":\"" << pos << "\"}";
        }
        j << "]}";
        return j.str();
    }

    cv::VideoCapture cap_;
    cv::dnn::Net net_;
    bool model_loaded_ = false;
    std::string model_path_;
    int camera_id_, frame_w_, frame_h_, infer_sz_, skip_;
    double fps_;
    float conf_thresh_, nms_thresh_;
    uint64_t frame_count_;
    std::string last_json_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr det_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr img_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VisionInputNode>());
    rclcpp::shutdown();
    return 0;
}
