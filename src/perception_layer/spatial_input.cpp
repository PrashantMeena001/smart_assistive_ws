// =============================================================================
// spatial_input.cpp — Smart Stick Spatial Sensing Node
// =============================================================================
// LiDAR + Ultrasonic sensor fusion for ground-level obstacle detection.
// Publishes directional obstacle data (left/center/right) with distance.
// Uses HC-SR04 ultrasonic sensors on GPIO (wiringPi) or simulated input.
//
// Topics published:
//   /spatial/obstacles (std_msgs/String — JSON: direction, distance, zone)
//   /spatial/emergency (std_msgs/String — emergency stop trigger)
// =============================================================================

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/range.hpp>

#ifdef HAS_WIRINGPI
#include <wiringPi.h>
#endif

#include <chrono>
#include <vector>
#include <string>
#include <sstream>
#include <iomanip>
#include <cmath>
#include <algorithm>
#include <numeric>
#include <array>

using namespace std::chrono_literals;

// Sensor configuration for 3-zone detection (left, center, right)
struct UltrasonicSensor {
    std::string direction;  // "left", "center", "right"
    int trig_pin;
    int echo_pin;
    float max_range;        // meters
    float min_range;        // meters
    float last_reading;     // meters
    std::array<float, 5> filter_buffer;  // median filter
    int filter_idx;
};

// Obstacle zone thresholds
struct ZoneThresholds {
    static constexpr float EMERGENCY = 0.30f;   // < 30cm  — immediate stop
    static constexpr float DANGER    = 0.80f;    // < 80cm  — strong warning
    static constexpr float CAUTION   = 1.50f;    // < 1.5m  — mild warning
    static constexpr float CLEAR     = 4.00f;    // > 1.5m  — safe
};

class SpatialInputNode : public rclcpp::Node {
public:
    SpatialInputNode() : Node("spatial_input_node") {
        // Parameters
        this->declare_parameter<double>("scan_rate_hz", 20.0);
        this->declare_parameter<float>("max_range", 4.0f);
        this->declare_parameter<float>("min_range", 0.02f);
        this->declare_parameter<int>("trig_left", 23);
        this->declare_parameter<int>("echo_left", 24);
        this->declare_parameter<int>("trig_center", 17);
        this->declare_parameter<int>("echo_center", 27);
        this->declare_parameter<int>("trig_right", 5);
        this->declare_parameter<int>("echo_right", 6);
        this->declare_parameter<bool>("use_lidar", false);

        double scan_rate = this->get_parameter("scan_rate_hz").as_double();
        float max_r = static_cast<float>(this->get_parameter("max_range").as_double());
        float min_r = static_cast<float>(this->get_parameter("min_range").as_double());
        use_lidar_ = this->get_parameter("use_lidar").as_bool();

        // Initialize 3 ultrasonic sensors (left, center, right on the stick)
        sensors_.resize(3);
        sensors_[0] = {"left",
            (int)this->get_parameter("trig_left").as_int(),
            (int)this->get_parameter("echo_left").as_int(),
            max_r, min_r, max_r, {}, 0};
        sensors_[1] = {"center",
            (int)this->get_parameter("trig_center").as_int(),
            (int)this->get_parameter("echo_center").as_int(),
            max_r, min_r, max_r, {}, 0};
        sensors_[2] = {"right",
            (int)this->get_parameter("trig_right").as_int(),
            (int)this->get_parameter("echo_right").as_int(),
            max_r, min_r, max_r, {}, 0};

        // Fill filter buffers
        for (auto& s : sensors_) s.filter_buffer.fill(max_r);

        // Publishers
        obstacle_pub_ = this->create_publisher<std_msgs::msg::String>("/spatial/obstacles", 10);
        emergency_pub_ = this->create_publisher<std_msgs::msg::String>("/spatial/emergency", 10);
        range_pub_ = this->create_publisher<sensor_msgs::msg::Range>("/spatial/range", 10);

        // Initialize GPIO
        initGPIO();

        // If LiDAR is available, subscribe to its topic
        if (use_lidar_) {
            // LiDAR data would come from a separate driver node
            RCLCPP_INFO(get_logger(), "LiDAR mode enabled — will fuse with ultrasonic");
        }

        // Main scan timer
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / scan_rate)),
            std::bind(&SpatialInputNode::scanCycle, this));

        RCLCPP_INFO(get_logger(), "Spatial input node started: 3-zone ultrasonic @ %.0f Hz", scan_rate);
    }

private:
    void initGPIO() {
#ifdef HAS_WIRINGPI
        if (wiringPiSetupGpio() == -1) {
            RCLCPP_ERROR(get_logger(), "wiringPi GPIO setup failed");
            gpio_ready_ = false;
            return;
        }
        for (auto& s : sensors_) {
            pinMode(s.trig_pin, OUTPUT);
            pinMode(s.echo_pin, INPUT);
            digitalWrite(s.trig_pin, LOW);
        }
        gpio_ready_ = true;
        RCLCPP_INFO(get_logger(), "GPIO initialized for ultrasonic sensors");
#else
        gpio_ready_ = false;
        RCLCPP_WARN(get_logger(), "No GPIO — using simulated sensor data");
#endif
    }

    // Read one ultrasonic sensor via GPIO trigger/echo
    float readUltrasonic(UltrasonicSensor& sensor) {
#ifdef HAS_WIRINGPI
        if (!gpio_ready_) return sensor.max_range;

        // Send 10µs trigger pulse
        digitalWrite(sensor.trig_pin, HIGH);
        delayMicroseconds(10);
        digitalWrite(sensor.trig_pin, LOW);

        // Wait for echo start (timeout 30ms)
        auto timeout = std::chrono::steady_clock::now() + std::chrono::milliseconds(30);
        while (digitalRead(sensor.echo_pin) == LOW) {
            if (std::chrono::steady_clock::now() > timeout) return sensor.max_range;
        }
        auto echo_start = std::chrono::steady_clock::now();

        // Wait for echo end
        timeout = echo_start + std::chrono::milliseconds(30);
        while (digitalRead(sensor.echo_pin) == HIGH) {
            if (std::chrono::steady_clock::now() > timeout) return sensor.max_range;
        }
        auto echo_end = std::chrono::steady_clock::now();

        // Calculate distance: speed of sound = 343 m/s, round trip
        double duration_us = std::chrono::duration<double, std::micro>(echo_end - echo_start).count();
        float distance = static_cast<float>(duration_us * 0.000343 / 2.0);

        // Clamp to valid range
        if (distance < sensor.min_range || distance > sensor.max_range) {
            distance = sensor.max_range;
        }
        return distance;
#else
        // Simulated reading — slowly varying sine wave for testing
        static int sim_tick = 0;
        sim_tick++;
        float base = 1.5f;
        float variation = 1.0f * std::sin(sim_tick * 0.05f + sensor.trig_pin);
        return std::clamp(base + variation, sensor.min_range, sensor.max_range);
#endif
    }

    // Median filter for noise rejection
    float medianFilter(UltrasonicSensor& sensor, float raw) {
        sensor.filter_buffer[sensor.filter_idx % 5] = raw;
        sensor.filter_idx++;

        std::array<float, 5> sorted = sensor.filter_buffer;
        std::sort(sorted.begin(), sorted.end());
        return sorted[2];  // median of 5
    }

    // Main scan cycle — read all sensors, fuse, and publish
    void scanCycle() {
        std::ostringstream json;
        json << "{\"timestamp\":" << this->now().nanoseconds()
             << ",\"sensors\":[";

        bool any_emergency = false;
        std::string emergency_dir;
        float emergency_dist = 999.0f;

        for (size_t i = 0; i < sensors_.size(); i++) {
            auto& s = sensors_[i];

            // Read and filter
            float raw = readUltrasonic(s);
            float filtered = medianFilter(s, raw);
            s.last_reading = filtered;

            // Classify zone
            std::string zone;
            if (filtered < ZoneThresholds::EMERGENCY) {
                zone = "emergency";
                any_emergency = true;
                if (filtered < emergency_dist) {
                    emergency_dist = filtered;
                    emergency_dir = s.direction;
                }
            } else if (filtered < ZoneThresholds::DANGER) {
                zone = "danger";
            } else if (filtered < ZoneThresholds::CAUTION) {
                zone = "caution";
            } else {
                zone = "clear";
            }

            if (i > 0) json << ",";
            json << "{\"direction\":\"" << s.direction
                 << "\",\"distance\":" << std::fixed << std::setprecision(2) << filtered
                 << ",\"zone\":\"" << zone << "\"}";

            // Publish individual range message
            sensor_msgs::msg::Range range_msg;
            range_msg.header.stamp = this->now();
            range_msg.header.frame_id = "stick_" + s.direction;
            range_msg.radiation_type = sensor_msgs::msg::Range::ULTRASOUND;
            range_msg.field_of_view = 0.26f;  // ~15 degrees
            range_msg.min_range = s.min_range;
            range_msg.max_range = s.max_range;
            range_msg.range = filtered;
            range_pub_->publish(range_msg);
        }

        json << "]}";

        auto msg = std_msgs::msg::String();
        msg.data = json.str();
        obstacle_pub_->publish(msg);

        // Emergency stop
        if (any_emergency) {
            auto emsg = std_msgs::msg::String();
            std::ostringstream ej;
            ej << "{\"type\":\"emergency_stop\",\"direction\":\"" << emergency_dir
               << "\",\"distance\":" << std::fixed << std::setprecision(2) << emergency_dist << "}";
            emsg.data = ej.str();
            emergency_pub_->publish(emsg);
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 500,
                "EMERGENCY: obstacle %.2f m %s", emergency_dist, emergency_dir.c_str());
        }
    }

    std::vector<UltrasonicSensor> sensors_;
    bool gpio_ready_ = false;
    bool use_lidar_ = false;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr obstacle_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr emergency_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr range_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SpatialInputNode>());
    rclcpp::shutdown();
    return 0;
}
