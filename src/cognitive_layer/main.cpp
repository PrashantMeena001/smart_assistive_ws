// =============================================================================
// main.cpp — Central Cognitive Decision Engine
// =============================================================================
// Fuses all sensor inputs and makes prioritized decisions:
//   Priority 1: Emergency stop (spatial < 30cm)
//   Priority 2: Hazard detection (YOLO vision)
//   Priority 3: SLAM navigation guidance
//   Priority 4: Morse communication relay
//
// Subscribes to all perception nodes, publishes action commands to
// interaction layer (haptic + audio).
//
// Operating Modes:
//   Mode 1 (BLIND):      Audio + basic vibration
//   Mode 2 (DEAF_BLIND): Full Morse code haptic communication
// =============================================================================

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <chrono>
#include <string>
#include <sstream>
#include <iomanip>
#include <vector>
#include <mutex>
#include <algorithm>
#include <cmath>
#include <map>
#include <functional>

using namespace std::chrono_literals;
using SteadyClock = std::chrono::steady_clock;

// =============================================================================
// Operating modes
// =============================================================================
enum class OperatingMode { BLIND = 1, DEAF_BLIND = 2 };

// Priority levels for decision engine
enum class Priority {
    EMERGENCY = 0,   // Immediate stop — highest
    HAZARD    = 1,   // Detected danger
    NAVIGATION = 2,  // SLAM waypoint guidance
    COMMUNICATION = 3, // Morse relay
    INFO      = 4    // General info — lowest
};

// Action command sent to interaction layer
struct ActionCommand {
    Priority priority;
    std::string type;         // "emergency","hazard","navigate","morse","info"
    std::string message;      // Human-readable or Morse-encodable
    std::string direction;    // "left","center","right","none"
    float distance;           // meters (0 if N/A)
    int64_t timestamp;
    int urgency;              // 0-10 (controls haptic intensity)

    bool operator<(const ActionCommand& other) const {
        return static_cast<int>(priority) < static_cast<int>(other.priority);
    }
};

// =============================================================================
// Simple JSON parser helpers (avoid external dependency)
// =============================================================================
namespace json_util {
    std::string getString(const std::string& json, const std::string& key) {
        std::string search = "\"" + key + "\":\"";
        auto pos = json.find(search);
        if (pos == std::string::npos) return "";
        pos += search.size();
        auto end = json.find("\"", pos);
        return (end != std::string::npos) ? json.substr(pos, end - pos) : "";
    }

    float getFloat(const std::string& json, const std::string& key) {
        std::string search = "\"" + key + "\":";
        auto pos = json.find(search);
        if (pos == std::string::npos) return -1.0f;
        pos += search.size();
        try { return std::stof(json.substr(pos)); }
        catch (...) { return -1.0f; }
    }

    int getInt(const std::string& json, const std::string& key) {
        std::string search = "\"" + key + "\":";
        auto pos = json.find(search);
        if (pos == std::string::npos) return -1;
        pos += search.size();
        try { return std::stoi(json.substr(pos)); }
        catch (...) { return -1; }
    }

    bool getBool(const std::string& json, const std::string& key) {
        std::string search = "\"" + key + "\":";
        auto pos = json.find(search);
        if (pos == std::string::npos) return false;
        pos += search.size();
        return json.substr(pos, 4) == "true";
    }

    // Extract array elements (simple JSON array of objects)
    std::vector<std::string> getArray(const std::string& json, const std::string& key) {
        std::vector<std::string> items;
        std::string search = "\"" + key + "\":[";
        auto pos = json.find(search);
        if (pos == std::string::npos) return items;
        pos += search.size();

        int depth = 0;
        size_t start = pos;
        for (size_t i = pos; i < json.size(); i++) {
            if (json[i] == '{') { if (depth == 0) start = i; depth++; }
            else if (json[i] == '}') {
                depth--;
                if (depth == 0) items.push_back(json.substr(start, i - start + 1));
            }
            else if (json[i] == ']' && depth == 0) break;
        }
        return items;
    }
}

// =============================================================================
// CognitiveEngineNode
// =============================================================================
class CognitiveEngineNode : public rclcpp::Node {
public:
    CognitiveEngineNode() : Node("cognitive_engine_node") {
        // Parameters
        this->declare_parameter<int>("operating_mode", 1);
        this->declare_parameter<double>("decision_rate_hz", 10.0);
        this->declare_parameter<float>("emergency_distance", 0.30f);
        this->declare_parameter<float>("hazard_confidence", 0.50f);
        this->declare_parameter<int>("max_commands_per_sec", 5);
        this->declare_parameter<double>("data_timeout_sec", 2.0);

        int mode_int = this->get_parameter("operating_mode").as_int();
        mode_ = (mode_int == 2) ? OperatingMode::DEAF_BLIND : OperatingMode::BLIND;
        double rate = this->get_parameter("decision_rate_hz").as_double();
        emergency_dist_ = static_cast<float>(this->get_parameter("emergency_distance").as_double());
        hazard_conf_ = static_cast<float>(this->get_parameter("hazard_confidence").as_double());
        max_cmds_ = this->get_parameter("max_commands_per_sec").as_int();
        data_timeout_ = this->get_parameter("data_timeout_sec").as_double();

        // Publishers — action commands for interaction layer
        haptic_pub_ = this->create_publisher<std_msgs::msg::String>("/action/haptic", 10);
        audio_pub_ = this->create_publisher<std_msgs::msg::String>("/action/audio", 10);
        status_pub_ = this->create_publisher<std_msgs::msg::String>("/cognitive/status", 5);

        // Subscribers — perception layer inputs
        vision_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/vision/detections", 10,
            std::bind(&CognitiveEngineNode::onVision, this, std::placeholders::_1));

        spatial_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/spatial/obstacles", 10,
            std::bind(&CognitiveEngineNode::onSpatial, this, std::placeholders::_1));

        emergency_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/spatial/emergency", 10,
            std::bind(&CognitiveEngineNode::onEmergency, this, std::placeholders::_1));

        imu_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/imu/orientation", 10,
            std::bind(&CognitiveEngineNode::onIMU, this, std::placeholders::_1));

        morse_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/morse/decoded", 10,
            std::bind(&CognitiveEngineNode::onMorseInput, this, std::placeholders::_1));

        morse_char_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/morse/char", 10,
            std::bind(&CognitiveEngineNode::onMorseChar, this, std::placeholders::_1));

        // Mode switch subscriber (can switch at runtime)
        mode_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/system/mode", 10,
            [this](const std_msgs::msg::String::SharedPtr msg) {
                if (msg->data == "blind") mode_ = OperatingMode::BLIND;
                else if (msg->data == "deaf_blind") mode_ = OperatingMode::DEAF_BLIND;
                RCLCPP_INFO(get_logger(), "Mode switched to: %s", msg->data.c_str());
            });

        // Decision engine timer
        decision_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / rate)),
            std::bind(&CognitiveEngineNode::decisionCycle, this));

        // Status timer
        status_timer_ = this->create_wall_timer(2s,
            std::bind(&CognitiveEngineNode::publishStatus, this));

        RCLCPP_INFO(get_logger(), "Cognitive engine started — mode: %s, rate: %.0f Hz",
            (mode_ == OperatingMode::DEAF_BLIND) ? "DEAF_BLIND" : "BLIND", rate);
    }

private:
    // =========================================================================
    // Perception callbacks — store latest data with timestamps
    // =========================================================================
    void onVision(const std_msgs::msg::String::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        vision_data_ = msg->data;
        vision_time_ = SteadyClock::now();
    }

    void onSpatial(const std_msgs::msg::String::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        spatial_data_ = msg->data;
        spatial_time_ = SteadyClock::now();
    }

    void onEmergency(const std_msgs::msg::String::SharedPtr msg) {
        // Emergency is handled immediately, not queued
        std::lock_guard<std::mutex> lock(data_mutex_);
        std::string dir = json_util::getString(msg->data, "direction");
        float dist = json_util::getFloat(msg->data, "distance");

        ActionCommand cmd;
        cmd.priority = Priority::EMERGENCY;
        cmd.type = "emergency";
        cmd.message = "STOP " + dir;
        cmd.direction = dir;
        cmd.distance = dist;
        cmd.timestamp = this->now().nanoseconds();
        cmd.urgency = 10;

        dispatchCommand(cmd);
    }

    void onIMU(const std_msgs::msg::String::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        imu_data_ = msg->data;
        imu_time_ = SteadyClock::now();
    }

    void onMorseInput(const std_msgs::msg::String::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        morse_text_ = json_util::getString(msg->data, "text");
        morse_time_ = SteadyClock::now();
    }

    void onMorseChar(const std_msgs::msg::String::SharedPtr msg) {
        // Handle special Morse commands immediately
        std::string type = json_util::getString(msg->data, "type");
        if (type == "command") {
            std::string cmd = json_util::getString(msg->data, "command");
            handleMorseCommand(cmd);
        }
    }

    // =========================================================================
    // Decision cycle — prioritized processing
    // =========================================================================
    void decisionCycle() {
        std::lock_guard<std::mutex> lock(data_mutex_);
        auto now_tp = SteadyClock::now();

        std::vector<ActionCommand> commands;

        // --- Priority 1: Spatial emergency (already handled in callback) ---

        // --- Priority 2: Spatial danger/caution zones ---
        if (isDataFresh(spatial_time_, now_tp)) {
            processSpatialData(commands);
        }

        // --- Priority 3: Vision hazards ---
        if (isDataFresh(vision_time_, now_tp)) {
            processVisionData(commands);
        }

        // --- Priority 4: IMU alerts ---
        if (isDataFresh(imu_time_, now_tp)) {
            processIMUData(commands);
        }

        // --- Priority 5: Morse communication ---
        if (isDataFresh(morse_time_, now_tp) && !morse_text_.empty()) {
            ActionCommand cmd;
            cmd.priority = Priority::COMMUNICATION;
            cmd.type = "morse_relay";
            cmd.message = morse_text_;
            cmd.direction = "none";
            cmd.distance = 0;
            cmd.timestamp = this->now().nanoseconds();
            cmd.urgency = 3;
            commands.push_back(cmd);
        }

        // Sort by priority and dispatch top commands
        std::sort(commands.begin(), commands.end());

        int dispatched = 0;
        for (const auto& cmd : commands) {
            if (dispatched >= max_cmds_) break;
            if (shouldSuppress(cmd)) continue;
            dispatchCommand(cmd);
            dispatched++;
        }
    }

    // =========================================================================
    // Process spatial obstacle data
    // =========================================================================
    void processSpatialData(std::vector<ActionCommand>& commands) {
        auto sensors = json_util::getArray(spatial_data_, "sensors");
        for (const auto& s : sensors) {
            std::string dir = json_util::getString(s, "direction");
            float dist = json_util::getFloat(s, "distance");
            std::string zone = json_util::getString(s, "zone");

            if (zone == "danger") {
                ActionCommand cmd;
                cmd.priority = Priority::HAZARD;
                cmd.type = "obstacle";
                cmd.direction = dir;
                cmd.distance = dist;
                cmd.urgency = 7;
                cmd.timestamp = this->now().nanoseconds();

                std::ostringstream msg;
                msg << "Obstacle " << dir << " " << std::fixed << std::setprecision(1) << dist << " meters";
                cmd.message = msg.str();
                commands.push_back(cmd);
            }
            else if (zone == "caution") {
                ActionCommand cmd;
                cmd.priority = Priority::NAVIGATION;
                cmd.type = "caution";
                cmd.direction = dir;
                cmd.distance = dist;
                cmd.urgency = 4;
                cmd.timestamp = this->now().nanoseconds();

                std::ostringstream msg;
                msg << "Caution " << dir << " " << std::fixed << std::setprecision(1) << dist << " meters";
                cmd.message = msg.str();
                commands.push_back(cmd);
            }
        }
    }

    // =========================================================================
    // Process vision detection data
    // =========================================================================
    void processVisionData(std::vector<ActionCommand>& commands) {
        auto detections = json_util::getArray(vision_data_, "detections");

        for (const auto& det : detections) {
            bool is_hazard = json_util::getBool(det, "is_hazard");
            float conf = json_util::getFloat(det, "confidence");
            if (conf < hazard_conf_) continue;

            std::string label = json_util::getString(det, "label");
            std::string pos = json_util::getString(det, "position");
            float dist = json_util::getFloat(det, "distance");

            ActionCommand cmd;
            cmd.type = is_hazard ? "hazard" : "object";
            cmd.priority = is_hazard ? Priority::HAZARD : Priority::INFO;
            cmd.direction = pos;
            cmd.distance = dist;
            cmd.timestamp = this->now().nanoseconds();

            if (is_hazard) {
                cmd.urgency = (dist < 1.5f) ? 8 : 5;
                cmd.message = label + " " + pos;
            } else {
                cmd.urgency = 2;
                cmd.message = label + " ahead";
            }
            commands.push_back(cmd);
        }
    }

    // =========================================================================
    // Process IMU orientation data
    // =========================================================================
    void processIMUData(std::vector<ActionCommand>& commands) {
        std::string alert = json_util::getString(imu_data_, "alert");
        if (alert == "none") return;

        ActionCommand cmd;
        cmd.direction = "none";
        cmd.distance = 0;
        cmd.timestamp = this->now().nanoseconds();

        if (alert == "fall") {
            cmd.priority = Priority::EMERGENCY;
            cmd.type = "fall";
            cmd.message = "Fall detected";
            cmd.urgency = 10;
        } else if (alert == "stumble") {
            cmd.priority = Priority::HAZARD;
            cmd.type = "stumble";
            cmd.message = "Stumble detected";
            cmd.urgency = 7;
        } else if (alert == "head_tilt") {
            cmd.priority = Priority::INFO;
            cmd.type = "tilt";
            cmd.message = "Watch overhead";
            cmd.urgency = 3;
        }
        commands.push_back(cmd);
    }

    // =========================================================================
    // Dispatch command to interaction layer
    // =========================================================================
    void dispatchCommand(const ActionCommand& cmd) {
        std::ostringstream json;
        json << "{\"type\":\"" << cmd.type
             << "\",\"message\":\"" << cmd.message
             << "\",\"direction\":\"" << cmd.direction
             << "\",\"distance\":" << std::fixed << std::setprecision(2) << cmd.distance
             << ",\"urgency\":" << cmd.urgency
             << ",\"priority\":" << static_cast<int>(cmd.priority)
             << ",\"mode\":\"" << ((mode_ == OperatingMode::DEAF_BLIND) ? "deaf_blind" : "blind")
             << "\",\"timestamp\":" << cmd.timestamp << "}";

        auto msg = std_msgs::msg::String();
        msg.data = json.str();

        // Always send to haptic (both modes use vibration)
        haptic_pub_->publish(msg);

        // Audio only in BLIND mode
        if (mode_ == OperatingMode::BLIND) {
            audio_pub_->publish(msg);
        }

        // Record for suppression
        last_dispatch_[cmd.type] = SteadyClock::now();
    }

    // =========================================================================
    // Suppress duplicate/spammy alerts
    // =========================================================================
    bool shouldSuppress(const ActionCommand& cmd) const {
        // Never suppress emergencies
        if (cmd.priority == Priority::EMERGENCY) return false;

        auto it = last_dispatch_.find(cmd.type);
        if (it == last_dispatch_.end()) return false;

        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            SteadyClock::now() - it->second).count();

        // Suppress if same type dispatched recently
        int cooldown_ms = (cmd.priority == Priority::HAZARD) ? 500 : 1000;
        return elapsed < cooldown_ms;
    }

    // =========================================================================
    // Handle special Morse commands
    // =========================================================================
    void handleMorseCommand(const std::string& cmd) {
        ActionCommand action;
        action.direction = "none";
        action.distance = 0;
        action.timestamp = this->now().nanoseconds();

        if (cmd == "SOS") {
            action.priority = Priority::EMERGENCY;
            action.type = "sos";
            action.message = "SOS Emergency";
            action.urgency = 10;
            RCLCPP_WARN(get_logger(), "SOS COMMAND RECEIVED");
        } else if (cmd == "HELP") {
            action.priority = Priority::HAZARD;
            action.type = "help";
            action.message = "Help requested";
            action.urgency = 8;
        } else if (cmd == "REPEAT") {
            action.priority = Priority::COMMUNICATION;
            action.type = "repeat";
            action.message = last_message_;
            action.urgency = 5;
        } else {
            return;
        }

        dispatchCommand(action);
        last_message_ = action.message;
    }

    bool isDataFresh(const SteadyClock::time_point& tp, const SteadyClock::time_point& now) const {
        auto age = std::chrono::duration<double>(now - tp).count();
        return age < data_timeout_ && tp.time_since_epoch().count() > 0;
    }

    void publishStatus() {
        auto now_tp = SteadyClock::now();
        std::ostringstream json;
        json << "{\"mode\":\"" << ((mode_ == OperatingMode::DEAF_BLIND) ? "deaf_blind" : "blind")
             << "\",\"vision_active\":" << (isDataFresh(vision_time_, now_tp) ? "true" : "false")
             << ",\"spatial_active\":" << (isDataFresh(spatial_time_, now_tp) ? "true" : "false")
             << ",\"imu_active\":" << (isDataFresh(imu_time_, now_tp) ? "true" : "false")
             << ",\"timestamp\":" << this->now().nanoseconds() << "}";

        auto msg = std_msgs::msg::String();
        msg.data = json.str();
        status_pub_->publish(msg);
    }

    // State
    OperatingMode mode_;
    float emergency_dist_, hazard_conf_;
    double data_timeout_;
    int max_cmds_;
    std::mutex data_mutex_;

    // Latest sensor data
    std::string vision_data_, spatial_data_, imu_data_, morse_text_;
    SteadyClock::time_point vision_time_, spatial_time_, imu_time_, morse_time_;

    // Suppression tracking
    std::map<std::string, SteadyClock::time_point> last_dispatch_;
    std::string last_message_;

    // Publishers
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr haptic_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr audio_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;

    // Subscribers
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr vision_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr spatial_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr emergency_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr imu_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr morse_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr morse_char_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_sub_;

    // Timers
    rclcpp::TimerBase::SharedPtr decision_timer_;
    rclcpp::TimerBase::SharedPtr status_timer_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CognitiveEngineNode>());
    rclcpp::shutdown();
    return 0;
}
