// =============================================================================
// haptic_output.cpp — Vibration Control & Morse Haptic Output Node
// =============================================================================
// Controls vibration motors for directional feedback and Morse code output.
// Supports 3 vibration motors (left, center, right) for spatial alerts
// and a dedicated Morse output motor for structured communication.
//
// Morse Timing:
//   Dot:        200 ms vibration
//   Dash:       600 ms vibration
//   Letter gap: 300 ms silence
//   Word gap:   700 ms silence
//
// Subscribes to: /action/haptic (from cognitive engine)
// =============================================================================

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#ifdef HAS_WIRINGPI
#include <wiringPi.h>
#include <softPwm.h>
#endif

#include <chrono>
#include <string>
#include <sstream>
#include <map>
#include <queue>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <functional>

using namespace std::chrono_literals;

// Morse code encoding table (char → dot/dash string)
static const std::map<char, std::string> CHAR_TO_MORSE = {
    {'A',".-"},{'B',"-..."},{'C',"-.-."},{'D',"-.."},{'E',"."},{'F',"..-."},
    {'G',"--."},{'H',"...."},{'I',".."},{'J',".---"},{'K',"-.-"},{'L',".-.."},
    {'M',"--"},{'N',"-."},{'O',"---"},{'P',".--."},{'Q',"--.-"},{'R',".-."},
    {'S',"..."},{'T',"-"},{'U',"..-"},{'V',"...-"},{'W',".--"},{'X',"-..-"},
    {'Y',"-.--"},{'Z',"--.."},
    {'0',"-----"},{'1',".----"},{'2',"..---"},{'3',"...--"},{'4',"....-"},
    {'5',"....."},{'6',"-...."},{'7',"--..."},{'8',"---.."},{'9',"----."},
    {' '," "},{'.', ".-.-.-"},{',',"--..--"},{'?',"..--.."}, {'!', "-.-.--"},
};

// Haptic command queued for output
struct HapticCommand {
    std::string type;        // "vibrate", "morse", "pattern"
    std::string direction;   // "left", "center", "right", "all"
    int intensity;           // 0-100 (PWM duty cycle %)
    int duration_ms;         // vibration duration
    std::string morse_text;  // text to encode as Morse (for deaf-blind)
    int priority;
};

class HapticOutputNode : public rclcpp::Node {
public:
    HapticOutputNode()
        : Node("haptic_output_node"), worker_running_(true)
    {
        // Parameters
        this->declare_parameter<int>("pin_left", 12);
        this->declare_parameter<int>("pin_center", 13);
        this->declare_parameter<int>("pin_right", 19);
        this->declare_parameter<int>("pin_morse", 26);
        this->declare_parameter<int>("dot_ms", 200);
        this->declare_parameter<int>("dash_ms", 600);
        this->declare_parameter<int>("letter_gap_ms", 300);
        this->declare_parameter<int>("word_gap_ms", 700);
        this->declare_parameter<int>("element_gap_ms", 100);

        pin_left_ = this->get_parameter("pin_left").as_int();
        pin_center_ = this->get_parameter("pin_center").as_int();
        pin_right_ = this->get_parameter("pin_right").as_int();
        pin_morse_ = this->get_parameter("pin_morse").as_int();
        dot_ms_ = this->get_parameter("dot_ms").as_int();
        dash_ms_ = this->get_parameter("dash_ms").as_int();
        letter_gap_ms_ = this->get_parameter("letter_gap_ms").as_int();
        word_gap_ms_ = this->get_parameter("word_gap_ms").as_int();
        element_gap_ms_ = this->get_parameter("element_gap_ms").as_int();

        // Initialize GPIO/PWM
        initMotors();

        // Subscribe to action commands from cognitive engine
        action_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/action/haptic", 10,
            std::bind(&HapticOutputNode::onAction, this, std::placeholders::_1));

        // Start worker thread for non-blocking Morse output
        worker_thread_ = std::thread(&HapticOutputNode::workerLoop, this);

        RCLCPP_INFO(get_logger(), "Haptic output node started — pins: L=%d C=%d R=%d M=%d",
            pin_left_, pin_center_, pin_right_, pin_morse_);
    }

    ~HapticOutputNode() {
        worker_running_ = false;
        cv_.notify_all();
        if (worker_thread_.joinable()) worker_thread_.join();
        allMotorsOff();
    }

private:
    void initMotors() {
#ifdef HAS_WIRINGPI
        if (wiringPiSetupGpio() == -1) {
            RCLCPP_ERROR(get_logger(), "GPIO setup failed");
            gpio_ready_ = false;
            return;
        }
        // Setup software PWM on each pin (range 0-100)
        softPwmCreate(pin_left_, 0, 100);
        softPwmCreate(pin_center_, 0, 100);
        softPwmCreate(pin_right_, 0, 100);
        softPwmCreate(pin_morse_, 0, 100);
        gpio_ready_ = true;
        RCLCPP_INFO(get_logger(), "GPIO PWM motors initialized");
#else
        gpio_ready_ = false;
        RCLCPP_WARN(get_logger(), "No GPIO — haptic output in simulation mode");
#endif
    }

    // Set motor intensity (0-100)
    void setMotor(int pin, int intensity) {
#ifdef HAS_WIRINGPI
        if (gpio_ready_) softPwmWrite(pin, std::clamp(intensity, 0, 100));
#endif
        RCLCPP_DEBUG(get_logger(), "Motor pin %d → %d%%", pin, intensity);
    }

    void allMotorsOff() {
        setMotor(pin_left_, 0);
        setMotor(pin_center_, 0);
        setMotor(pin_right_, 0);
        setMotor(pin_morse_, 0);
    }

    int getDirectionPin(const std::string& dir) const {
        if (dir == "left") return pin_left_;
        if (dir == "right") return pin_right_;
        return pin_center_;
    }

    // =========================================================================
    // Handle action command from cognitive engine
    // =========================================================================
    void onAction(const std_msgs::msg::String::SharedPtr msg) {
        // Parse JSON command
        std::string data = msg->data;

        // Quick parse
        std::string type, message, direction, mode;
        int urgency = 5;
        float distance = 0;

        // Extract fields
        auto extract_str = [&data](const std::string& key) -> std::string {
            std::string search = "\"" + key + "\":\"";
            auto pos = data.find(search);
            if (pos == std::string::npos) return "";
            pos += search.size();
            auto end = data.find("\"", pos);
            return (end != std::string::npos) ? data.substr(pos, end - pos) : "";
        };
        auto extract_int = [&data](const std::string& key) -> int {
            std::string search = "\"" + key + "\":";
            auto pos = data.find(search);
            if (pos == std::string::npos) return 0;
            pos += search.size();
            try { return std::stoi(data.substr(pos)); } catch (...) { return 0; }
        };

        type = extract_str("type");
        message = extract_str("message");
        direction = extract_str("direction");
        mode = extract_str("mode");
        urgency = extract_int("urgency");

        // Map urgency (0-10) to PWM intensity (30-100)
        int intensity = std::clamp(30 + urgency * 7, 30, 100);

        if (type == "emergency") {
            // Emergency: all motors full blast, short pulses
            HapticCommand cmd;
            cmd.type = "pattern";
            cmd.direction = "all";
            cmd.intensity = 100;
            cmd.duration_ms = 300;
            cmd.priority = 0;
            enqueueCommand(cmd);
        }
        else if (mode == "deaf_blind") {
            // Deaf-blind mode: encode message as Morse on morse motor
            HapticCommand cmd;
            cmd.type = "morse";
            cmd.morse_text = message;
            cmd.intensity = intensity;
            cmd.priority = 1;
            enqueueCommand(cmd);

            // Also give directional vibration
            if (direction != "none" && !direction.empty()) {
                HapticCommand dir_cmd;
                dir_cmd.type = "vibrate";
                dir_cmd.direction = direction;
                dir_cmd.intensity = intensity;
                dir_cmd.duration_ms = 200;
                dir_cmd.priority = 2;
                enqueueCommand(dir_cmd);
            }
        }
        else {
            // Blind mode: directional vibration only
            if (direction != "none" && !direction.empty()) {
                HapticCommand cmd;
                cmd.type = "vibrate";
                cmd.direction = direction;
                cmd.intensity = intensity;
                cmd.duration_ms = std::clamp(urgency * 50, 100, 500);
                cmd.priority = urgency > 7 ? 1 : 3;
                enqueueCommand(cmd);
            }
        }
    }

    // =========================================================================
    // Command queue management
    // =========================================================================
    void enqueueCommand(const HapticCommand& cmd) {
        std::lock_guard<std::mutex> lock(queue_mutex_);

        // Emergency preempts everything
        if (cmd.priority == 0) {
            // Clear queue, push emergency
            std::queue<HapticCommand> empty;
            command_queue_.swap(empty);
        }

        command_queue_.push(cmd);
        cv_.notify_one();
    }

    // =========================================================================
    // Worker thread — processes haptic commands sequentially
    // =========================================================================
    void workerLoop() {
        while (worker_running_) {
            HapticCommand cmd;
            {
                std::unique_lock<std::mutex> lock(queue_mutex_);
                cv_.wait_for(lock, 100ms, [this] {
                    return !command_queue_.empty() || !worker_running_;
                });
                if (!worker_running_) break;
                if (command_queue_.empty()) continue;
                cmd = command_queue_.front();
                command_queue_.pop();
            }

            if (cmd.type == "vibrate") {
                executeVibrate(cmd);
            } else if (cmd.type == "morse") {
                executeMorse(cmd);
            } else if (cmd.type == "pattern") {
                executePattern(cmd);
            }
        }
    }

    // =========================================================================
    // Execute simple vibration
    // =========================================================================
    void executeVibrate(const HapticCommand& cmd) {
        int pin = getDirectionPin(cmd.direction);
        setMotor(pin, cmd.intensity);
        std::this_thread::sleep_for(std::chrono::milliseconds(cmd.duration_ms));
        setMotor(pin, 0);
    }

    // =========================================================================
    // Execute Morse code output on morse motor
    // =========================================================================
    void executeMorse(const HapticCommand& cmd) {
        std::string text = cmd.morse_text;
        // Convert to uppercase
        for (auto& c : text) c = std::toupper(c);

        RCLCPP_INFO(get_logger(), "Morse output: \"%s\"", text.c_str());

        for (size_t i = 0; i < text.size() && worker_running_; i++) {
            char ch = text[i];

            if (ch == ' ') {
                // Word gap
                std::this_thread::sleep_for(std::chrono::milliseconds(word_gap_ms_));
                continue;
            }

            auto it = CHAR_TO_MORSE.find(ch);
            if (it == CHAR_TO_MORSE.end()) continue;

            const std::string& morse = it->second;

            for (size_t j = 0; j < morse.size() && worker_running_; j++) {
                if (morse[j] == '.') {
                    setMotor(pin_morse_, cmd.intensity);
                    std::this_thread::sleep_for(std::chrono::milliseconds(dot_ms_));
                    setMotor(pin_morse_, 0);
                } else if (morse[j] == '-') {
                    setMotor(pin_morse_, cmd.intensity);
                    std::this_thread::sleep_for(std::chrono::milliseconds(dash_ms_));
                    setMotor(pin_morse_, 0);
                }

                // Element gap (between dots and dashes within a letter)
                if (j < morse.size() - 1) {
                    std::this_thread::sleep_for(
                        std::chrono::milliseconds(element_gap_ms_));
                }
            }

            // Letter gap
            if (i < text.size() - 1 && text[i + 1] != ' ') {
                std::this_thread::sleep_for(
                    std::chrono::milliseconds(letter_gap_ms_));
            }
        }
    }

    // =========================================================================
    // Execute emergency pattern (rapid pulses on all motors)
    // =========================================================================
    void executePattern(const HapticCommand& cmd) {
        for (int pulse = 0; pulse < 5 && worker_running_; pulse++) {
            setMotor(pin_left_, cmd.intensity);
            setMotor(pin_center_, cmd.intensity);
            setMotor(pin_right_, cmd.intensity);
            setMotor(pin_morse_, cmd.intensity);
            std::this_thread::sleep_for(std::chrono::milliseconds(150));

            allMotorsOff();
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }

    // GPIO pins
    int pin_left_, pin_center_, pin_right_, pin_morse_;
    bool gpio_ready_ = false;

    // Morse timing
    int dot_ms_, dash_ms_, letter_gap_ms_, word_gap_ms_, element_gap_ms_;

    // Command queue
    std::queue<HapticCommand> command_queue_;
    std::mutex queue_mutex_;
    std::condition_variable cv_;
    std::atomic<bool> worker_running_;
    std::thread worker_thread_;

    // ROS2
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr action_sub_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HapticOutputNode>());
    rclcpp::shutdown();
    return 0;
}
