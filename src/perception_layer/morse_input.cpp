// =============================================================================
// morse_input.cpp — Morse Code Input Decoder Node
// =============================================================================
// Decodes button/touch-based Morse code input from deaf-blind users.
// Converts press durations into dots/dashes, decodes letters/words,
// and publishes decoded text for the cognitive layer.
//
// Morse Timing (optimized for haptic input):
//   Dot:        200 ms press
//   Dash:       600 ms press
//   Letter gap: 300 ms pause
//   Word gap:   700 ms pause
//
// Topics published:
//   /morse/decoded  (std_msgs/String — decoded text)
//   /morse/raw      (std_msgs/String — raw dot/dash sequence)
//   /morse/char     (std_msgs/String — latest decoded character)
// =============================================================================

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#ifdef HAS_WIRINGPI
#include <wiringPi.h>
#endif

#include <chrono>
#include <string>
#include <sstream>
#include <map>
#include <algorithm>
#include <functional>

using namespace std::chrono_literals;
using SteadyClock = std::chrono::steady_clock;
using TimePoint = std::chrono::steady_clock::time_point;

// =============================================================================
// International Morse Code lookup table
// =============================================================================
static const std::map<std::string, char> MORSE_TO_CHAR = {
    {".-", 'A'},     {"-...", 'B'},   {"-.-.", 'C'},   {"-..", 'D'},
    {".", 'E'},      {"..-.", 'F'},   {"--.", 'G'},    {"....", 'H'},
    {"..", 'I'},     {".---", 'J'},   {"-.-", 'K'},    {".-..", 'L'},
    {"--", 'M'},     {"-.", 'N'},     {"---", 'O'},    {".--.", 'P'},
    {"--.-", 'Q'},   {".-.", 'R'},    {"...", 'S'},    {"-", 'T'},
    {"..-", 'U'},    {"...-", 'V'},   {".--", 'W'},    {"-..-", 'X'},
    {"-.--", 'Y'},   {"--..", 'Z'},
    {"-----", '0'},  {".----", '1'},  {"..---", '2'},  {"...--", '3'},
    {"....-", '4'},  {".....", '5'},  {"-....", '6'},  {"--...", '7'},
    {"---..", '8'},  {"----.", '9'},
    {".-.-.-", '.'},  {"--..--", ','}, {"..--..", '?'},  {"-.-.--", '!'},
    // Special commands (mapped to control chars)
    {"..--.", '\x01'},   // SOS / emergency
    {".-.-", '\x02'},    // repeat last message
    {"...-.-", '\x03'},  // end of message
};

// Reverse lookup for common commands
static const std::map<std::string, std::string> SPECIAL_COMMANDS = {
    {"..--.", "SOS"},
    {"...---...", "SOS"},
    {".-.-", "REPEAT"},
    {"...-.-", "END"},
    {"-..---", "HELP"},
};

class MorseInputNode : public rclcpp::Node {
public:
    MorseInputNode() : Node("morse_input_node") {
        // Parameters — timing thresholds in milliseconds
        this->declare_parameter<int>("dot_max_ms", 350);
        this->declare_parameter<int>("dash_min_ms", 350);
        this->declare_parameter<int>("letter_gap_ms", 300);
        this->declare_parameter<int>("word_gap_ms", 700);
        this->declare_parameter<int>("button_pin", 25);
        this->declare_parameter<int>("poll_rate_hz", 100);

        dot_max_ms_ = this->get_parameter("dot_max_ms").as_int();
        dash_min_ms_ = this->get_parameter("dash_min_ms").as_int();
        letter_gap_ms_ = this->get_parameter("letter_gap_ms").as_int();
        word_gap_ms_ = this->get_parameter("word_gap_ms").as_int();
        button_pin_ = this->get_parameter("button_pin").as_int();
        int poll_rate = this->get_parameter("poll_rate_hz").as_int();

        // Publishers
        decoded_pub_ = this->create_publisher<std_msgs::msg::String>("/morse/decoded", 10);
        raw_pub_ = this->create_publisher<std_msgs::msg::String>("/morse/raw", 10);
        char_pub_ = this->create_publisher<std_msgs::msg::String>("/morse/char", 10);

        // State
        button_pressed_ = false;
        current_morse_ = "";
        decoded_text_ = "";

        // GPIO setup
        initButton();

        // Poll timer
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000 / poll_rate),
            std::bind(&MorseInputNode::pollButton, this));

        // Gap detection timer (check for letter/word gaps)
        gap_timer_ = this->create_wall_timer(50ms,
            std::bind(&MorseInputNode::checkGaps, this));

        RCLCPP_INFO(get_logger(),
            "Morse input node started: dot<%dms, dash>%dms, letter_gap=%dms, word_gap=%dms",
            dot_max_ms_, dash_min_ms_, letter_gap_ms_, word_gap_ms_);
    }

private:
    void initButton() {
#ifdef HAS_WIRINGPI
        if (wiringPiSetupGpio() == -1) {
            RCLCPP_ERROR(get_logger(), "GPIO setup failed");
            gpio_ready_ = false;
            return;
        }
        pinMode(button_pin_, INPUT);
        pullUpDnControl(button_pin_, PUD_UP);  // Internal pull-up
        gpio_ready_ = true;
        RCLCPP_INFO(get_logger(), "Morse button on GPIO %d", button_pin_);
#else
        gpio_ready_ = false;
        RCLCPP_WARN(get_logger(), "No GPIO — morse input via /morse/simulate topic");

        // Simulation subscriber for testing without hardware
        sim_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/morse/simulate", 10,
            [this](const std_msgs::msg::String::SharedPtr msg) {
                simulateInput(msg->data);
            });
#endif
    }

    bool readButton() {
#ifdef HAS_WIRINGPI
        if (!gpio_ready_) return false;
        return digitalRead(button_pin_) == LOW;  // Active low with pull-up
#else
        return sim_pressed_;
#endif
    }

    // Poll button state at high frequency for accurate timing
    void pollButton() {
        bool pressed = readButton();

        if (pressed && !button_pressed_) {
            // Button just pressed — record start time
            button_pressed_ = true;
            press_start_ = SteadyClock::now();
        }
        else if (!pressed && button_pressed_) {
            // Button just released — determine dot or dash
            button_pressed_ = false;
            release_time_ = SteadyClock::now();

            auto duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                release_time_ - press_start_).count();

            // Debounce — ignore very short presses
            if (duration_ms < 30) return;

            if (duration_ms < dot_max_ms_) {
                current_morse_ += ".";
            } else {
                current_morse_ += "-";
            }

            // Publish raw morse
            auto raw_msg = std_msgs::msg::String();
            raw_msg.data = current_morse_;
            raw_pub_->publish(raw_msg);

            has_pending_ = true;
        }
    }

    // Check for letter and word gaps
    void checkGaps() {
        if (!has_pending_ || button_pressed_) return;

        auto now = SteadyClock::now();
        auto gap_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            now - release_time_).count();

        if (gap_ms >= word_gap_ms_ && !word_gap_processed_) {
            // Word gap — decode current letter, add space
            decodeLetter();
            decoded_text_ += " ";
            publishDecoded();
            word_gap_processed_ = true;
            has_pending_ = false;
        }
        else if (gap_ms >= letter_gap_ms_ && !letter_gap_processed_) {
            // Letter gap — decode current letter
            decodeLetter();
            publishDecoded();
            letter_gap_processed_ = true;
            word_gap_processed_ = false;
        }
    }

    void decodeLetter() {
        if (current_morse_.empty()) return;

        // Check for special commands first
        auto cmd_it = SPECIAL_COMMANDS.find(current_morse_);
        if (cmd_it != SPECIAL_COMMANDS.end()) {
            RCLCPP_INFO(get_logger(), "Special command: %s (%s)",
                cmd_it->second.c_str(), current_morse_.c_str());

            auto cmd_msg = std_msgs::msg::String();
            cmd_msg.data = "{\"type\":\"command\",\"command\":\"" + cmd_it->second +
                          "\",\"morse\":\"" + current_morse_ + "\"}";
            char_pub_->publish(cmd_msg);

            current_morse_ = "";
            letter_gap_processed_ = false;
            return;
        }

        // Normal character decode
        auto it = MORSE_TO_CHAR.find(current_morse_);
        if (it != MORSE_TO_CHAR.end()) {
            char decoded = it->second;
            decoded_text_ += decoded;

            auto char_msg = std_msgs::msg::String();
            char_msg.data = "{\"type\":\"char\",\"char\":\"";
            char_msg.data += decoded;
            char_msg.data += "\",\"morse\":\"" + current_morse_ + "\"}";
            char_pub_->publish(char_msg);

            RCLCPP_DEBUG(get_logger(), "Decoded: '%c' from '%s'",
                decoded, current_morse_.c_str());
        } else {
            RCLCPP_WARN(get_logger(), "Unknown morse: '%s'", current_morse_.c_str());
        }

        current_morse_ = "";
        letter_gap_processed_ = false;
    }

    void publishDecoded() {
        auto msg = std_msgs::msg::String();
        std::ostringstream json;
        json << "{\"text\":\"" << decoded_text_
             << "\",\"length\":" << decoded_text_.size()
             << ",\"timestamp\":" << this->now().nanoseconds() << "}";
        msg.data = json.str();
        decoded_pub_->publish(msg);
    }

    // Simulation support for testing without hardware
    void simulateInput(const std::string& morse_str) {
        // Input like ".- -... -.-." for "ABC"
        for (char c : morse_str) {
            if (c == '.' || c == '-') {
                sim_pressed_ = true;
                int dur = (c == '.') ? 150 : 500;
                press_start_ = SteadyClock::now();
                std::this_thread::sleep_for(std::chrono::milliseconds(dur));
                sim_pressed_ = false;
                release_time_ = SteadyClock::now();
                current_morse_ += c;
                has_pending_ = true;
            } else if (c == ' ') {
                decodeLetter();
            }
        }
        if (!current_morse_.empty()) decodeLetter();
        decoded_text_ += " ";
        publishDecoded();
    }

    // Parameters
    int dot_max_ms_, dash_min_ms_, letter_gap_ms_, word_gap_ms_;
    int button_pin_;
    bool gpio_ready_ = false;

    // State
    bool button_pressed_ = false;
    bool sim_pressed_ = false;
    bool has_pending_ = false;
    bool letter_gap_processed_ = false;
    bool word_gap_processed_ = false;
    TimePoint press_start_;
    TimePoint release_time_;
    std::string current_morse_;
    std::string decoded_text_;

    // ROS2
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr decoded_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr raw_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr char_pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sim_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr gap_timer_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MorseInputNode>());
    rclcpp::shutdown();
    return 0;
}
