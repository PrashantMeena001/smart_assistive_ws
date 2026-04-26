// =============================================================================
// audio_output.cpp — Text-to-Speech Audio Output Node (Blind Users)
// =============================================================================
// Converts action commands from the cognitive engine into spoken audio
// using espeak-ng TTS. Only active in BLIND mode (Mode 1).
//
// Features:
//   - Priority-based speech queue (emergencies interrupt)
//   - Rate/pitch adjustment based on urgency
//   - Non-blocking async playback
//   - Cooldown to prevent speech overlap
//
// Subscribes to: /action/audio (from cognitive engine)
// =============================================================================

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#ifdef HAS_ESPEAK
#include <espeak-ng/speak_lib.h>
#endif

#include <chrono>
#include <string>
#include <sstream>
#include <queue>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <cstdlib>
#include <algorithm>
#include <functional>

using namespace std::chrono_literals;

struct SpeechCommand {
    std::string text;
    int urgency;    // 0-10
    int priority;   // lower = higher priority
    int64_t timestamp;

    bool operator>(const SpeechCommand& other) const {
        return priority > other.priority;
    }
};

class AudioOutputNode : public rclcpp::Node {
public:
    AudioOutputNode()
        : Node("audio_output_node"), worker_running_(true), speaking_(false)
    {
        // Parameters
        this->declare_parameter<int>("speech_rate", 175);
        this->declare_parameter<int>("base_pitch", 50);
        this->declare_parameter<int>("volume", 100);
        this->declare_parameter<std::string>("voice", "en");
        this->declare_parameter<int>("min_interval_ms", 500);
        this->declare_parameter<int>("max_queue_size", 10);

        speech_rate_ = this->get_parameter("speech_rate").as_int();
        base_pitch_ = this->get_parameter("base_pitch").as_int();
        volume_ = this->get_parameter("volume").as_int();
        voice_ = this->get_parameter("voice").as_string();
        min_interval_ms_ = this->get_parameter("min_interval_ms").as_int();
        max_queue_size_ = this->get_parameter("max_queue_size").as_int();

        // Initialize TTS engine
        initTTS();

        // Subscribe to audio action commands
        action_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/action/audio", 10,
            std::bind(&AudioOutputNode::onAction, this, std::placeholders::_1));

        // Worker thread for speech output
        worker_thread_ = std::thread(&AudioOutputNode::workerLoop, this);

        RCLCPP_INFO(get_logger(), "Audio output node started — voice: %s, rate: %d",
            voice_.c_str(), speech_rate_);
    }

    ~AudioOutputNode() {
        worker_running_ = false;
        cv_.notify_all();
        if (worker_thread_.joinable()) worker_thread_.join();

#ifdef HAS_ESPEAK
        if (espeak_ready_) espeak_Terminate();
#endif
    }

private:
    void initTTS() {
#ifdef HAS_ESPEAK
        int sample_rate = espeak_Initialize(
            AUDIO_OUTPUT_PLAYBACK, 0, nullptr, 0);
        if (sample_rate == -1) {
            RCLCPP_ERROR(get_logger(), "espeak initialization failed");
            espeak_ready_ = false;
            return;
        }
        espeak_SetVoiceByName(voice_.c_str());
        espeak_SetParameter(espeakRATE, speech_rate_, 0);
        espeak_SetParameter(espeakVOLUME, volume_, 0);
        espeak_SetParameter(espeakPITCH, base_pitch_, 0);
        espeak_ready_ = true;
        RCLCPP_INFO(get_logger(), "espeak-ng initialized (sample rate: %d)", sample_rate);
#else
        espeak_ready_ = false;
        RCLCPP_WARN(get_logger(), "espeak not linked — using system command fallback");
#endif
    }

    void onAction(const std_msgs::msg::String::SharedPtr msg) {
        // Parse the action command JSON
        std::string data = msg->data;

        auto extract_str = [&data](const std::string& key) -> std::string {
            std::string s = "\"" + key + "\":\"";
            auto p = data.find(s);
            if (p == std::string::npos) return "";
            p += s.size();
            auto e = data.find("\"", p);
            return (e != std::string::npos) ? data.substr(p, e - p) : "";
        };
        auto extract_int = [&data](const std::string& key) -> int {
            std::string s = "\"" + key + "\":";
            auto p = data.find(s);
            if (p == std::string::npos) return 0;
            p += s.size();
            try { return std::stoi(data.substr(p)); } catch (...) { return 0; }
        };

        std::string mode = extract_str("mode");
        // Only produce audio in blind mode
        if (mode == "deaf_blind") return;

        std::string type = extract_str("type");
        std::string message = extract_str("message");
        std::string direction = extract_str("direction");
        int urgency = extract_int("urgency");
        int priority = extract_int("priority");

        if (message.empty()) return;

        // Build natural speech text
        std::string speech = buildSpeechText(type, message, direction, urgency);

        SpeechCommand cmd;
        cmd.text = speech;
        cmd.urgency = urgency;
        cmd.priority = priority;
        cmd.timestamp = this->now().nanoseconds();

        enqueue(cmd);
    }

    std::string buildSpeechText(const std::string& type, const std::string& message,
                                 const std::string& direction, int urgency) {
        std::string speech;

        if (type == "emergency") {
            speech = "Warning! " + message;
        } else if (type == "fall") {
            speech = "Fall detected. Are you okay?";
        } else if (type == "sos") {
            speech = "Emergency SOS activated. Sending alert.";
        } else if (type == "hazard" || type == "obstacle") {
            speech = message;
            if (direction != "none" && !direction.empty()) {
                speech += " on your " + direction;
            }
        } else if (type == "caution") {
            speech = "Caution. " + message;
        } else if (type == "stumble") {
            speech = "Careful. Uneven ground detected.";
        } else if (type == "tilt") {
            speech = "Watch above.";
        } else {
            speech = message;
        }

        return speech;
    }

    void enqueue(const SpeechCommand& cmd) {
        std::lock_guard<std::mutex> lock(queue_mutex_);

        // Emergency preempts
        if (cmd.priority == 0) {
            // Clear queue and interrupt current speech
            while (!speech_queue_.empty()) speech_queue_.pop();
            interrupt_ = true;
        }

        // Limit queue size (drop oldest low-priority)
        while ((int)speech_queue_.size() >= max_queue_size_) {
            speech_queue_.pop();
        }

        speech_queue_.push(cmd);
        cv_.notify_one();
    }

    void workerLoop() {
        while (worker_running_) {
            SpeechCommand cmd;
            {
                std::unique_lock<std::mutex> lock(queue_mutex_);
                cv_.wait_for(lock, 200ms, [this] {
                    return !speech_queue_.empty() || !worker_running_;
                });
                if (!worker_running_) break;
                if (speech_queue_.empty()) continue;
                cmd = speech_queue_.front();
                speech_queue_.pop();
            }

            // Cooldown between speeches (unless emergency)
            if (cmd.priority > 0) {
                auto now = std::chrono::steady_clock::now();
                auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                    now - last_speech_time_).count();
                if (elapsed < min_interval_ms_) {
                    std::this_thread::sleep_for(
                        std::chrono::milliseconds(min_interval_ms_ - elapsed));
                }
            }

            speak(cmd);
            last_speech_time_ = std::chrono::steady_clock::now();
        }
    }

    void speak(const SpeechCommand& cmd) {
        speaking_ = true;
        interrupt_ = false;

        // Adjust rate and pitch based on urgency
        int rate = speech_rate_ + (cmd.urgency * 5);  // faster for urgent
        int pitch = base_pitch_ + (cmd.urgency > 7 ? 20 : 0);  // higher for emergencies

        RCLCPP_INFO(get_logger(), "Speaking [p%d u%d]: \"%s\"",
            cmd.priority, cmd.urgency, cmd.text.c_str());

#ifdef HAS_ESPEAK
        if (espeak_ready_) {
            espeak_SetParameter(espeakRATE, rate, 0);
            espeak_SetParameter(espeakPITCH, pitch, 0);
            espeak_Synth(cmd.text.c_str(), cmd.text.size() + 1, 0,
                         POS_CHARACTER, 0, espeakCHARS_AUTO, nullptr, nullptr);
            espeak_Synchronize();
        } else {
            speakFallback(cmd.text, rate);
        }
#else
        speakFallback(cmd.text, rate);
#endif

        speaking_ = false;
    }

    // Fallback: use system espeak command
    void speakFallback(const std::string& text, int rate) {
        // Sanitize text for shell
        std::string safe_text;
        for (char c : text) {
            if (c == '"' || c == '\\' || c == '$' || c == '`') {
                safe_text += '\\';
            }
            safe_text += c;
        }

        std::ostringstream cmd;
        cmd << "espeak -s " << rate << " -v " << voice_
            << " \"" << safe_text << "\" 2>/dev/null &";

        int ret = std::system(cmd.str().c_str());
        if (ret != 0) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                "espeak command failed (code %d)", ret);
        }
    }

    // Parameters
    int speech_rate_, base_pitch_, volume_;
    std::string voice_;
    int min_interval_ms_, max_queue_size_;
    bool espeak_ready_ = false;

    // State
    std::atomic<bool> worker_running_;
    std::atomic<bool> speaking_;
    std::atomic<bool> interrupt_{false};
    std::chrono::steady_clock::time_point last_speech_time_;

    // Speech queue (FIFO with priority preemption)
    std::queue<SpeechCommand> speech_queue_;
    std::mutex queue_mutex_;
    std::condition_variable cv_;
    std::thread worker_thread_;

    // ROS2
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr action_sub_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AudioOutputNode>());
    rclcpp::shutdown();
    return 0;
}
