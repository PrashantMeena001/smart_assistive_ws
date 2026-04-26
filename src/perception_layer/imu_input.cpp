// =============================================================================
// imu_input.cpp — IMU Orientation Tracking Node
// =============================================================================
// Reads MPU6050/MPU9250 IMU via I2C for pitch, roll, yaw tracking.
// Provides head tilt detection (overhead obstacle awareness) and
// fall/stumble detection for safety alerts.
//
// Topics published:
//   /imu/orientation (std_msgs/String — JSON: pitch, roll, yaw, alerts)
//   /imu/data        (sensor_msgs/Imu — standard ROS2 IMU message)
// =============================================================================

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#ifdef HAS_WIRINGPI
#include <wiringPiI2C.h>
#endif

#include <chrono>
#include <cmath>
#include <string>
#include <sstream>
#include <iomanip>
#include <array>

using namespace std::chrono_literals;

// MPU6050 register addresses
namespace MPU6050 {
    constexpr int ADDR          = 0x68;
    constexpr int PWR_MGMT_1    = 0x6B;
    constexpr int ACCEL_XOUT_H  = 0x3B;
    constexpr int GYRO_XOUT_H   = 0x43;
    constexpr int CONFIG        = 0x1A;
    constexpr int GYRO_CONFIG   = 0x1B;
    constexpr int ACCEL_CONFIG  = 0x1C;
    constexpr float ACCEL_SCALE = 16384.0f;  // ±2g
    constexpr float GYRO_SCALE  = 131.0f;    // ±250°/s
}

class ImuInputNode : public rclcpp::Node {
public:
    ImuInputNode() : Node("imu_input_node") {
        this->declare_parameter<double>("imu_rate_hz", 50.0);
        this->declare_parameter<float>("complementary_alpha", 0.96f);
        this->declare_parameter<float>("fall_threshold", 60.0f);
        this->declare_parameter<float>("stumble_gyro_threshold", 200.0f);
        this->declare_parameter<float>("head_tilt_threshold", 30.0f);

        double rate = this->get_parameter("imu_rate_hz").as_double();
        alpha_ = static_cast<float>(this->get_parameter("complementary_alpha").as_double());
        fall_thresh_ = static_cast<float>(this->get_parameter("fall_threshold").as_double());
        stumble_thresh_ = static_cast<float>(this->get_parameter("stumble_gyro_threshold").as_double());
        tilt_thresh_ = static_cast<float>(this->get_parameter("head_tilt_threshold").as_double());

        // Initialize state
        pitch_ = roll_ = yaw_ = 0.0f;
        last_time_ = this->now();

        // Publishers
        orient_pub_ = this->create_publisher<std_msgs::msg::String>("/imu/orientation", 10);
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu/data", 10);

        // Initialize I2C
        initIMU();

        // Timer
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / rate)),
            std::bind(&ImuInputNode::readAndPublish, this));

        RCLCPP_INFO(get_logger(), "IMU node started @ %.0f Hz, alpha=%.2f", rate, alpha_);
    }

private:
    void initIMU() {
#ifdef HAS_WIRINGPI
        i2c_fd_ = wiringPiI2CSetup(MPU6050::ADDR);
        if (i2c_fd_ < 0) {
            RCLCPP_ERROR(get_logger(), "I2C setup failed for MPU6050");
            imu_ready_ = false;
            return;
        }
        // Wake up MPU6050
        wiringPiI2CWriteReg8(i2c_fd_, MPU6050::PWR_MGMT_1, 0x00);
        // Set DLPF bandwidth ~44Hz for noise filtering
        wiringPiI2CWriteReg8(i2c_fd_, MPU6050::CONFIG, 0x03);
        // Accel ±2g, Gyro ±250°/s
        wiringPiI2CWriteReg8(i2c_fd_, MPU6050::ACCEL_CONFIG, 0x00);
        wiringPiI2CWriteReg8(i2c_fd_, MPU6050::GYRO_CONFIG, 0x00);
        imu_ready_ = true;
        RCLCPP_INFO(get_logger(), "MPU6050 initialized via I2C");

        // Calibrate gyro bias (collect 100 samples at startup)
        calibrateGyro();
#else
        imu_ready_ = false;
        RCLCPP_WARN(get_logger(), "No I2C — using simulated IMU data");
#endif
    }

#ifdef HAS_WIRINGPI
    int16_t readWord(int reg) {
        int hi = wiringPiI2CReadReg8(i2c_fd_, reg);
        int lo = wiringPiI2CReadReg8(i2c_fd_, reg + 1);
        return static_cast<int16_t>((hi << 8) | lo);
    }

    void calibrateGyro() {
        float gx_sum = 0, gy_sum = 0, gz_sum = 0;
        int n = 100;
        for (int i = 0; i < n; i++) {
            gx_sum += readWord(MPU6050::GYRO_XOUT_H) / MPU6050::GYRO_SCALE;
            gy_sum += readWord(MPU6050::GYRO_XOUT_H + 2) / MPU6050::GYRO_SCALE;
            gz_sum += readWord(MPU6050::GYRO_XOUT_H + 4) / MPU6050::GYRO_SCALE;
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        gyro_bias_[0] = gx_sum / n;
        gyro_bias_[1] = gy_sum / n;
        gyro_bias_[2] = gz_sum / n;
        RCLCPP_INFO(get_logger(), "Gyro calibrated — bias: [%.2f, %.2f, %.2f]",
            gyro_bias_[0], gyro_bias_[1], gyro_bias_[2]);
    }
#endif

    struct RawIMU {
        float ax, ay, az;  // m/s²
        float gx, gy, gz;  // °/s
    };

    RawIMU readRaw() {
        RawIMU raw{};
#ifdef HAS_WIRINGPI
        if (!imu_ready_) return raw;
        raw.ax = readWord(MPU6050::ACCEL_XOUT_H) / MPU6050::ACCEL_SCALE * 9.81f;
        raw.ay = readWord(MPU6050::ACCEL_XOUT_H + 2) / MPU6050::ACCEL_SCALE * 9.81f;
        raw.az = readWord(MPU6050::ACCEL_XOUT_H + 4) / MPU6050::ACCEL_SCALE * 9.81f;
        raw.gx = readWord(MPU6050::GYRO_XOUT_H) / MPU6050::GYRO_SCALE - gyro_bias_[0];
        raw.gy = readWord(MPU6050::GYRO_XOUT_H + 2) / MPU6050::GYRO_SCALE - gyro_bias_[1];
        raw.gz = readWord(MPU6050::GYRO_XOUT_H + 4) / MPU6050::GYRO_SCALE - gyro_bias_[2];
#else
        // Simulated stable orientation with slight drift
        static float t = 0;
        t += 0.02f;
        raw.ax = 0.1f * std::sin(t * 0.3f);
        raw.ay = 0.05f * std::cos(t * 0.2f);
        raw.az = 9.81f;
        raw.gx = 0.5f * std::sin(t * 0.1f);
        raw.gy = 0.3f * std::cos(t * 0.15f);
        raw.gz = 0.1f * std::sin(t * 0.05f);
#endif
        return raw;
    }

    void readAndPublish() {
        auto now = this->now();
        double dt = (now - last_time_).seconds();
        if (dt <= 0.0 || dt > 1.0) dt = 0.02;
        last_time_ = now;

        RawIMU raw = readRaw();

        // Complementary filter for pitch and roll
        float accel_pitch = std::atan2(raw.ay, std::sqrt(raw.ax*raw.ax + raw.az*raw.az)) * 180.0f / M_PI;
        float accel_roll = std::atan2(-raw.ax, raw.az) * 180.0f / M_PI;

        pitch_ = alpha_ * (pitch_ + raw.gx * dt) + (1.0f - alpha_) * accel_pitch;
        roll_  = alpha_ * (roll_  + raw.gy * dt) + (1.0f - alpha_) * accel_roll;
        yaw_  += raw.gz * dt;  // Gyro-only (no magnetometer)

        // Normalize yaw to [-180, 180]
        while (yaw_ > 180.0f) yaw_ -= 360.0f;
        while (yaw_ < -180.0f) yaw_ += 360.0f;

        // Detect alerts
        bool fall_detected = (std::abs(pitch_) > fall_thresh_ || std::abs(roll_) > fall_thresh_);
        bool stumble = (std::abs(raw.gx) > stumble_thresh_ || std::abs(raw.gy) > stumble_thresh_);
        bool head_tilted = (std::abs(pitch_) > tilt_thresh_);

        std::string alert = "none";
        if (fall_detected) alert = "fall";
        else if (stumble) alert = "stumble";
        else if (head_tilted) alert = "head_tilt";

        // Publish orientation JSON
        std::ostringstream json;
        json << "{\"timestamp\":" << now.nanoseconds()
             << ",\"pitch\":" << std::fixed << std::setprecision(1) << pitch_
             << ",\"roll\":" << roll_
             << ",\"yaw\":" << yaw_
             << ",\"alert\":\"" << alert << "\""
             << ",\"accel\":{\"x\":" << std::setprecision(2) << raw.ax
             << ",\"y\":" << raw.ay << ",\"z\":" << raw.az << "}"
             << "}";

        auto msg = std_msgs::msg::String();
        msg.data = json.str();
        orient_pub_->publish(msg);

        // Publish standard IMU message for SLAM
        sensor_msgs::msg::Imu imu_msg;
        imu_msg.header.stamp = now;
        imu_msg.header.frame_id = "imu_frame";

        tf2::Quaternion q;
        q.setRPY(roll_ * M_PI / 180.0, pitch_ * M_PI / 180.0, yaw_ * M_PI / 180.0);
        imu_msg.orientation.x = q.x();
        imu_msg.orientation.y = q.y();
        imu_msg.orientation.z = q.z();
        imu_msg.orientation.w = q.w();

        imu_msg.angular_velocity.x = raw.gx * M_PI / 180.0;
        imu_msg.angular_velocity.y = raw.gy * M_PI / 180.0;
        imu_msg.angular_velocity.z = raw.gz * M_PI / 180.0;

        imu_msg.linear_acceleration.x = raw.ax;
        imu_msg.linear_acceleration.y = raw.ay;
        imu_msg.linear_acceleration.z = raw.az;

        imu_pub_->publish(imu_msg);

        if (alert != "none") {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                "IMU alert: %s (pitch=%.1f, roll=%.1f)", alert.c_str(), pitch_, roll_);
        }
    }

    // State
    float pitch_, roll_, yaw_;
    float alpha_;
    float fall_thresh_, stumble_thresh_, tilt_thresh_;
    float gyro_bias_[3] = {0, 0, 0};
    rclcpp::Time last_time_;
    bool imu_ready_ = false;
    int i2c_fd_ = -1;

    // Publishers
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr orient_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImuInputNode>());
    rclcpp::shutdown();
    return 0;
}
