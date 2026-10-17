/*
NR27 BR 独立ステア 1モジュール分ノード

cmd_vel (ロボット座標系の vx, vy, wz) を、このモジュールの取り付け位置から見た
「ホイール速度」と「ステア角」に変換し、ros2can 経由でMD(ドライブ/ステア)に送る。
モジュール数が可変なので、1ノード=1モジュール。複数枚はパラメータを変えて複数起動する。

*/

// 標準
#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

// ROS
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"

namespace {

constexpr int kTxSlotNum = 24;
constexpr double kMinSpeed = 0.02;       // これ未満の速度ではステア目標角を更新しない [m/s]
constexpr double kSpeedFilterGain = 0.3; // ドライブ速度LPFの新しい値の重み

// 角度を [-pi, pi] に正規化する
double WrapPi(double angle) { return std::atan2(std::sin(angle), std::cos(angle)); }

// ホイールの目標(ステア角とドライブ速度)
struct WheelTarget {
    double angle;  // [rad]
    double speed;  // [m/s]  後退側は負
};

}  // namespace

class SwerveModule : public rclcpp::Node {
public:
    SwerveModule() : Node("br_swerve_module") {
        DeclareParameters();

        tx_data_.assign(kTxSlotNum, 0);

        const std::string id = std::to_string(device_id_);
        tx_pub_ = create_publisher<std_msgs::msg::Int16MultiArray>("serial_tx_" + id, 10);
        state_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>(
            "swerve/module_state_" + id, 10);

        cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10,
            std::bind(&SwerveModule::CmdCallback, this, std::placeholders::_1));
        // 原点セットからの相対カウント(int16のラップ展開済み)
        enc_sub_ = create_subscription<std_msgs::msg::Int32MultiArray>(
            "serial_rx_" + id + "_zeroed", 10,
            std::bind(&SwerveModule::EncCallback, this, std::placeholders::_1));

        timer_ = create_wall_timer(
            std::chrono::milliseconds(period_ms_),
            std::bind(&SwerveModule::ControlLoop, this));

        last_cmd_time_ = now();
        RCLCPP_INFO(get_logger(), "swerve module started: device %d, pos (%.2f, %.2f)",
                    device_id_, module_x_, module_y_);
    }

    void SendZero() {
        std::fill(tx_data_.begin(), tx_data_.end(), 0);
        PublishTx();
    }

private:
    // ===== 初期化 =====
    void DeclareParameters() {
        device_id_ = declare_parameter<int>("device_id", 1);

        // 取り付け位置(ロボット中心から) [m]
        module_x_ = declare_parameter<double>("module_x", 0.25);
        module_y_ = declare_parameter<double>("module_y", 0.25);

        // 【仮置き】スロット割当 (index = node * slots_per_node + local)
        const int node = declare_parameter<int>("node", 0);
        const int slots_per_node = declare_parameter<int>("slots_per_node", 5);
        tx_drive_slot_ = declare_parameter<int>("tx_drive_slot", node * slots_per_node + 0);
        tx_steer_slot_ = declare_parameter<int>("tx_steer_slot", node * slots_per_node + 1);
        rx_drive_slot_ = declare_parameter<int>("rx_drive_slot", node * slots_per_node + 1);
        rx_steer_slot_ = declare_parameter<int>("rx_steer_slot", node * slots_per_node + 2);

        // 【仮置き】機構
        wheel_radius_ = declare_parameter<double>("wheel_radius", 0.05);                  // [m]
        drive_counts_per_rev_ = declare_parameter<double>("drive_counts_per_rev", 1024.0);
        steer_counts_per_rev_ = declare_parameter<double>("steer_counts_per_rev", 1024.0);  // ステア軸1回転
        drive_sign_ = declare_parameter<int>("drive_sign", 1);
        steer_sign_ = declare_parameter<int>("steer_sign", 1);
        max_wheel_speed_ = declare_parameter<double>("max_wheel_speed", 2.0);  // duty_max に対応 [m/s]

        // 制御
        duty_max_ = declare_parameter<double>("duty_max", 255.0);
        steer_duty_max_ = declare_parameter<double>("steer_duty_max", 150.0);
        steer_kp_ = declare_parameter<double>("steer_kp", 120.0);  // [duty/rad]
        steer_kd_ = declare_parameter<double>("steer_kd", 10.0);   // [duty/(rad/s)]
        steer_deadband_ = declare_parameter<double>("steer_deadband", 0.02);  // [rad]
        cmd_timeout_ = declare_parameter<double>("cmd_timeout", 0.5);         // [s]
        period_ms_ = declare_parameter<int>("period_ms", 20);
    }

    // ===== 受信 =====
    void CmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        vx_ = msg->linear.x;
        vy_ = msg->linear.y;
        wz_ = msg->angular.z;
        last_cmd_time_ = now();
    }

    void EncCallback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
        const size_t required = static_cast<size_t>(std::max(rx_drive_slot_, rx_steer_slot_)) + 1;
        if (msg->data.size() < required) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "zeroed data too short (%zu)", msg->data.size());
            return;
        }
        UpdateSteerAngle(msg->data[rx_steer_slot_]);
        UpdateWheelSpeed(msg->data[rx_drive_slot_]);
    }

    void UpdateSteerAngle(int32_t steer_count) {
        steer_angle_ = WrapPi(steer_sign_ * 2.0 * M_PI * steer_count / steer_counts_per_rev_);
    }

    // ドライブエンコーダの差分から速度を求める
    void UpdateWheelSpeed(int32_t drive_count) {
        const rclcpp::Time t = now();
        if (drive_initialized_) {
            const double dt = (t - drive_last_time_).seconds();
            if (dt > 0.0) {
                const double revs = (drive_count - drive_last_count_) / drive_counts_per_rev_;
                const double speed = drive_sign_ * revs * 2.0 * M_PI * wheel_radius_ / dt;
                wheel_speed_ += kSpeedFilterGain * (speed - wheel_speed_);
            }
        }
        drive_last_count_ = drive_count;
        drive_last_time_ = t;
        drive_initialized_ = true;
    }

    // ===== 制御 =====
    void ControlLoop() {
        const rclcpp::Time t = now();
        const double dt = (t - last_loop_time_).seconds();
        last_loop_time_ = t;

        // 指令が途切れたら停止(ステアは現状維持)
        if ((t - last_cmd_time_).seconds() > cmd_timeout_) {
            vx_ = vy_ = wz_ = 0.0;
        }

        WheelTarget target = ComputeWheelTarget();
        const double steer_error = ApplyShortestRotation(target);
        const double steer_duty = ComputeSteerDuty(steer_error, dt);
        const double drive_duty = ComputeDriveDuty(target.speed, steer_error);

        tx_data_[tx_drive_slot_] = static_cast<int16_t>(drive_duty);
        tx_data_[tx_steer_slot_] = static_cast<int16_t>(steer_duty);
        PublishTx();
        PublishState(drive_duty);
    }

    // cmd_vel から、このモジュール位置でのホイール速度ベクトルを求める
    WheelTarget ComputeWheelTarget() {
        const double wheel_vx = vx_ - wz_ * module_y_;
        const double wheel_vy = vy_ + wz_ * module_x_;
        const double speed = std::hypot(wheel_vx, wheel_vy);

        // 停止付近では目標角を保持して、ステアを振らせない
        if (speed < kMinSpeed) {
            return {target_angle_, 0.0};
        }
        target_angle_ = std::atan2(wheel_vy, wheel_vx);
        return {target_angle_, speed};
    }

    // 目標角との差が90°を超えるなら逆向きに回し、ドライブを反転する。
    // 戻り値は補正後のステア誤差 [rad]。
    double ApplyShortestRotation(WheelTarget &target) const {
        double error = WrapPi(target.angle - steer_angle_);
        if (std::fabs(error) > M_PI / 2.0) {
            error -= std::copysign(M_PI, error);
            target.speed = -target.speed;
        }
        return error;
    }

    // ステアのPD制御。微分は角度の変化から求める。
    double ComputeSteerDuty(double error, double dt) {
        double steer_rate = 0.0;
        if (dt > 0.0 && prev_angle_valid_) {
            steer_rate = WrapPi(steer_angle_ - prev_angle_) / dt;
        }
        prev_angle_ = steer_angle_;
        prev_angle_valid_ = true;

        if (std::fabs(error) <= steer_deadband_) {
            return 0.0;
        }
        const double duty = steer_kp_ * error - steer_kd_ * steer_rate;
        return steer_sign_ * std::clamp(duty, -steer_duty_max_, steer_duty_max_);
    }

    // ドライブはオープンループ。ステアが目標を向くまで cos(誤差) で抑える。
    double ComputeDriveDuty(double speed, double steer_error) const {
        const double scaled = speed * std::max(0.0, std::cos(steer_error));
        const double ratio = std::clamp(scaled / max_wheel_speed_, -1.0, 1.0);
        return ratio * duty_max_ * drive_sign_;
    }

    // ===== 送信 =====
    void PublishTx() {
        std_msgs::msg::Int16MultiArray msg;
        msg.data = tx_data_;
        tx_pub_->publish(msg);
    }

    // [ステア角, ホイール速度, 目標角, ドライブduty]
    void PublishState(double drive_duty) {
        std_msgs::msg::Float32MultiArray msg;
        msg.data = {static_cast<float>(steer_angle_), static_cast<float>(wheel_speed_),
                    static_cast<float>(target_angle_), static_cast<float>(drive_duty)};
        state_pub_->publish(msg);
    }

    // ===== パラメータ =====
    int device_id_;
    double module_x_, module_y_;
    int tx_drive_slot_, tx_steer_slot_, rx_drive_slot_, rx_steer_slot_;
    double wheel_radius_, drive_counts_per_rev_, steer_counts_per_rev_, max_wheel_speed_;
    int drive_sign_, steer_sign_;
    double duty_max_, steer_duty_max_, steer_kp_, steer_kd_, steer_deadband_, cmd_timeout_;
    int period_ms_;

    // ===== 状態 =====
    double vx_ = 0.0, vy_ = 0.0, wz_ = 0.0;  // 最新の cmd_vel
    double target_angle_ = 0.0;
    double steer_angle_ = 0.0;
    double wheel_speed_ = 0.0;
    double prev_angle_ = 0.0;
    bool prev_angle_valid_ = false;
    double drive_last_count_ = 0.0;
    bool drive_initialized_ = false;
    rclcpp::Time drive_last_time_{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_cmd_time_{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_loop_time_{0, 0, RCL_ROS_TIME};

    std::vector<int16_t> tx_data_;

    // ===== ROS =====
    rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr tx_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr state_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr enc_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SwerveModule>();
    rclcpp::spin(node);
    node->SendZero();
    rclcpp::shutdown();
    return 0;
}
