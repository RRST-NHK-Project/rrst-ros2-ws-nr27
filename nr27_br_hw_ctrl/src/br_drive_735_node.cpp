// 駆動 (マブチ735) のハードウェアノード。
// ros2can CAN_HOST 配下の MES基板 (MD1 = PWM出力、ENC1 = エンコーダ) を使う。
//
//   sub  drive/duty  (Float32)  MDへのduty (符号=向き)
//   pub  drive/speed (Float32)  エンコーダから求めた車輪速度 [m/s]
//
// 速度制御は br_pid ノードの役割。ここは duty を送り、速度を測るだけ。
//
// 安全:
//   - duty が届かなければ 0 を送る
//   - 終了時は 0 を送る (stop_on_shutdown.hpp)

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include "nr27_br_hw_ctrl/latest_value.hpp"
#include "nr27_br_hw_ctrl/stop_on_shutdown.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"

namespace nr27_br_hw_ctrl {

constexpr size_t kTxSlotCount = 24;
constexpr double kSpeedFilterGain = 0.3;  // 速度ローパスの新しい値の重み

class Drive735Node : public rclcpp::Node {
 public:
  Drive735Node() : Node("br_drive_735") {
    const std::string id =
        std::to_string(declare_parameter<int>("device_id", 1));
    tx_slot_ = declare_parameter<int>("tx_slot", 0);
    rx_slot_ = declare_parameter<int>("rx_slot", 1);
    wheel_radius_ = declare_parameter<double>("wheel_radius", 0.05);
    counts_per_rev_ = declare_parameter<double>("counts_per_rev", 1024.0);
    duty_sign_ = declare_parameter<int>("duty_sign", 1);
    encoder_sign_ = declare_parameter<int>("encoder_sign", 1);
    duty_max_ = declare_parameter<double>("duty_max", 255.0);
    timeout_ = declare_parameter<double>("timeout", 0.5);

    tx_pub_ =
        create_publisher<std_msgs::msg::Int16MultiArray>("serial_tx_" + id, 10);
    // PCNT は ±32767 でリセットされるので、ros2can が連続値に直した
    // _unwrapped を使う
    encoder_sub_ = create_subscription<std_msgs::msg::Int32MultiArray>(
        "serial_rx_" + id + "_unwrapped", 10,
        [this](const std_msgs::msg::Int32MultiArray& msg) { OnEncoder(msg); });
    duty_sub_ = create_subscription<std_msgs::msg::Float32>(
        "drive/duty", 10, [this](const std_msgs::msg::Float32& msg) {
          duty_.Set(msg.data, now());
        });
    speed_pub_ = create_publisher<std_msgs::msg::Float32>("drive/speed", 10);
    timer_ =
        create_wall_timer(std::chrono::milliseconds(20), [this]() { Run(); });
    stop_ = std::make_unique<StopOnShutdown>(*this, tx_pub_, kTxSlotCount);
  }

 private:
  void OnEncoder(const std_msgs::msg::Int32MultiArray& msg) {
    if (msg.data.size() <= static_cast<size_t>(rx_slot_)) return;
    const int32_t count = msg.data[rx_slot_];

    if (count_.has_value()) {
      const double dt = (now() - count_.stamp()).seconds();
      const double revs = (count - count_.value()) / counts_per_rev_;
      const double speed =
          encoder_sign_ * revs * 2.0 * M_PI * wheel_radius_ / dt;
      speed_ += kSpeedFilterGain * (speed - speed_);
    }
    count_.Set(count, now());

    std_msgs::msg::Float32 out;
    out.data = static_cast<float>(speed_);
    speed_pub_->publish(out);
  }

  void Run() {
    if (stop_->is_stopping()) return;

    std_msgs::msg::Int16MultiArray msg;
    msg.data.assign(kTxSlotCount, 0);  // 全ゼロ = duty 0
    if (duty_.IsFresh(now(), timeout_)) {
      const double duty =
          std::clamp(duty_sign_ * duty_.value(), -duty_max_, duty_max_);
      msg.data[tx_slot_] = static_cast<int16_t>(std::lround(duty));
    }
    tx_pub_->publish(msg);
  }

  int tx_slot_ = 0;  // MES基板 MD1 のスロット (ノード番号*5+0)
  int rx_slot_ = 1;  // MES基板 ENC1 のスロット (ノード番号*5+1)
  double wheel_radius_ = 0.05;      // [m]
  double counts_per_rev_ = 1024.0;  // 車輪1回転のENCカウント
  int duty_sign_ = 1;               // +dutyで前進なら1
  int encoder_sign_ = 1;            // 前進でカウントが増えるなら1
  double duty_max_ = 255.0;         // MD_PWM_MAX (8bit)
  double timeout_ = 0.5;            // [s]

  double speed_ = 0.0;          // 測った車輪速度 [m/s]
  LatestValue<double> duty_;    // 最後に受けた duty
  LatestValue<int32_t> count_;  // 最後のエンコーダカウント

  rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr tx_pub_;
  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr encoder_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr duty_sub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr speed_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<StopOnShutdown> stop_;
};

}  // namespace nr27_br_hw_ctrl

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<nr27_br_hw_ctrl::Drive735Node>());
  rclcpp::shutdown();
  return 0;
}
