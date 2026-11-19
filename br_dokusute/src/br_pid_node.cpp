// 汎用PIDノード (1入力1出力)。common/PID.hpp の PIDController を使う。
//
//   sub  setpoint     (Float32)  目標
//   sub  measurement  (Float32)  計測値
//   pub  output       (Float32)  操作量 = kff * 目標 + PID
//
// launch で remap して使う (例: 駆動の速度 → duty)。
// 目標が timeout 以上届かなければ何も出さない (下流が止まる)。
// 計測値が届かないときは、フィードフォワードだけを出す。

#include <algorithm>
#include <chrono>
#include <memory>

#include "common/PID.hpp"
#include "br_dokusute/latest_value.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

namespace br_dokusute {

class PidNode : public rclcpp::Node {
 public:
  PidNode() : Node("br_pid") {
    const float kp = declare_parameter<double>("kp", 0.0);
    const float ki = declare_parameter<double>("ki", 0.0);
    const float kd = declare_parameter<double>("kd", 0.0);
    output_limit_ = declare_parameter<double>("output_limit", 255.0);
    pid_ = PIDController(kp, ki, kd, output_limit_);
    kff_ = declare_parameter<double>("kff", 0.0);
    timeout_ = declare_parameter<double>("timeout", 0.5);

    setpoint_sub_ = create_subscription<std_msgs::msg::Float32>(
        "setpoint", 10, [this](const std_msgs::msg::Float32& msg) {
          setpoint_.Set(msg.data, now());
        });
    measurement_sub_ = create_subscription<std_msgs::msg::Float32>(
        "measurement", 10, [this](const std_msgs::msg::Float32& msg) {
          measurement_.Set(msg.data, now());
        });
    output_pub_ = create_publisher<std_msgs::msg::Float32>("output", 10);
    timer_ = create_wall_timer(std::chrono::duration<double>(kPeriod),
                               [this]() { Run(); });
  }

 private:
  static constexpr double kPeriod = 0.02;  // [s]

  void Run() {
    if (!setpoint_.IsFresh(now(), timeout_)) {
      pid_.reset();
      return;  // 何も出さない
    }

    double output = kff_ * setpoint_.value();
    if (measurement_.IsFresh(now(), timeout_)) {
      pid_.set_target(setpoint_.value());
      output += pid_.update(measurement_.value(), kPeriod);
    } else {
      pid_.reset();
    }

    std_msgs::msg::Float32 msg;
    msg.data = std::clamp(output, -output_limit_, output_limit_);
    output_pub_->publish(msg);
  }

  PIDController pid_{0.0f, 0.0f, 0.0f, 0.0f};
  double kff_ = 0.0;  // 目標1あたりのフィードフォワード
  double output_limit_ = 255.0;
  double timeout_ = 0.5;  // [s]

  LatestValue<double> setpoint_;
  LatestValue<double> measurement_;

  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr setpoint_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr measurement_sub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr output_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace br_dokusute

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<br_dokusute::PidNode>());
  rclcpp::shutdown();
  return 0;
}
