// 独ステ1ユニットの運動学ノード。
//
//   sub  cmd_vel            (Twist)    ロボット全体の速度指令
//   sub  steer/angle        (Float32)  今のステア角 [rad]
//   pub  steer/angle_target (Float32)  ステア角の目標 [rad]
//   pub  drive/speed_target (Float32)  ホイール速度の目標 [m/s]。逆転は負
//
// ステア角は ±steer_limit の範囲に収める (配線のねじれ防止)。
// 範囲内で今の角度に一番近い向きを選び、必要なら車輪を逆転させる。
// ステア角が届いていない間は何も出さない (下流のノードが止まる)。

#include <algorithm>
#include <cmath>
#include <memory>

#include "geometry_msgs/msg/twist.hpp"
#include "tr_dokusute/latest_value.hpp"
#include "tr_dokusute/swerve_kinematics.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

namespace tr_dokusute {

// これ未満の車輪速度ではステア目標を更新しない [m/s]。
// 速度0付近では atan2 の向きが定まらず、ステアが暴れるため。
constexpr double kMinWheelSpeed = 0.02;

class SwerveKinematicsNode : public rclcpp::Node {
 public:
  SwerveKinematicsNode() : Node("tr_swerve_kinematics") {
    module_x_ = declare_parameter<double>("module_x", 0.25);
    module_y_ = declare_parameter<double>("module_y", 0.25);
    steer_limit_ = declare_parameter<double>("steer_limit", M_PI);
    timeout_ = declare_parameter<double>("timeout", 0.5);
    if (steer_limit_ < M_PI / 2.0) {
      RCLCPP_WARN(get_logger(), "steer_limit が小さすぎるので pi/2 にします");
      steer_limit_ = M_PI / 2.0;
    }

    angle_sub_ = create_subscription<std_msgs::msg::Float32>(
        "steer/angle", 10, [this](const std_msgs::msg::Float32& msg) {
          angle_.Set(msg.data, now());
        });
    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10,
        [this](const geometry_msgs::msg::Twist& msg) { OnCmdVel(msg); });
    angle_target_pub_ =
        create_publisher<std_msgs::msg::Float32>("steer/angle_target", 10);
    speed_target_pub_ =
        create_publisher<std_msgs::msg::Float32>("drive/speed_target", 10);
  }

 private:
  void OnCmdVel(const geometry_msgs::msg::Twist& msg) {
    // 今のステア角がわからないと、どちらへ回すか決められない
    if (!angle_.IsFresh(now(), timeout_)) return;
    const double angle = angle_.value();

    // 指令が途切れていたなら、古い目標ではなく今の角度から始める
    // (止まっている間に手で回されていても、急に戻らないように)
    if (!cmd_.IsFresh(now(), timeout_)) target_angle_ = angle;
    cmd_.Set(msg, now());

    const BodyVelocity body{msg.linear.x, msg.linear.y, msg.angular.z};
    const WheelCommand wheel = ComputeWheelVelocity(body, module_x_, module_y_);

    double speed = 0.0;
    if (wheel.speed >= kMinWheelSpeed) {
      const WheelCommand choice =
          ChooseSteerAngle(wheel.angle, angle, steer_limit_);
      target_angle_ = choice.angle;
      speed = choice.speed * wheel.speed;
    }
    // ステアが目標を向くまで cos(ずれ) で抑える (90°以上ずれていれば0)
    speed *= std::max(0.0, std::cos(target_angle_ - angle));

    std_msgs::msg::Float32 angle_target;
    std_msgs::msg::Float32 speed_target;
    angle_target.data = static_cast<float>(target_angle_);
    speed_target.data = static_cast<float>(speed);
    angle_target_pub_->publish(angle_target);
    speed_target_pub_->publish(speed_target);
  }

  double module_x_ = 0.25;  // ロボット中心からの位置 [m] (前が+)
  double module_y_ = 0.25;  // [m] (左が+)
  double steer_limit_ = M_PI;  // ステアの可動範囲 ±[rad]
  double timeout_ = 0.5;       // [s]
  double target_angle_ = 0.0;  // 最後に出したステア角の目標 [rad]

  LatestValue<float> angle_;                    // 今のステア角 [rad]
  LatestValue<geometry_msgs::msg::Twist> cmd_;  // 最後の cmd_vel

  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr angle_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr angle_target_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr speed_target_pub_;
};

}  // namespace tr_dokusute

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<tr_dokusute::SwerveKinematicsNode>());
  rclcpp::shutdown();
  return 0;
}
