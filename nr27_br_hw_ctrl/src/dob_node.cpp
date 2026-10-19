/* 足回り用の外乱オブザーバ (DOB) ノード。

 入出力:
   sub  cmd_vel_ref      (geometry_msgs/Twist)  上位からの速度指令
   sub  odom             (nav_msgs/Odometry)    計測速度 (twist はベース座標系)
   pub  cmd_vel          (geometry_msgs/Twist)  外乱補償後の速度指令
   pub  dob/disturbance  (geometry_msgs/Twist)  外乱推定値 (確認用)

 公称モデル (各軸独立): Pn(s) = K / (T s + 1)   入力: 速度指令 u, 出力: 速度 v
 外乱 d は入力側に加わるものとする:  v = Pn (u + d)

 推定:  d_hat = Q(s) * (Pn^-1(s) v - u),   Q(s) = 1 / (tau s + 1)

 Pn^-1 は非プロパーなので、次のように分解して微分を使わずに実装する。
   Q * Pn^-1 * v = (1/K) * (T * s/(tau s + 1) + 1/(tau s + 1)) v
   1/(tau s + 1) v   = LPF(v)
   s/(tau s + 1) v   = (v - LPF(v)) / tau
   Q * u             = LPF(u)
補償:  u = u_ref - d_hat*/

#include <algorithm>
#include <array>
#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

namespace {

constexpr int kNumAxes = 3;  // vx, vy, wz

// 1軸分の外乱オブザーバ。
class DisturbanceObserver {
 public:
  DisturbanceObserver(double gain, double time_constant, double q_tau)
      : gain_(gain), time_constant_(time_constant), q_tau_(q_tau) {}

  // u: 実際にプラントへ入力した指令, v: 計測速度, dt: 周期 [s]。
  // 外乱推定値を返す。
  double Update(double u, double v, double dt) {
    const double alpha = dt / (q_tau_ + dt);  // 後退オイラーの 1次LPF
    v_filtered_ += alpha * (v - v_filtered_);
    u_filtered_ += alpha * (u - u_filtered_);
    const double v_dot_filtered = (v - v_filtered_) / q_tau_;
    d_hat_ = (time_constant_ * v_dot_filtered + v_filtered_) / gain_ -
             u_filtered_;
    return d_hat_;
  }

  void Reset() {
    v_filtered_ = 0.0;
    u_filtered_ = 0.0;
    d_hat_ = 0.0;
  }

  double d_hat() const { return d_hat_; }

 private:
  double gain_;
  double time_constant_;
  double q_tau_;
  double v_filtered_ = 0.0;
  double u_filtered_ = 0.0;
  double d_hat_ = 0.0;
};

}  // namespace

class DisturbanceObserverNode : public rclcpp::Node {
 public:
  DisturbanceObserverNode() : Node("disturbance_observer") {
    declare_parameter<double>("control_rate", 20.0);  // [Hz]
    declare_parameter<double>("timeout", 0.5);         // [s]
    declare_parameter<std::vector<double>>("gain", {1.0, 1.0, 1.0});
    declare_parameter<std::vector<double>>("time_constant", {0.1, 0.1, 0.1});
    declare_parameter<std::vector<double>>("q_tau", {0.1, 0.1, 0.1});
    declare_parameter<std::vector<double>>("max_disturbance", {0.5, 0.5, 1.0});

    const double rate = get_parameter("control_rate").as_double();
    timeout_ = get_parameter("timeout").as_double();
    dt_ = 1.0 / rate;

    const auto gain = GetAxisParameter("gain");
    const auto time_constant = GetAxisParameter("time_constant");
    const auto q_tau = GetAxisParameter("q_tau");
    max_disturbance_ = GetAxisParameter("max_disturbance");

    for (int i = 0; i < kNumAxes; ++i) {
      observers_.emplace_back(gain[i], time_constant[i], q_tau[i]);
    }

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    disturbance_pub_ =
        create_publisher<geometry_msgs::msg::Twist>("dob/disturbance", 10);
    ref_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel_ref", 10,
        [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
          OnRef(*msg);
        });
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "odom", 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
          OnOdom(*msg);
        });
    timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(dt_)),
        [this]() { ControlLoop(); });
  }

 private:
  std::vector<double> GetAxisParameter(const std::string& name) {
    auto values = get_parameter(name).as_double_array();
    if (values.size() != kNumAxes) {
      RCLCPP_FATAL(get_logger(), "parameter '%s' must have %d elements",
                   name.c_str(), kNumAxes);
      throw std::runtime_error("invalid parameter size: " + name);
    }
    return values;
  }

  void OnRef(const geometry_msgs::msg::Twist& msg) {
    ref_[0] = msg.linear.x;
    ref_[1] = msg.linear.y;
    ref_[2] = msg.angular.z;
    last_ref_time_ = now();
    has_ref_ = true;
  }

  void OnOdom(const nav_msgs::msg::Odometry& msg) {
    measured_[0] = msg.twist.twist.linear.x;
    measured_[1] = msg.twist.twist.linear.y;
    measured_[2] = msg.twist.twist.angular.z;
    last_odom_time_ = now();
    has_odom_ = true;
  }

  void ControlLoop() {
    const rclcpp::Time t = now();
    const bool ref_ok =
        has_ref_ && (t - last_ref_time_).seconds() < timeout_;
    const bool odom_ok =
        has_odom_ && (t - last_odom_time_).seconds() < timeout_;

    // 入力が途切れたら観測器をリセットして停止指令を出す。
    if (!ref_ok || !odom_ok) {
      for (auto& observer : observers_) observer.Reset();
      PublishCommand({0.0, 0.0, 0.0});
      PublishDisturbance();
      return;
    }

    // 直前の推定値で補償し (1ステップ遅れ)、実際に出した u で推定を更新する。
    std::array<double, kNumAxes> u;
    for (int i = 0; i < kNumAxes; ++i) {
      const double d = std::clamp(observers_[i].d_hat(), -max_disturbance_[i],
                                  max_disturbance_[i]);
      u[i] = ref_[i] - d;
    }
    PublishCommand(u);

    for (int i = 0; i < kNumAxes; ++i) {
      observers_[i].Update(u[i], measured_[i], dt_);
    }
    PublishDisturbance();
  }

  void PublishCommand(const std::array<double, kNumAxes>& u) {
    geometry_msgs::msg::Twist msg;
    msg.linear.x = u[0];
    msg.linear.y = u[1];
    msg.angular.z = u[2];
    cmd_pub_->publish(msg);
  }

  void PublishDisturbance() {
    geometry_msgs::msg::Twist msg;
    msg.linear.x = observers_[0].d_hat();
    msg.linear.y = observers_[1].d_hat();
    msg.angular.z = observers_[2].d_hat();
    disturbance_pub_->publish(msg);
  }

  //Ros
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr disturbance_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr ref_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::vector<DisturbanceObserver> observers_;
  std::vector<double> max_disturbance_;
  std::array<double, kNumAxes> ref_ = {0.0, 0.0, 0.0};
  std::array<double, kNumAxes> measured_ = {0.0, 0.0, 0.0};
  rclcpp::Time last_ref_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_odom_time_{0, 0, RCL_ROS_TIME};
  bool has_ref_ = false;
  bool has_odom_ = false;
  double timeout_ = 0.5;
  double dt_ = 0.01;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DisturbanceObserverNode>());
  rclcpp::shutdown();
  return 0;
}