// ステア (DAMIAO DM2325 + DM3520 ESC) のハードウェアノード。
// ros2can MODE_DAMIAO のESP32 と serial_tx/rx でやりとりする。
//
//   sub  steer/angle_target (Float32)  ステア角の目標 [rad]
//   pub  steer/angle        (Float32)  今のステア角 [rad]
//   srv  steer/set_zero     (Trigger)  今の向きを「前」として登録する
//
// 位置制御はESCの中で回るので、ここではPIDしない (POS_VEL で目標を送るだけ)。
//
// 安全:
//   - 目標が届かない・DMの帰還が無い・故障中は、DISABLE (脱力) を送る
//   - 故障したらラッチして、ノードを再起動するまで DISABLE のままにする
//     (故障のまま繰り返し有効化しないため)
//   - 終了時は DISABLE を送る (stop_on_shutdown.hpp)

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "tr_dokusute/damiao_slots.hpp"
#include "tr_dokusute/latest_value.hpp"
#include "tr_dokusute/stop_on_shutdown.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace tr_dokusute {

class DmSteerNode : public rclcpp::Node {
 public:
  DmSteerNode() : Node("tr_steer_dm2325") {
    const std::string id =
        std::to_string(declare_parameter<int>("device_id", 201));
    motor_index_ = declare_parameter<int>("motor_index", 0);
    gear_ratio_ = declare_parameter<double>("gear_ratio", 1.0);
    sign_ = declare_parameter<int>("sign", 1);
    offset_ = declare_parameter<double>("offset_rad", 0.0);
    vel_limit_ = declare_parameter<double>("vel_limit", 5.0);
    timeout_ = declare_parameter<double>("timeout", 0.5);

    tx_pub_ =
        create_publisher<std_msgs::msg::Int16MultiArray>("serial_tx_" + id, 10);
    // MODE_DAMIAO の位置は折り返し幅が違うので _unwrapped/_zeroed は使えない
    rx_sub_ = create_subscription<std_msgs::msg::Int16MultiArray>(
        "serial_rx_" + id, 10,
        [this](const std_msgs::msg::Int16MultiArray& msg) { OnFeedback(msg); });
    target_sub_ = create_subscription<std_msgs::msg::Float32>(
        "steer/angle_target", 10, [this](const std_msgs::msg::Float32& msg) {
          target_.Set(msg.data, now());
        });
    angle_pub_ = create_publisher<std_msgs::msg::Float32>("steer/angle", 10);
    zero_srv_ = create_service<std_srvs::srv::Trigger>(
        "steer/set_zero",
        [this](const std_srvs::srv::Trigger::Request::SharedPtr,
               std_srvs::srv::Trigger::Response::SharedPtr res) {
          OnSetZero(*res);
        });
    timer_ =
        create_wall_timer(std::chrono::milliseconds(20), [this]() { Run(); });
    stop_ =
        std::make_unique<StopOnShutdown>(*this, tx_pub_, damiao::kSlotCount);
  }

 private:
  void OnFeedback(const std_msgs::msg::Int16MultiArray& msg) {
    if (msg.data.size() < damiao::kSlotCount) return;
    const auto fb = damiao::GetFeedback(msg.data, motor_index_);

    // 受信回数 (counter) が進んでいれば、モータから帰還が届いている
    if (!feedback_.has_value() || fb.counter != feedback_.value()) {
      feedback_.Set(fb.counter, now());
    }
    position_ = fb.position;

    if (damiao::IsFault(fb.state) && !faulted_) {
      faulted_ = true;
      RCLCPP_ERROR(get_logger(),
                   "DMモータ故障 (状態%d)。脱力します。"
                   "復帰するにはこのノードを再起動してください",
                   fb.state);
    }
    if (faulted_) return;  // 故障中は角度を出さない (下流が止まる)

    std_msgs::msg::Float32 angle;
    angle.data = (position_ - offset_) * sign_ / gear_ratio_;
    angle_pub_->publish(angle);
  }

  void Run() {
    if (stop_->is_stopping()) return;

    std_msgs::msg::Int16MultiArray msg;
    msg.data.assign(damiao::kSlotCount, 0);  // 全ゼロ = DISABLE
    if (CanMove()) {
      damiao::SetCommand(msg.data, motor_index_,
                         damiao::Mode::kPositionVelocity,
                         ToDmPosition(target_.value()), vel_limit_);
    }
    tx_pub_->publish(msg);
  }

  // 目標が新しく、DMから帰還が届いていて、故障していないときだけ動かす。
  bool CanMove() {
    return target_.IsFresh(now(), timeout_) &&
           feedback_.IsFresh(now(), timeout_) && !faulted_;
  }

  // ステア角 [rad] → DMの位置 [rad, 出力軸]
  double ToDmPosition(double angle) {
    const double position = offset_ + sign_ * gear_ratio_ * angle;
    if (std::fabs(position) > damiao::kPositionMax) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
                            "目標(%.2frad)がDMの位置範囲±%.1fradを超えています",
                            position, damiao::kPositionMax);
    }
    return std::clamp(position, -damiao::kPositionMax, damiao::kPositionMax);
  }

  // 今のDM位置を「前」とみなす。手で車輪を前に揃えてから呼ぶ (動作中は不可)。
  // 値は保存されない。毎回呼ぶか、ログの値を yaml の offset_rad に書く。
  void OnSetZero(std_srvs::srv::Trigger::Response& res) {
    res.success = false;
    if (target_.IsFresh(now(), timeout_)) {
      res.message = "動作中は原点セットできません";
    } else if (!feedback_.has_value()) {
      res.message = "DMの帰還が来ていません";
    } else {
      offset_ = position_;
      res.success = true;
      res.message = "offset_rad = " + std::to_string(offset_);
      RCLCPP_INFO(get_logger(), "%s", res.message.c_str());
    }
  }

  int motor_index_ = 0;      // config.hpp の DAMIAO_MOTORS の行 (0始まり)
  double gear_ratio_ = 1.0;  // 出力軸→ステア軸の減速比
  int sign_ = 1;         // DMの+回転でステアが反時計回りなら1
  double offset_ = 0.0;  // 車輪が前を向くときのDM位置 [rad, 出力軸]
  double vel_limit_ = 5.0;  // POS_VEL の速度上限 [rad/s, 出力軸]
  double timeout_ = 0.5;    // [s]

  double position_ = 0.0;  // DMの位置帰還 [rad, 出力軸]
  bool faulted_ = false;
  LatestValue<float> target_;  // ステア角の目標 [rad]
  // DMの受信回数。進んでいれば、モータから帰還が届いている
  LatestValue<int16_t> feedback_;

  rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr tx_pub_;
  rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr rx_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr target_sub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr angle_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr zero_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<StopOnShutdown> stop_;
};

}  // namespace tr_dokusute

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<tr_dokusute::DmSteerNode>());
  rclcpp::shutdown();
  return 0;
}
