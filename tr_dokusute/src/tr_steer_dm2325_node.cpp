// ステア (DAMIAO DM2325 + DM3520 ESC) のハードウェアノード。
// ros2can MODE_DAMIAO のESP32 と serial_tx/rx でやりとりする。
//
//   sub  steer/angle_target (Float32)  ステア角の目標 [rad]
//   pub  steer/angle        (Float32)  今のステア角 [rad]
//   srv  steer/set_zero     (Trigger)  今の向きを「前」として登録する
//   srv  steer/home         (Trigger)  原点合わせをやり直す
//
// 位置制御はESCの中で回るので、ここではPIDしない (POS_VEL で目標を送るだけ)。
//
// 原点合わせ (steer_homing.hpp):
//   起動したら自動で1回、操舵を回してフォトセンサのフラグ (45°の円弧) を探し、
//   フラグの中心を flag_center_rad の向きとして offset を決める。
//   終わるまで (失敗したら set_zero か steer/home が成功するまで) は
//   steer/angle_target を受け付けない。home_on_start: false なら従来通り
//   offset_rad をそのまま使う。
//
// 安全:
//   - 目標が届かない・DMの帰還が無い・故障中は、DISABLE (脱力) を送る
//   - 故障したらラッチして、ノードを再起動するまで DISABLE のままにする
//     (故障のまま繰り返し有効化しないため)
//   - 終了時は DISABLE を送る (stop_on_shutdown.hpp)
//   - 原点合わせ中に帰還・センサの値が途絶えた・時間切れ・故障したら、
//     原点合わせを失敗にして脱力する

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "tr_dokusute/damiao_slots.hpp"
#include "tr_dokusute/latest_value.hpp"
#include "tr_dokusute/steer_homing.hpp"
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
        std::to_string(declare_parameter<int>("device_id", 205));
    motor_index_ = declare_parameter<int>("motor_index", 0);
    gear_ratio_ = declare_parameter<double>("gear_ratio", 1.0);
    sign_ = declare_parameter<int>("sign", 1);
    offset_ = declare_parameter<double>("offset_rad", 0.0);
    vel_limit_ = declare_parameter<double>("vel_limit", 5.0);
    timeout_ = declare_parameter<double>("timeout", 0.5);

    // 原点合わせ。フラグの幅 = 円弧の長さ / 半径 (64.01mm / 81.5mm = 45°)
    const bool home_on_start = declare_parameter<bool>("home_on_start", true);
    const double flag_arc = declare_parameter<double>("flag_arc_mm", 64.01);
    const double flag_diameter =
        declare_parameter<double>("flag_diameter_mm", 163.0);
    flag_center_ = declare_parameter<double>("flag_center_rad", 0.0);
    homing_vel_ = declare_parameter<double>("homing_vel", 1.0);
    homing_timeout_ = declare_parameter<double>("homing_timeout", 30.0);
    homing_config_.flag_width = gear_ratio_ * flag_arc / (flag_diameter / 2.0);
    homing_config_.sweep =
        gear_ratio_ * 2.0 * M_PI *
        declare_parameter<double>("homing_sweep_turns", 1.3);
    // 折り返す時に端から余分に戻る量。センサの遅れで行き過ぎる分より大きくする
    // (1rad/s なら 0.39s の遅れまで大丈夫)
    homing_config_.margin = 0.5 * homing_config_.flag_width;
    homing_config_.direction = declare_parameter<int>("homing_direction", 1);
    homing_config_.width_tolerance =
        declare_parameter<double>("homing_width_tolerance", 0.3);
    homed_ = !home_on_start;
    homing_requested_ = home_on_start;

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
    home_srv_ = create_service<std_srvs::srv::Trigger>(
        "steer/home",
        [this](const std_srvs::srv::Trigger::Request::SharedPtr,
               std_srvs::srv::Trigger::Response::SharedPtr res) {
          OnHome(*res);
        });
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
    photo_blocked_ = damiao::IsPhotoSensorBlocked(msg.data);
    if (photo_blocked_.has_value()) last_photo_ = now();
    if (homing_.has_value() && photo_blocked_.has_value()) {
      homing_->Update(position_, photo_blocked_.value());
    }

    if (damiao::IsFault(fb.state) && !faulted_) {
      faulted_ = true;
      RCLCPP_ERROR(get_logger(),
                   "DMモータ故障 (状態%d)。脱力します。"
                   "復帰するにはこのノードを再起動してください",
                   fb.state);
    }
    if (faulted_) return;  // 故障中は角度を出さない (下流が止まる)
    if (!homed_) return;   // 原点が決まるまでは角度を出さない

    std_msgs::msg::Float32 angle;
    angle.data = (position_ - offset_) * sign_ / gear_ratio_;
    angle_pub_->publish(angle);
  }

  void Run() {
    if (stop_->is_stopping()) return;

    std_msgs::msg::Int16MultiArray msg;
    msg.data.assign(damiao::kSlotCount, 0);  // 全ゼロ = DISABLE
    if (homing_requested_ || homing_.has_value()) {
      StepHoming(msg.data);
    } else if (CanMove()) {
      damiao::SetCommand(msg.data, motor_index_,
                         damiao::Mode::kPositionVelocity,
                         ToDmPosition(target_.value()), vel_limit_);
    }
    tx_pub_->publish(msg);
  }

  // 目標が新しく、DMから帰還が届いていて、故障していないときだけ動かす。
  bool CanMove() {
    return homed_ && target_.IsFresh(now(), timeout_) &&
           feedback_.IsFresh(now(), timeout_) && !faulted_;
  }

  bool PhotoFresh() {
    return photo_blocked_.has_value() &&
           (now() - last_photo_).seconds() < timeout_;
  }

  // 原点合わせを1歩進める (20ms ごと)。動かす時は tx に目標を書く。
  void StepHoming(std::vector<int16_t>& tx) {
    if (faulted_) {
      FinishHoming("DMモータが故障中");
      return;
    }
    if (!homing_.has_value()) {
      // 帰還とセンサの値がそろうまで待つ (起動直後)
      if (!feedback_.IsFresh(now(), timeout_) || !PhotoFresh()) {
        if (!homing_wait_start_.has_value()) homing_wait_start_ = now();
        if ((now() - homing_wait_start_.value()).seconds() > 5.0) {
          FinishHoming(PhotoFresh() ? "DMの帰還が来ない"
                                    : "フォトセンサの値が来ない "
                                      "(ファームがセンサ対応か確認)");
        }
        return;
      }
      const double end = position_ + homing_config_.direction *
                                         homing_config_.sweep;
      if (std::fabs(end) > kHomingPositionLimit) {
        FinishHoming("探す範囲が位置の上限を超える (位置 " +
                     std::to_string(position_) +
                     " rad)。ESCの電源を入れ直して位置を0に戻す");
        return;
      }
      homing_.emplace(homing_config_, position_, photo_blocked_.value());
      homing_requested_ = false;
      homing_start_ = now();
      RCLCPP_INFO(get_logger(),
                  "原点合わせを始めます (位置 %.3f rad、フラグ幅 %.3f rad)",
                  position_, homing_config_.flag_width);
    }

    if (!feedback_.IsFresh(now(), timeout_) || !PhotoFresh()) {
      FinishHoming("原点合わせ中に帰還かセンサの値が途絶えた");
      return;
    }
    if ((now() - homing_start_).seconds() > homing_timeout_) {
      FinishHoming("時間切れ");
      return;
    }
    if (homing_->failed()) {
      FinishHoming(homing_->error());
      return;
    }
    if (homing_->done()) {
      offset_ = homing_->center() - sign_ * gear_ratio_ * flag_center_;
      homed_ = true;
      RCLCPP_INFO(get_logger(),
                  "原点合わせ完了: フラグの中心 %.3f rad (幅 %.3f rad) → "
                  "offset_rad = %.3f",
                  homing_->center(), homing_->measured_width(), offset_);
      homing_.reset();
      return;
    }
    damiao::SetCommand(tx, motor_index_, damiao::Mode::kPositionVelocity,
                       homing_->target(), homing_vel_);
  }

  // 原点合わせを失敗で終える。脱力して、steer/home か set_zero を待つ。
  void FinishHoming(const std::string& reason) {
    RCLCPP_ERROR(get_logger(),
                 "原点合わせ失敗: %s。脱力します。"
                 "ros2 service call /steer/home std_srvs/srv/Trigger "
                 "でやり直せます",
                 reason.c_str());
    homing_.reset();
    homing_requested_ = false;
    homing_wait_start_.reset();
  }

  void OnHome(std_srvs::srv::Trigger::Response& res) {
    res.success = false;
    if (homing_requested_ || homing_.has_value()) {
      res.message = "原点合わせ中です";
    } else if (faulted_) {
      res.message = "DMモータが故障中です。ノードを再起動してください";
    } else {
      homed_ = false;
      homing_requested_ = true;
      homing_wait_start_.reset();
      res.success = true;
      res.message = "原点合わせを始めます";
    }
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
    if (homing_requested_ || homing_.has_value()) {
      res.message = "原点合わせ中です";
    } else if (target_.IsFresh(now(), timeout_)) {
      res.message = "動作中は原点セットできません";
    } else if (!feedback_.has_value()) {
      res.message = "DMの帰還が来ていません";
    } else {
      offset_ = position_;
      homed_ = true;
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

  // 原点合わせで探す範囲の端がこれを超えたら始めない。ファームは内部の位置が
  // ±12rad を超えると角度制御を拒否する
  static constexpr double kHomingPositionLimit = 12.0;

  double position_ = 0.0;  // DMの位置帰還 [rad, 出力軸]
  std::optional<bool> photo_blocked_;  // フォトセンサ (遮光 = true)
  rclcpp::Time last_photo_{0, 0, RCL_ROS_TIME};

  SteerHomingConfig homing_config_;
  double flag_center_ = 0.0;   // フラグの中心のステア角 [rad] (前 = 0)
  double homing_vel_ = 1.0;    // 原点合わせの速さ [rad/s, 出力軸]
  double homing_timeout_ = 30.0;  // [s]
  bool homed_ = false;            // offset が決まっているか
  bool homing_requested_ = false;  // 原点合わせを始める (帰還待ち)
  std::optional<SteerHoming> homing_;  // 原点合わせ中だけ値がある
  std::optional<rclcpp::Time> homing_wait_start_;
  rclcpp::Time homing_start_{0, 0, RCL_ROS_TIME};
  bool faulted_ = false;
  LatestValue<float> target_;  // ステア角の目標 [rad]
  // DMの受信回数。進んでいれば、モータから帰還が届いている
  LatestValue<int16_t> feedback_;

  rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr tx_pub_;
  rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr rx_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr target_sub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr angle_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr zero_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr home_srv_;
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
