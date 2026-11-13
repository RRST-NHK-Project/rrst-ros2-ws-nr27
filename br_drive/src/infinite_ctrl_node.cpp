// =====================================================================
// infinite_ctrl_node : 独立ステアリング(独ステ)1ユニットをPS4で動かすノード
//   damiao_ctrl の example_motor_control をもとに、独ステ引き継ぎ資料
//   (dokusute_damiao_handover.md)の決まりに合わせて書き換えたもの。
// =====================================================================
// ■ モータの役割(引き継ぎ資料 3章・9.4節)
//   モータ1 = 操舵 DM2325 → 角度制御(POS_VEL)
//   モータ2 = 駆動 DM3519 → 速度制御(VEL)。DM3519は角度制御に使えない(4.1節)
//
// ■ 操作
//   R1を押している間だけ動く(離すと2台とも脱力)。起動直後とE-STOPの後は、
//   R1を一度離して押し直す。
//   左スティック : 倒した方向へ車輪を向ける(上=起動時の向き、右=+90°)
//   R2           : 駆動の速さ(押し込むほど速い。最大 kDriveMaxRadps)
//   Options      : E-STOP(R1を離して押し直すと再開)
//
// ■ 安全のための決まり(引き継ぎ資料に合わせて全部入れてある)
//   ・指令は50回/秒で送り続ける(0.5秒届かないと駆動ノードが脱力させる)。
//   ・始める前に2台とも返事が届いていること、操舵の内部の位置が±12rad以内を確かめる。
//   ・操舵は「今の向きから±90°以内」へだけ回し、超える時は車輪を逆転させる。
//     目標は±11.5rad以内に収め、何周も回さない(4.1節・10.3節)。
//   ・【DM3519の暴走対策(9.1節)】駆動は低速(kDriveMaxRadps)に制限し、加速度も
//     制限して、前進⇔後退を素早く切り替えない。さらに暴走の見張りを入れてある
//     (実速度が指令と逆向きで10rad/s以上ずれる、または上限+5rad/sを超えるのが
//     0.3秒続いたら、2台とも脱力してノードを終える)。
//   ・操舵の向きが目標から kAlignTolRad 以上ずれている間は、駆動を回さない。
//   ・PS4: joyが0.5秒途絶えたら止める。/joy の送り手が2つ以上いたら入力を無視する。
//     R2は、一度「離した状態」を受け取るまで0として扱う(起動直後の誤った値の対策)。
//   ・故障・返事の途絶(状態15を含む)で、2台とも脱力して終える。
//   ・終わる時・Ctrl+Cの時は、必ず脱力を送る。
//
// ■ 動かす前に(引き継ぎ資料 0章)
//   車輪を浮かせ、緊急停止の方法(R1を離す / Ctrl+C / バッテリーのスイッチ)を
//   確認してから。DM3519はトルクが大きいので固定すること。
//
// ■ 使い方(ros2can と damiao_driver_node を先に起動。PS4ノードは起動しない)
//   ros2 launch damiao_ctrl damiao_ctrl.launch.py        ← use_joy:=true は付けない
//   ros2 run joy joy_node
//   ros2 run br_drive infinite_ctrl_node
// =====================================================================

#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

//自作
#include "br_drive/zakinema.h"

using namespace std::chrono_literals;

namespace Infinite_Zakisute
{

// ---------------------------------------------------------------------
// 調整する値(まずは低速のまま試すこと)
// ---------------------------------------------------------------------
constexpr double kPeriodSec = 0.02;          // 指令を送る周期(50回/秒)
constexpr double kDriveMaxRadps = 5.0;       // 駆動の最高速度[rad/s](9.1節: 低速から)
constexpr double kDriveAccelRadps2 = 5.0;    // 駆動の加速度の上限[rad/s^2]
constexpr double kDriveSign = 1.0;           // 車輪の前後が逆なら -1.0 にする
constexpr double kSteerSpeedRadps = 5.0;     // 操舵の速さの上限[rad/s]
constexpr double kSteerLimitRad = 11.5;      // 操舵の目標の範囲[rad](±12radより少し内側)
constexpr double kAlignTolRad = 0.3;         // 操舵がこれ以上ずれている間は駆動しない[rad]
constexpr double kReverseHysteresisRad = 0.2;  // 車輪の逆転を切り替えにくくする余裕[rad]
constexpr double kStickDeadzone = 0.5;       // 左スティックをこれ以上倒したら向きを変える
constexpr double kJoyTimeoutSec = 0.5;       // joyがこれ以上来なければ止める
// 暴走の見張り(9.1節 調べ方4)
constexpr double kRunawayDiffRadps = 10.0;   // 指令と逆向きで、これ以上ずれたら異常
constexpr double kRunawayOverRadps = 5.0;    // |実速度| > 最高速度 + これ なら異常
constexpr double kRunawayHoldSec = 0.3;      // 異常がこれだけ続いたら止める

// PS4(joy_node)のボタン・スティックの番号(damiao_ctrlの設定と同じ)
constexpr size_t kAxisLsX = 0;
constexpr size_t kAxisLsY = 1;
constexpr size_t kAxisR2 = 5;
constexpr size_t kButtonR1 = 5;
constexpr size_t kButtonOptions = 9;

// ---------------------------------------------------------------------
// モータ1台分の「リモコン」。damiao/motorN/cmd に指令を送り、
// damiao/motorN/state から今の状態を受け取る。
// ---------------------------------------------------------------------
class MotorHandle
{
public:
  MotorHandle(rclcpp::Node & node, int number)
  : number_(number)
  {
    const std::string prefix = "damiao/motor" + std::to_string(number);
    cmd_pub_ = node.create_publisher<std_msgs::msg::Float32MultiArray>(prefix + "/cmd", 10);
    state_sub_ = node.create_subscription<std_msgs::msg::Float32MultiArray>(
      prefix + "/state", 10,
      [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        if (msg->data.size() >= 6) {
          position_ = msg->data[0];
          velocity_ = msg->data[1];
          torque_ = msg->data[2];
          state_ = static_cast<int>(msg->data[3]);
          feedback_ok_ = msg->data[4] > 0.5f;
          internal_pos_ = msg->data[5];
          last_state_time_ = std::chrono::steady_clock::now();
        }
      });
  }

  // ---- 指令を決める(実際に送るのは send()) ----
  void set_velocity(double radps) {cmd_ = {0.0f, static_cast<float>(radps), 0.0f};}
  void set_angle(double rad, double speed_radps)
  {
    cmd_ = {1.0f, static_cast<float>(rad), static_cast<float>(speed_radps)};
  }
  void set_disable() {cmd_ = {-1.0f, 0.0f, 0.0f};}

  // 今の指令を送る。タイマーから50回/秒で呼ぶ。
  void send()
  {
    std_msgs::msg::Float32MultiArray msg;
    msg.data = {cmd_[0], cmd_[1], cmd_[2]};
    cmd_pub_->publish(msg);
  }

  // ---- 今の状態 ----
  double position() const {return position_;}   // 位置[rad](±12.5radで一周)
  double velocity() const {return velocity_;}   // 速度[rad/s]
  double torque() const {return torque_;}       // トルク[N・m]
  int state() const {return state_;}            // 0=脱力 1=有効 それ以外=故障(15=止めたまま)
  // モータから返事が届いているか(0.3秒以内に届いていて、帰還OK)
  bool ready() const
  {
    const auto age = std::chrono::steady_clock::now() - last_state_time_;
    return feedback_ok_ && age < 300ms;
  }
  // 角度制御を使ってよいか(モータ内部の位置が分かっていて±12rad以内)
  bool angle_ok() const {return !std::isnan(internal_pos_) && std::abs(internal_pos_) <= 12.0;}
  int number() const {return number_;}

private:
  int number_;
  std::array<float, 3> cmd_{-1.0f, 0.0f, 0.0f};  // 最初は脱力
  double position_{0.0};
  double velocity_{0.0};
  double torque_{0.0};
  int state_{0};
  bool feedback_ok_{false};
  double internal_pos_{std::nan("")};
  std::chrono::steady_clock::time_point last_state_time_{};
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr cmd_pub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr state_sub_;
};

// ---------------------------------------------------------------------
// ノード本体
// ---------------------------------------------------------------------
class InfiniteCtrlNode : public rclcpp::Node
{
public:
  InfiniteCtrlNode()
  : Node("infinite_ctrl_node"), steer_(*this, 1), drive_(*this, 2)
  {
    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10,
      std::bind(&InfiniteCtrlNode::ps4_listener_callback, this, std::placeholders::_1));
    // 50回/秒(20msごと)に on_timer() を呼ぶ
    timer_ = create_wall_timer(20ms, std::bind(&InfiniteCtrlNode::on_timer, this));
    // Ctrl+Cで止めた時も、通信が切れる前に必ず脱力を送る
    shutdown_cb_ = get_node_base_interface()->get_context()->add_pre_shutdown_callback(
      [this]() {
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        finished_ = true;
        for (int i = 0; i < 3; ++i) {
          steer_.set_disable(); drive_.set_disable(); steer_.send(); drive_.send();
          std::this_thread::sleep_for(20ms);
        }
      });
    created_ = std::chrono::steady_clock::now();
    RCLCPP_INFO(get_logger(), "infinite_ctrl_node を開始します。モータから返事が届くのを待っています…");
  }

  ~InfiniteCtrlNode() override
  {
    get_node_base_interface()->get_context()->remove_pre_shutdown_callback(shutdown_cb_);
  }

private:
  // ===================================================================
  // PS4の入力を受け取る(ここでは値をしまうだけ。指令は on_timer() で送る)
  // ===================================================================
  void ps4_listener_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (msg->axes.size() <= kAxisR2 || msg->buttons.size() <= kButtonOptions) {
      return;  // 想定と違うコントローラ
    }
    ls_x_ = -1.0 * msg->axes[kAxisLsX];   // 右に倒すと+
    ls_y_ = msg->axes[kAxisLsY];          // 上に倒すと+
    // R2は離すと+1、押し込むと-1。起動直後は0のまま届くことがあり、そのままだと
    // 半分押した扱いになるので、一度「離した(+1)」を受け取るまでは0として扱う。
    if (msg->axes[kAxisR2] > 0.99f) {
      r2_seen_released_ = true;
    }
    r2_ = r2_seen_released_ ? (-1.0 * msg->axes[kAxisR2] + 1.0) / 2.0 : 0.0;
    const bool r1 = msg->buttons[kButtonR1] != 0;
    const bool options = msg->buttons[kButtonOptions] != 0;
    // R1を離したら、次に押した時に動けるようにする(E-STOPもここで解除)
    if (!r1) {
      armed_ = true;
      estop_ = false;
    }
    if (options) {
      if (!estop_) {
        RCLCPP_WARN(get_logger(), "E-STOP(R1を離して押し直すと再開)");
      }
      estop_ = true;
      armed_ = false;
    }
    r1_ = r1;
    last_joy_time_ = std::chrono::steady_clock::now();
    joy_received_ = true;
  }

  // ===================================================================
  // 操舵の目標を決める。heading = 起動時の向きから見た、車輪を向けたい方向[rad]。
  // 「heading + k×半回転」の中から、今の目標に一番近く、±kSteerLimitRad に
  // 収まるものを選ぶ(kが奇数なら車輪を逆転させる)。見つからなければ false。
  // ===================================================================
  bool choose_steer(double heading, double & target, bool & reverse) const
  {
    double best_cost = std::numeric_limits<double>::infinity();
    bool found = false;
    for (int k = -8; k <= 8; ++k) {
      const double cand = steer_zero_ + heading + k * M_PI;
      if (std::abs(cand) > kSteerLimitRad) {
        continue;
      }
      const bool rev = (k % 2) != 0;
      // 逆転の切り替えを起こしにくくする(前進⇔後退を素早く切り替えないため)
      const double cost = std::abs(cand - steer_target_) +
        (rev != steer_reverse_ ? kReverseHysteresisRad : 0.0);
      if (cost < best_cost) {
        best_cost = cost;
        target = cand;
        reverse = rev;
        found = true;
      }
    }
    return found;
  }

  // ===================================================================
  // 【動き方を変える時はここ】R1を押している間、50回/秒で呼ばれる。
  // ===================================================================
  void control_step()
  {
    double drive_target = 0.0;
    const double mag = std::hypot(ls_x_, ls_y_);
    if (mag > kStickDeadzone) {
      const double heading = std::atan2(ls_x_, ls_y_);  // 上=0、右=+90°
      double target = steer_target_;
      bool reverse = steer_reverse_;
      if (choose_steer(heading, target, reverse)) {
        steer_target_ = target;
        steer_reverse_ = reverse;
      }
      // 操舵が目標の向きに着いてから駆動する
      if (std::abs(steer_.position() - steer_target_) < kAlignTolRad) {
        drive_target = kDriveSign * (steer_reverse_ ? -1.0 : 1.0) * r2_ * kDriveMaxRadps;
      }
    }
    // 加速度を制限して、指令を急に変えない(前進⇔後退も必ずゆっくり0を通る)
    const double step = kDriveAccelRadps2 * kPeriodSec;
    if (drive_target > drive_cmd_ + step) {
      drive_cmd_ += step;
    } else if (drive_target < drive_cmd_ - step) {
      drive_cmd_ -= step;
    } else {
      drive_cmd_ = drive_target;
    }
    steer_.set_angle(steer_target_, kSteerSpeedRadps);
    drive_.set_velocity(drive_cmd_);
  }
  // ===================================================================

  // 駆動(DM3519)の暴走の見張り。異常が kRunawayHoldSec 続いたら true。
  bool runaway_detected(std::chrono::steady_clock::time_point now)
  {
    const double v = drive_.velocity();
    const bool opposite = std::abs(v - drive_cmd_) >= kRunawayDiffRadps && v * drive_cmd_ <= 0.0;
    const bool too_fast = std::abs(v) > kDriveMaxRadps + kRunawayOverRadps;
    if (!(opposite || too_fast)) {
      runaway_watch_ = false;
      return false;
    }
    if (!runaway_watch_) {
      runaway_watch_ = true;
      runaway_since_ = now;
    }
    return now - runaway_since_ >= std::chrono::duration<double>(kRunawayHoldSec);
  }

  void on_timer()
  {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (finished_) {
      return;
    }
    const auto now = std::chrono::steady_clock::now();

    if (!started_) {
      // 始める前の確認: 2台とも返事が届くまで待つ(5秒待っても来なければやめる)
      if (steer_.ready() && drive_.ready()) {
        if (!steer_.angle_ok()) {
          RCLCPP_ERROR(get_logger(), "操舵(モータ1)の内部の位置が±12radを超えている(または不明)"
            "ので、角度制御が使えません。ESCの電源を入れ直してください");
          finish();
          return;
        }
        steer_zero_ = steer_.position();  // 起動時の向きを「スティック上」にする
        steer_target_ = steer_zero_;
        started_ = true;
        RCLCPP_INFO(get_logger(), "準備できました(操舵の始めの向き %+.3frad)。"
          "R1を押している間だけ動きます", steer_zero_);
      } else if (now - created_ > 5s) {
        RCLCPP_ERROR(get_logger(), "5秒待ってもモータから返事が来ません"
          "(ros2can・damiao_driver_nodeの起動、モータの電源、ROS_AUTOMATIC_DISCOVERY_RANGEを確認)");
        finish();
        return;
      }
      steer_.send(); drive_.send();  // 待っている間も脱力を送り続ける(止めたままの解除にもなる)
      return;
    }

    // 故障したら全部止める(状態15 = ESP32・駆動ノードが止めたまま、も含む)
    if (steer_.state() > 1 || drive_.state() > 1 || !steer_.ready() || !drive_.ready()) {
      RCLCPP_ERROR(get_logger(), "故障または返事の途絶を検出したので止めます(状態 操舵%d 駆動%d)",
        steer_.state(), drive_.state());
      finish();
      return;
    }

    // PS4の入力が使えるか
    bool input_ok = joy_received_ &&
      now - last_joy_time_ < std::chrono::duration<double>(kJoyTimeoutSec);
    if (count_publishers("joy") > 1) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "/joy の送り手が2つ以上います。入力を無視します(ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST を確認)");
      input_ok = false;
    }
    const bool active = input_ok && r1_ && armed_ && !estop_;

    if (!active) {
      // R1を離した・E-STOP・入力が途絶えた → 2台とも脱力
      steer_.set_disable();
      drive_.set_disable();
      drive_cmd_ = 0.0;
      runaway_watch_ = false;
      was_active_ = false;
    } else {
      if (!was_active_) {
        // 脱力中に手で回されているかもしれないので、今の向きから始める
        if (!steer_.angle_ok() || std::abs(steer_.position()) > kSteerLimitRad) {
          RCLCPP_ERROR(get_logger(), "操舵の位置が範囲(±%.1frad)の外です。ESCの電源を入れ直してください",
            kSteerLimitRad);
          finish();
          return;
        }
        steer_target_ = steer_.position();
        steer_reverse_ = false;
        was_active_ = true;
      }
      control_step();
      if (runaway_detected(now)) {
        RCLCPP_ERROR(get_logger(), "駆動(DM3519)の暴走を検出したので止めます"
          "(指令%+.2frad/s 実速度%+.2frad/s トルク%+.2fN・m)",
          drive_cmd_, drive_.velocity(), drive_.torque());
        finish();
        return;
      }
    }
    steer_.send(); drive_.send();

    // 1秒ごとに様子を表示
    if (now - last_print_ >= 1s) {
      last_print_ = now;
      RCLCPP_INFO(
        get_logger(), "%s 操舵: 目標%+6.3f 位置%+6.3frad  駆動: 指令%+6.2f 実速度%+6.2frad/s",
        active ? "[動作]" : "[脱力]", steer_target_, steer_.position(), drive_cmd_,
        drive_.velocity());
    }
  }

  // 2台とも脱力を送ってからノードを終える
  void finish()
  {
    finished_ = true;
    steer_.set_disable(); drive_.set_disable();
    for (int i = 0; i < 5; ++i) {
      steer_.send(); drive_.send();
      std::this_thread::sleep_for(20ms);
    }
    RCLCPP_INFO(get_logger(), "終わりました(2台とも脱力)");
    rclcpp::shutdown();
  }

  MotorHandle steer_;   // モータ1: 操舵 DM2325(角度制御)
  MotorHandle drive_;   // モータ2: 駆動 DM3519(速度制御)
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::PreShutdownCallbackHandle shutdown_cb_;
  // タイマー・joy・Ctrl+C時の処理が同時に動かないようにする鍵。finish()の中の
  // rclcpp::shutdown()が同じスレッドで終了処理(鍵を取る)を呼ぶので、同じスレッドなら
  // 何度でも取れるrecursive_mutexにしている(普通のmutexだと固まる)。
  std::recursive_mutex mutex_;
  bool started_{false};
  bool finished_{false};
  // PS4の入力
  double ls_x_{0.0};
  double ls_y_{0.0};
  double r2_{0.0};
  bool r1_{false};
  bool r2_seen_released_{false};
  bool armed_{false};   // R1を一度離したか(起動直後・E-STOP後は押し直しが必要)
  bool estop_{false};
  bool joy_received_{false};
  std::chrono::steady_clock::time_point last_joy_time_{};
  // 操舵・駆動の状態
  bool was_active_{false};
  double steer_zero_{0.0};     // 起動時の操舵の向き(スティック上 = この向き)
  double steer_target_{0.0};   // 今の操舵の目標[rad]
  bool steer_reverse_{false};  // 車輪を逆転させているか
  double drive_cmd_{0.0};      // 今送っている駆動の速度[rad/s](加速度制限後)
  bool runaway_watch_{false};
  std::chrono::steady_clock::time_point runaway_since_{};
  std::chrono::steady_clock::time_point created_;
  std::chrono::steady_clock::time_point last_print_{};
};

}  // namespace Infinite_Zakisute

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Infinite_Zakisute::InfiniteCtrlNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
