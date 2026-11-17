// =====================================================================
// infinite_ctrl_node : 独立ステアリング(独ステ)1ユニットをPS4で動かすノード
//   damiao_ctrl の example_motor_control をもとに、独ステ引き継ぎ資料
//   (dokusute_damiao_handover.md)の決まりに合わせて書き換えたもの。
// =====================================================================
// ■ モータの役割(引き継ぎ資料 3章・9.4節)
//   モータ1 = 操舵 DM2325 → 角度制御(POS_VEL)
//   モータ2 = 駆動 DM3519 → 速度制御(VEL)。DM3519は角度制御に使えない(4.1節)
//
// ■ 2つの使い方(パラメータ use_drive)
//   操舵だけ(既定): ros2 run br_drive infinite_ctrl_node
//     駆動(DM3519)には脱力しか送らない。駆動のESCの電源が切れていても動く。
//   駆動も使う     : ros2 run br_drive infinite_ctrl_node --ros-args -p use_drive:=true
//     【2026-11-17】DM3519が回らないまま7.3N・mを出し続けて発煙した。原因(動力線の順番・
//     エンコーダの校正)が分かるまで use_drive:=true にしないこと。
//
// ■ 操作
//   R1を押している間だけ動く(離すと2台とも脱力)。起動直後とE-STOPの後は、
//   R1を一度離して押し直す。
//   左スティック : 倒した方向へ車輪を向ける(上=起動時の向き、右=+90°)
//   R2           : 駆動の速さ(押し込むほど速い。最大 kDriveMaxRadps)。use_drive:=true の時だけ
//   Options      : E-STOP(R1を離して押し直すと再開)
//   △           : 原点合わせを始める(R1を押したまま。終わるか失敗するまでスティックは効かない)
//
// ■ 原点合わせ(操舵の原点センサー C-MSX951N-2M-R、2026-11-18追加)
//   センサーの値は ESP32(soki_damiao_201)が serial_rx_201 の23番に入れる
//   (1 = 空、2 = 原点。黒線 = ENC2_A)。このノードが直接読む(origin_sensor_node は要らない)。
//   △を押すと、駆動は脱力のまま、操舵だけを回してセンサーの「OFF→ON」の所を探す:
//     1. 始めにONなら、−向きに回してOFFになるまで抜ける
//     2. +向きに速く(kHomingFastRadps)回して、ONになる所を探す
//     3. 見つけた時は、センサーに気づくまでの遅れの分だけ行き過ぎている(センサーの幅より
//        行き過ぎることもある)。なので見つけた所から kHomingBackoffRad だけ−向きに戻って
//        kHomingSettleSec 止まり(遅れて届くセンサーの値が追いつくのを待つ)、まだONなら
//        さらに−向きに戻って、OFFになっているのを確かめてから、+向きにゆっくり(kHomingSlowRadps)回し直して、
//        ONになった所を原点とする(速さと正確さの両立)
//     4. +向きの端(操舵モータ +kSteerLimitRad)まで行っても無ければ、−向きに戻りながら探し、
//        見つけたら 1 からやり直す(いつも「+向きに回してONになる所」を原点にするため)
//     5. −向きの端まで行っても無い・kHomingTimeoutSec を超えた → 失敗(脱力して原点はそのまま)
//   見つけた所の車輪の向きを origin_offset_deg(既定0 = 前)として、スティックの「上」を決め直す。
//   原点合わせの前は、今まで通り「起動時の向き」がスティックの「上」。
//   ※センサーの値が来ていない(古いファーム・ros2canが止まっている)時は、原点合わせは始めない。
//   ※白線を使わないので、黒線の断線は「ずっと空」に見える → 見つからずに失敗として止まる。
//
// ■ 安全のための決まり(引き継ぎ資料に合わせて全部入れてある)
//   ・駆動は「R2を押していて、実際に回す時」だけ有効にする。それ以外(操舵だけ動かしている
//     時、R2を離して止まった後)は脱力にして、駆動に力を出させない。
//   ・【固まりの見張り】駆動が有効な間、トルクが kStallTorqueNm 以上なのに速度が
//     kStallVelRadps 未満のまま kStallHoldSec 続いたら、2台とも脱力してノードを終える
//     (回らないまま大きな電流を流し続けて、モータを焼かないため)。
//   ・指令は50回/秒で送り続ける(0.5秒届かないと駆動ノードが脱力させる)。
//   ・始める前に返事が届いていること、操舵の内部の位置が±12rad以内を確かめる。
//   ・操舵は「今の向きから±90°以内」へだけ回し、超える時は車輪を逆転させる。
//     操舵モータの目標は±11.5rad以内に収め、何周も回さない(4.1節・10.3節)。
//     歯車 13:49 なので、車輪の向きでは起動時から約±175°まで。
//   ・歯車の比(操舵 13:49、駆動 23:33)は kSteerGear / kDriveGear で計算に入れてある。
//   ・【DM3519の暴走対策(9.1節)】駆動は低速(kDriveMaxRadps)に制限し、加速度も
//     制限して、前進⇔後退を素早く切り替えない。さらに暴走の見張りを入れてある
//     (駆動が有効な間に実速度が指令と逆向きで10rad/s以上ずれる、または脱力中も含めて
//     上限+5rad/sを超えるのが0.1秒続いたら、2台とも脱力してノードを終える)。
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
//
// ■ パラメータ
//   use_drive         : 駆動(DM3519)も使うか(既定 false)
//   origin_offset_deg : 原点センサーがONになる所の車輪の向き[°](前 = 0、右 = +90。既定 0)
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
#include "std_msgs/msg/int16_multi_array.hpp"

//自作
#include "br_drive/zakinema.h"

using namespace std::chrono_literals;

namespace Infinite_Zakisute
{

// ---------------------------------------------------------------------
// 調整する値(まずは低速のまま試すこと)
// ---------------------------------------------------------------------
constexpr double kPeriodSec = 0.02;          // 指令を送る周期(50回/秒)

// 歯車の比(モータ側の歯数 : 車輪側の歯数)。モータの値 = 車輪の値 × 比
constexpr double kSteerGear = 49.0 / 13.0;   // 操舵 13:49。モータが約3.77回転で車輪の向きが1回転
constexpr double kDriveGear = 33.0 / 23.0;   // 駆動 23:33。モータが約1.43回転で車輪が1回転
constexpr double kSteerSign = 1.0;           // 右に倒して車輪が左を向くなら -1.0 にする
constexpr double kDriveSign = 1.0;           // 車輪の前後が逆なら -1.0 にする

// 駆動(モータの出力軸の値。暴走の見張りもこの値で行う)
constexpr double kDriveMaxRadps = 5.0;       // 駆動モータの最高速度[rad/s](9.1節: 低速から)
                                             // → 車輪では 5.0 ÷ 1.43 ≒ 3.5rad/s
constexpr double kDriveAccelRadps2 = 5.0;    // 駆動モータの加速度の上限[rad/s^2]
// 操舵(モータの出力軸の値)
constexpr double kSteerLimitRad = 11.5;      // 操舵モータの目標の範囲[rad](±12radより少し内側)
                                             // → 車輪の向きでは ±11.5 ÷ 3.77 ≒ ±3.05rad(±175°)
// 操舵(車輪の向きの値)
constexpr double kSteerSpeedRadps = 1.5;     // 車輪の向きを変える速さの上限[rad/s](モータでは×3.77)
constexpr double kAlignTolRad = 0.3;         // 車輪の向きがこれ以上ずれている間は駆動しない[rad]
constexpr double kReverseHysteresisRad = 0.2;  // 車輪の逆転を切り替えにくくする余裕[rad]
constexpr double kStickDeadzone = 0.5;       // 左スティックをこれ以上倒したら向きを変える
constexpr double kJoyTimeoutSec = 0.5;       // joyがこれ以上来なければ止める
constexpr double kR2OnThreshold = 0.05;      // R2がこれより押されている時だけ駆動する(0〜1)
// 暴走の見張り(9.1節 調べ方4)
constexpr double kRunawayDiffRadps = 10.0;   // 指令と逆向きで、これ以上ずれたら異常
constexpr double kRunawayOverRadps = 5.0;    // |実速度| > 最高速度 + これ なら異常
constexpr double kRunawayHoldSec = 0.1;      // 異常がこれだけ続いたら止める
// 固まりの見張り(2026-11-17の発煙の対策。駆動モータの値)
constexpr double kStallTorqueNm = 3.0;       // トルクがこれ以上なのに
constexpr double kStallVelRadps = 0.5;       // 速度がこれ未満のまま
constexpr double kStallHoldSec = 0.5;        // これだけ続いたら止める

// 原点合わせ(原点センサー)
constexpr int kOriginDeviceId = 201;         // ESP32のDEVICE_ID → serial_rx_201 を読む
constexpr size_t kOriginSlot = 23;           // 原点センサーの値が入る番号(damiao.cppと同じ)
constexpr int16_t kOriginEmpty = 1;          // 1 = 空
constexpr int16_t kOriginDetected = 2;       // 2 = 原点(黒線がON)
constexpr double kOriginTimeoutSec = 0.5;    // センサーの値がこれ以上古ければ使わない
constexpr double kHomingFastRadps = 3.5;     // 原点を探す時の車輪の向きの速さ[rad/s](約200°/s)
                                             // → モータでは約13.2rad/s(上限45.6の約3割)
constexpr double kHomingSlowRadps = 0.3;     // 見つけた後、合わせ直す時の速さ[rad/s](約17°/s)
// 速く探して見つけた後に戻る量[rad](車輪の向き)。
// 速く回している時の行き過ぎがこれより大きくても、戻った所で止まってセンサーの値を待つので原点はずれない
// (まだONならさらに戻る)。ただ、行き過ぎがセンサーの幅+この量より大きいと、端まで
// 行ってから戻って見つけ直すので時間がかかる → その時はこの値を大きくする。
// (2026-11-18: 21° → 7° → 3° → 1° に変更)
constexpr double kHomingBackoffRad = 1.0 * M_PI / 180.0;  // 1°
constexpr double kHomingSettleSec = 0.15;    // 戻った所で止まって、センサーの値が追いつくのを待つ時間
constexpr double kHomingTimeoutSec = 20.0;   // これを超えたら失敗(端から端まで約2.5秒)

// PS4(joy_node)のボタン・スティックの番号(damiao_ctrlの設定と同じ)
constexpr size_t kAxisLsX = 0;
constexpr size_t kAxisLsY = 1;
constexpr size_t kAxisR2 = 5;
constexpr size_t kButtonTriangle = 2;  // △(原点合わせ)
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
    // false(既定) = 操舵だけ。駆動を使う時だけ -p use_drive:=true を付ける
    use_drive_ = declare_parameter<bool>("use_drive", false);
    if (use_drive_) {
      RCLCPP_WARN(get_logger(), "駆動(DM3519)も使います。車輪を浮かせ、すぐ止められるようにしてください");
    }
    origin_offset_rad_ = declare_parameter<double>("origin_offset_deg", 0.0) * M_PI / 180.0;
    // 原点センサー(ESP32が serial_rx_201 の23番に入れる)
    origin_sub_ = create_subscription<std_msgs::msg::Int16MultiArray>(
      "serial_rx_" + std::to_string(kOriginDeviceId), 10,
      std::bind(&InfiniteCtrlNode::origin_listener_callback, this, std::placeholders::_1));
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
    const bool triangle = msg->buttons[kButtonTriangle] != 0;
    if (triangle && !triangle_prev_) {
      homing_requested_ = true;  // 押した瞬間だけ。実際に始めるのは on_timer()
    }
    triangle_prev_ = triangle;
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
  // 原点センサーの値を受け取る(serial_rx_201 の23番。1 = 空、2 = 原点)
  // ===================================================================
  void origin_listener_callback(const std_msgs::msg::Int16MultiArray::SharedPtr msg)
  {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (msg->data.size() <= kOriginSlot) {
      return;
    }
    const int16_t raw = msg->data[kOriginSlot];
    if (raw != kOriginEmpty && raw != kOriginDetected) {
      // 0 = 原点センサーを読まないファーム
      origin_valid_ = false;
      return;
    }
    origin_on_ = (raw == kOriginDetected);
    origin_valid_ = true;
    last_origin_time_ = std::chrono::steady_clock::now();
  }

  // 原点センサーの値が使えるか(新しい値が届いているか)
  bool origin_ok(std::chrono::steady_clock::time_point now) const
  {
    return origin_valid_ &&
           now - last_origin_time_ < std::chrono::duration<double>(kOriginTimeoutSec);
  }

  // ===================================================================
  // 原点合わせを1歩進める(R1を押している間、50回/秒)。駆動は脱力のまま。
  // 操舵の目標を少しずつ動かし、センサーの OFF→ON(+向き)の所を探す。
  // ===================================================================
  enum class Homing {kIdle, kBackoff, kSeek, kReturn};

  void homing_step(std::chrono::steady_clock::time_point now)
  {
    drive_cmd_ = 0.0;
    drive_enabled_ = false;
    drive_.set_disable();

    if (!origin_ok(now)) {
      homing_fail("原点センサーの値が届かなくなりました(ros2can・ESP32のファームを確認)");
      return;
    }
    if (now - homing_started_ > std::chrono::duration<double>(kHomingTimeoutSec)) {
      homing_fail("時間がかかりすぎました");
      return;
    }

    // 目標を動かす向き(+1 / −1)。1回(20ms)で動かす量はモータの値で
    // 車輪の向きの速さ。原点を決める「ゆっくり合わせ直し」の時だけ遅くする
    const double speed =
      (homing_ == Homing::kSeek && homing_fine_) ? kHomingSlowRadps : kHomingFastRadps;
    const double step = speed * kSteerGear * kPeriodSec;
    switch (homing_) {
      case Homing::kBackoff:   // −向きに戻る
        if (backoff_to_target_) {
          // 見つけた後: 決めた所まで戻ったら止まり、センサーの値が追いつくまで待つ
          // (遅れて届く値は、行き過ぎた先の「OFF」のことがあるので、動いている間は信じない)
          if (kSteerSign * (steer_.position() - backoff_target_) > 0.0) {
            steer_target_ -= kSteerSign * step;
            settle_started_ = false;
            break;
          }
          if (!settle_started_) {
            settle_started_ = true;
            settle_since_ = now;
          }
          if (now - settle_since_ < std::chrono::duration<double>(kHomingSettleSec)) {
            break;  // 止まって待つ
          }
          backoff_to_target_ = false;  // 待った後の値で決める(ONならさらに戻る)
          settle_started_ = false;
        }
        if (!origin_on_) {
          RCLCPP_INFO(get_logger(), "原点合わせ: センサーから抜けました。+向きに探します");
          homing_ = Homing::kSeek;
        } else {
          steer_target_ -= kSteerSign * step;
        }
        break;
      case Homing::kSeek:      // +向きに回して、ONになる所を探す
        if (origin_on_) {
          if (homing_fine_) {
            homing_done(steer_.position());
            return;
          }
          // 速く回していたので少し行き過ぎている → ゆっくり抜けてから合わせ直す
          RCLCPP_INFO(get_logger(), "原点合わせ: 見つけました。ゆっくり合わせ直します");
          homing_fine_ = true;
          homing_ = Homing::kBackoff;
          steer_target_ = steer_.position();
          backoff_target_ = steer_.position() - kSteerSign * kSteerGear * kHomingBackoffRad;
          backoff_to_target_ = true;
          break;
        }
        steer_target_ += kSteerSign * step;
        if (std::abs(steer_target_) > kSteerLimitRad) {
          steer_target_ = std::copysign(kSteerLimitRad, steer_target_);  // 端で止めて折り返す
          RCLCPP_INFO(get_logger(), "原点合わせ: +向きの端まで来ました。−向きに戻りながら探します");
          homing_ = Homing::kReturn;
        }
        break;
      case Homing::kReturn:    // −向きに戻りながら探す。見つけたら抜けてから+向きでやり直す
        if (origin_on_) {
          RCLCPP_INFO(get_logger(), "原点合わせ: センサーが反応しました。抜けてから+向きで合わせ直します");
          homing_ = Homing::kBackoff;
        } else {
          steer_target_ -= kSteerSign * step;
        }
        break;
      case Homing::kIdle:
        return;
    }
    // −向きの端より先へは行かない
    if (std::abs(steer_target_) > kSteerLimitRad) {
      homing_fail("操舵の動ける範囲の端まで回しても、原点センサーが見つかりませんでした"
        "(センサーの位置・黒線・電源を確認)");
      return;
    }
    // 目標は今の位置から離れすぎないようにする(モータが追いつかない時に先へ行かない)
    const double max_lead = 5.0 * step;
    if (steer_target_ > steer_.position() + max_lead) {
      steer_target_ = steer_.position() + max_lead;
    } else if (steer_target_ < steer_.position() - max_lead) {
      steer_target_ = steer_.position() - max_lead;
    }
    steer_.set_angle(steer_target_, 2.0 * speed * kSteerGear);
  }

  void homing_done(double edge_motor_pos)
  {
    // 見つけた所の車輪の向きが origin_offset になるように、スティックの「上」を決め直す
    steer_zero_ = edge_motor_pos - kSteerSign * kSteerGear * origin_offset_rad_;
    steer_target_ = edge_motor_pos;  // その場で止まる
    steer_reverse_ = false;
    homing_ = Homing::kIdle;
    homed_ = true;
    RCLCPP_INFO(get_logger(), "原点合わせができました(センサーの位置 モータ%+.3frad = 車輪%+.1f°)。"
      "スティックの「上」を決め直しました", edge_motor_pos, origin_offset_rad_ * 180.0 / M_PI);
  }

  void homing_fail(const char * reason)
  {
    RCLCPP_ERROR(get_logger(), "原点合わせをやめました: %s。原点は前のままです", reason);
    homing_ = Homing::kIdle;
    steer_target_ = steer_.position();  // その場で止まる
    // R1を押し直すまで動かさない(何が起きたか人が確かめてから)
    armed_ = false;
  }

  // 車輪の向き[rad](起動時=0) → 操舵モータの位置[rad]
  double wheel_to_motor(double wheel_angle) const
  {
    return steer_zero_ + kSteerSign * kSteerGear * wheel_angle;
  }
  // 操舵モータの位置[rad] → 車輪の向き[rad](起動時=0)
  double motor_to_wheel(double motor_pos) const
  {
    return kSteerSign * (motor_pos - steer_zero_) / kSteerGear;
  }

  // ===================================================================
  // 操舵の目標を決める。heading = 起動時の向きから見た、車輪を向けたい方向[rad]。
  // 「heading + k×半回転」の中から、今の目標に一番近く、操舵モータの位置が
  // ±kSteerLimitRad に収まるものを選ぶ(kが奇数なら車輪を逆転させる)。
  // target は操舵モータの位置[rad]で返す。見つからなければ false。
  // ===================================================================
  bool choose_steer(double heading, double & target, bool & reverse) const
  {
    double best_cost = std::numeric_limits<double>::infinity();
    bool found = false;
    for (int k = -8; k <= 8; ++k) {
      const double cand = wheel_to_motor(heading + k * M_PI);
      if (std::abs(cand) > kSteerLimitRad) {
        continue;
      }
      const bool rev = (k % 2) != 0;
      // 車輪の向きで比べる。逆転の切り替えを起こしにくくする(前進⇔後退を素早く切り替えないため)
      const double cost = std::abs(cand - steer_target_) / kSteerGear +
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
      // 操舵が目標の向きに着いてから駆動する(ずれは車輪の向きで比べる)
      if (use_drive_ && r2_ > kR2OnThreshold &&
        std::abs(steer_.position() - steer_target_) / kSteerGear < kAlignTolRad)
      {
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
    steer_.set_angle(steer_target_, kSteerSpeedRadps * kSteerGear);  // 速さはモータの値にして送る

    // 駆動は「回す指令がある間(R2を離した後、ゆっくり0に戻るまでを含む)」だけ有効にする。
    // 指令が0になったら脱力にして、止まったまま力を出し続けないようにする。
    drive_enabled_ = use_drive_ && drive_cmd_ != 0.0;
    if (drive_enabled_) {
      drive_.set_velocity(drive_cmd_);
    } else {
      drive_.set_disable();
    }
  }
  // ===================================================================

  // 「異常」が hold_sec 続いたら true を返す見張りの共通部分
  static bool held_for(
    bool abnormal, bool & watching, std::chrono::steady_clock::time_point & since,
    std::chrono::steady_clock::time_point now, double hold_sec)
  {
    if (!abnormal) {
      watching = false;
      return false;
    }
    if (!watching) {
      watching = true;
      since = now;
    }
    return now - since >= std::chrono::duration<double>(hold_sec);
  }

  // 駆動(DM3519)の暴走の見張り。速すぎるのは脱力中も見る。逆向きは駆動が有効な間だけ見る。
  bool runaway_detected(std::chrono::steady_clock::time_point now)
  {
    const double v = drive_.velocity();
    const bool opposite = drive_enabled_ &&
      std::abs(v - drive_cmd_) >= kRunawayDiffRadps && v * drive_cmd_ <= 0.0;
    const bool too_fast = std::abs(v) > kDriveMaxRadps + kRunawayOverRadps;
    return held_for(opposite || too_fast, runaway_watch_, runaway_since_, now, kRunawayHoldSec);
  }

  // 駆動(DM3519)の固まりの見張り。駆動が有効なのに、大きな力を出したまま回っていない。
  bool stall_detected(std::chrono::steady_clock::time_point now)
  {
    const bool stalled = drive_enabled_ &&
      std::abs(drive_.torque()) >= kStallTorqueNm && std::abs(drive_.velocity()) < kStallVelRadps;
    return held_for(stalled, stall_watch_, stall_since_, now, kStallHoldSec);
  }

  void on_timer()
  {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (finished_) {
      return;
    }
    const auto now = std::chrono::steady_clock::now();

    if (!started_) {
      // 始める前の確認: 使うモータの返事が届くまで待つ(5秒待っても来なければやめる)
      // 操舵だけの時は、駆動の返事は待たない(駆動のESCの電源が切れていてもよい)
      if (steer_.ready() && (!use_drive_ || drive_.ready())) {
        if (!steer_.angle_ok()) {
          RCLCPP_ERROR(get_logger(), "操舵(モータ1)の内部の位置が±12radを超えている(または不明)"
            "ので、角度制御が使えません。ESCの電源を入れ直してください");
          finish();
          return;
        }
        steer_zero_ = steer_.position();  // 起動時の向きを「スティック上」にする
        steer_target_ = steer_zero_;
        started_ = true;
        RCLCPP_INFO(get_logger(), "準備できました(操舵の始めの向き %+.3frad、%s)。"
          "R1を押している間だけ動きます", steer_zero_,
          use_drive_ ? "駆動も使う" : "操舵だけ。駆動には脱力だけを送る");
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
    // 操舵だけの時は、駆動の故障・返事の途絶では止めない(使っていないため)
    const bool drive_bad = use_drive_ && (drive_.state() > 1 || !drive_.ready());
    if (steer_.state() > 1 || !steer_.ready() || drive_bad) {
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
      drive_enabled_ = false;
      stall_watch_ = false;
      was_active_ = false;
      if (homing_ != Homing::kIdle) {
        RCLCPP_WARN(get_logger(), "原点合わせを中断しました(R1を離した・E-STOP・入力の途絶)");
        homing_ = Homing::kIdle;
      }
      homing_requested_ = false;
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
      if (homing_requested_) {
        homing_requested_ = false;
        if (homing_ == Homing::kIdle) {
          if (!origin_ok(now)) {
            RCLCPP_ERROR(get_logger(), "原点センサーの値が来ていないので、原点合わせを始められません"
              "(serial_rx_%d の%zu番。ESP32のファームが原点センサーに対応しているか確認)",
              kOriginDeviceId, kOriginSlot);
          } else {
            homing_ = origin_on_ ? Homing::kBackoff : Homing::kSeek;
            homing_fine_ = false;  // まず速く探す
            backoff_to_target_ = false;
            settle_started_ = false;
            homing_started_ = now;
            steer_target_ = steer_.position();
            RCLCPP_INFO(get_logger(), "原点合わせを始めます(R1を押したまま。離すと中断)");
          }
        }
      }
      if (homing_ != Homing::kIdle) {
        homing_step(now);
      } else {
        control_step();
      }
      if (stall_detected(now)) {
        RCLCPP_ERROR(get_logger(), "駆動(DM3519)が大きな力を出したまま回っていないので止めます"
          "(指令%+.2frad/s 実速度%+.2frad/s トルク%+.2fN・m)。モータが熱くなっていないか、"
          "動力線・エンコーダの校正を確かめてください", drive_cmd_, drive_.velocity(), drive_.torque());
        finish();
        return;
      }
    }
    // 暴走の見張りは、動作中・脱力中どちらでも行う(駆動の返事が来ている時だけ)
    if (drive_.ready() && runaway_detected(now)) {
      RCLCPP_ERROR(get_logger(), "駆動(DM3519)の暴走を検出したので止めます"
        "(指令%+.2frad/s 実速度%+.2frad/s トルク%+.2fN・m)",
        drive_cmd_, drive_.velocity(), drive_.torque());
      finish();
      return;
    }
    steer_.send(); drive_.send();

    // 1秒ごとに様子を表示
    if (now - last_print_ >= 1s) {
      last_print_ = now;
      // 車輪の値で表示する(かっこ内はモータの値)
      RCLCPP_INFO(
        get_logger(), "%s 車輪の向き: 目標%+7.1f° 今%+7.1f°(モータ%+6.2frad)  "
        "車輪の速さ: 指令%+5.2f 実%+5.2frad/s(モータ%+5.2frad/s)  "
        "状態 操舵%d 駆動%d  駆動トルク%+5.2fN・m  原点センサー %s%s",
        active ? "[動作]" : "[脱力]",
        motor_to_wheel(steer_target_) * 180.0 / M_PI, motor_to_wheel(steer_.position()) * 180.0 / M_PI,
        steer_.position(), drive_cmd_ / kDriveGear, drive_.velocity() / kDriveGear, drive_.velocity(),
        steer_.state(), drive_.state(), drive_.torque(),
        !origin_ok(now) ? "なし" : (origin_on_ ? "ON" : "OFF"),
        homing_ != Homing::kIdle ? "(原点合わせ中)" : (homed_ ? "(合わせ済み)" : ""));
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
  rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr origin_sub_;
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
  bool triangle_prev_{false};
  bool homing_requested_{false};  // △が押された(次の on_timer() で原点合わせを始める)
  // 原点センサー・原点合わせ
  bool origin_valid_{false};
  bool origin_on_{false};
  std::chrono::steady_clock::time_point last_origin_time_{};
  Homing homing_{Homing::kIdle};
  bool homing_fine_{false};
  bool backoff_to_target_{false};  // true = backoff_target_ まで戻ってから探し直す
  double backoff_target_{0.0};
  bool settle_started_{false};
  std::chrono::steady_clock::time_point settle_since_{};     // 見つけた後に戻る所(操舵モータの位置[rad])  // true = 見つけた後のゆっくり合わせ直し中
  std::chrono::steady_clock::time_point homing_started_{};
  bool homed_{false};
  double origin_offset_rad_{0.0};  // パラメータ origin_offset_deg[rad]
  std::chrono::steady_clock::time_point last_joy_time_{};
  // 操舵・駆動の状態
  bool was_active_{false};
  double steer_zero_{0.0};     // 起動時の操舵の向き(スティック上 = この向き)
  double steer_target_{0.0};   // 今の操舵の目標[rad]
  bool steer_reverse_{false};  // 車輪を逆転させているか
  double drive_cmd_{0.0};      // 今送っている駆動の速度[rad/s](加速度制限後)
  bool use_drive_{false};      // パラメータ use_drive(false = 操舵だけ)
  bool drive_enabled_{false};  // 今、駆動に速度の指令を送っているか(false = 脱力を送っている)
  bool runaway_watch_{false};
  std::chrono::steady_clock::time_point runaway_since_{};
  bool stall_watch_{false};
  std::chrono::steady_clock::time_point stall_since_{};
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
