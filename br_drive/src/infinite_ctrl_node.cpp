// =====================================================================
// infinite_ctrl_node : 独立ステアリング(独ステ)1ユニットをPS4で動かすノード
//   damiao_ctrl の example_motor_control をもとに、独ステ引き継ぎ資料
//   (dokusute_damiao_handover.md)の決まりに合わせて書き換えたもの。
// =====================================================================
// ■ モータの役割(引き継ぎ資料 3章・9.4節)
//   モータ1 = 操舵 DM2325 → 角度制御(POS_VEL)か速度制御(VEL)。kSteerMode で切り替える(下)
//   モータ2 = 駆動 DM3519 → 速度制御(VEL)。DM3519は角度制御に使えない(4.1節)
//
// ■ 操舵の2つの制御のしかた(定数 kSteerMode。変えたらビルドし直す)
//   SteerMode::kAngle … ESCの角度制御。正確。
//     動けるのは起動時の向きから約±175°まで。届かない向きは、車輪を逆転させて向く。
//   SteerMode::kVelocity(今の設定) … ESCの速度制御 + このノードの中で位置を制御(P制御)。
//     何周でも同じ向きに回れる(全方位に無限旋回。車輪の逆転はしない)。
//     ・位置は、±12.5radで一周する帰還の位置をつなぎ合わせて数える。
//     ・同じ向きに何百rad(モータ)も回すと、ESCの中の位置の数え方が粗くなり、ゆっくり
//       回した時に位置の読みが変わらなくなる(実際には回っている。引き継ぎ資料・damiao.cpp参照)。
//       → 速度の読みと位置の読みが食い違ったら止める見張りを入れてある。
//         内部の位置が kInternalPosWarnRad を超えたら警告するので、ESCの電源を入れ直す(0に戻る)。
//     ・止まる位置の正確さは角度制御より少し落ちる(50回/秒で速度を指令するため)。
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
//                  スティックを倒していない時にR2を押すと、今の車輪の向き(最後に向けた向き)の
//                  まま駆動する
//   Options      : E-STOP(R1を離して押し直すと再開)
//   △           : 原点合わせを始める(R1を押したまま。終わるか失敗するまでスティックは効かない)
//   ※起動してモータと通信できたら、最初にR1を押した時に自動で1回だけ原点合わせをする
//     (△は要らない。途中でR1を離したら、次にR1を押した時に続きからやり直す)
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
//     4. 角度制御の時: +向きの端(操舵モータ +kSteerLimitRad)まで行っても無ければ、−向きに
//        戻りながら探し、見つけたら 1 からやり直す(いつも「+向きに回してONになる所」を原点にするため)
//        速度制御の時: 端は無いので、+向きに車輪 kHomingMaxTurns 周まで回して探す
//     5. 見つからない・kHomingTimeoutSec を超えた → 失敗(脱力して原点はそのまま)
//   見つけた所の車輪の向きを origin_offset_deg(既定0 = 前)として、スティックの「上」を決め直す。
//   原点合わせの前は、今まで通り「起動時の向き」がスティックの「上」。
//   ※センサーの値が来ていない(古いファーム・ros2canが止まっている)時は、原点合わせは始めない。
//   ※白線を使わないので、黒線の断線は「ずっと空」に見える → 見つからずに失敗として止まる。
//
// ■ 安全のための決まり(引き継ぎ資料に合わせて全部入れてある)
//   ・駆動は「R2を押していて、実際に回す時」と「操舵が回っていて、操舵と駆動のつながりを
//     打ち消す時(drive_coupling が 0 でない時だけ)」だけ有効にする。それ以外(止まっている時、
//     原点合わせの時)は脱力にして、駆動に力を出させない。
//   ・【固まりの見張り】駆動が有効な間、トルクが kStallTorqueNm 以上なのに速度が
//     kStallVelRadps 未満のまま kStallHoldSec 続いたら、2台とも脱力してノードを終える
//     (回らないまま大きな電流を流し続けて、モータを焼かないため)。
//   ・指令は50回/秒で送り続ける(0.5秒届かないと駆動ノードが脱力させる)。
//   ・始める前に返事が届いていること、(角度制御の時は)操舵の内部の位置が±12rad以内を確かめる。
//   ・角度制御の時: 操舵は「今の向きから±90°以内」へだけ回し、超える時は車輪を逆転させる。
//     操舵モータの目標は±11.5rad以内に収め、何周も回さない(4.1節・10.3節)。
//     歯車 13:49 なので、車輪の向きでは起動時から約±175°まで。
//   ・速度制御の時: 一番近い回り方で向きを変える(何周でもよい)。位置の数え間違い
//     (1回で kPosJumpRad 以上の変化)・位置の読みの止まり(上)を見つけたら、2台とも脱力して終える。
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
// ■ 操舵と駆動のつながりの打ち消し(drive_coupling)
//   駆動の軸は操舵の回転軸の中を通っている(DM-S3519 → 上の大きな歯車2枚(同じ部品、1:1)
//   → 縦の軸 → 傘歯車 → 23:33 → 車輪)。そのため、駆動のモータが止まっていても、操舵で
//   旋回部が回ると、傘歯車が縦の軸の周りを回って車輪が回ってしまう。
//   打ち消すには、縦の軸を旋回部と同じ速さで回せばよい。上の歯車が1:1なので、駆動モータに
//     「車輪の向きが変わる速さ[rad/s] × drive_coupling」
//   を足して送る(傘歯車や 23:33 の比には関係しない)。
//   drive_coupling の大きさは 1.0(上の歯車が1:1)、向き(+か−か)は実機で決める:
//     車輪を浮かせ use_drive:=true drive_coupling:=1.0 で操舵だけ回し、車輪がほぼ止まって
//     いれば +1.0、0 の時より大きく回るなら −1.0。
//   2026-11-19 実機で −1.0 が正しいと確かめたので、既定は −1.0(0 にすると打ち消さない)。
//   操舵の速さは操舵モータの帰還の速度から求める。操舵が止まって kCouplingHoldSec たつと
//   駆動は脱力に戻る。原点合わせの間は打ち消さない(駆動は脱力)。
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
//   drive_coupling    : 操舵と駆動のつながりの打ち消し(上の説明。既定 −1.0(実機で確認済み)。0 = 打ち消さない)
// =====================================================================

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdio>
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
// 駆動モータ DM-S3519-1EC は中に減速機(3591/187 ≒ 19.2)がある。速度は出口の値で返ってくるが、
// 位置は中の回転子の値で返ってくる(2026-11-19 実機: 車輪1回転で位置が約178rad変わった)。
// ログに出す駆動の位置は、これで割って出口の値に直す(制御には位置を使っていない)。
constexpr double kDriveEncRatio = 3591.0 / 187.0;
constexpr double kSteerSign = 1.0;           // 右に倒して車輪が左を向くなら -1.0 にする
constexpr double kDriveSign = 1.0;           // 車輪の前後が逆なら -1.0 にする

// 駆動(モータの出力軸の値。暴走の見張りもこの値で行う)
constexpr double kDriveMaxRadps = 10.0;      // 駆動モータの最高速度[rad/s](9.1節: 低速から。2026-11-19 5.0→7.5→10.0)
                                             // → 車輪では 10.0 ÷ 1.43 ≒ 7.0rad/s
constexpr double kDriveAccelRadps2 = 7.5;    // 駆動モータの加速度の上限[rad/s^2](2026-11-19 5.0→7.5)
// 操舵(モータの出力軸の値)
constexpr double kSteerLimitRad = 11.5;      // 操舵モータの目標の範囲[rad](±12radより少し内側)
                                             // → 車輪の向きでは ±11.5 ÷ 3.77 ≒ ±3.05rad(±175°)
// 操舵(車輪の向きの値)
constexpr double kSteerSpeedRadps = 1.95;    // 車輪の向きを変える速さの上限[rad/s](モータでは×3.77。2026-11-19 1.5→1.95(1.3倍))
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
// 操舵と駆動のつながりの打ち消し(駆動モータの値)
constexpr double kCouplingMinRadps = 0.05;   // 打ち消しの量がこれ以上の時に駆動を有効にする
constexpr double kCouplingHoldSec = 0.3;     // 操舵が止まってから、これだけたったら駆動を脱力に戻す

// ---------------------------------------------------------------------
// 操舵の制御のしかた【ここで切り替える】(説明はファイルの先頭)
//   SteerMode::kAngle    : 角度制御。正確。起動時から約±175°まで、届かない向きは車輪を逆転
//   SteerMode::kVelocity : 速度制御 + ノードの中で位置制御。何周でも回れる(逆転なし)
// ---------------------------------------------------------------------
enum class SteerMode {kAngle, kVelocity};
constexpr SteerMode kSteerMode = SteerMode::kVelocity;
constexpr bool kSteerVel = (kSteerMode == SteerMode::kVelocity);

// 速度制御(kVelocity)の時だけ使う値(モータの出力軸の値)
constexpr double kSteerKp = 6.0;              // 位置のずれ1radあたりの速度指令[rad/s]
constexpr double kSteerHoldTolRad = 0.01;     // ずれがこれ未満なら速度0にする[rad]
constexpr double kPosWrapRad = 12.5;          // 帰還の位置は±これで一周する(ESCのP_MAX)
constexpr double kPosJumpRad = 3.0;           // 1回の帰還でこれ以上動いたら数え間違いとして止める
constexpr double kPosStuckCheckSec = 0.3;     // 位置の読みが止まっていないか見る間隔[s]
constexpr double kPosStuckMinRad = 0.3;       // 速度の読みからこれ以上動いたはずなのに
constexpr double kPosStuckRatio = 0.5;        // 位置の読みがその半分も変わっていなければ止める
constexpr double kInternalPosWarnRad = 100.0; // 内部の位置がこれを超えたら警告(電源を入れ直すと0)
constexpr double kHomingMaxTurns = 1.3;       // 原点合わせで車輪を何周まで回して探すか

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
          const auto t = std::chrono::steady_clock::now();
          const bool ok = msg->data[4] > 0.5f;
          // 始める前(locked_ でない間)は、帰還が無い・途切れた後の値でつなぎ合わせない。
          // ノードを起動してからESCの24Vを入れると、帰還が無い間の値(0など)から、電源が入った
          // ESCの本当の位置へ一気に飛ぶため、ここでつなぎ合わせをやり直す。
          if (!locked_ &&
            (!ok || (have_pos_ && t - last_state_time_ > std::chrono::milliseconds(100))))
          {
            have_pos_ = false;
            ok_since_ = t;
          }
          if (ok && !feedback_ok_) {
            ok_since_ = t;  // 帰還が来始めた時刻
          }
          position_ = msg->data[0];
          // 位置をつなぎ合わせる(帰還の位置は±kPosWrapRadで一周する)
          if (!ok && !locked_) {
            // 帰還が無い時の値は使わない(次に帰還が来た時に、そこからつなぎ合わせる)
          } else if (!have_pos_) {
            unwrapped_ = position_;
            have_pos_ = true;
          } else {
            double d = position_ - last_raw_pos_;
            if (d > kPosWrapRad) {
              d -= 2.0 * kPosWrapRad;
            } else if (d < -kPosWrapRad) {
              d += 2.0 * kPosWrapRad;
            }
            if (std::abs(d) > kPosJumpRad) {
              pos_jump_ = true;  // 帰還が抜けた・数え間違い
            }
            unwrapped_ += d;
            // 速度の読みを足し合わせる(位置の読みと比べて、読みが止まっていないかを見る)
            const double dt = std::chrono::duration<double>(t - last_state_time_).count();
            vel_integral_ += msg->data[1] * std::clamp(dt, 0.0, 0.1);
          }
          last_raw_pos_ = position_;
          velocity_ = msg->data[1];
          torque_ = msg->data[2];
          state_ = static_cast<int>(msg->data[3]);
          feedback_ok_ = ok;
          internal_pos_ = msg->data[5];
          last_state_time_ = t;
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
  double position_unwrapped() const {return unwrapped_;}  // つなぎ合わせた位置[rad](何周でも)
  bool pos_jump() const {return pos_jump_;}     // 位置の数え間違いがあったか
  double vel_integral() const {return vel_integral_;}  // 速度の読みを足し合わせたもの[rad]
  double internal_pos() const {return internal_pos_;}  // モータ内部の位置[rad](不明ならNaN)
  double velocity() const {return velocity_;}   // 速度[rad/s]
  double torque() const {return torque_;}       // トルク[N・m]
  int state() const {return state_;}            // 0=脱力 1=有効 それ以外=故障(15=止めたまま)
  // モータから返事が届いているか(0.3秒以内に届いていて、帰還OK)
  bool ready() const
  {
    const auto age = std::chrono::steady_clock::now() - last_state_time_;
    return feedback_ok_ && age < 300ms;
  }
  // 帰還が kSettleSec 以上続けて届いているか(始める前の確認用。24Vを入れた直後の値を避ける)
  bool settled() const
  {
    return ready() && have_pos_ &&
      std::chrono::steady_clock::now() - ok_since_ >= std::chrono::milliseconds(500);
  }
  // 始める時に呼ぶ。ここまでの数え間違いの印を消し、以後はつなぎ合わせをやり直さない
  // (始めた後の帰還の抜けは、数え間違い・返事の途絶として止める)
  void lock_position()
  {
    locked_ = true;
    pos_jump_ = false;
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
  bool have_pos_{false};
  double last_raw_pos_{0.0};
  double unwrapped_{0.0};
  bool pos_jump_{false};
  bool locked_{false};          // 始めた後か(true ならつなぎ合わせをやり直さない)
  std::chrono::steady_clock::time_point ok_since_{};  // 帰還が来始めた時刻
  double vel_integral_{0.0};
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
    drive_coupling_ = declare_parameter<double>("drive_coupling", -1.0);
    if (use_drive_ && drive_coupling_ != 0.0) {
      RCLCPP_WARN(get_logger(), "操舵と駆動のつながりを打ち消します(drive_coupling=%+.2f)。"
        "操舵だけ回している時も駆動に力が出ます", drive_coupling_);
    }
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
    drive_out_ = 0.0;
    coupling_cmd_ = 0.0;
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
    const double move = kSteerSign * speed * kSteerGear;  // 回す時の速さ(モータ、+向き)
    double ff = 0.0;  // 速度制御の時に足す速度(止まって待つ時は0)
    switch (homing_) {
      case Homing::kBackoff:   // −向きに戻る
        if (backoff_to_target_) {
          // 見つけた後: 決めた所まで戻ったら止まり、センサーの値が追いつくまで待つ
          // (遅れて届く値は、行き過ぎた先の「OFF」のことがあるので、動いている間は信じない)
          if (kSteerSign * (steer_pos() - backoff_target_) > 0.0) {
            steer_target_ -= kSteerSign * step;
            ff = -move;
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
          ff = -move;
        }
        break;
      case Homing::kSeek:      // +向きに回して、ONになる所を探す
        if (origin_on_) {
          if (homing_fine_) {
            homing_done(steer_pos());
            return;
          }
          // 速く回していたので少し行き過ぎている → ゆっくり抜けてから合わせ直す
          RCLCPP_INFO(get_logger(), "原点合わせ: 見つけました。ゆっくり合わせ直します");
          homing_fine_ = true;
          homing_ = Homing::kBackoff;
          steer_target_ = steer_pos();
          backoff_target_ = steer_pos() - kSteerSign * kSteerGear * kHomingBackoffRad;
          backoff_to_target_ = true;
          break;
        }
        steer_target_ += kSteerSign * step;
        ff = move;
        if (!kSteerVel && std::abs(steer_target_) > kSteerLimitRad) {
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
          ff = -move;
        }
        break;
      case Homing::kIdle:
        return;
    }
    if (kSteerVel) {
      // 速度制御: 端は無いので、回した量で決める
      if (std::abs(steer_pos() - homing_start_pos_) > kHomingMaxTurns * 2.0 * M_PI * kSteerGear) {
        homing_fail("車輪を1周以上回しても、原点センサーが見つかりませんでした"
          "(センサーの位置・黒線・電源を確認)");
        return;
      }
    } else if (std::abs(steer_target_) > kSteerLimitRad) {
      // 角度制御: −向きの端より先へは行かない
      homing_fail("操舵の動ける範囲の端まで回しても、原点センサーが見つかりませんでした"
        "(センサーの位置・黒線・電源を確認)");
      return;
    }
    // 目標は今の位置から離れすぎないようにする(モータが追いつかない時に先へ行かない)
    const double max_lead = 5.0 * step;
    if (steer_target_ > steer_pos() + max_lead) {
      steer_target_ = steer_pos() + max_lead;
    } else if (steer_target_ < steer_pos() - max_lead) {
      steer_target_ = steer_pos() - max_lead;
    }
    steer_output(2.0 * speed * kSteerGear, ff);
  }

  void homing_done(double edge_motor_pos)
  {
    // 見つけた所の車輪の向きが origin_offset になるように、スティックの「上」を決め直す
    steer_zero_ = edge_motor_pos - kSteerSign * kSteerGear * origin_offset_rad_;
    steer_target_ = edge_motor_pos;  // その場で止まる
    steer_reverse_ = false;
    homing_ = Homing::kIdle;
    homed_ = true;
    auto_homing_pending_ = false;
    RCLCPP_INFO(get_logger(), "原点合わせができました(センサーの位置 モータ%+.3frad = 車輪%+.1f°)。"
      "スティックの「上」を決め直しました", edge_motor_pos, origin_offset_rad_ * 180.0 / M_PI);
  }

  void homing_fail(const char * reason)
  {
    RCLCPP_ERROR(get_logger(), "原点合わせをやめました: %s。原点は前のままです", reason);
    homing_ = Homing::kIdle;
    auto_homing_pending_ = false;  // 失敗したら自動ではやり直さない(△で手動ならできる)
    steer_target_ = steer_pos();  // その場で止まる
    // R1を押し直すまで動かさない(何が起きたか人が確かめてから)
    armed_ = false;
  }

  // 操舵モータの今の位置[rad]。速度制御の時はつなぎ合わせた位置(何周でも)
  double steer_pos() const
  {
    return kSteerVel ? steer_.position_unwrapped() : steer_.position();
  }

  // 操舵に指令を出す(目標 steer_target_ へ向かう)。max_radps = 速さの上限(モータ)。
  // ff_radps = 速度制御の時に足す速度(原点合わせで一定の速さで回す時)。
  void steer_output(double max_radps, double ff_radps = 0.0)
  {
    if (!kSteerVel) {
      steer_.set_angle(steer_target_, max_radps);
      return;
    }
    const double err = steer_target_ - steer_pos();
    double v = ff_radps + kSteerKp * err;
    if (ff_radps == 0.0 && std::abs(err) < kSteerHoldTolRad) {
      v = 0.0;
    }
    steer_vel_cmd_ = std::clamp(v, -max_radps, max_radps);
    steer_.set_velocity(steer_vel_cmd_);
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
    const bool stick_on = mag > kStickDeadzone;
    const bool r2_on = use_drive_ && r2_ > kR2OnThreshold;
    if (stick_on) {
      const double heading = std::atan2(ls_x_, ls_y_);  // 上=0、右=+90°
      if (kSteerVel) {
        // 速度制御: 今の目標から一番近い回り方で向ける(何周でもよい。逆転しない)
        const double cur = motor_to_wheel(steer_target_);
        steer_target_ = wheel_to_motor(cur + std::remainder(heading - cur, 2.0 * M_PI));
        steer_reverse_ = false;
      } else {
        double target = steer_target_;
        bool reverse = steer_reverse_;
        if (choose_steer(heading, target, reverse)) {
          steer_target_ = target;
          steer_reverse_ = reverse;
        }
      }
    }
    // スティックを倒していない時は向きの目標を変えない(今の向きのまま駆動する)。
    // 操舵が目標の向きに着いてから駆動する(ずれは車輪の向きで比べる)
    if (r2_on && std::abs(steer_pos() - steer_target_) / kSteerGear < kAlignTolRad) {
      drive_target = kDriveSign * (steer_reverse_ ? -1.0 : 1.0) * r2_ * kDriveMaxRadps;
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
    steer_output(kSteerSpeedRadps * kSteerGear);  // 速さはモータの値にして送る

    // 操舵と駆動のつながりの打ち消し: 車輪の向きが変わる速さ × drive_coupling を駆動に足す
    const auto now = std::chrono::steady_clock::now();
    const double steer_wheel_radps = kSteerSign * steer_.velocity() / kSteerGear;
    coupling_cmd_ = use_drive_ ? drive_coupling_ * steer_wheel_radps : 0.0;
    if (std::abs(coupling_cmd_) >= kCouplingMinRadps) {
      coupling_until_ = now + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(kCouplingHoldSec));
    }
    const bool coupling_active = use_drive_ && drive_coupling_ != 0.0 && now < coupling_until_;

    // 駆動は「回す指令がある間(R2を離した後、ゆっくり0に戻るまでを含む)」と
    // 「操舵が回っていて打ち消す間」だけ有効にする。どちらでもなければ脱力にして、
    // 止まったまま力を出し続けないようにする。
    drive_enabled_ = use_drive_ && (drive_cmd_ != 0.0 || coupling_active);
    if (drive_enabled_) {
      drive_out_ = drive_cmd_ + (coupling_active ? coupling_cmd_ : 0.0);
      drive_.set_velocity(drive_out_);
    } else {
      drive_out_ = 0.0;
      drive_.set_disable();
    }
  }
  // ===================================================================

  // 駆動の位置[モータの出口 rad](ログ用。帰還の位置は回転子の値なので減速比で割る)
  double drive_pos_out() const {return drive_.position_unwrapped() / kDriveEncRatio;}

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
      std::abs(v - drive_out_) >= kRunawayDiffRadps && v * drive_out_ <= 0.0;
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

  // 速度制御の時の見張り。止めた(finish した)ら true
  bool steer_velocity_watch(std::chrono::steady_clock::time_point now)
  {
    if (steer_.pos_jump()) {
      RCLCPP_ERROR(get_logger(), "操舵の位置を数え間違えました(帰還の抜け・速すぎる回転)。"
        "原点が分からなくなったので止めます");
      finish();
      return true;
    }
    if (now - stuck_check_time_ >= std::chrono::duration<double>(kPosStuckCheckSec)) {
      const double dv = steer_.vel_integral() - stuck_check_vi_;  // 速度の読みから見た動き
      const double dp = steer_pos() - stuck_check_pos_;           // 位置の読みの動き
      if (std::abs(dv) > kPosStuckMinRad && std::abs(dp) < kPosStuckRatio * std::abs(dv)) {
        RCLCPP_ERROR(get_logger(), "操舵の位置の読みが、速度の読みと合いません(速度から%+.2frad、"
          "位置は%+.2frad)。ESCの中の位置の数え方が粗くなっている恐れがあるので止めます。"
          "ESCの電源を入れ直してください(内部の位置 %+.1frad)", dv, dp, steer_.internal_pos());
        finish();
        return true;
      }
      stuck_check_time_ = now;
      stuck_check_pos_ = steer_pos();
      stuck_check_vi_ = steer_.vel_integral();
    }
    if (!std::isnan(steer_.internal_pos()) && std::abs(steer_.internal_pos()) > kInternalPosWarnRad) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000,
        "操舵の内部の位置が%+.0fradです(±%.0fradを超えた)。このまま増えると、ゆっくり回した時に"
        "位置の読みが変わらなくなります。止めてESCの電源を入れ直してください",
        steer_.internal_pos(), kInternalPosWarnRad);
    }
    return false;
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
      // 24Vを入れた直後の値を避けるため、帰還が0.5秒続けて届いてから始める
      if (steer_.settled() && (!use_drive_ || drive_.settled())) {
        if (!kSteerVel && !steer_.angle_ok()) {
          RCLCPP_ERROR(get_logger(), "操舵(モータ1)の内部の位置が±12radを超えている(または不明)"
            "ので、角度制御が使えません。ESCの電源を入れ直してください");
          finish();
          return;
        }
        steer_.lock_position();
        drive_.lock_position();
        steer_zero_ = steer_pos();  // 起動時の向きを「スティック上」にする
        steer_target_ = steer_zero_;
        started_ = true;
        stuck_check_time_ = now;
        stuck_check_pos_ = steer_pos();
        stuck_check_vi_ = steer_.vel_integral();
        auto_homing_pending_ = true;  // 最初にR1を押した時に1回だけ原点合わせをする
        RCLCPP_INFO(get_logger(), "操舵は%s", kSteerVel ?
          "速度制御(何周でも回れる。車輪の逆転なし)" : "角度制御(起動時から約±175°。届かない向きは車輪を逆転)");
        RCLCPP_INFO(get_logger(), "準備できました(操舵の始めの向き %+.3frad、%s)。"
          "R1を押している間だけ動きます。最初にR1を押すと原点合わせをします", steer_zero_,
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

    // 速度制御の時の見張り: 位置の数え間違い・位置の読みの止まり・内部の位置の大きさ
    if (kSteerVel && steer_velocity_watch(now)) {
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
      drive_out_ = 0.0;
      coupling_cmd_ = 0.0;
      coupling_until_ = {};
      drive_enabled_ = false;
      stall_watch_ = false;
      was_active_ = false;
      steer_vel_cmd_ = 0.0;
      if (homing_ != Homing::kIdle) {
        RCLCPP_WARN(get_logger(), "原点合わせを中断しました(R1を離した・E-STOP・入力の途絶)");
        homing_ = Homing::kIdle;
      }
      homing_requested_ = false;
    } else {
      if (!was_active_) {
        // 脱力中に手で回されているかもしれないので、今の向きから始める
        if (!kSteerVel && (!steer_.angle_ok() || std::abs(steer_pos()) > kSteerLimitRad)) {
          RCLCPP_ERROR(get_logger(), "操舵の位置が範囲(±%.1frad)の外です。ESCの電源を入れ直してください",
            kSteerLimitRad);
          finish();
          return;
        }
        steer_target_ = steer_pos();
        steer_reverse_ = false;
        was_active_ = true;
      }
      if (auto_homing_pending_ && homing_ == Homing::kIdle) {
        homing_requested_ = true;  // 起動後の1回目の原点合わせ
      }
      if (homing_requested_) {
        homing_requested_ = false;
        if (homing_ == Homing::kIdle) {
          if (!origin_ok(now)) {
            RCLCPP_ERROR(get_logger(), "原点センサーの値が来ていないので、原点合わせを始められません"
              "(serial_rx_%d の%zu番。ESP32のファームが原点センサーに対応しているか確認)",
              kOriginDeviceId, kOriginSlot);
            auto_homing_pending_ = false;  // 自動ではもうやらない(△で手動ならできる)
          } else {
            homing_ = origin_on_ ? Homing::kBackoff : Homing::kSeek;
            homing_fine_ = false;  // まず速く探す
            backoff_to_target_ = false;
            settle_started_ = false;
            homing_started_ = now;
            homing_start_pos_ = steer_pos();
            steer_target_ = steer_pos();
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
          "動力線・エンコーダの校正を確かめてください", drive_out_, drive_.velocity(), drive_.torque());
        RCLCPP_ERROR(get_logger(), "  止まった時の駆動の位置: モータ%+.3frad(帰還の値 %+.3frad 回転子)"
          "  回し始めてから %+.3frad(車輪 %+.1f°)",
          drive_pos_out(), drive_.position(), drive_pos_out() - drive_start_pos_,
          (drive_pos_out() - drive_start_pos_) / kDriveGear * 180.0 / M_PI);
        finish();
        return;
      }
    }
    // 暴走の見張りは、動作中・脱力中どちらでも行う(駆動の返事が来ている時だけ)
    if (drive_.ready() && runaway_detected(now)) {
      RCLCPP_ERROR(get_logger(), "駆動(DM3519)の暴走を検出したので止めます"
        "(指令%+.2frad/s 実速度%+.2frad/s トルク%+.2fN・m)",
        drive_out_, drive_.velocity(), drive_.torque());
      finish();
      return;
    }
    steer_.send(); drive_.send();

    // 駆動を回している間は、0.1秒ごとに駆動の様子を出す(固まりは0.6秒ほどで止まるため)
    if (drive_enabled_ && !drive_was_enabled_) {
      drive_start_pos_ = drive_pos_out();
      last_drive_print_ = now - 1s;
      RCLCPP_INFO(get_logger(), "[駆動] 回し始め: 位置 モータ%+.3frad(帰還の値 %+.3frad 回転子)",
        drive_pos_out(), drive_.position());
    }
    drive_was_enabled_ = drive_enabled_;
    if (drive_enabled_ && now - last_drive_print_ >= 100ms) {
      last_drive_print_ = now;
      const double moved = drive_pos_out() - drive_start_pos_;
      RCLCPP_INFO(get_logger(), "[駆動] 指令%+5.2f(うち打ち消し%+5.2f) 実%+6.2frad/s  トルク%+5.2fN・m  "
        "位置 モータ%+8.3frad(帰還の値 %+.3f 回転子)  回し始めから%+.3frad(車輪%+6.1f°)  状態%d",
        drive_out_, drive_out_ - drive_cmd_, drive_.velocity(), drive_.torque(),
        drive_pos_out(), drive_.position(),
        moved, moved / kDriveGear * 180.0 / M_PI, drive_.state());
    }

    // 1秒ごとに様子を表示
    if (now - last_print_ >= 1s) {
      last_print_ = now;
      // 速度制御の時は、操舵へ送っている速度と内部の位置も出す
      char extra[192] = "";
      int n = 0;
      if (kSteerVel) {
        n = std::snprintf(extra, sizeof(extra), "  操舵の速度指令%+5.2frad/s 内部の位置%+.1frad",
          steer_vel_cmd_, steer_.internal_pos());
      }
      if (n >= 0 && static_cast<size_t>(n) < sizeof(extra)) {
        std::snprintf(extra + n, sizeof(extra) - n, "  駆動の位置 モータ%+.3frad(帰還の値 %+.3f 回転子)",
          drive_pos_out(), drive_.position());
      }
      // 車輪の値で表示する(かっこ内はモータの値)
      RCLCPP_INFO(
        get_logger(), "%s 車輪の向き: 目標%+7.1f° 今%+7.1f°(モータ%+6.2frad)  "
        "車輪の速さ: 指令%+5.2f 実%+5.2frad/s(モータ%+5.2frad/s)  "
        "状態 操舵%d 駆動%d  駆動トルク%+5.2fN・m  原点センサー %s%s%s",
        active ? "[動作]" : "[脱力]",
        motor_to_wheel(steer_target_) * 180.0 / M_PI, motor_to_wheel(steer_pos()) * 180.0 / M_PI,
        steer_pos(), drive_cmd_ / kDriveGear, drive_.velocity() / kDriveGear, drive_.velocity(),
        steer_.state(), drive_.state(), drive_.torque(),
        !origin_ok(now) ? "なし" : (origin_on_ ? "ON" : "OFF"),
        homing_ != Homing::kIdle ? "(原点合わせ中)" : (homed_ ? "(合わせ済み)" : ""),
        extra);
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
  bool homing_requested_{false};
  bool auto_homing_pending_{false};  // 起動後の自動の原点合わせがまだ終わっていない  // △が押された(次の on_timer() で原点合わせを始める)
  // 原点センサー・原点合わせ
  bool origin_valid_{false};
  bool origin_on_{false};
  std::chrono::steady_clock::time_point last_origin_time_{};
  Homing homing_{Homing::kIdle};
  bool homing_fine_{false};
  double homing_start_pos_{0.0};   // 原点合わせを始めた所(操舵モータ[rad])
  double steer_vel_cmd_{0.0};      // 速度制御の時に操舵へ送っている速度[rad/s]
  std::chrono::steady_clock::time_point stuck_check_time_{};
  double stuck_check_pos_{0.0};
  double stuck_check_vi_{0.0};
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
  double drive_cmd_{0.0};      // R2による駆動の速度[rad/s](加速度制限後。打ち消しは含まない)
  double drive_out_{0.0};      // 実際に駆動へ送っている速度[rad/s](drive_cmd_ + 打ち消し)
  double drive_coupling_{-1.0}; // パラメータ drive_coupling(操舵と駆動のつながりの打ち消し)
  double coupling_cmd_{0.0};   // 打ち消しの量[駆動モータ rad/s]
  std::chrono::steady_clock::time_point coupling_until_{};  // この時刻まで打ち消しのため駆動を有効にする
  bool use_drive_{false};      // パラメータ use_drive(false = 操舵だけ)
  bool drive_enabled_{false};  // 今、駆動に速度の指令を送っているか(false = 脱力を送っている)
  bool runaway_watch_{false};
  std::chrono::steady_clock::time_point runaway_since_{};
  bool stall_watch_{false};
  std::chrono::steady_clock::time_point stall_since_{};
  std::chrono::steady_clock::time_point created_;
  std::chrono::steady_clock::time_point last_print_{};
  std::chrono::steady_clock::time_point last_drive_print_{};
  bool drive_was_enabled_{false};   // 前の周期に駆動を回していたか
  double drive_start_pos_{0.0};     // 駆動を回し始めた時の位置[モータの出口 rad]
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
