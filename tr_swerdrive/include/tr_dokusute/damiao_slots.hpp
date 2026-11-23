// ros2can MODE_DAMIAO (DM3520 ESC + DM2325) の24スロット割り当て。
//
// 正本は ros2can のファーム (firmware/xiao-esp32-s3_can2io/src/damiao.cpp)。
// rrst-ros2-ws/damiao_ctrl の slot_layout.hpp と README は、速度を 0.001 単位と
// 書いているが古い (現行のファームは速度系が 0.01 単位)。ファームを変えたら
// ここも合わせること。tests で単位を確認している。
//
// 1台6スロット x 最大4台。位置・速度は出力軸基準。
// 指令 (ROS -> ESP32):
//   0: mode_word (0=DISABLE, 1=VEL, 2=POS_VEL)。全ゼロ=DISABLEが安全な既定値
//   1: target    (VEL: 0.01 rad/s/LSB, POS_VEL: 0.001 rad/LSB)
//   2: vel_limit (POS_VELの速さ上限。0.01 rad/s/LSB)
// 帰還 (ESP32 -> ROS):
//   0: motor_state (0=Disable, 1=Enable, 15=ESP32が止めている(ラッチ),
//                   それ以外=モータの故障。13=通信喪失)
//   1: fb_counter  (モータから返事を受け取るたび+1。脱力中も進む)
//   2: 内部の位置  (0.1 rad/LSB。不明なら -32768。ここでは使わない)
//   3: position    (0.001 rad/LSB、±12.5rad=P_MAX で折り返す)
//   4: velocity    (0.01 rad/s/LSB)
//   5: torque      (0.001 N・m/LSB)
//
// 帰還の23番は、操舵の原点のフォトセンサ (下の GetPhotoSensor)。

#ifndef TR_DOKUSUTE__DAMIAO_SLOTS_HPP_
#define TR_DOKUSUTE__DAMIAO_SLOTS_HPP_

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <optional>
#include <vector>

namespace tr_dokusute {
namespace damiao {

constexpr size_t kSlotCount = 24;
constexpr size_t kSlotsPerMotor = 6;
constexpr size_t kMaxMotors = kSlotCount / kSlotsPerMotor;  // 4
constexpr double kPositionScale = 0.001;                    // [rad/LSB]
constexpr double kVelocityScale = 0.01;                     // [rad/s/LSB]
// P_MAX(RID21) [rad]。位置の帰還と目標はこの範囲に収まる
constexpr double kPositionMax = 12.5;

enum class Mode : int16_t {
  kDisabled = 0,
  kVelocity = 1,
  kPositionVelocity = 2,
};

constexpr int16_t kStateDisabled = 0;
constexpr int16_t kStateEnabled = 1;

inline bool IsFault(int16_t state) {
  return state != kStateDisabled && state != kStateEnabled;
}

inline int16_t ToI16(double v) {
  v = std::clamp(std::round(v), static_cast<double>(INT16_MIN),
                 static_cast<double>(INT16_MAX));
  return static_cast<int16_t>(v);
}

inline void SetCommand(std::vector<int16_t>& tx, size_t motor, Mode mode,
                       double target, double vel_limit) {
  const size_t off = motor * kSlotsPerMotor;
  const double target_scale =
      mode == Mode::kVelocity ? kVelocityScale : kPositionScale;
  tx[off + 0] = static_cast<int16_t>(mode);
  tx[off + 1] = ToI16(target / target_scale);
  tx[off + 2] = ToI16(vel_limit / kVelocityScale);
}

struct Feedback {
  int16_t state{kStateDisabled};
  int16_t counter{0};
  double position{0.0};
  double velocity{0.0};
};

inline Feedback GetFeedback(const std::vector<int16_t>& rx, size_t motor) {
  const size_t off = motor * kSlotsPerMotor;
  Feedback fb;
  fb.state = rx[off + 0];
  fb.counter = rx[off + 1];
  fb.position = rx[off + 3] * kPositionScale;
  fb.velocity = rx[off + 4] * kVelocityScale;
  return fb;
}

// ---- 操舵の原点のフォトセンサ (NPN、黒線 = 遮光時ON) ----
// ESP32 (TR は tr_damiao_205、BR は soki_damiao_201) が帰還の kPhotoSensorSlot 番に
// 入れる。値の決まりは
// ros2can のブランチ feature/damiao-per-motor-maxvel の damiao.cpp
// (DAMIAO_ORIGIN_SENSOR_SLOT) と、br_swervedrive の infinite_ctrl_node に合わせてある:
//   0 = センサを読まないファーム、またはまだ値が決まっていない
//   1 = 入光 (スリットは空)
//   2 = 遮光 (スリットに物がある = 原点)
// 黒線の断線・センサの電源切れも 1 (入光) に見え、区別できない。
// モータの帰還と同じ帯にあるので、モータが4台のときは使えない
// (ファームの static_assert が止める)。
constexpr size_t kPhotoSensorSlot = 23;
constexpr int16_t kPhotoSensorClear = 1;
constexpr int16_t kPhotoSensorBlocked = 2;

// 遮光中なら true、入光なら false。値が決まっていない・決まりに無い値なら
// nullopt (0 を「入光」とみなすと、センサの無いファームで原点を見落とす)。
inline std::optional<bool> IsPhotoSensorBlocked(
    const std::vector<int16_t>& rx) {
  if (rx.size() <= kPhotoSensorSlot) return std::nullopt;
  switch (rx[kPhotoSensorSlot]) {
    case kPhotoSensorBlocked:
      return true;
    case kPhotoSensorClear:
      return false;
    default:
      return std::nullopt;
  }
}

}  // namespace damiao
}  // namespace tr_dokusute

#endif  // TR_DOKUSUTE__DAMIAO_SLOTS_HPP_
