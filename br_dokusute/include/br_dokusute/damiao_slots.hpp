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

#ifndef NR27_BR_HW_CTRL__DAMIAO_SLOTS_HPP_
#define NR27_BR_HW_CTRL__DAMIAO_SLOTS_HPP_

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

namespace nr27_br_hw_ctrl {
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

}  // namespace damiao
}  // namespace nr27_br_hw_ctrl

#endif  // NR27_BR_HW_CTRL__DAMIAO_SLOTS_HPP_
