// I 項だけに別の上限を付けた PID (ROSに依存しない部分)。
//
// common/PID.hpp の PIDController は、積分を「出力の上限 / Ki」で止めるので、
// I 項が出せる量 (Ki * 積分) は出力の上限と同じになる。駆動の速度制御では、
// 引っかかっている間に I 項が duty の上限まで溜まると、外れた瞬間に飛び出す。
// そこで P・D 用と I 用の PIDController を分けて、I 用の出力の上限を
// integral_limit にする (common は rrst-ros2-ws で直す決まりなので、ここで包む)。
//
//   出力 = clamp(clamp(P + D, ±output_limit) + clamp(I, ±integral_limit),
//                ±output_limit)
//
// integral_limit = output_limit なら、I 項の上限は PIDController 1つと同じ。

#ifndef TR_DOKUSUTE__LIMITED_PID_HPP_
#define TR_DOKUSUTE__LIMITED_PID_HPP_

#include <algorithm>

#include "common/PID.hpp"

namespace tr_dokusute {

class LimitedPid {
 public:
  // output_limit: 出力の上限。integral_limit: I 項の上限。
  // どちらも出力と同じ単位 (駆動なら duty) で、0 以上。
  LimitedPid(float kp, float ki, float kd, float output_limit,
             float integral_limit)
      : output_limit_(output_limit),
        proportional_derivative_(kp, 0.0f, kd, output_limit),
        integral_(0.0f, ki, 0.0f, integral_limit) {}

  void set_target(float target) {
    proportional_derivative_.set_target(target);
    integral_.set_target(target);
  }

  void reset() {
    proportional_derivative_.reset();
    integral_.reset();
  }

  // dt: 前回からの時間 [s]
  float update(float current, float dt) {
    const float output = proportional_derivative_.update(current, dt) +
                         integral_.update(current, dt);
    return std::clamp(output, -output_limit_, output_limit_);
  }

 private:
  float output_limit_;
  PIDController proportional_derivative_;
  PIDController integral_;
};

}  // namespace tr_dokusute

#endif  // TR_DOKUSUTE__LIMITED_PID_HPP_
