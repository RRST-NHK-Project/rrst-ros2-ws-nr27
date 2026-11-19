#include "nr27_br_hw_ctrl/swerve_kinematics.hpp"

#include <cmath>
#include <limits>

namespace nr27_br_hw_ctrl {

WheelCommand ComputeWheelVelocity(const BodyVelocity& body, double x,
                                  double y) {
  const double wheel_vx = body.vx - body.wz * y;
  const double wheel_vy = body.vy + body.wz * x;
  return {std::atan2(wheel_vy, wheel_vx), std::hypot(wheel_vx, wheel_vy)};
}

WheelCommand ChooseSteerAngle(double goal_angle, double current_angle,
                              double steer_limit) {
  // goal_angle は atan2 の結果で [-pi, pi]、steer_limit は高々数回転ぶんを
  // 想定しているので、k は ±3 まで見れば足りる。
  constexpr int kMaxHalfTurns = 3;

  WheelCommand best{current_angle, 1.0};
  double best_distance = std::numeric_limits<double>::infinity();
  for (int k = -kMaxHalfTurns; k <= kMaxHalfTurns; ++k) {
    const double candidate = goal_angle + k * M_PI;
    if (std::fabs(candidate) > steer_limit) continue;

    const double distance = std::fabs(candidate - current_angle);
    if (distance < best_distance) {
      best_distance = distance;
      best.angle = candidate;
      best.speed = (k % 2 == 0) ? 1.0 : -1.0;
    }
  }
  return best;
}

}  // namespace nr27_br_hw_ctrl
