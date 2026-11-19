// 独ステの運動学 (ROSに依存しない計算部分)。
//
// 座標系はROSと同じ: x=前, y=左, 角度は反時計回りが正, 前向きが0。

#ifndef BR_DOKUSUTE__SWERVE_KINEMATICS_HPP_
#define BR_DOKUSUTE__SWERVE_KINEMATICS_HPP_

namespace br_dokusute {

// ロボット全体の速度指令 (cmd_vel の vx, vy, wz)。
struct BodyVelocity {
  double vx = 0.0;  // [m/s]
  double vy = 0.0;  // [m/s]
  double wz = 0.0;  // [rad/s]
};

// 1輪が向くべき角度と回す速さ。
struct WheelCommand {
  double angle = 0.0;  // ステア角 [rad]
  double speed = 0.0;  // ホイール速度 [m/s]。逆転させるときは負
};

// ロボット中心から (x, y) にある車輪の速度ベクトルを、角度と速さで返す。
// 剛体の関係 v_wheel = v + w × r を使う。速さが0に近いと角度は意味を持たない。
WheelCommand ComputeWheelVelocity(const BodyVelocity& body, double x, double y);

// 車輪を goal_angle の方向へ進めるための、ステア角と回転の向きを選ぶ。
//
// goal_angle + k*pi はどれも同じ動きになる (kが奇数なら車輪を逆転させる)。
// その中から ±steer_limit の範囲内で、今の角度 current_angle に
// 一番近いものを選ぶ。
// steer_limit は pi/2 以上にすること (候補は pi おきなので、
// 範囲内に必ず1つ入る)。
//
// 戻り値の speed は +1 (正転) か -1 (逆転)。呼び出し側で速さを掛ける。
WheelCommand ChooseSteerAngle(double goal_angle, double current_angle,
                              double steer_limit);

}  // namespace br_dokusute

#endif  // BR_DOKUSUTE__SWERVE_KINEMATICS_HPP_
