#pragma once
// zakinema : 独ステ4輪の運動学(中身は src/zakinema.cpp)
#include <array>

#include "br_drive/matrix.h"

struct WheelCmd
{
  double angle;
  double speed;
};

struct OdomDelta
{
  double dx;
  double dy;
  double d_yaw;
  double vx_r;
  double vy_r;
  double d_rad;
  double q_z;
  double q_w;
};

// ---------------------------------------------------------------------
// SwerveDrive_IK
// 【役割】機体を動かしたい速度から、4つの車輪それぞれの向きと速さを計算する(逆運動学)。
//
// 【変数】
//   vel_x        : 機体の前進方向の速度[m/s](前が+)
//   vel_y        : 機体の横方向の速度[m/s](左が+)
//   delta_radian : 機体の角速度[rad/s](左回りが+)
//   L            : 前輪と後輪の距離[m]
//   W            : 左右の車輪の距離[m]
//
// 【戻り値】WheelCmd が4つ入った配列。番号は 0=左前 1=右前 2=左後 3=右後
//   .angle : 車輪の向き[rad](前=0、左=+、-π〜+π)→ 操舵モータの目標にする
//   .speed : 車輪の速さ[m/s](0以上)→ 車輪の半径[m]で割ると駆動モータの指令[rad/s]になる
//
// 【使い方】
//   auto wheels = SwerveDrive_IK(0.3, 0.0, 0.5, 0.5, 0.4);
//   double steer = wheels[0].angle;          // 左前の操舵の目標[rad]
//   double drive = wheels[0].speed / 0.05;   // 左前の駆動の指令[rad/s](車輪の半径0.05m)
// ---------------------------------------------------------------------
std::array<WheelCmd, 4> SwerveDrive_IK(
  double vel_x, double vel_y, double delta_radian, double L, double W);

// ---------------------------------------------------------------------
// Odom_IK
// 【役割】各車輪の実際の向きと速さから、dt秒間に機体がどれだけ動いたかを計算する(オドメトリ)。
//
// 【変数】
//   wheels : 各車輪の実際の向きと速さ(SwerveDrive_IK の戻り値と同じ形)
//            .angle = 操舵の向き[rad]、.speed = 車輪の速さ[m/s](後ろ向きに回っている時は-)
//            番号は 0=左前 1=右前 2=左後 3=右後
//   dt     : 前回の計算からの時間[s]
//   yaw    : 今の機体の向き[rad](起動時の向きが0)
//   L      : 前輪と後輪の距離[m]
//   W      : 左右の車輪の距離[m]
//
// 【戻り値】OdomDelta
//   dt秒間の変位(起動時の向きを基準にした座標)→ 足し合わせると今の位置・向きになる
//     .dx    : x方向に動いた距離[m]
//     .dy    : y方向に動いた距離[m]
//     .d_yaw : 向きの変化[rad]
//   今の機体の速度(機体から見た向き。x=前、y=左)→ nav_msgs/Odometry の twist に入れる
//     .vx_r  : 前進方向の速度[m/s]
//     .vy_r  : 横方向の速度[m/s]
//     .d_rad : 角速度[rad/s](左回りが+)
//   dt秒後の向き(yaw + d_yaw)をクォータニオンにしたもの(z軸まわりの回転。x=y=0)
//   → Odometry の pose.orientation に入れる
//     .q_z   : sin((yaw + d_yaw)/2)
//     .q_w   : cos((yaw + d_yaw)/2)
//
// 【使い方】
//   std::array<WheelCmd, 4> now;
//   now[0] = {steer_pos0, drive_vel0 * 0.05};   // 左前: 操舵の位置[rad]、駆動の速度[rad/s]×半径
//   …(1〜3も同じ)
//   OdomDelta d = Odom_IK(now, dt, yaw, 0.5, 0.4);
//   x += d.dx;  y += d.dy;  yaw += d.d_yaw;
//   odom.twist.twist.linear.x  = d.vx_r;
//   odom.twist.twist.linear.y  = d.vy_r;
//   odom.twist.twist.angular.z = d.d_rad;
//   odom.pose.pose.orientation.z = d.q_z;   // yaw += d.d_yaw した後の向きと同じ
//   odom.pose.pose.orientation.w = d.q_w;
// ---------------------------------------------------------------------
OdomDelta Odom_IK(const std::array<WheelCmd, 4> & wheels, double dt, double yaw, double L, double W);
