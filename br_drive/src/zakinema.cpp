//独ステ４輪制御プログラムを作成するときに使う予定。機体の速度と角速度から逆運動学を用いて、目標の各車輪の速度を計算する関数。
//一応かつて作成したオドメトリの逆運動学も追加しておく
//(変数・戻り値の意味は include/br_drive/zakinema.h を参照)
#include "br_drive/zakinema.h"

#include <cmath>

namespace
{
// 逆運動学の行列 A(8×3)。[各車輪のvx, vy を並べた8×1] = A × [vel_x, vel_y, delta_radian]
// このファイルの中だけで使う
matrix make_A(double L, double W)
{
  const double a = L / 2, b = W / 2;
  // 車輪iの位置を(xi, yi)とすると vx_i = vx - ω·yi、vy_i = vy + ω·xi
  return matrix({{1, 0, -b},   // 0: 左前 ( a,  b) のvx
                 {0, 1,  a},   //               のvy
                 {1, 0,  b},   // 1: 右前 ( a, -b) のvx
                 {0, 1,  a},   //               のvy
                 {1, 0, -b},   // 2: 左後 (-a,  b) のvx
                 {0, 1, -a},   //               のvy
                 {1, 0,  b},   // 3: 右後 (-a, -b) のvx
                 {0, 1, -a}}); //               のvy
}
}  // namespace

// 変数のLは前輪と後輪の距離、Wは左右の車輪同士の距離。vel_xは前進方向の速度、vel_yは横方向の速度、delta_radianは角速度(rad/s)
std::array<WheelCmd, 4> SwerveDrive_IK(
  double vel_x, double vel_y, double delta_radian, double L, double W)
{
  matrix V({{vel_x},
            {vel_y},
            {delta_radian}});
  matrix A = make_A(L, W);

  matrix V_wheel = A * V;  // 逆運動学の計算(8×1: 各車輪の vx, vy)

  std::array<WheelCmd, 4> out{};
  for (int i = 0; i < 4; ++i) {
    const double vx = V_wheel(2 * i, 0);
    const double vy = V_wheel(2 * i + 1, 0);
    out[i].speed = std::hypot(vx, vy);
    out[i].angle = (out[i].speed < 1e-9) ? 0.0 : std::atan2(vy, vx);
  }
  return out;
}

OdomDelta Odom_IK(const std::array<WheelCmd, 4> & wheels, double dt, double yaw, double L, double W)
{
  // 各車輪の速度をベクトル化(8×1: 各車輪の vx, vy)
  matrix V_wheel({{wheels[0].speed * std::cos(wheels[0].angle)},
                  {wheels[0].speed * std::sin(wheels[0].angle)},
                  {wheels[1].speed * std::cos(wheels[1].angle)},
                  {wheels[1].speed * std::sin(wheels[1].angle)},
                  {wheels[2].speed * std::cos(wheels[2].angle)},
                  {wheels[2].speed * std::sin(wheels[2].angle)},
                  {wheels[3].speed * std::cos(wheels[3].angle)},
                  {wheels[3].speed * std::sin(wheels[3].angle)}});

  // 8本の式に対して未知数は3つなので、最小二乗の解 FK_inv = (AᵀA)⁻¹Aᵀ(3×8)を使う
  matrix A = make_A(L, W);
  matrix At = A.T();
  matrix FK_inv = (At * A).inv() * At;

  // タイヤの速度ベクトルから、ロボットを原点とした基準での直交座標系の速度ベクトルへ変換
  matrix V_r = FK_inv * V_wheel;

  const double vx_r = V_r(0, 0);  //(i+1,j+1にあたる成分を取り出す)
  const double vy_r = V_r(1, 0);
  const double d_rad = V_r(2, 0);

  const double dx_r = vx_r * dt;
  const double dy_r = vy_r * dt;

  // ロボットを原点とした基準での直交座標系の変位ベクトル。z成分は角度の変化量d_rad*dt
  matrix dR_r({{dx_r},
               {dy_r},
               {d_rad * dt}});

  // 3×3のyaw回転行列(動力学で出てくる運動座標系A-ξηから固定座標系O-xyへの変換行列)
  matrix R({{std::cos(yaw), -std::sin(yaw), 0},
            {std::sin(yaw),  std::cos(yaw), 0},
            {0,              0,             1}});

  // ロボットを原点とした基準での直交座標系の変位ベクトルに、現在のロボットの初期方向からの傾き(yaw)をかけることで座標変換
  matrix dR = R * dR_r;

  // dt秒後の機体の向き(yaw + d_yaw)をクォータニオンにしたもの(z軸まわりの回転なので x=y=0)
  const double new_yaw = yaw + dR(2, 0);
  const double q_z = std::sin(new_yaw / 2.0);
  const double q_w = std::cos(new_yaw / 2.0);

  OdomDelta out{};
  out.dx = dR(0, 0);
  out.dy = dR(1, 0);
  out.d_yaw = dR(2, 0);
  out.vx_r = vx_r;
  out.vy_r = vy_r;
  out.d_rad = d_rad;
  out.q_z = q_z;
  out.q_w = q_w;
  return out;
}
