#include <gtest/gtest.h>

#include <cmath>

#include "tr_dokusute/swerve_kinematics.hpp"

namespace tr_dokusute {
namespace {

constexpr double kEps = 1e-9;
constexpr double kDeg = M_PI / 180.0;

TEST(ComputeWheelVelocityTest, Translation) {
  const WheelCommand w = ComputeWheelVelocity({0.0, 1.0, 0.0}, 0.25, 0.25);
  EXPECT_NEAR(w.angle, 90 * kDeg, kEps);  // 左へ
  EXPECT_NEAR(w.speed, 1.0, kEps);
}

TEST(ComputeWheelVelocityTest, SpinInPlaceIsTangent) {
  // 反時計回りのその場旋回: 左前の車輪 (0.25, 0.25) は左後ろ (135°) へ動く
  const WheelCommand w = ComputeWheelVelocity({0.0, 0.0, 1.0}, 0.25, 0.25);
  EXPECT_NEAR(w.angle, 135 * kDeg, kEps);
  EXPECT_NEAR(w.speed, std::hypot(0.25, 0.25), kEps);
}

TEST(ChooseSteerAngleTest, ForwardKeepsAngle) {
  const WheelCommand c = ChooseSteerAngle(0.0, 0.0, M_PI);
  EXPECT_NEAR(c.angle, 0.0, kEps);
  EXPECT_EQ(c.speed, 1.0);
}

TEST(ChooseSteerAngleTest, BackwardReversesInsteadOfTurning) {
  // 後ろへ進むときは 180° 回さず、向きはそのままで逆転する
  const WheelCommand c = ChooseSteerAngle(M_PI, 0.0, M_PI);
  EXPECT_NEAR(c.angle, 0.0, kEps);
  EXPECT_EQ(c.speed, -1.0);
}

TEST(ChooseSteerAngleTest, NeverTurnsMoreThan90Degrees) {
  // 斜め左後ろ (135°) は、右斜め前 (-45°) を向いて逆転する
  const WheelCommand c = ChooseSteerAngle(135 * kDeg, 0.0, M_PI);
  EXPECT_NEAR(c.angle, -45 * kDeg, kEps);
  EXPECT_EQ(c.speed, -1.0);
}

TEST(ChooseSteerAngleTest, UnwindsAtLimit) {
  // 170° まで来た車輪を 190° (= -170°) の方向へ進めたいとき、±180° を
  // 超えられないので、10° まで巻き戻して逆転する
  const WheelCommand c = ChooseSteerAngle(-170 * kDeg, 170 * kDeg, M_PI);
  EXPECT_NEAR(c.angle, 10 * kDeg, kEps);
  EXPECT_EQ(c.speed, -1.0);
}

TEST(ChooseSteerAngleTest, StaysWithinLimit) {
  for (double goal = -M_PI; goal <= M_PI; goal += 5 * kDeg) {
    for (double current = -M_PI; current <= M_PI; current += 5 * kDeg) {
      const WheelCommand c = ChooseSteerAngle(goal, current, M_PI / 2.0);
      EXPECT_LE(std::fabs(c.angle), M_PI / 2.0 + kEps);
    }
  }
}

}  // namespace
}  // namespace tr_dokusute
