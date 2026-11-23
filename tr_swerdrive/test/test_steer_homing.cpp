#include <gtest/gtest.h>

#include <cmath>
#include <deque>

#include "tr_dokusute/steer_homing.hpp"

namespace tr_dokusute {
namespace {

constexpr double kFlagWidth = 64.01 / (163.0 / 2.0);  // 45°
constexpr double kTol = 0.011;  // 模擬の1歩 (0.02rad) の半分 + 丸め

// 目標へ一定の速さで動く操舵と、遅れて値が届くフォトセンサの模擬。
// 1周期に speed*dt (0.02rad) ずつ動くので、端は最大で半歩ずれる。
struct Sim {
  double flag_begin = 2.0;  // フラグの範囲 [rad]
  double flag_end = 2.0 + kFlagWidth;
  double speed = 1.0;  // [rad/s]
  double dt = 0.02;    // 帰還の周期 [s]
  int delay_steps = 2;  // センサの値が届くまでの遅れ (周期の数)

  bool InFlag(double p) const { return p >= flag_begin && p <= flag_end; }

  // 原点合わせが終わるか失敗するまで回す。
  SteerHoming Run(double start, const SteerHomingConfig& config) {
    double position = start;
    std::deque<bool> pipe(delay_steps + 1, InFlag(start));
    SteerHoming homing(config, position, pipe.front());
    for (int i = 0; i < 100000 && !homing.done() && !homing.failed(); ++i) {
      const double step = speed * dt;
      const double diff = homing.target() - position;
      position += std::fabs(diff) < step ? diff : std::copysign(step, diff);
      pipe.push_back(InFlag(position));
      pipe.pop_front();
      homing.Update(position, pipe.front());
    }
    return homing;
  }
};

SteerHomingConfig Config() {
  SteerHomingConfig config;
  config.flag_width = kFlagWidth;
  config.margin = 0.5 * kFlagWidth;  // ノードと同じ
  return config;
}

TEST(SteerHoming, FindsFlagCenterFromOutside) {
  Sim sim;
  const SteerHoming homing = sim.Run(0.0, Config());
  ASSERT_TRUE(homing.done()) << homing.error();
  EXPECT_NEAR(homing.center(), 2.0 + kFlagWidth / 2.0, kTol);
  EXPECT_NEAR(homing.measured_width(), kFlagWidth, 0.03);
  EXPECT_DOUBLE_EQ(homing.target(), homing.center());
}

TEST(SteerHoming, SensorDelayCancelsOutByGoingBothWays) {
  Sim sim;
  sim.delay_steps = 5;  // 0.1s 遅れ = 片道で 0.1rad 行き過ぎる
  sim.speed = 1.0;
  const SteerHoming homing = sim.Run(0.0, Config());
  ASSERT_TRUE(homing.done()) << homing.error();
  EXPECT_NEAR(homing.center(), 2.0 + kFlagWidth / 2.0, kTol);
}

TEST(SteerHoming, LeavesFlagFirstWhenStartingInside) {
  Sim sim;
  const SteerHoming homing = sim.Run(2.3, Config());
  ASSERT_TRUE(homing.done()) << homing.error();
  EXPECT_NEAR(homing.center(), 2.0 + kFlagWidth / 2.0, kTol);
}

TEST(SteerHoming, WorksInNegativeDirection) {
  Sim sim;
  SteerHomingConfig config = Config();
  config.direction = -1;
  const SteerHoming homing = sim.Run(4.0, config);
  ASSERT_TRUE(homing.done()) << homing.error();
  EXPECT_NEAR(homing.center(), 2.0 + kFlagWidth / 2.0, kTol);
}

TEST(SteerHoming, FailsWhenNoFlagWithinSweep) {
  Sim sim;
  sim.flag_begin = 100.0;
  sim.flag_end = 101.0;
  const SteerHoming homing = sim.Run(0.0, Config());
  EXPECT_TRUE(homing.failed());
  EXPECT_NE(homing.error().find("見つからない"), std::string::npos);
}

TEST(SteerHoming, FailsWhenFlagWidthIsWrong) {
  Sim sim;
  sim.flag_end = sim.flag_begin + 0.2;  // 45° より細い物を拾った
  const SteerHoming homing = sim.Run(0.0, Config());
  EXPECT_TRUE(homing.failed());
  EXPECT_NE(homing.error().find("幅"), std::string::npos);
}

TEST(SteerHoming, FailsWhenSensorStaysBlocked) {
  Sim sim;
  sim.flag_begin = -100.0;
  sim.flag_end = 100.0;
  const SteerHoming homing = sim.Run(0.0, Config());
  EXPECT_TRUE(homing.failed());
}

}  // namespace
}  // namespace tr_dokusute
