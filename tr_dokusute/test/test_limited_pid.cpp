#include <gtest/gtest.h>

#include "tr_dokusute/limited_pid.hpp"

namespace tr_dokusute {
namespace {

constexpr float kDt = 0.02f;  // tr_pid の周期 [s]

TEST(LimitedPid, IntegralTermStopsAtIntegralLimit) {
  // P も D も 0 で、誤差 1 を長く続ける。I 項は integral_limit で止まる
  LimitedPid pid(0.0f, 10.0f, 0.0f, 255.0f, 50.0f);
  pid.set_target(1.0f);
  float output = 0.0f;
  for (int i = 0; i < 1000; ++i) output = pid.update(0.0f, kDt);
  EXPECT_FLOAT_EQ(output, 50.0f);
}

TEST(LimitedPid, IntegralDoesNotWindUpBeyondLimit) {
  // 上限に張り付いた後で誤差の向きが変わったら、すぐに下がり始める
  // (積分が上限より奥に溜まっていない)。積分は台形なので、向きが変わった
  // 1歩目は前の誤差と打ち消し合って動かず、2歩目から下がる
  LimitedPid pid(0.0f, 10.0f, 0.0f, 255.0f, 50.0f);
  pid.set_target(1.0f);
  for (int i = 0; i < 1000; ++i) pid.update(0.0f, kDt);
  pid.set_target(-1.0f);
  pid.update(0.0f, kDt);
  EXPECT_LT(pid.update(0.0f, kDt), 50.0f);
}

TEST(LimitedPid, ProportionalTermUsesOutputLimit) {
  // P 項は integral_limit ではなく output_limit で止まる
  LimitedPid pid(100.0f, 0.0f, 0.0f, 255.0f, 50.0f);
  pid.set_target(1.0f);
  EXPECT_FLOAT_EQ(pid.update(0.0f, kDt), 100.0f);
  pid.set_target(10.0f);
  EXPECT_FLOAT_EQ(pid.update(0.0f, kDt), 255.0f);
}

TEST(LimitedPid, SumIsClampedToOutputLimit) {
  LimitedPid pid(200.0f, 10.0f, 0.0f, 255.0f, 100.0f);
  pid.set_target(1.0f);
  float output = 0.0f;
  for (int i = 0; i < 1000; ++i) output = pid.update(0.0f, kDt);
  EXPECT_FLOAT_EQ(output, 255.0f);  // 200 + 100 を 255 で止める
}

TEST(LimitedPid, MatchesPidControllerWhenLimitsAreEqual) {
  // integral_limit = output_limit で、出力が上限に届かないなら
  // PIDController 1つと同じ値になる
  LimitedPid limited(2.0f, 1.0f, 0.1f, 255.0f, 255.0f);
  PIDController plain(2.0f, 1.0f, 0.1f, 255.0f);
  limited.set_target(3.0f);
  plain.set_target(3.0f);
  float current = 0.0f;
  for (int i = 0; i < 200; ++i) {
    EXPECT_NEAR(limited.update(current, kDt), plain.update(current, kDt),
                1e-4f);
    current += 0.01f;
  }
}

TEST(LimitedPid, ResetClearsIntegral) {
  LimitedPid pid(0.0f, 10.0f, 0.0f, 255.0f, 50.0f);
  pid.set_target(1.0f);
  for (int i = 0; i < 100; ++i) pid.update(0.0f, kDt);
  pid.reset();
  pid.set_target(0.0f);
  EXPECT_FLOAT_EQ(pid.update(0.0f, kDt), 0.0f);
}

}  // namespace
}  // namespace tr_dokusute
