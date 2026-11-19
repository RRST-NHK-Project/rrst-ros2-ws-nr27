// ros2can ファーム (damiao.cpp) の単位と一致しているかの確認。
// 速度系は 0.01 rad/s/LSB、位置は 0.001 rad/LSB。
#include <gtest/gtest.h>

#include <vector>

#include "tr_dokusute/damiao_slots.hpp"

namespace tr_dokusute {
namespace damiao {
namespace {

std::vector<int16_t> Zeros() { return std::vector<int16_t>(kSlotCount, 0); }

TEST(DamiaoSlotsTest, PositionCommandUsesFirmwareScales) {
  auto tx = Zeros();
  SetCommand(tx, 0, Mode::kPositionVelocity, 1.234, 5.0);
  EXPECT_EQ(tx[0], 2);
  EXPECT_EQ(tx[1], 1234);  // 0.001 rad/LSB
  EXPECT_EQ(tx[2], 500);   // 0.01 rad/s/LSB (5 rad/s)
}

TEST(DamiaoSlotsTest, VelocityCommandUsesVelocityScale) {
  auto tx = Zeros();
  SetCommand(tx, 0, Mode::kVelocity, 2.0, 0.0);
  EXPECT_EQ(tx[0], 1);
  EXPECT_EQ(tx[1], 200);  // 0.01 rad/s/LSB
}

TEST(DamiaoSlotsTest, SecondMotorUsesNextSixSlots) {
  auto tx = Zeros();
  SetCommand(tx, 1, Mode::kPositionVelocity, 1.0, 1.0);
  EXPECT_EQ(tx[6], 2);
  EXPECT_EQ(tx[7], 1000);
  EXPECT_EQ(tx[0], 0);
}

TEST(DamiaoSlotsTest, DisabledIsAllZero) {
  auto tx = Zeros();
  SetCommand(tx, 0, Mode::kDisabled, 0.0, 0.0);
  for (int16_t v : tx) EXPECT_EQ(v, 0);
}

TEST(DamiaoSlotsTest, FeedbackIsDecodedWithFirmwareScales) {
  auto rx = Zeros();
  rx[0] = 1;
  rx[1] = 42;
  rx[3] = -3141;  // -3.141 rad
  rx[4] = 250;    // 2.5 rad/s
  const Feedback fb = GetFeedback(rx, 0);
  EXPECT_EQ(fb.state, 1);
  EXPECT_EQ(fb.counter, 42);
  EXPECT_NEAR(fb.position, -3.141, 1e-9);
  EXPECT_NEAR(fb.velocity, 2.5, 1e-9);
}

TEST(DamiaoSlotsTest, FaultStates) {
  EXPECT_FALSE(IsFault(0));
  EXPECT_FALSE(IsFault(1));
  EXPECT_TRUE(IsFault(13));  // 通信喪失
  EXPECT_TRUE(IsFault(15));  // ESP32が止めている
}

TEST(DamiaoSlotsTest, PhotoSensorFollowsFirmwareValues) {
  auto rx = Zeros();
  EXPECT_FALSE(IsPhotoSensorBlocked(rx).has_value());  // 0 = 未対応・未確定
  rx[23] = 1;
  EXPECT_EQ(IsPhotoSensorBlocked(rx), false);  // 入光
  rx[23] = 2;
  EXPECT_EQ(IsPhotoSensorBlocked(rx), true);  // 遮光
  rx[23] = 7;
  EXPECT_FALSE(IsPhotoSensorBlocked(rx).has_value());
  EXPECT_FALSE(IsPhotoSensorBlocked(std::vector<int16_t>(23, 2)).has_value());
}

}  // namespace
}  // namespace damiao
}  // namespace tr_dokusute
