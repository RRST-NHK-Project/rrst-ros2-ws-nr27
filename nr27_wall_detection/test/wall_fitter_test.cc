#include "nr27_wall_detection/wall_fitter.h"

#include <cmath>
#include <random>
#include <vector>

#include "gtest/gtest.h"

namespace nr27_wall_detection {
namespace {

constexpr double kAngleTolerance = DegToRad(0.5);
constexpr double kDistanceTolerance = 0.01;

// n . p = d の直線上に、壁方向 t in [t_begin, t_end] の点を並べる。
void AddWall(double normal_deg, double distance, double t_begin, double t_end,
             double spacing, double noise_sigma, std::mt19937* rng,
             std::vector<Point2D>* points) {
  const double normal = DegToRad(normal_deg);
  const double nx = std::cos(normal);
  const double ny = std::sin(normal);
  std::normal_distribution<double> noise(0.0, noise_sigma);
  for (double t = t_begin; t <= t_end; t += spacing) {
    const double e = noise_sigma > 0.0 ? noise(*rng) : 0.0;
    points->push_back(
        {(distance + e) * nx - t * ny, (distance + e) * ny + t * nx});
  }
}

FitterConfig MakeConfig(double target_normal_deg) {
  FitterConfig config;
  config.target_normal_angle = DegToRad(target_normal_deg);
  return config;
}

TEST(WrapPiTest, FoldsIntoHalfOpenRange) {
  EXPECT_NEAR(WrapPi(3.0 * kPi), -kPi, 1e-12);
  EXPECT_NEAR(WrapPi(-3.5 * kPi), 0.5 * kPi, 1e-12);
  EXPECT_NEAR(WrapPi(0.1), 0.1, 1e-12);
}

TEST(WallFitterTest, FrontWall) {
  std::mt19937 rng(1);
  std::vector<Point2D> points;
  AddWall(0.0, 1.0, -0.6, 0.6, 0.01, 0.0, &rng, &points);
  WallFitter fitter(MakeConfig(0.0), 42);
  const auto wall = fitter.FindWall(points, nullptr);
  ASSERT_TRUE(wall.has_value());
  EXPECT_NEAR(wall->normal_angle, 0.0, kAngleTolerance);
  EXPECT_NEAR(wall->distance, 1.0, kDistanceTolerance);
  EXPECT_NEAR(wall->segment_length, 1.2, 0.02);
}

// 旧実装 (a > 0 正規化) で +90 / -90 が入れ替わっていた横壁のケース。
TEST(WallFitterTest, SideWallNormalIsContinuous) {
  std::mt19937 rng(2);
  for (double tilt : {-2.0, -0.5, 0.0, 0.5, 2.0}) {
    std::vector<Point2D> left;
    AddWall(90.0 + tilt, 0.8, -1.0, 1.0, 0.01, 0.0, &rng, &left);
    WallFitter fitter(MakeConfig(90.0), 42);
    const auto wall = fitter.FindWall(left, nullptr);
    ASSERT_TRUE(wall.has_value()) << "tilt=" << tilt;
    EXPECT_NEAR(WrapPi(wall->normal_angle - DegToRad(90.0 + tilt)), 0.0,
                kAngleTolerance)
        << "tilt=" << tilt;
    EXPECT_NEAR(wall->distance, 0.8, kDistanceTolerance);
  }
}

TEST(WallFitterTest, RightWallHasNegativeNormal) {
  std::mt19937 rng(3);
  std::vector<Point2D> points;
  AddWall(-90.0, 0.5, -1.0, 1.0, 0.01, 0.0, &rng, &points);
  WallFitter fitter(MakeConfig(-90.0), 42);
  const auto wall = fitter.FindWall(points, nullptr);
  ASSERT_TRUE(wall.has_value());
  EXPECT_NEAR(wall->normal_angle, DegToRad(-90.0), kAngleTolerance);
}

TEST(WallFitterTest, TiltedWallWithNoise) {
  std::mt19937 rng(4);
  std::vector<Point2D> points;
  AddWall(12.0, 1.5, -0.8, 0.8, 0.007, 0.01, &rng, &points);
  WallFitter fitter(MakeConfig(0.0), 42);
  const auto wall = fitter.FindWall(points, nullptr);
  ASSERT_TRUE(wall.has_value());
  EXPECT_NEAR(wall->normal_angle, DegToRad(12.0), DegToRad(1.0));
  EXPECT_NEAR(wall->distance, 1.5, 0.01);
}

// 凹形: 側面の方が点数が多くても、狙いの向き (正面) の面を選ぶ。
TEST(WallFitterTest, ConcaveShapePicksFacingSurface) {
  std::mt19937 rng(5);
  std::vector<Point2D> points;
  AddWall(0.0, 1.2, -0.4, 0.4, 0.01, 0.005, &rng, &points);     // 奥の面
  AddWall(90.0, 0.4, -1.2, -0.3, 0.005, 0.005, &rng, &points);  // 左の側面
  AddWall(-90.0, 0.4, 0.3, 1.2, 0.005, 0.005, &rng, &points);   // 右の側面
  WallFitter fitter(MakeConfig(0.0), 42);
  std::vector<WallLine> candidates;
  const auto wall = fitter.FindWall(points, &candidates);
  ASSERT_TRUE(wall.has_value());
  EXPECT_GE(candidates.size(), 3u);
  EXPECT_NEAR(wall->normal_angle, 0.0, DegToRad(1.0));
  EXPECT_NEAR(wall->distance, 1.2, 0.01);
}

TEST(WallFitterTest, ShortSegmentIsRejected) {
  std::mt19937 rng(6);
  std::vector<Point2D> points;
  AddWall(0.0, 1.0, -0.05, 0.05, 0.005, 0.0, &rng, &points);  // 柱
  FitterConfig config = MakeConfig(0.0);
  config.min_inliers = 5;
  WallFitter fitter(config, 42);
  EXPECT_FALSE(fitter.FindWall(points, nullptr).has_value());
}

TEST(WallFitterTest, TrackWallFollowsPrior) {
  std::mt19937 rng(7);
  std::vector<Point2D> points;
  AddWall(3.0, 1.0, -0.6, 0.6, 0.01, 0.005, &rng, &points);
  AddWall(0.0, 3.0, -2.0, 2.0, 0.01, 0.005, &rng, &points);  // 奥の別の壁
  WallFitter fitter(MakeConfig(0.0), 42);
  WallLine prior;
  prior.normal_angle = DegToRad(2.0);
  prior.distance = 1.01;
  const auto wall = fitter.TrackWall(points, prior, 1.5);
  ASSERT_TRUE(wall.has_value());
  EXPECT_NEAR(wall->normal_angle, DegToRad(3.0), DegToRad(0.5));
  EXPECT_NEAR(wall->distance, 1.0, 0.01);
}

TEST(ScanToPointsTest, MountYawRotatesIntoBaseFrame) {
  // LiDAR 正面 (beam 0) に 1 m。LiDAR は左向き (+90deg) 取り付け。
  ScanFilterConfig config;
  config.mount_x = 0.1;
  config.mount_yaw = DegToRad(90.0);
  config.fov_center = DegToRad(90.0);
  const std::vector<float> ranges = {1.0f};
  const auto points = ScanToPoints(0.0, 0.01, ranges, 0.02, 30.0, config);
  ASSERT_EQ(points.size(), 1u);
  EXPECT_NEAR(points[0].x, 0.1, 1e-9);
  EXPECT_NEAR(points[0].y, 1.0, 1e-9);
}

TEST(ScanToPointsTest, UpsideDownFlipsBeamAngle) {
  ScanFilterConfig config;
  config.upside_down = true;
  const std::vector<float> ranges = {1.0f};
  const auto points =
      ScanToPoints(DegToRad(30.0), 0.01, ranges, 0.02, 30.0, config);
  ASSERT_EQ(points.size(), 1u);
  EXPECT_LT(points[0].y, 0.0);
}

TEST(ScanToPointsTest, FovAndRangeGate) {
  ScanFilterConfig config;
  config.fov_half = DegToRad(45.0);
  config.range_max = 2.0;
  // -90, 0, +90 deg の 3 本。
  const std::vector<float> ranges = {1.0f, 1.0f, 1.0f};
  EXPECT_EQ(
      ScanToPoints(DegToRad(-90.0), DegToRad(90.0), ranges, 0.02, 30.0, config)
          .size(),
      1u);
  const std::vector<float> far = {3.0f};
  EXPECT_TRUE(ScanToPoints(0.0, 0.01, far, 0.02, 30.0, config).empty());
}

TEST(WallStateFilterTest, PredictCompensatesRotationAndTranslation) {
  WallStateFilter filter(StateFilterConfig{});
  WallLine wall;
  wall.normal_angle = 0.0;
  wall.distance = 1.0;
  ASSERT_TRUE(filter.Update(wall, 0.0));
  filter.Predict(0.2, 0.0, DegToRad(10.0));
  EXPECT_NEAR(filter.normal_angle(), DegToRad(-10.0), 1e-9);
  EXPECT_NEAR(filter.distance(), 0.8, 1e-9);
}

TEST(WallStateFilterTest, RejectsOutliersThenReinitializes) {
  StateFilterConfig config;
  config.max_consecutive_rejects = 3;
  WallStateFilter filter(config);
  WallLine wall;
  wall.normal_angle = 0.0;
  wall.distance = 1.0;
  ASSERT_TRUE(filter.Update(wall, 0.000));

  WallLine jumped = wall;
  jumped.normal_angle = DegToRad(40.0);
  EXPECT_FALSE(filter.Update(jumped, 0.025));
  EXPECT_FALSE(filter.Update(jumped, 0.050));
  EXPECT_NEAR(filter.normal_angle(), 0.0, 1e-9);
  EXPECT_TRUE(filter.Update(jumped, 0.075));  // 3 回連続 -> 本物とみなす
  EXPECT_NEAR(filter.normal_angle(), DegToRad(40.0), 1e-9);
}

TEST(WallStateFilterTest, ExpiresAfterTimeout) {
  StateFilterConfig config;
  config.lost_timeout = 0.2;
  WallStateFilter filter(config);
  WallLine wall;
  wall.distance = 1.0;
  ASSERT_TRUE(filter.Update(wall, 1.0));
  EXPECT_TRUE(filter.IsValid(1.1));
  EXPECT_FALSE(filter.IsValid(1.3));
}

TEST(WallStateFilterTest, LowPassConvergesWithTimeConstant) {
  StateFilterConfig config;
  config.angle_time_constant = 0.1;
  WallStateFilter filter(config);
  WallLine wall;
  wall.distance = 1.0;
  ASSERT_TRUE(filter.Update(wall, 0.0));
  wall.normal_angle = DegToRad(5.0);
  // 40 Hz で 0.1 s (= 時定数) 経過すると約 63% 追従。
  for (int i = 1; i <= 4; ++i) filter.Update(wall, 0.025 * i);
  const double ratio = filter.normal_angle() / DegToRad(5.0);
  EXPECT_GT(ratio, 0.55);
  EXPECT_LT(ratio, 0.70);
}

}  // namespace
}  // namespace nr27_wall_detection
