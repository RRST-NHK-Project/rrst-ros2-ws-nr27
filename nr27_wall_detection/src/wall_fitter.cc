#include "nr27_wall_detection/wall_fitter.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

namespace nr27_wall_detection {
namespace {

// 法線を原点 -> 壁 の向きにそろえる。
void OrientTowardWall(double* nx, double* ny, double* d) {
  if (*d < 0.0) {
    *nx = -*nx;
    *ny = -*ny;
    *d = -*d;
  }
}

// 内側ループ用。cos/sin を毎回計算しないよう法線ベクトルを保持する。
struct NormalForm {
  explicit NormalForm(const WallLine& line)
      : nx(std::cos(line.normal_angle)),
        ny(std::sin(line.normal_angle)),
        d(line.distance) {}
  double Residual(const Point2D& p) const { return nx * p.x + ny * p.y - d; }
  double nx;
  double ny;
  double d;
};

// line の近傍 (|residual| < gate) にある点から、インライア数・端点・
// 壁方向の広がりを埋める。
void FillSupport(const std::vector<Point2D>& points, double gate,
                 WallLine* line) {
  const NormalForm form(*line);
  const double nx = form.nx;
  const double ny = form.ny;
  // 壁に沿った方向ベクトル。
  const double tx = -ny;
  const double ty = nx;

  int count = 0;
  double t_min = std::numeric_limits<double>::infinity();
  double t_max = -std::numeric_limits<double>::infinity();
  for (const Point2D& p : points) {
    if (std::abs(form.Residual(p)) >= gate) continue;
    ++count;
    const double t = tx * p.x + ty * p.y;
    t_min = std::min(t_min, t);
    t_max = std::max(t_max, t);
  }

  line->inlier_count = count;
  if (count == 0) {
    line->segment_length = 0.0;
    return;
  }
  const double foot_x = line->distance * nx;
  const double foot_y = line->distance * ny;
  line->endpoint_begin = {foot_x + t_min * tx, foot_y + t_min * ty};
  line->endpoint_end = {foot_x + t_max * tx, foot_y + t_max * ty};
  line->segment_length = t_max - t_min;
}

}  // namespace

double WrapPi(double angle) {
  angle = std::fmod(angle + kPi, 2.0 * kPi);
  if (angle < 0.0) angle += 2.0 * kPi;
  return angle - kPi;
}

double WallLine::Residual(const Point2D& p) const {
  return std::cos(normal_angle) * p.x + std::sin(normal_angle) * p.y - distance;
}

std::vector<Point2D> ScanToPoints(double angle_min, double angle_increment,
                                  const std::vector<float>& ranges,
                                  double sensor_range_min,
                                  double sensor_range_max,
                                  const ScanFilterConfig& config) {
  std::vector<Point2D> points;
  points.reserve(ranges.size());

  const double cos_mount = std::cos(config.mount_yaw);
  const double sin_mount = std::sin(config.mount_yaw);
  const bool full_fov = config.fov_half >= kPi;

  for (std::size_t i = 0; i < ranges.size(); ++i) {
    const double r = ranges[i];
    if (!std::isfinite(r)) continue;
    if (r < sensor_range_min || r > sensor_range_max) continue;
    if (r < config.range_min || r > config.range_max) continue;

    double beam = angle_min + static_cast<double>(i) * angle_increment;
    if (config.upside_down) beam = -beam;

    const double sx = r * std::cos(beam);
    const double sy = r * std::sin(beam);
    const Point2D p{config.mount_x + cos_mount * sx - sin_mount * sy,
                    config.mount_y + sin_mount * sx + cos_mount * sy};

    if (!full_fov) {
      const double bearing = std::atan2(p.y, p.x);
      if (std::abs(WrapPi(bearing - config.fov_center)) > config.fov_half) {
        continue;
      }
    }
    if (config.use_roi && (p.x < config.roi_x_min || p.x > config.roi_x_max ||
                           p.y < config.roi_y_min || p.y > config.roi_y_max)) {
      continue;
    }
    points.push_back(p);
  }
  return points;
}

WallFitter::WallFitter(const FitterConfig& config, std::uint32_t seed)
    : config_(config), rng_(seed) {}

std::optional<WallLine> WallFitter::FitTotalLeastSquares(
    const std::vector<Point2D>& points) {
  if (points.size() < 2) return std::nullopt;

  double mean_x = 0.0;
  double mean_y = 0.0;
  for (const Point2D& p : points) {
    mean_x += p.x;
    mean_y += p.y;
  }
  mean_x /= static_cast<double>(points.size());
  mean_y /= static_cast<double>(points.size());

  double sxx = 0.0;
  double syy = 0.0;
  double sxy = 0.0;
  for (const Point2D& p : points) {
    const double dx = p.x - mean_x;
    const double dy = p.y - mean_y;
    sxx += dx * dx;
    syy += dy * dy;
    sxy += dx * dy;
  }
  if (sxx + syy < 1e-12) return std::nullopt;  // 全点が一点に重なっている

  // 共分散の最大固有値方向 = 壁の向き。法線はそれに直交。
  const double direction = 0.5 * std::atan2(2.0 * sxy, sxx - syy);
  double nx = -std::sin(direction);
  double ny = std::cos(direction);
  double d = nx * mean_x + ny * mean_y;
  OrientTowardWall(&nx, &ny, &d);

  WallLine line;
  line.normal_angle = std::atan2(ny, nx);
  line.distance = d;
  line.inlier_count = static_cast<int>(points.size());
  return line;
}

std::optional<WallLine> WallFitter::RansacSeed(
    const std::vector<Point2D>& points) {
  if (points.size() < 2) return std::nullopt;

  std::uniform_int_distribution<std::size_t> pick(0, points.size() - 1);
  const double min_separation = 2.0 * config_.inlier_threshold;

  std::optional<WallLine> best;
  for (int iter = 0; iter < config_.ransac_iterations; ++iter) {
    const Point2D& p1 = points[pick(rng_)];
    const Point2D& p2 = points[pick(rng_)];
    const double dx = p2.x - p1.x;
    const double dy = p2.y - p1.y;
    const double length = std::hypot(dx, dy);
    // 近すぎる 2 点は向きがノイズで決まるので使わない。
    if (length < min_separation) continue;

    double nx = -dy / length;
    double ny = dx / length;
    double d = nx * p1.x + ny * p1.y;
    OrientTowardWall(&nx, &ny, &d);

    WallLine candidate;
    candidate.normal_angle = std::atan2(ny, nx);
    candidate.distance = d;
    int count = 0;
    for (const Point2D& p : points) {
      if (std::abs(nx * p.x + ny * p.y - d) < config_.inlier_threshold) {
        ++count;
      }
    }
    candidate.inlier_count = count;
    if (!best || count > best->inlier_count) best = candidate;
  }
  return best;
}

std::optional<WallLine> WallFitter::Refine(const std::vector<Point2D>& points,
                                           const WallLine& seed,
                                           double gate) const {
  const NormalForm form(seed);
  std::vector<Point2D> near;
  near.reserve(points.size());
  for (const Point2D& p : points) {
    if (std::abs(form.Residual(p)) < gate) near.push_back(p);
  }
  std::optional<WallLine> refined = FitTotalLeastSquares(near);
  if (!refined) return std::nullopt;
  FillSupport(points, config_.inlier_threshold, &*refined);
  return refined;
}

bool WallFitter::IsAcceptable(const WallLine& line,
                              std::size_t total_points) const {
  return RejectReason(line, total_points).empty();
}

std::string WallFitter::RejectReason(const WallLine& line,
                                     std::size_t total_points) const {
  if (total_points == 0) return "no points";
  if (line.inlier_count < config_.min_inliers) return "few inliers";
  const double ratio = static_cast<double>(line.inlier_count) /
                       static_cast<double>(total_points);
  if (ratio < config_.min_inlier_ratio) return "low ratio";
  if (line.segment_length < config_.min_segment_length) return "too short";
  const double deviation =
      std::abs(WrapPi(line.normal_angle - config_.target_normal_angle));
  if (deviation > config_.normal_tolerance) return "wrong direction";
  return "";
}

std::optional<WallLine> WallFitter::FindWall(
    const std::vector<Point2D>& points, std::vector<WallLine>* candidates) {
  if (candidates != nullptr) candidates->clear();

  std::vector<Point2D> remaining = points;
  std::optional<WallLine> best;

  for (int k = 0; k < config_.max_candidates; ++k) {
    if (static_cast<int>(remaining.size()) < config_.min_inliers) break;

    const std::optional<WallLine> seed = RansacSeed(remaining);
    if (!seed || seed->inlier_count < config_.min_inliers) break;

    const std::optional<WallLine> candidate =
        Refine(remaining, *seed, config_.inlier_threshold);
    if (!candidate || candidate->inlier_count < config_.min_inliers) break;

    if (candidates != nullptr) candidates->push_back(*candidate);
    if (IsAcceptable(*candidate, points.size()) &&
        (!best || candidate->inlier_count > best->inlier_count)) {
      best = candidate;
    }

    // この面の点を除いて次の面を探す。
    const NormalForm form(*candidate);
    std::vector<Point2D> rest;
    rest.reserve(remaining.size());
    for (const Point2D& p : remaining) {
      if (std::abs(form.Residual(p)) >= config_.inlier_threshold) {
        rest.push_back(p);
      }
    }
    if (rest.size() == remaining.size()) break;  // 進捗なし
    remaining.swap(rest);
  }
  return best;
}

std::optional<WallLine> WallFitter::TrackWall(
    const std::vector<Point2D>& points, const WallLine& prior,
    double gate_scale) const {
  const std::optional<WallLine> refined =
      Refine(points, prior, config_.inlier_threshold * gate_scale);
  if (!refined || !IsAcceptable(*refined, points.size())) return std::nullopt;
  return refined;
}

WallStateFilter::WallStateFilter(const StateFilterConfig& config)
    : config_(config) {}

void WallStateFilter::Predict(double dx, double dy, double dyaw) {
  if (!initialized_) return;
  // 前回のロボット座標系で、新しい原点 (dx, dy) から壁までの距離。
  const double moved_toward_wall =
      std::cos(normal_angle_) * dx + std::sin(normal_angle_) * dy;
  distance_ = std::max(0.0, distance_ - moved_toward_wall);
  // ロボットが CCW に dyaw 回ると、ロボットから見た法線は CW に dyaw 回る。
  normal_angle_ = WrapPi(normal_angle_ - dyaw);
}

bool WallStateFilter::Update(const WallLine& measurement, double stamp_sec) {
  if (!initialized_ || stamp_sec - last_update_sec_ > config_.lost_timeout) {
    Initialize(measurement, stamp_sec);
    return true;
  }

  const double angle_innovation =
      WrapPi(measurement.normal_angle - normal_angle_);
  const double distance_innovation = measurement.distance - distance_;
  if (std::abs(angle_innovation) > config_.max_angle_jump ||
      std::abs(distance_innovation) > config_.max_distance_jump) {
    ++consecutive_rejects_;
    if (consecutive_rejects_ >= config_.max_consecutive_rejects) {
      Initialize(measurement, stamp_sec);
      return true;
    }
    return false;
  }
  consecutive_rejects_ = 0;

  const double dt = std::max(0.0, stamp_sec - last_update_sec_);
  auto gain = [dt](double time_constant) {
    return time_constant <= 0.0 ? 1.0 : dt / (time_constant + dt);
  };
  normal_angle_ = WrapPi(normal_angle_ +
                         gain(config_.angle_time_constant) * angle_innovation);
  distance_ += gain(config_.distance_time_constant) * distance_innovation;
  last_update_sec_ = stamp_sec;
  return true;
}

bool WallStateFilter::IsValid(double stamp_sec) const {
  return initialized_ && stamp_sec - last_update_sec_ <= config_.lost_timeout;
}

void WallStateFilter::Reset() {
  initialized_ = false;
  consecutive_rejects_ = 0;
}

WallLine WallStateFilter::line() const {
  WallLine line;
  line.normal_angle = normal_angle_;
  line.distance = distance_;
  return line;
}

void WallStateFilter::Initialize(const WallLine& measurement,
                                 double stamp_sec) {
  initialized_ = true;
  normal_angle_ = measurement.normal_angle;
  distance_ = measurement.distance;
  last_update_sec_ = stamp_sec;
  consecutive_rejects_ = 0;
}

}  // namespace nr27_wall_detection
