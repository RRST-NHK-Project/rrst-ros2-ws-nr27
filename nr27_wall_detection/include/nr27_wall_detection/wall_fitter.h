// ROS 非依存の壁検出コア。
//
// 座標系はロボット基準 (base frame): x = 前方 [m], y = 左方向 [m]。
// 壁は「原点から壁へ向かう単位法線 n と垂線距離 d」で表す:
//
//   n = (cos(normal_angle), sin(normal_angle)),  n . p = d,  d >= 0
//
// ax + by + c = 0 を「a > 0 に正規化」する旧方式は、横壁 (a ~= 0) で法線の
// 向きが +90deg / -90deg を行き来する。ここでは法線を常に壁側へ向けることで
// 前・左・右・後ろどの壁でも向きが一意に決まるようにしている。

#ifndef NR27_WALL_DETECTION_WALL_FITTER_H_
#define NR27_WALL_DETECTION_WALL_FITTER_H_

#include <cstdint>
#include <optional>
#include <random>
#include <string>
#include <vector>

namespace nr27_wall_detection {

inline constexpr double kPi = 3.14159265358979323846;

// 角度を [-pi, pi) に折り返す。
double WrapPi(double angle);

inline constexpr double DegToRad(double deg) { return deg * kPi / 180.0; }
inline constexpr double RadToDeg(double rad) { return rad * 180.0 / kPi; }

struct Point2D {
  double x = 0.0;
  double y = 0.0;
};

struct WallLine {
  double normal_angle = 0.0;  // 原点 -> 壁 の法線方向 [rad]
  double distance = 0.0;      // 原点から壁までの垂線距離 [m] (>= 0)
  int inlier_count = 0;
  double segment_length = 0.0;  // インライアの壁方向の広がり [m]
  Point2D endpoint_begin;       // インライアの端点 (可視化用)
  Point2D endpoint_end;

  // 符号付き距離 (壁より原点側なら負)。
  double Residual(const Point2D& p) const;
};

// LaserScan -> base frame の点群変換とマスクの設定。
struct ScanFilterConfig {
  // LiDAR の取り付け位置・向き (base frame)。
  double mount_x = 0.0;
  double mount_y = 0.0;
  double mount_yaw = 0.0;
  // 逆さ付けのとき true (スキャン角の符号を反転)。
  bool upside_down = false;
  // 距離ゲート (LiDAR からの距離) [m]。
  double range_min = 0.05;
  double range_max = 5.0;
  // 視野: base frame で fov_center 方向 +-fov_half だけ残す [rad]。
  double fov_center = 0.0;
  double fov_half = DegToRad(60.0);
  // 矩形 ROI (base frame) [m]。
  bool use_roi = false;
  double roi_x_min = -5.0;
  double roi_x_max = 5.0;
  double roi_y_min = -5.0;
  double roi_y_max = 5.0;
};

// sensor_msgs/LaserScan の中身を base frame の点群に変換する。
std::vector<Point2D> ScanToPoints(double angle_min, double angle_increment,
                                  const std::vector<float>& ranges,
                                  double sensor_range_min,
                                  double sensor_range_max,
                                  const ScanFilterConfig& config);

struct FitterConfig {
  int ransac_iterations = 100;
  double inlier_threshold = 0.03;  // [m]
  int min_inliers = 15;
  double min_inlier_ratio = 0.15;  // 入力点数に対するインライア比
  double min_segment_length = 0.30;  // 柱など短い面を弾く [m]
  int max_candidates = 4;  // 逐次 RANSAC で抜き出す面の上限
  // 狙う壁の法線方向 (0 = 前の壁, +pi/2 = 左の壁, -pi/2 = 右の壁)。
  double target_normal_angle = 0.0;
  // 法線がこれ以上ずれた面は対象外 (凹形の側面などを弾く) [rad]。
  double normal_tolerance = DegToRad(35.0);
};

class WallFitter {
 public:
  WallFitter(const FitterConfig& config, std::uint32_t seed);

  WallFitter(const WallFitter&) = delete;
  WallFitter& operator=(const WallFitter&) = delete;

  // 逐次 RANSAC で面を抜き出し、狙いの向きに合う面のうち最も支持点の多い
  // ものを返す。candidates が非 null なら抜き出した全候補を格納する。
  std::optional<WallLine> FindWall(const std::vector<Point2D>& points,
                                   std::vector<WallLine>* candidates);

  // prior の近傍 (inlier_threshold * gate_scale) の点だけで再フィットする。
  // RANSAC を省略するウォームスタート用。条件を満たさなければ nullopt。
  std::optional<WallLine> TrackWall(const std::vector<Point2D>& points,
                                    const WallLine& prior,
                                    double gate_scale) const;

  // 総最小二乗 (TLS) で直線を当てはめる。点が 2 未満なら nullopt。
  static std::optional<WallLine> FitTotalLeastSquares(
      const std::vector<Point2D>& points);

  // line が採用条件を満たさない理由 (rviz 表示用)。満たすなら空文字列。
  std::string RejectReason(const WallLine& line,
                           std::size_t total_points) const;

  const FitterConfig& config() const { return config_; }
  void set_config(const FitterConfig& config) { config_ = config; }

 private:
  std::optional<WallLine> RansacSeed(const std::vector<Point2D>& points);
  // seed 近傍の点で TLS 再フィットし、インライア数と端点を埋める。
  std::optional<WallLine> Refine(const std::vector<Point2D>& points,
                                 const WallLine& seed, double gate) const;
  bool IsAcceptable(const WallLine& line, std::size_t total_points) const;

  FitterConfig config_;
  std::mt19937 rng_;
};

struct StateFilterConfig {
  // 一次遅れ LPF の時定数 [s]。0 なら平滑化なし。
  double angle_time_constant = 0.03;
  double distance_time_constant = 0.05;
  // 予測値からこれ以上飛んだ観測は外れ値として捨てる。
  double max_angle_jump = DegToRad(10.0);
  double max_distance_jump = 0.20;
  // 外れ値が連続でこの回数続いたら、本当に変化したとみなして観測で初期化。
  int max_consecutive_rejects = 5;
  // この時間観測が採用されなければ状態を捨てる [s]。
  double lost_timeout = 0.25;
};

// 壁の (法線角, 距離) の時系列フィルタ。
// オドメトリの差分で予測 -> スキャンで補正、の順に呼ぶ。
class WallStateFilter {
 public:
  explicit WallStateFilter(const StateFilterConfig& config);

  // 前回からロボットが (dx, dy) [前回のロボット座標系] 動き、dyaw 回転した。
  void Predict(double dx, double dy, double dyaw);

  // 観測を取り込む。採用したら true。
  bool Update(const WallLine& measurement, double stamp_sec);

  // stamp_sec の時点で状態が有効か (lost_timeout 以内に採用があるか)。
  bool IsValid(double stamp_sec) const;

  void Reset();

  bool initialized() const { return initialized_; }
  double normal_angle() const { return normal_angle_; }
  double distance() const { return distance_; }
  // 予測込みの現在推定を WallLine として返す (ウォームスタートの事前値用)。
  WallLine line() const;

  void set_config(const StateFilterConfig& config) { config_ = config; }

 private:
  void Initialize(const WallLine& measurement, double stamp_sec);

  StateFilterConfig config_;
  bool initialized_ = false;
  double normal_angle_ = 0.0;
  double distance_ = 0.0;
  double last_update_sec_ = 0.0;
  int consecutive_rejects_ = 0;
};

}  // namespace nr27_wall_detection

#endif  // NR27_WALL_DETECTION_WALL_FITTER_H_
