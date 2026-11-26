// nr27 用 壁検出ノード (北陽 UST-30LX-X01 / urg_node2 想定)。
//
// <サブスクライブ>
//   scan  (sensor_msgs/LaserScan)  : urg_node2 のスキャン
//   odom  (nav_msgs/Odometry)      : use_odom_prediction=true のときだけ使用
//
// <パブリッシュ>
//   wall_detection/angle    (std_msgs/Float64) : ヨー誤差 [rad]
//       = wrap(壁法線の向き - target_normal_deg)。
//       ロボットがこの値だけ CCW に回ると、target_normal_deg=0 なら壁に正対、
//       +-90 なら壁と平行になる。ヨー制御の誤差としてそのまま使える。
//   wall_detection/distance (std_msgs/Float64) : base 原点から壁までの垂線距離
//   [m] wall_detection/valid    (std_msgs/Bool)    : 毎スキャン。false の間は
//       angle/distance を出さないので、制御側は valid を見て止めること。
//   (publish_debug=true のとき)
//   wall_detection/filtered_points (sensor_msgs/PointCloud2) : マスク後の点群
//   wall_detection/markers   (visualization_msgs/MarkerArray) :
//       視野の扇形 (黄)・ROI (橙)・候補面 (水色 = 条件 OK / 赤 = 弾いた、
//       理由つき)・今回の観測 (緑 = 採用 / 橙 = 外れ値)・推定の法線 (白矢印)・
//       状態の文字。rviz は launch で一緒に起動する
//   wall_detection/ransac_params
//   (std_msgs/Float64MultiArray)
//       : [a, b, c, inliers, total] (ax + by + c = 0, 旧ノードと同じ並び)
//
// 処理の流れ: 点群化 (取り付け補正・FOV・距離ゲート・ROI)
//   -> オドメトリで前回推定を予測 -> ウォームスタート (前回の面を追従)
//   -> 失敗時は逐次 RANSAC で面を列挙し target 方向の面を選択
//   -> 外れ値ゲート付き一次遅れ LPF -> publish

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <functional>
#include <memory>
#include <optional>
#include <random>
#include <string>
#include <vector>

#include "nav_msgs/msg/odometry.hpp"
#include "nr27_wall_detection/wall_fitter.h"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace nr27_wall_detection {
namespace {

constexpr char kMarkerNs[] = "wall_detection";
constexpr int kFovMarkerId = 0;
constexpr int kRoiMarkerId = 1;
constexpr int kWallMarkerId = 2;
constexpr int kEstimateMarkerId = 3;
constexpr int kStatusMarkerId = 4;
constexpr int kCandidateMarkerIdBase =
    10;  // 候補 k は 10+2k (線), 11+2k (文字)
constexpr int kLogThrottleMs = 1000;

struct Pose2D {
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
};

double YawFromQuaternion(const geometry_msgs::msg::Quaternion& q) {
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

geometry_msgs::msg::Point ToPointMsg(const Point2D& p) {
  geometry_msgs::msg::Point msg;
  msg.x = p.x;
  msg.y = p.y;
  msg.z = 0.0;
  return msg;
}

}  // namespace

class WallDetectionNode : public rclcpp::Node {
 public:
  WallDetectionNode()
      : Node("wall_detection_node"), state_filter_(StateFilterConfig{}) {
    DeclareParameters();
    LoadParameters();

    const int seed = get_parameter("random_seed").as_int();
    fitter_ = std::make_unique<WallFitter>(
        fitter_config_,
        seed != 0 ? static_cast<std::uint32_t>(seed) : std::random_device{}());

    angle_pub_ =
        create_publisher<std_msgs::msg::Float64>("wall_detection/angle", 10);
    distance_pub_ =
        create_publisher<std_msgs::msg::Float64>("wall_detection/distance", 10);
    valid_pub_ =
        create_publisher<std_msgs::msg::Bool>("wall_detection/valid", 10);
    points_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "wall_detection/filtered_points", 10);
    markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "wall_detection/markers", 10);
    params_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
        "wall_detection/ransac_params", 10);

    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        get_parameter("scan_topic").as_string(), rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::LaserScan::ConstSharedPtr msg) {
          OnScan(*msg);
        });
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        get_parameter("odom_topic").as_string(), rclcpp::SensorDataQoS(),
        [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) { OnOdom(*msg); });

    // 実行中に target_normal_deg などを変えられるようにする
    // (FSM から「前の壁に正対」->「横の壁に沿う」の切り替え等)。
    validate_handle_ = add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter>& params) {
          return ValidateParameters(params);
        });
    apply_handle_ = add_post_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter>&) {
          LoadParameters();
          fitter_->set_config(fitter_config_);
          state_filter_.set_config(filter_config_);
          ResetTracking();
          RCLCPP_INFO(get_logger(), "parameters updated: target=%.1f deg",
                      RadToDeg(fitter_config_.target_normal_angle));
        });

    RCLCPP_INFO(get_logger(),
                "wall_detection_node (nr27) started. target=%.1f deg, "
                "fov=+-%.1f deg, odom_prediction=%s",
                RadToDeg(fitter_config_.target_normal_angle),
                RadToDeg(scan_config_.fov_half),
                use_odom_prediction_ ? "on" : "off");
  }

 private:
  void DeclareParameters() {
    declare_parameter<std::string>("scan_topic", "scan");
    declare_parameter<std::string>("odom_topic", "odom");
    // 空なら LaserScan の frame_id を使う。mount_* を設定したら base_link に。
    declare_parameter<std::string>("base_frame", "");
    declare_parameter<int>("random_seed", 0);  // 0 = 毎回ランダム
    declare_parameter<bool>("publish_debug", true);

    // 取り付け (base frame)
    declare_parameter<double>("mount_x", 0.0);
    declare_parameter<double>("mount_y", 0.0);
    declare_parameter<double>("mount_yaw_deg", 0.0);
    declare_parameter<bool>("upside_down", false);

    // 狙う壁と視野
    declare_parameter<double>("target_normal_deg", 0.0);
    declare_parameter<double>("normal_tolerance_deg", 35.0);
    declare_parameter<double>("fov_half_deg", 60.0);
    declare_parameter<double>("range_min", 0.05);
    declare_parameter<double>("range_max", 5.0);
    declare_parameter<bool>("use_roi", false);
    declare_parameter<double>("roi_x_min", -5.0);
    declare_parameter<double>("roi_x_max", 5.0);
    declare_parameter<double>("roi_y_min", -5.0);
    declare_parameter<double>("roi_y_max", 5.0);

    // RANSAC
    declare_parameter<int>("ransac_iterations", 100);
    declare_parameter<double>("inlier_threshold", 0.03);
    declare_parameter<int>("min_inliers", 15);
    declare_parameter<double>("min_inlier_ratio", 0.15);
    declare_parameter<double>("min_segment_length", 0.30);
    declare_parameter<int>("max_candidates", 4);

    // ウォームスタート
    declare_parameter<bool>("warm_start", true);
    declare_parameter<double>("warm_start_gate_scale", 1.5);
    declare_parameter<int>("warm_start_max_frames", 20);

    // 時系列フィルタ
    declare_parameter<bool>("use_odom_prediction", false);
    declare_parameter<double>("angle_time_constant", 0.03);
    declare_parameter<double>("distance_time_constant", 0.05);
    declare_parameter<double>("max_angle_jump_deg", 10.0);
    declare_parameter<double>("max_distance_jump", 0.20);
    declare_parameter<int>("max_consecutive_rejects", 5);
    declare_parameter<double>("lost_timeout", 0.25);
  }

  void LoadParameters() {
    base_frame_ = get_parameter("base_frame").as_string();
    publish_debug_ = get_parameter("publish_debug").as_bool();

    const double target =
        DegToRad(get_parameter("target_normal_deg").as_double());

    scan_config_.mount_x = get_parameter("mount_x").as_double();
    scan_config_.mount_y = get_parameter("mount_y").as_double();
    scan_config_.mount_yaw =
        DegToRad(get_parameter("mount_yaw_deg").as_double());
    scan_config_.upside_down = get_parameter("upside_down").as_bool();
    scan_config_.range_min = get_parameter("range_min").as_double();
    scan_config_.range_max = get_parameter("range_max").as_double();
    scan_config_.fov_center = target;
    scan_config_.fov_half = DegToRad(get_parameter("fov_half_deg").as_double());
    scan_config_.use_roi = get_parameter("use_roi").as_bool();
    scan_config_.roi_x_min = get_parameter("roi_x_min").as_double();
    scan_config_.roi_x_max = get_parameter("roi_x_max").as_double();
    scan_config_.roi_y_min = get_parameter("roi_y_min").as_double();
    scan_config_.roi_y_max = get_parameter("roi_y_max").as_double();

    fitter_config_.ransac_iterations =
        get_parameter("ransac_iterations").as_int();
    fitter_config_.inlier_threshold =
        get_parameter("inlier_threshold").as_double();
    fitter_config_.min_inliers = get_parameter("min_inliers").as_int();
    fitter_config_.min_inlier_ratio =
        get_parameter("min_inlier_ratio").as_double();
    fitter_config_.min_segment_length =
        get_parameter("min_segment_length").as_double();
    fitter_config_.max_candidates = get_parameter("max_candidates").as_int();
    fitter_config_.target_normal_angle = target;
    fitter_config_.normal_tolerance =
        DegToRad(get_parameter("normal_tolerance_deg").as_double());

    warm_start_ = get_parameter("warm_start").as_bool();
    warm_start_gate_scale_ = get_parameter("warm_start_gate_scale").as_double();
    warm_start_max_frames_ = get_parameter("warm_start_max_frames").as_int();

    use_odom_prediction_ = get_parameter("use_odom_prediction").as_bool();
    filter_config_.angle_time_constant =
        get_parameter("angle_time_constant").as_double();
    filter_config_.distance_time_constant =
        get_parameter("distance_time_constant").as_double();
    filter_config_.max_angle_jump =
        DegToRad(get_parameter("max_angle_jump_deg").as_double());
    filter_config_.max_distance_jump =
        get_parameter("max_distance_jump").as_double();
    filter_config_.max_consecutive_rejects =
        get_parameter("max_consecutive_rejects").as_int();
    filter_config_.lost_timeout = get_parameter("lost_timeout").as_double();
  }

  rcl_interfaces::msg::SetParametersResult ValidateParameters(
      const std::vector<rclcpp::Parameter>& params) const {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    for (const rclcpp::Parameter& p : params) {
      const std::string& name = p.get_name();
      const bool is_int =
          p.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER;
      const bool is_double =
          p.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE;
      if (name == "scan_topic" || name == "odom_topic" ||
          name == "random_seed") {
        result.successful = false;
        result.reason = name + " は起動時のみ設定できます";
      } else if ((name == "ransac_iterations" || name == "max_candidates" ||
                  name == "warm_start_max_frames" ||
                  name == "max_consecutive_rejects") &&
                 is_int && p.as_int() < 1) {
        result.successful = false;
        result.reason = name + " は 1 以上";
      } else if (name == "min_inliers" && is_int && p.as_int() < 2) {
        result.successful = false;
        result.reason = "min_inliers は 2 以上";
      } else if ((name == "inlier_threshold" || name == "lost_timeout") &&
                 is_double && p.as_double() <= 0.0) {
        result.successful = false;
        result.reason = name + " は正の値";
      }
      if (!result.successful) break;
    }
    return result;
  }

  void OnOdom(const nav_msgs::msg::Odometry& msg) {
    latest_odom_ = Pose2D{msg.pose.pose.position.x, msg.pose.pose.position.y,
                          YawFromQuaternion(msg.pose.pose.orientation)};
  }

  void ResetTracking() {
    state_filter_.Reset();
    warm_start_frames_ = 0;
  }

  // 前回スキャン時からのオドメトリ差分で推定値を進める。
  void PredictWithOdometry() {
    if (!use_odom_prediction_ || !latest_odom_) return;
    const Pose2D current = *latest_odom_;
    if (odom_at_last_scan_) {
      const Pose2D& prev = *odom_at_last_scan_;
      const double wx = current.x - prev.x;
      const double wy = current.y - prev.y;
      const double c = std::cos(prev.yaw);
      const double s = std::sin(prev.yaw);
      // world の差分を前回のロボット座標系へ。
      state_filter_.Predict(c * wx + s * wy, -s * wx + c * wy,
                            WrapPi(current.yaw - prev.yaw));
    }
    odom_at_last_scan_ = current;
  }

  void OnScan(const sensor_msgs::msg::LaserScan& scan) {
    const double stamp = rclcpp::Time(scan.header.stamp).seconds();
    // rosbag のループ再生などで時刻が戻ったら状態を捨てる。
    if (last_scan_stamp_ && stamp < *last_scan_stamp_) ResetTracking();
    last_scan_stamp_ = stamp;

    if (state_filter_.initialized() && !state_filter_.IsValid(stamp)) {
      ResetTracking();
    }
    PredictWithOdometry();

    const std::vector<Point2D> points =
        ScanToPoints(scan.angle_min, scan.angle_increment, scan.ranges,
                     scan.range_min, scan.range_max, scan_config_);

    std::optional<WallLine> measurement;
    std::vector<WallLine> candidates;
    bool warm_started = false;

    if (warm_start_ && state_filter_.initialized() &&
        warm_start_frames_ < warm_start_max_frames_) {
      measurement = fitter_->TrackWall(points, state_filter_.line(),
                                       warm_start_gate_scale_);
      warm_started = measurement.has_value();
    }
    if (!measurement) {
      measurement = fitter_->FindWall(points, &candidates);
    }
    warm_start_frames_ = warm_started ? warm_start_frames_ + 1 : 0;

    bool accepted = false;
    if (measurement) {
      accepted = state_filter_.Update(*measurement, stamp);
      if (!accepted) {
        RCLCPP_DEBUG(get_logger(), "outlier rejected: normal=%.1f deg d=%.2f m",
                     RadToDeg(measurement->normal_angle),
                     measurement->distance);
      }
    } else {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), kLogThrottleMs,
                           "壁が見つからない (points=%zu)", points.size());
    }

    const bool valid = state_filter_.IsValid(stamp);
    std_msgs::msg::Bool valid_msg;
    valid_msg.data = valid;
    valid_pub_->publish(valid_msg);

    if (valid) {
      std_msgs::msg::Float64 angle_msg;
      angle_msg.data = WrapPi(state_filter_.normal_angle() -
                              fitter_config_.target_normal_angle);
      angle_pub_->publish(angle_msg);

      std_msgs::msg::Float64 distance_msg;
      distance_msg.data = state_filter_.distance();
      distance_pub_->publish(distance_msg);

      RCLCPP_DEBUG(get_logger(), "yaw_error=%.2f deg d=%.3f m (%s)",
                   RadToDeg(angle_msg.data), distance_msg.data,
                   warm_started ? "warm" : "ransac");
    }

    if (publish_debug_) {
      std_msgs::msg::Header header = scan.header;
      if (!base_frame_.empty()) header.frame_id = base_frame_;
      DebugFrame frame;
      frame.points = &points;
      frame.candidates = &candidates;
      frame.measurement = measurement;
      frame.accepted = accepted;
      frame.warm_started = warm_started;
      frame.valid = valid;
      PublishDebug(header, frame);
    }
  }

  // rviz に出す 1 スキャン分の判定の中身。
  struct DebugFrame {
    const std::vector<Point2D>* points = nullptr;
    const std::vector<WallLine>* candidates = nullptr;
    std::optional<WallLine> measurement;  // 今回の観測 (外れ値で捨てたものも)
    bool accepted = false;      // 時系列フィルタが採用したか
    bool warm_started = false;  // 前回の面の追従で見つけたか
    bool valid = false;
  };

  void PublishDebug(const std_msgs::msg::Header& header, const DebugFrame& f) {
    PublishPoints(header, *f.points);

    if (f.measurement && f.accepted) {
      std_msgs::msg::Float64MultiArray params;
      const double a = std::cos(state_filter_.normal_angle());
      const double b = std::sin(state_filter_.normal_angle());
      params.data = {a, b, -state_filter_.distance(),
                     static_cast<double>(f.measurement->inlier_count),
                     static_cast<double>(f.points->size())};
      params_pub_->publish(params);
    }

    // 毎回 DELETEALL してから描き直す (前のスキャンの候補が残らないように)。
    visualization_msgs::msg::MarkerArray markers;
    visualization_msgs::msg::Marker clear;
    clear.header = header;
    clear.ns = kMarkerNs;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(clear);

    markers.markers.push_back(MakeFovMarker(header));
    if (scan_config_.use_roi) markers.markers.push_back(MakeRoiMarker(header));
    AddCandidateMarkers(header, f, &markers);
    if (f.measurement) {
      markers.markers.push_back(MakeMeasurementMarker(header, f));
    }
    if (f.valid) markers.markers.push_back(MakeEstimateArrow(header));
    markers.markers.push_back(MakeStatusText(header, f));
    markers_pub_->publish(markers);
  }

  void PublishPoints(const std_msgs::msg::Header& header,
                     const std::vector<Point2D>& points) {
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header = header;
    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(points.size());
    sensor_msgs::PointCloud2Iterator<float> it_x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> it_y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> it_z(cloud, "z");
    for (const Point2D& p : points) {
      *it_x = static_cast<float>(p.x);
      *it_y = static_cast<float>(p.y);
      *it_z = 0.0f;
      ++it_x;
      ++it_y;
      ++it_z;
    }
    points_pub_->publish(cloud);
  }

  static visualization_msgs::msg::Marker MakeBaseMarker(
      const std_msgs::msg::Header& header, int id, int type, float r, float g,
      float b) {
    visualization_msgs::msg::Marker m;
    m.header = header;
    m.ns = kMarkerNs;
    m.id = id;
    m.type = type;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose.orientation.w = 1.0;
    m.scale.x = 0.02;
    m.color.r = r;
    m.color.g = g;
    m.color.b = b;
    m.color.a = 1.0f;
    return m;
  }

  static visualization_msgs::msg::Marker MakeText(
      const std_msgs::msg::Header& header, int id, const Point2D& at,
      const std::string& text, float r, float g, float b, double height) {
    visualization_msgs::msg::Marker m = MakeBaseMarker(
        header, id, visualization_msgs::msg::Marker::TEXT_VIEW_FACING, r, g, b);
    m.pose.position.x = at.x;
    m.pose.position.y = at.y;
    m.pose.position.z = 0.1;
    m.scale.z = height;
    m.text = text;
    return m;
  }

  // 点を拾う範囲 (fov_center ± fov_half、range_max まで) の扇形。黄色。
  visualization_msgs::msg::Marker MakeFovMarker(
      const std_msgs::msg::Header& header) const {
    visualization_msgs::msg::Marker m = MakeBaseMarker(
        header, kFovMarkerId, visualization_msgs::msg::Marker::LINE_STRIP, 0.9f,
        0.8f, 0.1f);
    m.scale.x = 0.01;
    const ScanFilterConfig& c = scan_config_;
    const double half = std::min(c.fov_half, kPi);
    const Point2D origin{c.mount_x, c.mount_y};
    m.points.push_back(ToPointMsg(origin));
    constexpr int kArcSteps = 24;
    for (int i = 0; i <= kArcSteps; ++i) {
      const double a = c.fov_center - half + 2.0 * half * i / kArcSteps;
      m.points.push_back(ToPointMsg({origin.x + c.range_max * std::cos(a),
                                     origin.y + c.range_max * std::sin(a)}));
    }
    m.points.push_back(ToPointMsg(origin));
    return m;
  }

  visualization_msgs::msg::Marker MakeRoiMarker(
      const std_msgs::msg::Header& header) const {
    visualization_msgs::msg::Marker m = MakeBaseMarker(
        header, kRoiMarkerId, visualization_msgs::msg::Marker::LINE_STRIP, 1.0f,
        0.6f, 0.0f);
    const ScanFilterConfig& c = scan_config_;
    for (const Point2D& p :
         {Point2D{c.roi_x_min, c.roi_y_min}, Point2D{c.roi_x_max, c.roi_y_min},
          Point2D{c.roi_x_max, c.roi_y_max}, Point2D{c.roi_x_min, c.roi_y_max},
          Point2D{c.roi_x_min, c.roi_y_min}}) {
      m.points.push_back(ToPointMsg(p));
    }
    return m;
  }

  // RANSAC の候補の面。条件に合う面は水色、弾いた面は赤で、理由を横に書く。
  // ウォームスタート (前回の面の追従) のスキャンでは候補を探さないので出ない。
  void AddCandidateMarkers(const std_msgs::msg::Header& header,
                           const DebugFrame& f,
                           visualization_msgs::msg::MarkerArray* out) const {
    for (std::size_t k = 0; k < f.candidates->size(); ++k) {
      const WallLine& line = (*f.candidates)[k];
      const std::string reason = fitter_->RejectReason(line, f.points->size());
      const bool ok = reason.empty();
      const float r = ok ? 0.2f : 1.0f;
      const float g = ok ? 0.7f : 0.2f;
      const float b = ok ? 1.0f : 0.2f;
      const int id = kCandidateMarkerIdBase + 2 * static_cast<int>(k);

      visualization_msgs::msg::Marker m = MakeBaseMarker(
          header, id, visualization_msgs::msg::Marker::LINE_STRIP, r, g, b);
      m.scale.x = 0.025;
      m.points.push_back(ToPointMsg(line.endpoint_begin));
      m.points.push_back(ToPointMsg(line.endpoint_end));
      out->markers.push_back(m);

      const double ratio = f.points->empty()
                               ? 0.0
                               : static_cast<double>(line.inlier_count) /
                                     static_cast<double>(f.points->size());
      const double dev = RadToDeg(
          WrapPi(line.normal_angle - fitter_config_.target_normal_angle));
      char buf[160];
      std::snprintf(buf, sizeof(buf),
                    "#%zu %s\nn=%d (%.0f%%) len=%.2fm dev=%+.0fdeg", k,
                    ok ? "OK" : reason.c_str(), line.inlier_count,
                    100.0 * ratio, line.segment_length, dev);
      const Point2D mid{0.5 * (line.endpoint_begin.x + line.endpoint_end.x),
                        0.5 * (line.endpoint_begin.y + line.endpoint_end.y)};
      out->markers.push_back(MakeText(header, id + 1, mid, buf, r, g, b, 0.06));
    }
  }

  // 今回の観測の面。時系列フィルタが採用したら緑、外れ値で捨てたら橙。
  visualization_msgs::msg::Marker MakeMeasurementMarker(
      const std_msgs::msg::Header& header, const DebugFrame& f) const {
    visualization_msgs::msg::Marker m = MakeBaseMarker(
        header, kWallMarkerId, visualization_msgs::msg::Marker::LINE_STRIP,
        f.accepted ? 0.1f : 1.0f, f.accepted ? 1.0f : 0.5f, 0.1f);
    m.scale.x = 0.05;
    m.points.push_back(ToPointMsg(f.measurement->endpoint_begin));
    m.points.push_back(ToPointMsg(f.measurement->endpoint_end));
    return m;
  }

  // 出力している推定 (フィルタ後) の法線: 原点から壁への垂線。白。
  visualization_msgs::msg::Marker MakeEstimateArrow(
      const std_msgs::msg::Header& header) const {
    visualization_msgs::msg::Marker m = MakeBaseMarker(
        header, kEstimateMarkerId, visualization_msgs::msg::Marker::ARROW, 1.0f,
        1.0f, 1.0f);
    m.scale.x = 0.02;  // 軸の太さ
    m.scale.y = 0.05;  // 矢じりの幅
    m.scale.z = 0.08;  // 矢じりの長さ
    const double a = state_filter_.normal_angle();
    const double d = state_filter_.distance();
    m.points.push_back(ToPointMsg({0.0, 0.0}));
    m.points.push_back(ToPointMsg({d * std::cos(a), d * std::sin(a)}));
    return m;
  }

  // 原点の横に出す状態の文字。
  visualization_msgs::msg::Marker MakeStatusText(
      const std_msgs::msg::Header& header, const DebugFrame& f) const {
    char buf[256];
    const char* source = f.warm_started ? "track" : "ransac";
    if (f.valid) {
      std::snprintf(
          buf, sizeof(buf),
          "VALID  target=%+.0fdeg\nangle=%+.2fdeg  dist=%.3fm\n"
          "points=%zu  %s%s",
          RadToDeg(fitter_config_.target_normal_angle),
          RadToDeg(WrapPi(state_filter_.normal_angle() -
                          fitter_config_.target_normal_angle)),
          state_filter_.distance(), f.points->size(), source,
          f.measurement && !f.accepted ? "  (outlier this scan)" : "");
    } else {
      std::snprintf(buf, sizeof(buf),
                    "LOST  target=%+.0fdeg\npoints=%zu  candidates=%zu\n%s",
                    RadToDeg(fitter_config_.target_normal_angle),
                    f.points->size(), f.candidates->size(),
                    f.points->size() <
                            static_cast<std::size_t>(fitter_config_.min_inliers)
                        ? "few points in FOV (check FOV / range / mount)"
                        : "no candidate passed (see red labels)");
    }
    const float g = f.valid ? 1.0f : 0.3f;
    return MakeText(header, kStatusMarkerId, {-0.3, 0.3}, buf, 1.0f, g,
                    f.valid ? 1.0f : 0.3f, 0.08);
  }

  // 設定
  ScanFilterConfig scan_config_;
  FitterConfig fitter_config_;
  StateFilterConfig filter_config_;
  std::string base_frame_;
  bool publish_debug_ = true;
  bool warm_start_ = true;
  double warm_start_gate_scale_ = 1.5;
  int warm_start_max_frames_ = 20;
  bool use_odom_prediction_ = false;

  // 状態
  std::unique_ptr<WallFitter> fitter_;
  WallStateFilter state_filter_;
  int warm_start_frames_ = 0;
  std::optional<double> last_scan_stamp_;
  std::optional<Pose2D> latest_odom_;
  std::optional<Pose2D> odom_at_last_scan_;

  // ROS
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr angle_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr distance_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr valid_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr points_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      markers_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr params_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
      validate_handle_;
  rclcpp::node_interfaces::PostSetParametersCallbackHandle::SharedPtr
      apply_handle_;
};

}  // namespace nr27_wall_detection

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<nr27_wall_detection::WallDetectionNode>());
  rclcpp::shutdown();
  return 0;
}
