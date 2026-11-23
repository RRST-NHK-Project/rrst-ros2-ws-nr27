// 操舵の原点合わせ (ROSに依存しない判定部分)。
//
// 操舵の円板 (直径163mm) に、フォトセンサを遮る円弧のフラグ (64.01mm = 45°)
// が付いている。フラグの「中心」を原点にする。
//
//   1. 起動時にフラグの中にいたら、-向きに回ってフラグから出る
//   2. +向きに回して、フラグに入る端と出る端の位置を記録する
//   3. 折り返して -向きに回し、もう一度入る端と出る端を記録する
//   4. 4つの端の平均をフラグの中心とする
//
// センサの値は遅れて届く (ファームのデバウンス 3x5ms など)ので、
// 端は進む向きに少し行き過ぎた所で見つかる。往復で行き過ぎの向きが逆になるので、
// 平均すると打ち消し合う。
// 測ったフラグの幅が分かっている幅 (45°) から外れたら失敗にする
// (別の物を拾った・センサの誤動作・ギア比の設定違いに気づくため)。
//
// 位置はすべて DM の出力軸の位置 [rad] (帰還の position と同じ)。

#ifndef TR_DOKUSUTE__STEER_HOMING_HPP_
#define TR_DOKUSUTE__STEER_HOMING_HPP_

#include <cmath>
#include <string>

namespace tr_dokusute {

struct SteerHomingConfig {
  double flag_width = M_PI / 4.0;  // フラグの幅 [rad, DM出力軸]
  double width_tolerance = 0.3;    // 幅の許容誤差 (割合)。0.3 = ±30%
  double sweep = 2.0 * M_PI * 1.3;  // フラグを探して +向きに回す最大量 [rad]
  double margin = 0.2;  // 折り返す時に端から余分に戻る量 [rad]
  int direction = 1;    // 最初に回す向き (+1 / -1)
  double reach_tolerance = 0.02;  // 目標に着いたとみなす誤差 [rad]
};

class SteerHoming {
 public:
  enum class Phase { kLeaveFlag, kForward, kBackward, kDone, kFailed };

  // start_position: 始めた時の位置。start_blocked: 始めた時にフラグの中か。
  SteerHoming(const SteerHomingConfig& config, double start_position,
              bool start_blocked)
      : config_(config),
        dir_(config.direction >= 0 ? 1 : -1),
        prev_blocked_(start_blocked) {
    if (start_blocked) {
      phase_ = Phase::kLeaveFlag;
      target_ = start_position - dir_ * (config_.flag_width + config_.margin);
    } else {
      StartForward(start_position);
    }
  }

  // 帰還が届くたびに呼ぶ。position と blocked は同じ帰還の値を使うこと。
  void Update(double position, bool blocked) {
    const bool entered = !prev_blocked_ && blocked;
    const bool exited = prev_blocked_ && !blocked;
    prev_blocked_ = blocked;

    switch (phase_) {
      case Phase::kLeaveFlag:
        if (!blocked) {
          StartForward(position);
        } else if (Reached(position)) {
          Fail("フラグから出られない (センサが遮光のまま)");
        }
        break;
      case Phase::kForward:
        if (entered) {
          forward_enter_ = position;
          has_forward_enter_ = true;
        } else if (exited && has_forward_enter_) {
          forward_exit_ = position;
          if (!CheckWidth(forward_exit_ - forward_enter_)) break;
          phase_ = Phase::kBackward;
          target_ = forward_enter_ - dir_ * config_.margin;
        } else if (Reached(position)) {
          Fail(has_forward_enter_ ? "フラグの出口が見つからない"
                                  : "フラグが見つからない");
        }
        break;
      case Phase::kBackward:
        if (entered) {
          backward_enter_ = position;
          has_backward_enter_ = true;
        } else if (exited && has_backward_enter_) {
          const double backward_exit = position;
          if (!CheckWidth(backward_enter_ - backward_exit)) break;
          center_ = (forward_enter_ + forward_exit_ + backward_enter_ +
                     backward_exit) /
                    4.0;
          measured_width_ = (std::fabs(forward_exit_ - forward_enter_) +
                             std::fabs(backward_enter_ - backward_exit)) /
                            2.0;
          phase_ = Phase::kDone;
          target_ = center_;
        } else if (Reached(position)) {
          Fail("戻る時にフラグが見つからない");
        }
        break;
      case Phase::kDone:
      case Phase::kFailed:
        break;
    }
  }

  Phase phase() const { return phase_; }
  bool done() const { return phase_ == Phase::kDone; }
  bool failed() const { return phase_ == Phase::kFailed; }
  // 今向かうべき位置 [rad]
  double target() const { return target_; }
  // フラグの中心 [rad] (done() の時だけ意味がある)
  double center() const { return center_; }
  // 測ったフラグの幅 [rad] (往復の平均。done() の時だけ)
  double measured_width() const { return measured_width_; }
  const std::string& error() const { return error_; }

 private:
  void StartForward(double position) {
    phase_ = Phase::kForward;
    target_ = position + dir_ * config_.sweep;
  }

  bool Reached(double position) const {
    return std::fabs(position - target_) < config_.reach_tolerance;
  }

  // 進む向きに測った幅 (signed) が、分かっている幅に合っているか。
  bool CheckWidth(double signed_width) {
    const double width = signed_width * dir_;
    const double error = std::fabs(width - config_.flag_width);
    if (error > config_.flag_width * config_.width_tolerance) {
      Fail("フラグの幅が合わない (測定 " + std::to_string(width) +
           " rad、正しくは " + std::to_string(config_.flag_width) + " rad)");
      return false;
    }
    return true;
  }

  void Fail(const std::string& error) {
    phase_ = Phase::kFailed;
    error_ = error;
  }

  SteerHomingConfig config_;
  int dir_;
  Phase phase_ = Phase::kForward;
  double target_ = 0.0;
  bool prev_blocked_;

  bool has_forward_enter_ = false;
  bool has_backward_enter_ = false;
  double forward_enter_ = 0.0;
  double forward_exit_ = 0.0;
  double backward_enter_ = 0.0;
  double center_ = 0.0;
  double measured_width_ = 0.0;
  std::string error_;
};

}  // namespace tr_dokusute

#endif  // TR_DOKUSUTE__STEER_HOMING_HPP_
