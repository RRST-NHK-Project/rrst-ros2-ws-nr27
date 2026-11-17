// トピックで最後に受け取った値と、その時刻。
// 「一定時間届かなければ止める」判定を、各ノードで書かなくて済むようにする。

#ifndef NR27_BR_HW_CTRL__LATEST_VALUE_HPP_
#define NR27_BR_HW_CTRL__LATEST_VALUE_HPP_

#include "rclcpp/rclcpp.hpp"

namespace nr27_br_hw_ctrl {

template <typename T>
class LatestValue {
 public:
  void Set(const T& value, const rclcpp::Time& stamp) {
    value_ = value;
    stamp_ = stamp;
    has_value_ = true;
  }

  // 一度も受け取っていない、または timeout [s] より古ければ false。
  bool IsFresh(const rclcpp::Time& now, double timeout) const {
    return has_value_ && (now - stamp_).seconds() <= timeout;
  }

  bool has_value() const { return has_value_; }
  const T& value() const { return value_; }
  const rclcpp::Time& stamp() const { return stamp_; }

 private:
  T value_{};
  rclcpp::Time stamp_{0, 0, RCL_ROS_TIME};
  bool has_value_ = false;
};

}  // namespace nr27_br_hw_ctrl

#endif  // NR27_BR_HW_CTRL__LATEST_VALUE_HPP_
