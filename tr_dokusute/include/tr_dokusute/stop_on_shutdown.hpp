// ノードの終了時 (Ctrl+C) に、停止指令 (全スロット0) を送る。
//
// rclcpp::spin が返った後や rclcpp::on_shutdown では、通信がもう切れていて
// publish が届かない。通信を切る前に呼ばれる pre-shutdown コールバックで送る
// (damiao_ctrl で2026-10-14に実機確認済み)。
//
// 使い方: メンバに持ち、通常の送信の先頭で is_stopping() を見て、
// true なら何も送らない (停止指令を上書きしないため)。

#ifndef BR_DOKUSUTE__STOP_ON_SHUTDOWN_HPP_
#define BR_DOKUSUTE__STOP_ON_SHUTDOWN_HPP_

#include <atomic>
#include <chrono>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"

namespace br_dokusute {

class StopOnShutdown {
 public:
  using Publisher = rclcpp::Publisher<std_msgs::msg::Int16MultiArray>;

  StopOnShutdown(rclcpp::Node& node, Publisher::SharedPtr publisher,
                 size_t slot_count)
      : context_(node.get_node_base_interface()->get_context()),
        publisher_(publisher),
        slot_count_(slot_count) {
    handle_ = context_->add_pre_shutdown_callback([this]() { Send(); });
  }

  ~StopOnShutdown() { context_->remove_pre_shutdown_callback(handle_); }

  StopOnShutdown(const StopOnShutdown&) = delete;
  StopOnShutdown& operator=(const StopOnShutdown&) = delete;

  bool is_stopping() const { return stopping_; }

 private:
  // シグナル処理のスレッドから呼ばれる。
  void Send() {
    stopping_ = true;
    std_msgs::msg::Int16MultiArray msg;
    msg.data.assign(slot_count_, 0);
    for (int i = 0; i < 3; ++i) {  // 直後にプロセスが終わるので取りこぼし対策
      publisher_->publish(msg);
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
  }

  rclcpp::Context::SharedPtr context_;
  Publisher::SharedPtr publisher_;
  size_t slot_count_;
  rclcpp::PreShutdownCallbackHandle handle_;
  std::atomic<bool> stopping_{false};
};

}  // namespace br_dokusute

#endif  // BR_DOKUSUTE__STOP_ON_SHUTDOWN_HPP_
