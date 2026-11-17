// =====================================================================
// origin_sensor_node : 操舵の原点センサー(MISUMI C-MSX951N-2M-R)の状態を配信するノード
// =====================================================================
// ■ 仕組み
//   センサーの黒線(出力1・遮光時ON) → SOKI基板のENC2_A(GPIO5) → ESP32(MODE_DAMIAO、
//   soki_damiao_201 の damiao.cpp)が serial_rx_201 の23番(最後)に入れる
//   → このノードが読んで br_drive/origin_sensor(Bool)に出す。白線は使わない。
//   23番の値の意味(ESP32のファーム damiao.cpp と同じ決まり):
//     0 = 原点センサーの機能が無いファーム(または未受信) → 配信しない
//     1 = スリットは空(黒がOFF)                 → false を出す
//     2 = スリットに物がある = 原点(黒がON)     → true を出す
//   ※黒線の断線・センサーの電源切れも 1(false)になり、区別できない。
//
// ■ トピック
//   出す: br_drive/origin_sensor (std_msgs/Bool)  true = 原点(スリットに物がある)
//         変わった時すぐ + publish_period_sec ごと。serial_rx が途絶えた時・値がおかしい時は出さない
//
// ■ 使い方(ros2can を先に起動。damiao_driver_node と一緒に動かしてよい)
//   ros2 run br_drive origin_sensor_node
//   ros2 topic echo /br_drive/origin_sensor
//
// ■ パラメータ
//   device_id          : ESP32のDEVICE_ID(既定 201 → serial_rx_201 を読む)
//   slot               : 何番の値を読むか(既定 23。ESP32のDAMIAO_ORIGIN_SENSOR_SLOTと同じにする)
//   publish_period_sec : 変わらない時に出し直す間隔(既定 0.1秒)
//   timeout_sec        : serial_rx がこれ以上来なければ警告する(既定 0.5秒)
// =====================================================================

#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"

using namespace std::chrono_literals;

namespace Infinite_Zakisute
{

class OriginSensorNode : public rclcpp::Node
{
public:
  OriginSensorNode()
  : Node("origin_sensor_node")
  {
    device_id_ = declare_parameter<int>("device_id", 201);
    slot_ = declare_parameter<int>("slot", 23);
    publish_period_sec_ = declare_parameter<double>("publish_period_sec", 0.1);
    timeout_sec_ = declare_parameter<double>("timeout_sec", 0.5);

    pub_ = create_publisher<std_msgs::msg::Bool>("br_drive/origin_sensor", 10);
    const std::string rx_topic = "serial_rx_" + std::to_string(device_id_);
    sub_ = create_subscription<std_msgs::msg::Int16MultiArray>(
      rx_topic, 10,
      std::bind(&OriginSensorNode::on_rx, this, std::placeholders::_1));
    timer_ = create_wall_timer(100ms, std::bind(&OriginSensorNode::on_timer, this));

    RCLCPP_INFO(get_logger(), "%s の %d 番を読んで br_drive/origin_sensor に出します",
      rx_topic.c_str(), slot_);
  }

private:
  void on_rx(const std_msgs::msg::Int16MultiArray::SharedPtr msg)
  {
    last_rx_time_ = now_steady();
    if (slot_ < 0 || static_cast<size_t>(slot_) >= msg->data.size()) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
        "slot=%d がデータの範囲(0〜%zu)の外です", slot_, msg->data.size() - 1);
      return;
    }
    const int16_t raw = msg->data[static_cast<size_t>(slot_)];
    if (raw != 1 && raw != 2) {
      // 0 = 原点センサーを読まないファーム。それ以外 = slot の番号違い
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "%d 番の値が %d です(1 か 2 のはず)。ESP32のファームが原点センサーに対応していない"
        "(DAMIAO_ORIGIN_SENSOR_DISABLE)か、slot の番号が違います", slot_, raw);
      have_value_ = false;
      return;
    }
    const bool on = (raw == 2);
    if (!have_value_ || on != value_) {
      RCLCPP_INFO(get_logger(), "原点センサー: %s", on ? "原点(物あり)" : "空");
      value_ = on;
      have_value_ = true;
      publish();  // 変わった時はすぐ出す
    }
  }

  void on_timer()
  {
    const auto now = now_steady();
    if (last_rx_time_.time_since_epoch().count() != 0 &&
      now - last_rx_time_ > std::chrono::duration<double>(timeout_sec_))
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "serial_rx_%d が %.1f秒以上来ていません(ros2canを確認)", device_id_, timeout_sec_);
      have_value_ = false;  // 古い値は出さない
    }
    if (have_value_ && now - last_pub_time_ >= std::chrono::duration<double>(publish_period_sec_)) {
      publish();
    }
  }

  void publish()
  {
    std_msgs::msg::Bool msg;
    msg.data = value_;
    pub_->publish(msg);
    last_pub_time_ = now_steady();
  }

  static std::chrono::steady_clock::time_point now_steady()
  {
    return std::chrono::steady_clock::now();
  }

  int device_id_{201};
  int slot_{23};
  double publish_period_sec_{0.1};
  double timeout_sec_{0.5};
  bool have_value_{false};
  bool value_{false};
  std::chrono::steady_clock::time_point last_rx_time_{};
  std::chrono::steady_clock::time_point last_pub_time_{};
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_;
  rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace Infinite_Zakisute

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Infinite_Zakisute::OriginSensorNode>());
  rclcpp::shutdown();
  return 0;
}
