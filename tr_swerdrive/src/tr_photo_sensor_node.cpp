// 操舵の原点のフォトセンサ (MISUMI C-MSX672N-2M など、NPN・黒線 = 遮光時ON)
// を確かめるノード。
//
//   sub  serial_rx_<device_id>  (Int16MultiArray)  ESP32 (MODE_DAMIAO) の帰還
//   pub  photo_sensor/blocked   (Bool)  true = 遮光中 (原点)
//
// 値の決まりは damiao_slots.hpp の IsPhotoSensorBlocked。
// 遮光・入光が決まっている帰還を受けるたびに publish し、状態が変わったときだけ
// ログに出す。値が決まっていない (センサを読まないファーム、未確定) ときは
// publish しない (入光と取り違えると原点を見落とすため)。
//
// 起動 (ros2can を先に起動):
//   ros2 run tr_dokusute tr_photo_sensor
//   ros2 topic echo /photo_sensor/blocked

#include <memory>
#include <optional>
#include <string>

#include "tr_dokusute/damiao_slots.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"

namespace tr_dokusute {

class PhotoSensorNode : public rclcpp::Node {
 public:
  PhotoSensorNode() : Node("tr_photo_sensor") {
    const std::string topic =
        "serial_rx_" + std::to_string(declare_parameter<int>("device_id", 205));
    blocked_pub_ =
        create_publisher<std_msgs::msg::Bool>("photo_sensor/blocked", 10);
    rx_sub_ = create_subscription<std_msgs::msg::Int16MultiArray>(
        topic, 10,
        [this](const std_msgs::msg::Int16MultiArray& msg) { OnRx(msg); });
    RCLCPP_INFO(get_logger(), "%s の %zu 番を読む", topic.c_str(),
                damiao::kPhotoSensorSlot);
  }

 private:
  void OnRx(const std_msgs::msg::Int16MultiArray& msg) {
    const std::optional<bool> blocked = damiao::IsPhotoSensorBlocked(msg.data);
    if (!received_ || blocked != last_) {
      LogChange(blocked, msg);
      received_ = true;
      last_ = blocked;
    }
    if (!blocked.has_value()) return;

    std_msgs::msg::Bool out;
    out.data = *blocked;
    blocked_pub_->publish(out);
  }

  void LogChange(const std::optional<bool>& blocked,
                 const std_msgs::msg::Int16MultiArray& msg) {
    if (blocked.has_value()) {
      RCLCPP_INFO(get_logger(), "%s", *blocked ? "遮光" : "入光");
      return;
    }
    const int raw = msg.data.size() > damiao::kPhotoSensorSlot
                        ? msg.data[damiao::kPhotoSensorSlot]
                        : -1;
    RCLCPP_WARN(get_logger(),
                "フォトセンサの値が決まっていない (%zu 番 = %d)。"
                "センサを読むファームか確認すること",
                damiao::kPhotoSensorSlot, raw);
  }

  bool received_ = false;  // 帰還を1回でも受けたか
  std::optional<bool> last_;  // 最後に受けた状態 (nullopt = 決まっていない)

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr blocked_pub_;
  rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr rx_sub_;
};

}  // namespace tr_dokusute

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<tr_dokusute::PhotoSensorNode>());
  rclcpp::shutdown();
  return 0;
}
