/*
NR27 BR 独ステ PS4テレオペ (動作確認用の最小構成)

R1を押している間だけ動く。
  左スティック     : 平行移動 (倒した分だけ速くなる)
  右スティック左右 : 旋回

joyを受け取ったときだけ cmd_vel を送る。R1を離すと0を送る。
コントローラやjoy_nodeが止まると送信も止まり、下流のノードが
timeout (0.5秒) で止める。
*/

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>

// ROS
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

// スティックを倒し切ったときの速度 (初回は控えめ)
#define MAX_SPEED 0.5      // [m/s]
#define MAX_YAW_SPEED 1.0  // [rad/s]

// スティックのデッドゾーン
#define DEADZONE_L 0.1
#define DEADZONE_R 0.1

class JoyTeleop : public rclcpp::Node {
 public:
  JoyTeleop() : Node("br_joy_teleop") {
    // joyノードのSubscribe
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "joy", 10,
        std::bind(&JoyTeleop::ps4_listener_callback, this,
                  std::placeholders::_1));

    // br_swerve_kinematicsへpublish
    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    RCLCPP_INFO(get_logger(),
                "br_joy_teleop started. R1を押している間だけ動きます");
  }

 private:
  void ps4_listener_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
    if (msg->axes.size() < 8 || msg->buttons.size() < 13) {
      return;  // コントローラが違うと番号がずれるので何もしない
    }

    // コントローラーの入力を取得、使わない入力はコメントアウト推奨
    // ROSのjoyは 上=+, 左=+ (cmd_velの x前, y左, 反時計回り+ と同じ向き)
    float LS_X = msg->axes[0];
    float LS_Y = msg->axes[1];
    float RS_X = msg->axes[3];
    // float RS_Y = msg->axes[4];

    // bool CROSS = msg->buttons[0];
    // bool CIRCLE = msg->buttons[1];
    // bool TRIANGLE = msg->buttons[2];
    // bool SQUARE = msg->buttons[3];

    // bool LEFT = msg->axes[6] == 1.0;
    // bool RIGHT = msg->axes[6] == -1.0;
    // bool UP = msg->axes[7] == 1.0;
    // bool DOWN = msg->axes[7] == -1.0;

    // bool L1 = msg->buttons[4];
    bool R1 = msg->buttons[5];

    // float L2 = (-1 * msg->axes[2] + 1) / 2;
    // float R2 = (-1 * msg->axes[5] + 1) / 2;

    // bool SHARE = msg->buttons[8];
    // bool OPTION = msg->buttons[9];
    // bool PS = msg->buttons[10];

    // bool L3 = msg->buttons[11];
    // bool R3 = msg->buttons[12];

    // デッドゾーン
    if (std::fabs(LS_X) < DEADZONE_L) LS_X = 0.0;
    if (std::fabs(LS_Y) < DEADZONE_L) LS_Y = 0.0;
    if (std::fabs(RS_X) < DEADZONE_R) RS_X = 0.0;

    // 以降、cmd_velを作る
    geometry_msgs::msg::Twist cmd;
    if (R1) {
      cmd.linear.x = MAX_SPEED * LS_Y;
      cmd.linear.y = MAX_SPEED * LS_X;
      cmd.angular.z = MAX_YAW_SPEED * RS_X;
    }

    cmd_pub_->publish(cmd);

    // デバッグ用
    // RCLCPP_INFO(get_logger(), "R1=%d, cmd_vel=[%.2f, %.2f, %.2f]", R1,
    //             cmd.linear.x, cmd.linear.y, cmd.angular.z);
  }

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  // figletでノード名を表示
  std::string figletout = "figlet BR Joy Teleop";
  int result = std::system(figletout.c_str());
  if (result != 0) {
    std::cerr << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
              << std::endl;
    std::cerr << "Please install 'figlet' with the following command:"
              << std::endl;
    std::cerr << "sudo apt install figlet" << std::endl;
    std::cerr << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
              << std::endl;
  }

  rclcpp::spin(std::make_shared<JoyTeleop>());
  rclcpp::shutdown();
  return 0;
}
