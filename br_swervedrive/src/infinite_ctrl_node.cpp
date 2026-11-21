/*
infinite_ctrl_node : BRの独立ステアリング(独ステ)1ユニットをPS4で動かすノード
  モータ1 = 操舵 DM2325(速度制御か角度制御。kSteerMode で切り替える)
  モータ2 = 駆動 DM-S3519(速度制御)
  定数(歯車の比・速さ・加速度・見張りのしきい値など)は include/br_swervedrive/infinite_ctrl_params.hpp

■ 使い方(ros2can を先に起動しておく。damiao_ctrl のPS4ノードは起動しない)
  ros2 launch damiao_ctrl damiao_ctrl.launch.py        ← use_joy:=true は付けない
  ros2 run joy joy_node
  ros2 run br_swervedrive infinite_ctrl_node            ← 操舵だけ: --ros-args -p use_drive:=false
  動かす前に、車輪を浮かせて固定し、緊急停止の方法(R1を離す / Ctrl+C / バッテリーのスイッチ)を確認する。

■ パラメータ
  use_drive         : 駆動も使うか(既定 true。false = 操舵だけ。駆動には脱力だけを送る)
  origin_offset_deg : 原点センサーがONになる所の車輪の向き[°](前 = 0、右 = +90。既定 0)
  drive_coupling    : 操舵と駆動のつながりの打ち消し(下の説明。既定 −1.0。0 = 打ち消さない)

■ 操作
  R1を押している間だけ動く(離すと2台とも脱力)。起動直後とE-STOPの後は、R1を一度離して押し直す。
  左スティック : 倒した方向へ車輪を向ける(上 = 原点の向き、右 = +90°)
  R2           : 駆動の速さ(押し込むほど速い。最大 kDriveMaxRadps)。
                 スティックを倒していない時は、今の車輪の向きのまま駆動する
  Options      : E-STOP(R1を離して押し直すと再開)
  △           : 原点合わせ(R1を押したまま。終わるか失敗するまでスティックは効かない)
  ※起動後、最初にR1を押した時に自動で1回だけ原点合わせをする(途中でR1を離したら、次に押した時にやり直す)

■ 操舵の2つの制御のしかた(kSteerMode)
  SteerMode::kVelocity(今の設定) … ESCの速度制御 + このノードの中で位置を制御(P制御)。
    ・何周でも同じ向きに回れる(全方位に無限旋回)。位置は、±12.5radで一周する帰還の位置をつなぎ合わせて数える。
    ・進む向きから左右に120°(kReverseThresholdRad)を超えた向きを指令されたら、大きく旋回せずに
      車輪を逆転させ、残り(60°未満)だけ旋回する。前進⇔後退は加速度の制限で必ずゆっくり0を通る。
    ・同じ向きに何百rad(モータ)も回すと、ESCの中の位置の数え方が粗くなり、ゆっくり回した時に
      位置の読みが変わらなくなる(実際には回っている)。速度の読みと位置の読みが食い違ったら止める。
      内部の位置が kInternalPosWarnRad を超えたら警告するので、ESCの電源を入れ直す(0に戻る)。
  SteerMode::kAngle … ESCの角度制御。正確だが、動けるのは起動時の向きから約±175°まで。
    「今の向きから±90°以内」へだけ回し、超える時は車輪を逆転させる。

■ 原点合わせ(操舵の原点センサー C-MSX951N-2M-R)
  センサーの値は ESP32(soki_damiao_201)が serial_rx_201 の23番に入れる(1 = 空、2 = 原点。黒線 = ENC2_A)。
  駆動は脱力のまま、操舵だけを回してセンサーの「OFF→ON」の所を探す:
    1. 始めにONなら、−向きに回してOFFになるまで抜ける
    2. +向きに速く(kHomingFastRadps)回して、ONになる所を探す
    3. 見つけた時は、センサーに気づくまでの遅れの分だけ行き過ぎている。見つけた所から kHomingBackoffRad だけ
       −向きに戻って kHomingSettleSec 止まり(遅れて届くセンサーの値を待つ)、まだONならさらに戻る。
       OFFを確かめてから+向きにゆっくり(kHomingSlowRadps)回し直し、ONになった所を原点とする
    4. 角度制御の時: +向きの端まで行っても無ければ、−向きに戻りながら探し、見つけたら 1 からやり直す
       速度制御の時: +向きに車輪 kHomingMaxTurns 周まで回して探す
    5. 見つからない・kHomingTimeoutSec を超えた → 失敗(脱力して原点はそのまま)
  見つけた所の車輪の向きを origin_offset_deg として、スティックの「上」を決め直す
  (原点合わせの前は、起動時の向きが「上」)。
  ※センサーの値が来ていない時は始めない。黒線が断線すると「ずっと空」に見え、見つからずに失敗する。
  原点合わせの後は、センサーの端を通るたびに、数えている位置とのずれを「[原点の通過]」としてログに出す。
  原点の端を通った時だけ、ゆっくり(kOriginCorrectMaxRadps 以下)で、ずれが kOriginCorrectMaxDeg 以下なら
  ずれを直す(「[原点の補正]」)。速い時は、センサーの値の遅れの分だけ先で気づくので直さない
  (遅れを測ったら kOriginDelaySec に入れると、その分を引いて直す)。反対の端は幅を出すだけ。

■ 操舵と駆動のつながりの打ち消し(drive_coupling)
  駆動の軸は操舵の回転軸の中を通っている(DM-S3519 → 上の歯車2枚(1:1) → 縦の軸 → 傘歯車 → 23:33 → 車輪)。
  そのため、駆動のモータが止まっていても、旋回部が回ると車輪が回ってしまう。
  打ち消すには縦の軸を旋回部と同じ速さで回せばよいので、駆動モータに
    「車輪の向きが変わる速さ[rad/s] × drive_coupling」
  を足して送る(大きさは上の歯車が1:1なので 1.0)。
  向きの決め方: 車輪を浮かせて drive_coupling:=1.0 で操舵だけ回し、車輪がほぼ止まっていれば +1.0、
  0 の時より大きく回るなら −1.0(このユニットは −1.0)。
  操舵が止まって kCouplingHoldSec たつと駆動は脱力に戻る。原点合わせの間は打ち消さない。
  ドライバは「R2の分 + 打ち消しの分」の合計を kDriverLimitRadps で切るので、合計が上限を超えそうな時は
  R2の分を下げて打ち消しを優先する(速く走りながら操舵を回す時だけ、少し遅くなる)。

■ 安全のための決まり
  ・駆動は「R2で回す時」と「打ち消す時」だけ有効にし、それ以外(止まっている時、原点合わせの時)は脱力にする。
  ・固まりの見張り: 駆動が有効な間、トルクが kStallTorqueNm 以上なのに速度が kStallVelRadps 未満のまま
    kStallHoldSec 続いたら、2台とも脱力してノードを終える。
  ・暴走の見張り: 駆動が有効な間に実速度が指令と逆向きで kRunawayDiffRadps 以上ずれる、または脱力中も含めて
    上限 + kRunawayOverRadps を超えるのが kRunawayHoldSec 続いたら、2台とも脱力してノードを終える。
  ・駆動の速さと加速度を制限して、前進⇔後退を素早く切り替えない。
  ・操舵の向きが目標から kAlignTolRad 以上ずれている間は、駆動を回さない。
  ・速度制御の時: 位置の数え間違い(1回で kPosJumpRad 以上の変化)・位置の読みの止まりで、2台とも脱力して終える。
  ・始める前に、帰還が0.5秒続けて届いていること(角度制御の時は、操舵の内部の位置が±12rad以内)を確かめる。
  ・指令は50回/秒で送り続ける(0.5秒届かないと damiao_driver_node が脱力させる)。
  ・PS4: joyが0.5秒途絶えたら止める。/joy の送り手が2つ以上いたら入力を無視する。
    R2は、一度「離した状態」を受け取るまで0として扱う(起動直後の誤った値の対策)。
  ・故障・返事の途絶(状態15を含む)で、2台とも脱力して終える。終わる時・Ctrl+Cの時は、必ず脱力を送る。

Copyright (c) 2026 RRST-NHK-Project
SPDX-License-Identifier: MIT
*/

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

// ROS
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/int16_multi_array.hpp>

// 自作
#include "br_swervedrive/infinite_ctrl_params.hpp" // 定数(歯車の比・速さ・見張りのしきい値など)
#include "br_swervedrive/zakinema.h"

// 以下マイコンに合わせて設定
// RX_DEVICE_ID は原点センサーの値を送ってくるESP32(soki_damiao_201)のシリアルフレームDEVICE_ID
// (ros2can の config.hpp の DEVICE_ID)。damiao_driver_node の device_id と同じ。
#define RX_DEVICE_ID 201 // 受信先ESP32のDEVICE_ID(serial_rx_201 の23番に原点センサーの値が入る)

// damiao_driver_node のモータの番号(damiao/motorN/cmd・state のN)
#define STEER_MOTOR_NUMBER 1 // 操舵 DM2325
#define DRIVE_MOTOR_NUMBER 2 // 駆動 DM-S3519

#define PUBLISH_RATE_MS 20 // publish周期(ms)。damiao_driver_node は0.5秒指令が来ないと脱力させる

// スティックのデッドゾーン
#define DEADZONE_L 0.5 // 左スティックをこれ以上倒したら車輪の向きを変える

using namespace std::chrono_literals;

namespace Infinite_Zakisute {

constexpr double kPeriodSec = PUBLISH_RATE_MS / 1000.0; // 指令を送る周期[s]

// ---------------------------------------------------------------------
// モータ1台分の「リモコン」。damiao/motorN/cmd に指令を送り、
// damiao/motorN/state から今の状態を受け取る。
// ---------------------------------------------------------------------
class MotorHandle {
public:
    MotorHandle(rclcpp::Node &node, int number)
        : number_(number) {
        const std::string prefix = "damiao/motor" + std::to_string(number);
        cmd_pub_ = node.create_publisher<std_msgs::msg::Float32MultiArray>(prefix + "/cmd", 10);
        state_sub_ = node.create_subscription<std_msgs::msg::Float32MultiArray>(
            prefix + "/state", 10,
            [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
                if (msg->data.size() >= 6) {
                    const auto t = std::chrono::steady_clock::now();
                    const bool ok = msg->data[4] > 0.5f;
                    // 始める前(locked_ でない間)は、帰還が無い・途切れた後の値でつなぎ合わせない。
                    // ノードを起動してからESCの24Vを入れると、帰還が無い間の値(0など)から、電源が入った
                    // ESCの本当の位置へ一気に飛ぶため、ここでつなぎ合わせをやり直す。
                    if (!locked_ &&
                        (!ok || (have_pos_ && t - last_state_time_ > std::chrono::milliseconds(100)))) {
                        have_pos_ = false;
                        ok_since_ = t;
                    }
                    if (ok && !feedback_ok_) {
                        ok_since_ = t; // 帰還が来始めた時刻
                    }
                    position_ = msg->data[0];
                    // 位置をつなぎ合わせる(帰還の位置は±kPosWrapRadで一周する)
                    if (!ok && !locked_) {
                        // 帰還が無い時の値は使わない(次に帰還が来た時に、そこからつなぎ合わせる)
                    } else if (!have_pos_) {
                        unwrapped_ = position_;
                        have_pos_ = true;
                    } else {
                        double d = position_ - last_raw_pos_;
                        if (d > kPosWrapRad) {
                            d -= 2.0 * kPosWrapRad;
                        } else if (d < -kPosWrapRad) {
                            d += 2.0 * kPosWrapRad;
                        }
                        if (std::abs(d) > kPosJumpRad) {
                            pos_jump_ = true; // 帰還が抜けた・数え間違い
                        }
                        unwrapped_ += d;
                        // 速度の読みを足し合わせる(位置の読みと比べて、読みが止まっていないかを見る)
                        const double dt = std::chrono::duration<double>(t - last_state_time_).count();
                        vel_integral_ += msg->data[1] * std::clamp(dt, 0.0, 0.1);
                    }
                    last_raw_pos_ = position_;
                    velocity_ = msg->data[1];
                    torque_ = msg->data[2];
                    state_ = static_cast<int>(msg->data[3]);
                    feedback_ok_ = ok;
                    internal_pos_ = msg->data[5];
                    last_state_time_ = t;
                }
            });
    }

    // ---- 指令を決める(実際に送るのは send()) ----
    void set_velocity(double radps) { cmd_ = {0.0f, static_cast<float>(radps), 0.0f}; }
    void set_angle(double rad, double speed_radps) {
        cmd_ = {1.0f, static_cast<float>(rad), static_cast<float>(speed_radps)};
    }
    void set_disable() { cmd_ = {-1.0f, 0.0f, 0.0f}; }

    // 今の指令を送る。タイマーから50回/秒で呼ぶ。
    void send() {
        std_msgs::msg::Float32MultiArray msg;
        msg.data = {cmd_[0], cmd_[1], cmd_[2]};
        cmd_pub_->publish(msg);
    }

    // ---- 今の状態 ----
    double position() const { return position_; }            // 位置[rad](±12.5radで一周)
    double position_unwrapped() const { return unwrapped_; } // つなぎ合わせた位置[rad](何周でも)
    bool pos_jump() const { return pos_jump_; }              // 位置の数え間違いがあったか
    double vel_integral() const { return vel_integral_; }    // 速度の読みを足し合わせたもの[rad]
    double internal_pos() const { return internal_pos_; }    // モータ内部の位置[rad](不明ならNaN)
    double velocity() const { return velocity_; }            // 速度[rad/s]
    double torque() const { return torque_; }                // トルク[N・m]
    int state() const { return state_; }                     // 0=脱力 1=有効 それ以外=故障(15=止めたまま)
    // モータから返事が届いているか(0.3秒以内に届いていて、帰還OK)
    bool ready() const {
        const auto age = std::chrono::steady_clock::now() - last_state_time_;
        return feedback_ok_ && age < 300ms;
    }
    // 帰還が0.5秒以上続けて届いているか(始める前の確認用。24Vを入れた直後の値を避ける)
    bool settled() const {
        return ready() && have_pos_ &&
               std::chrono::steady_clock::now() - ok_since_ >= std::chrono::milliseconds(500);
    }
    // 始める時に呼ぶ。ここまでの数え間違いの印を消し、以後はつなぎ合わせをやり直さない
    // (始めた後の帰還の抜けは、数え間違い・返事の途絶として止める)
    void lock_position() {
        locked_ = true;
        pos_jump_ = false;
    }
    // 角度制御を使ってよいか(モータ内部の位置が分かっていて±12rad以内)
    bool angle_ok() const { return !std::isnan(internal_pos_) && std::abs(internal_pos_) <= 12.0; }
    int number() const { return number_; }
    // 最後に帰還を受け取ってからの時間[s]
    double state_age() const {
        return std::chrono::duration<double>(std::chrono::steady_clock::now() - last_state_time_).count();
    }

private:
    int number_;
    std::array<float, 3> cmd_{-1.0f, 0.0f, 0.0f}; // 最初は脱力
    double position_{0.0};
    double velocity_{0.0};
    double torque_{0.0};
    int state_{0};
    bool feedback_ok_{false};
    double internal_pos_{std::nan("")};
    bool have_pos_{false};
    double last_raw_pos_{0.0};
    double unwrapped_{0.0};
    bool pos_jump_{false};
    bool locked_{false};                               // 始めた後か(true ならつなぎ合わせをやり直さない)
    std::chrono::steady_clock::time_point ok_since_{}; // 帰還が来始めた時刻
    double vel_integral_{0.0};
    std::chrono::steady_clock::time_point last_state_time_{};
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr cmd_pub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr state_sub_;
};

// ---------------------------------------------------------------------
// ノード本体
// ---------------------------------------------------------------------
class SwerveDriveControl : public rclcpp::Node {
public:
    SwerveDriveControl()
        : Node("infinite_ctrl_node"), steer_(*this, STEER_MOTOR_NUMBER), drive_(*this, DRIVE_MOTOR_NUMBER) {
        // true(既定) = 駆動も使う。操舵だけにする時は -p use_drive:=false を付ける
        use_drive_ = declare_parameter<bool>("use_drive", true);
        if (use_drive_) {
            RCLCPP_WARN(get_logger(), "駆動(DM3519)も使います。車輪を浮かせ、すぐ止められるようにしてください");
        }
        origin_offset_rad_ = declare_parameter<double>("origin_offset_deg", 0.0) * M_PI / 180.0;
        drive_coupling_ = declare_parameter<double>("drive_coupling", -1.0);
        if (use_drive_ && drive_coupling_ != 0.0) {
            RCLCPP_WARN(get_logger(), "操舵と駆動のつながりを打ち消します(drive_coupling=%+.2f)。"
                                      "操舵だけ回している時も駆動に力が出ます",
                        drive_coupling_);
        }
        // joyノードのSubscribe
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10,
            std::bind(&SwerveDriveControl::ps4_listener_callback, this, std::placeholders::_1));

        // damiao_driver_nodeへのpublish・からのSubscribeは MotorHandle の中(damiao/motorN/cmd・state)

        // timer_callbackを呼び出すタイマーを作成(50回/秒)
        timer_ = create_wall_timer(
            std::chrono::milliseconds(PUBLISH_RATE_MS),
            std::bind(&SwerveDriveControl::publisher_timer_callback, this));

        // ros2canからSubscribe(原点センサー。ESP32が serial_rx_201 の23番に入れる)
        origin_sub_ = this->create_subscription<std_msgs::msg::Int16MultiArray>(
            "serial_rx_" + std::to_string(RX_DEVICE_ID), 10,
            std::bind(&SwerveDriveControl::sensor_callback, this, std::placeholders::_1));

        // Ctrl+Cで止めた時も、通信が切れる前に必ず脱力を送る
        shutdown_cb_ = get_node_base_interface()->get_context()->add_pre_shutdown_callback(
            [this]() {
                std::lock_guard<std::recursive_mutex> lock(mutex_);
                finished_ = true;
                for (int i = 0; i < 3; ++i) {
                    steer_.set_disable();
                    drive_.set_disable();
                    steer_.send();
                    drive_.send();
                    std::this_thread::sleep_for(20ms);
                }
            });
        created_ = std::chrono::steady_clock::now();
        RCLCPP_INFO(get_logger(), "infinite_ctrl_node を開始します。モータから返事が届くのを待っています…");
    }

    ~SwerveDriveControl() override {
        get_node_base_interface()->get_context()->remove_pre_shutdown_callback(shutdown_cb_);
    }

private:
    // ===================================================================
    // PS4の入力を受け取る(ここでは値をしまうだけ。指令は publisher_timer_callback() で送る)
    // ===================================================================
    void ps4_listener_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        if (msg->axes.size() < 6 || msg->buttons.size() < 10) {
            return; // 想定と違うコントローラ
        }

        // コントローラーの入力を取得、使わない入力はコメントアウト推奨
        float LS_X = -1 * msg->axes[0]; // 右に倒すと+
        float LS_Y = msg->axes[1];      // 上に倒すと+
        // float RS_X = -1 * msg->axes[3];
        // float RS_Y = msg->axes[4];

        // bool CROSS = msg->buttons[0];
        // bool CIRCLE = msg->buttons[1];
        bool TRIANGLE = msg->buttons[2]; // 原点合わせ
        // bool SQUARE = msg->buttons[3];

        // bool LEFT = msg->axes[6] == 1.0;
        // bool RIGHT = msg->axes[6] == -1.0;
        // bool UP = msg->axes[7] == 1.0;
        // bool DOWN = msg->axes[7] == -1.0;

        // bool L1 = msg->buttons[4];
        bool R1 = msg->buttons[5]; // 押している間だけ動く

        // float L2_DIGITAL = (-1 * msg->axes[2] + 1) / 2;
        float R2_DIGITAL = (-1 * msg->axes[5] + 1) / 2; // 駆動の速さ(0〜1)

        // bool L2 = msg->buttons[6];
        // bool R2 = msg->buttons[7];

        // bool SHARE = msg->buttons[8];
        bool OPTION = msg->buttons[9]; // E-STOP
        // bool PS = msg->buttons[10];

        // bool L3 = msg->buttons[11];
        // bool R3 = msg->buttons[12];

        // 以降、入力を使った処理(ここでは値をしまうだけ。指令は publisher_timer_callback() で送る)
        ls_x_ = LS_X;
        ls_y_ = LS_Y;
        // R2は離すと+1、押し込むと-1。起動直後は0のまま届くことがあり、そのままだと
        // 半分押した扱いになるので、一度「離した(+1)」を受け取るまでは0として扱う。
        if (msg->axes[5] > 0.99f) {
            r2_seen_released_ = true;
        }
        r2_ = r2_seen_released_ ? R2_DIGITAL : 0.0;
        if (TRIANGLE && !triangle_prev_) {
            homing_requested_ = true; // 押した瞬間だけ。実際に始めるのは publisher_timer_callback()
        }
        triangle_prev_ = TRIANGLE;
        // R1を離したら、次に押した時に動けるようにする(E-STOPもここで解除)
        if (!R1) {
            armed_ = true;
            estop_ = false;
        }
        if (OPTION) {
            if (!estop_) {
                RCLCPP_WARN(get_logger(), "E-STOP(R1を離して押し直すと再開)");
            }
            estop_ = true;
            armed_ = false;
        }
        r1_ = R1;
        last_joy_time_ = std::chrono::steady_clock::now();
        joy_received_ = true;

        // 入力の処理ここまで
    }

    // ===================================================================
    // 原点センサーの値を受け取る(serial_rx_201 の23番。1 = 空、2 = 原点)
    // ===================================================================
    void sensor_callback(const std_msgs::msg::Int16MultiArray::SharedPtr msg) {
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        if (msg->data.size() <= kOriginSlot) {
            return;
        }
        // 以降、受信データを使った処理を記述
        const int16_t raw = msg->data[kOriginSlot];
        if (raw != kOriginEmpty && raw != kOriginDetected) {
            // 0 = 原点センサーを読まないファーム
            origin_valid_ = false;
            return;
        }
        const auto now = std::chrono::steady_clock::now();
        const bool on = (raw == kOriginDetected);
        if (origin_ok(now) && on != origin_on_) {
            origin_pass_log(on); // 切り替わった(センサーの端を通った)
        }
        origin_on_ = on;
        origin_valid_ = true;
        last_origin_time_ = now;
        // 受信データ処理ここまで
    }

    // ===================================================================
    // 原点センサーの端を通るたびに、数えている位置とのずれをログに出す。
    //   原点(A) = 原点合わせで「+向きに回してONになった所」。+向きの OFF→ON と −向きの ON→OFF は A。
    //     A を通った時だけ、ずれを直す(ゆっくり通った時・ずれが小さい時だけ。下の条件)。
    //   反対の端(B) = A + センサーの幅。+向きの ON→OFF と −向きの OFF→ON は B。
    //     幅はセンサーの形で変わる(設計の改良中)ので、B は幅をログに出すだけで、直すのには使わない。
    //   センサーの値は遅れて届く(ファームの確定約15ms + 通信 + 位置は20msごと)ので、回っている時は
    //   回った向きに少し先で気づく。「遅れだとすると」は ずれ ÷ 速さ。速さを変えても同じ値なら、ずれは遅れだけ。
    //   位置は、最後に届いた帰還から、その後の時間 × 速さだけ進めて見積もる。
    // ===================================================================
    void origin_pass_log(bool on) {
        if (!started_ || !homed_ || homing_ != Homing::kIdle) {
            return;
        }
        const double wheel_radps = kSteerSign * steer_.velocity() / kSteerGear; // 車輪の向きの速さ(+向きが+)
        if (std::abs(wheel_radps) < kOriginPassMinRadps) {
            return; // ほぼ止まっている(端の上で揺れている)時は出さない
        }
        const bool plus = wheel_radps > 0.0;
        const double per_turn = 2.0 * M_PI * kSteerGear; // 車輪1周分の操舵モータの位置[rad]
        // 位置の帰還は20msごとなので、最後に届いてからの分を速さで足して、端を通った時の位置にする
        const double pos_now = steer_pos() + steer_.velocity() * std::clamp(steer_.state_age(), 0.0, 0.05);
        const double from_origin = pos_now - steer_origin_pos_;  // 原点からの操舵モータの位置[rad]
        const double turns = std::round(from_origin / per_turn); // 原点から何周目か
        const double deg_per_motor = kSteerSign / kSteerGear * 180.0 / M_PI;
        const double speed_dps = wheel_radps * 180.0 / M_PI;
        if (on == plus) {
            // A: 原点の端。ずれ = 一番近い「原点 + 整数周」からの差
            const double err_motor = std::remainder(from_origin, per_turn);
            const double err_deg = err_motor * deg_per_motor;
            // 遅れの分を差し引いた、本当のずれ。原点合わせの時も kHomingSlowRadps で回って遅れて気づいているので、
            // 速さの差 × 遅れ だけを引く(kOriginDelaySec が0なら、引かない)
            const double delay_motor =
                (steer_.velocity() - kSteerSign * kHomingSlowRadps * kSteerGear) * kOriginDelaySec;
            const double delta_motor = err_motor - delay_motor;
            const double delta_deg = delta_motor * deg_per_motor;
            const bool slow = std::abs(wheel_radps) <= kOriginCorrectMaxRadps;
            const bool small = std::abs(delta_deg) <= kOriginCorrectMaxDeg;
            RCLCPP_INFO(get_logger(), "[原点の通過] %s(%s向き・原点の端) %+.0f周目 ずれ%+6.2f°(遅れを引いて%+6.2f°) "
                                      "速さ%+6.0f°/s(遅れだとすると%+5.0fms) 内部の位置%+.1frad%s",
                        on ? "OFF→ON" : "ON→OFF", plus ? "+" : "−", turns, err_deg, delta_deg, speed_dps,
                        err_deg / speed_dps * 1000.0, steer_.internal_pos(),
                        !slow ? "(速いので直さない)" : "");
            if (slow && small) {
                // 原点と「スティックの上」を同じだけずらす。スティックを倒している時は、次の周期で
                // 正しい向きへ向け直す(ずれは kOriginCorrectMaxDeg 以下なので、小さく動くだけ)
                steer_origin_pos_ += delta_motor;
                steer_zero_ += delta_motor;
                RCLCPP_INFO(get_logger(), "[原点の補正] ずれ%+.2f°を直しました", delta_deg);
            } else if (slow) {
                RCLCPP_WARN(get_logger(), "[原点の補正] ずれ%+.2f°が大きすぎる(±%.1f°を超えた)ので直しません。"
                                          "位置の数え間違いの恐れがあります。△で原点合わせをやり直してください",
                            delta_deg, kOriginCorrectMaxDeg);
            }
        } else {
            // B: 反対の端。原点から+向きに測った角度 = センサーの幅(遅れの分だけ、+向きでは大きく、−向きでは小さく出る)
            double width_deg = std::fmod(from_origin * deg_per_motor, 360.0);
            if (width_deg < 0.0) {
                width_deg += 360.0;
            }
            RCLCPP_INFO(get_logger(), "[原点の通過] %s(%s向き・反対の端) 原点からの幅%6.2f° 速さ%+6.0f°/s 内部の位置%+.1frad",
                        on ? "OFF→ON" : "ON→OFF", plus ? "+" : "−", width_deg, speed_dps, steer_.internal_pos());
        }
    }

    // 原点センサーの値が使えるか(新しい値が届いているか)
    bool origin_ok(std::chrono::steady_clock::time_point now) const {
        return origin_valid_ &&
               now - last_origin_time_ < std::chrono::duration<double>(kOriginTimeoutSec);
    }

    // ===================================================================
    // 原点合わせを1歩進める(R1を押している間、50回/秒)。駆動は脱力のまま。
    // 操舵の目標を少しずつ動かし、センサーの OFF→ON(+向き)の所を探す。
    // ===================================================================
    enum class Homing { kIdle,
                        kBackoff,
                        kSeek,
                        kReturn };

    void homing_step(std::chrono::steady_clock::time_point now) {
        drive_cmd_ = 0.0;
        drive_out_ = 0.0;
        coupling_cmd_ = 0.0;
        drive_enabled_ = false;
        drive_.set_disable();

        if (!origin_ok(now)) {
            homing_fail("原点センサーの値が届かなくなりました(ros2can・ESP32のファームを確認)");
            return;
        }
        if (now - homing_started_ > std::chrono::duration<double>(kHomingTimeoutSec)) {
            homing_fail("時間がかかりすぎました");
            return;
        }

        // 目標を動かす向き(+1 / −1)。1回(20ms)で動かす量はモータの値で
        // 車輪の向きの速さ。原点を決める「ゆっくり合わせ直し」の時だけ遅くする
        const double speed =
            (homing_ == Homing::kSeek && homing_fine_) ? kHomingSlowRadps : kHomingFastRadps;
        const double step = speed * kSteerGear * kPeriodSec;
        const double move = kSteerSign * speed * kSteerGear; // 回す時の速さ(モータ、+向き)
        double ff = 0.0;                                     // 速度制御の時に足す速度(止まって待つ時は0)
        switch (homing_) {
        case Homing::kBackoff: // −向きに戻る
            if (backoff_to_target_) {
                // 見つけた後: 決めた所まで戻ったら止まり、センサーの値が追いつくまで待つ
                // (遅れて届く値は、行き過ぎた先の「OFF」のことがあるので、動いている間は信じない)
                if (kSteerSign * (steer_pos() - backoff_target_) > 0.0) {
                    steer_target_ -= kSteerSign * step;
                    ff = -move;
                    settle_started_ = false;
                    break;
                }
                if (!settle_started_) {
                    settle_started_ = true;
                    settle_since_ = now;
                }
                if (now - settle_since_ < std::chrono::duration<double>(kHomingSettleSec)) {
                    break; // 止まって待つ
                }
                backoff_to_target_ = false; // 待った後の値で決める(ONならさらに戻る)
                settle_started_ = false;
            }
            if (!origin_on_) {
                RCLCPP_INFO(get_logger(), "原点合わせ: センサーから抜けました。+向きに探します");
                homing_ = Homing::kSeek;
            } else {
                steer_target_ -= kSteerSign * step;
                ff = -move;
            }
            break;
        case Homing::kSeek: // +向きに回して、ONになる所を探す
            if (origin_on_) {
                if (homing_fine_) {
                    homing_done(steer_pos());
                    return;
                }
                // 速く回していたので少し行き過ぎている → ゆっくり抜けてから合わせ直す
                RCLCPP_INFO(get_logger(), "原点合わせ: 見つけました。ゆっくり合わせ直します");
                homing_fine_ = true;
                homing_ = Homing::kBackoff;
                steer_target_ = steer_pos();
                backoff_target_ = steer_pos() - kSteerSign * kSteerGear * kHomingBackoffRad;
                backoff_to_target_ = true;
                break;
            }
            steer_target_ += kSteerSign * step;
            ff = move;
            if (!kSteerVel && std::abs(steer_target_) > kSteerLimitRad) {
                steer_target_ = std::copysign(kSteerLimitRad, steer_target_); // 端で止めて折り返す
                RCLCPP_INFO(get_logger(), "原点合わせ: +向きの端まで来ました。−向きに戻りながら探します");
                homing_ = Homing::kReturn;
            }
            break;
        case Homing::kReturn: // −向きに戻りながら探す。見つけたら抜けてから+向きでやり直す
            if (origin_on_) {
                RCLCPP_INFO(get_logger(), "原点合わせ: センサーが反応しました。抜けてから+向きで合わせ直します");
                homing_ = Homing::kBackoff;
            } else {
                steer_target_ -= kSteerSign * step;
                ff = -move;
            }
            break;
        case Homing::kIdle:
            return;
        }
        if (kSteerVel) {
            // 速度制御: 端は無いので、回した量で決める
            if (std::abs(steer_pos() - homing_start_pos_) > kHomingMaxTurns * 2.0 * M_PI * kSteerGear) {
                homing_fail("車輪を1周以上回しても、原点センサーが見つかりませんでした"
                            "(センサーの位置・黒線・電源を確認)");
                return;
            }
        } else if (std::abs(steer_target_) > kSteerLimitRad) {
            // 角度制御: −向きの端より先へは行かない
            homing_fail("操舵の動ける範囲の端まで回しても、原点センサーが見つかりませんでした"
                        "(センサーの位置・黒線・電源を確認)");
            return;
        }
        // 目標は今の位置から離れすぎないようにする(モータが追いつかない時に先へ行かない)
        const double max_lead = 5.0 * step;
        if (steer_target_ > steer_pos() + max_lead) {
            steer_target_ = steer_pos() + max_lead;
        } else if (steer_target_ < steer_pos() - max_lead) {
            steer_target_ = steer_pos() - max_lead;
        }
        steer_output(2.0 * speed * kSteerGear, ff);
    }

    void homing_done(double edge_motor_pos) {
        // 見つけた所の車輪の向きが origin_offset になるように、スティックの「上」を決め直す
        steer_zero_ = edge_motor_pos - kSteerSign * kSteerGear * origin_offset_rad_;
        steer_origin_pos_ = edge_motor_pos; // 通るたびのずれの確認に使う
        steer_target_ = edge_motor_pos;     // その場で止まる
        steer_reverse_ = false;
        homing_ = Homing::kIdle;
        homed_ = true;
        auto_homing_pending_ = false;
        RCLCPP_INFO(get_logger(), "原点合わせができました(センサーの位置 モータ%+.3frad = 車輪%+.1f°)。"
                                  "スティックの「上」を決め直しました",
                    edge_motor_pos, origin_offset_rad_ * 180.0 / M_PI);
    }

    void homing_fail(const char *reason) {
        RCLCPP_ERROR(get_logger(), "原点合わせをやめました: %s。原点は前のままです", reason);
        homing_ = Homing::kIdle;
        auto_homing_pending_ = false; // 失敗したら自動ではやり直さない(△で手動ならできる)
        steer_target_ = steer_pos();  // その場で止まる
        // R1を押し直すまで動かさない(何が起きたか人が確かめてから)
        armed_ = false;
    }

    // 操舵モータの今の位置[rad]。速度制御の時はつなぎ合わせた位置(何周でも)
    double steer_pos() const {
        return kSteerVel ? steer_.position_unwrapped() : steer_.position();
    }

    // 操舵に指令を出す(目標 steer_target_ へ向かう)。max_radps = 速さの上限(モータ)。
    // ff_radps = 速度制御の時に足す速度(原点合わせで一定の速さで回す時)。
    void steer_output(double max_radps, double ff_radps = 0.0) {
        if (!kSteerVel) {
            steer_.set_angle(steer_target_, max_radps);
            return;
        }
        const double err = steer_target_ - steer_pos();
        double v = ff_radps + kSteerKp * err;
        if (ff_radps == 0.0 && std::abs(err) < kSteerHoldTolRad) {
            v = 0.0;
        }
        steer_vel_cmd_ = std::clamp(v, -max_radps, max_radps);
        steer_.set_velocity(steer_vel_cmd_);
    }

    // 車輪の向き[rad](起動時=0) → 操舵モータの位置[rad]
    double wheel_to_motor(double wheel_angle) const {
        return steer_zero_ + kSteerSign * kSteerGear * wheel_angle;
    }
    // 操舵モータの位置[rad] → 車輪の向き[rad](起動時=0)
    double motor_to_wheel(double motor_pos) const {
        return kSteerSign * (motor_pos - steer_zero_) / kSteerGear;
    }

    // ===================================================================
    // 操舵の目標を決める。heading = 起動時の向きから見た、車輪を向けたい方向[rad]。
    // 「heading + k×半回転」の中から、今の目標に一番近く、操舵モータの位置が
    // ±kSteerLimitRad に収まるものを選ぶ(kが奇数なら車輪を逆転させる)。
    // target は操舵モータの位置[rad]で返す。見つからなければ false。
    // ===================================================================
    bool choose_steer(double heading, double &target, bool &reverse) const {
        double best_cost = std::numeric_limits<double>::infinity();
        bool found = false;
        for (int k = -8; k <= 8; ++k) {
            const double cand = wheel_to_motor(heading + k * M_PI);
            if (std::abs(cand) > kSteerLimitRad) {
                continue;
            }
            const bool rev = (k % 2) != 0;
            // 車輪の向きで比べる。逆転の切り替えを起こしにくくする(前進⇔後退を素早く切り替えないため)
            const double cost = std::abs(cand - steer_target_) / kSteerGear +
                                (rev != steer_reverse_ ? kReverseHysteresisRad : 0.0);
            if (cost < best_cost) {
                best_cost = cost;
                target = cand;
                reverse = rev;
                found = true;
            }
        }
        return found;
    }

    // ===================================================================
    // 【動き方を変える時はここ】R1を押している間、50回/秒で呼ばれる。
    // ===================================================================
    void control_step() {
        double drive_target = 0.0;
        const double mag = std::hypot(ls_x_, ls_y_);
        const bool stick_on = mag > DEADZONE_L;
        const bool r2_on = use_drive_ && r2_ > kR2OnThreshold;
        if (stick_on) {
            const double heading = std::atan2(ls_x_, ls_y_); // 上=0、右=+90°
            if (kSteerVel) {
                // 速度制御: 今の目標から一番近い回り方で向ける(何周でもよい)。
                // 進む向き(逆転中は車輪の向き + 180°)から kReverseThresholdRad を超えて離れた向きを
                // 指令されたら、大きく旋回せずに車輪を逆転させて、残り(60°未満)だけ旋回する。
                // 逆転してから元に戻るには、また 120° を超えて離れる必要がある(60°〜120°の余裕があるので、
                // スティックが境目で揺れても前進⇔後退を繰り返さない)。
                // 前進⇔後退の切り替えは、下の加速度の制限で必ずゆっくり0を通る(急に逆に回さない)。
                const double cur = motor_to_wheel(steer_target_);
                const double travel = cur + (steer_reverse_ ? M_PI : 0.0); // 今の進む向き
                double d = std::remainder(heading - travel, 2.0 * M_PI);
                if (std::abs(d) > kReverseThresholdRad) {
                    steer_reverse_ = !steer_reverse_;
                    RCLCPP_INFO(get_logger(), "[逆転] 進む向きから%+.0f°の向きを指令されたので、車輪を%sにして"
                                              "%+.0f°だけ旋回します(駆動の今の指令%+.2frad/s → 加速度の制限で0を通って切り替え)",
                                d * 180.0 / M_PI, steer_reverse_ ? "逆転" : "正転",
                                std::remainder(d + M_PI, 2.0 * M_PI) * 180.0 / M_PI, drive_cmd_);
                    d = std::remainder(d + M_PI, 2.0 * M_PI); // 逆転した後の、進む向きからのずれ
                }
                steer_target_ = wheel_to_motor(cur + d);
            } else {
                double target = steer_target_;
                bool reverse = steer_reverse_;
                if (choose_steer(heading, target, reverse)) {
                    steer_target_ = target;
                    steer_reverse_ = reverse;
                }
            }
        }
        // スティックを倒していない時は向きの目標を変えない(今の向きのまま駆動する)。
        // 操舵が目標の向きに着いてから駆動する(ずれは車輪の向きで比べる)
        if (r2_on && std::abs(steer_pos() - steer_target_) / kSteerGear < kAlignTolRad) {
            drive_target = kDriveSign * (steer_reverse_ ? -1.0 : 1.0) * r2_ * kDriveMaxRadps;
        }
        // 加速度を制限して、指令を急に変えない(前進⇔後退も必ずゆっくり0を通る)。
        // 駆動のESCが有効(状態1)になるまでは0のまま待つ。脱力から有効になるまで0.5秒ほどかかり、
        // その間に指令だけ上げると、有効になった瞬間に速い指令が入る(暴走の見張りも誤って働く)。
        // 回している途中で有効でなくなった時も0に戻し、また有効になってから0から上げ直す。
        const auto now = std::chrono::steady_clock::now();
        const bool drive_ready = drive_.state() == 1;
        const bool drive_wanted = drive_target != 0.0;
        if (drive_wanted && !drive_ready) {
            if (!drive_waiting_) {
                drive_waiting_ = true;
                drive_wait_since_ = now;
            } else if (now - drive_wait_since_ > std::chrono::duration<double>(kDriveEnableWarnSec)) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                                     "駆動のESCが有効になるのを待っています(%.1f秒。状態%d)",
                                     std::chrono::duration<double>(now - drive_wait_since_).count(), drive_.state());
            }
        } else {
            drive_waiting_ = false;
        }
        // 操舵と駆動のつながりの打ち消し: 車輪の向きが変わる速さ × drive_coupling を駆動に足す
        const double steer_wheel_radps = kSteerSign * steer_.velocity() / kSteerGear;
        coupling_cmd_ = use_drive_ ? drive_coupling_ * steer_wheel_radps : 0.0;
        if (std::abs(coupling_cmd_) >= kCouplingMinRadps) {
            coupling_until_ = now + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                        std::chrono::duration<double>(kCouplingHoldSec));
        }
        const bool coupling_active = use_drive_ && drive_coupling_ != 0.0 && now < coupling_until_;

        // 打ち消しの分の余裕を取る: ドライバは「R2の分 + 打ち消しの分」の合計を kDriverLimitRadps で切るので、
        // 速く走りながら操舵を回すと打ち消しの分が切られ、車輪の回転がずれる。合計が上限を超えそうな時だけ、
        // R2の分を下げて打ち消しを優先する(打ち消しが逆向きの時は下げない)。下げる時も加速度の制限で
        // ゆっくり下げるので、操舵を回し始めてから0.1秒ほどは、打ち消しが少し切られることがある。
        const double coupling_now = coupling_active ? coupling_cmd_ : 0.0;
        const double limited_target =
            std::clamp(drive_target, -kDriverLimitRadps - coupling_now, kDriverLimitRadps - coupling_now);
        if (limited_target != drive_target) {
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                                 "[駆動] 打ち消しの分(%+.2frad/s)の余裕を取るため、駆動を%+.2f → %+.2frad/sに下げています",
                                 coupling_now, drive_target, limited_target);
            drive_target = limited_target;
        }

        const double step = kDriveAccelRadps2 * kPeriodSec;
        if (!drive_ready) {
            drive_cmd_ = 0.0;
        } else if (drive_target > drive_cmd_ + step) {
            drive_cmd_ += step;
        } else if (drive_target < drive_cmd_ - step) {
            drive_cmd_ -= step;
        } else {
            drive_cmd_ = drive_target;
        }
        steer_output(kSteerSpeedRadps * kSteerGear); // 速さはモータの値にして送る

        // 駆動は「回したい間(ESCが有効になるのを待つ間と、R2を離した後ゆっくり0に戻るまでを含む)」と
        // 「操舵が回っていて打ち消す間」だけ有効にする。どちらでもなければ脱力にして、
        // 止まったまま力を出し続けないようにする。
        drive_enabled_ = use_drive_ && (drive_wanted || drive_cmd_ != 0.0 || coupling_active);
        if (drive_enabled_) {
            drive_out_ = drive_cmd_ + (coupling_active ? coupling_cmd_ : 0.0);
            drive_.set_velocity(drive_out_);
        } else {
            drive_out_ = 0.0;
            drive_.set_disable();
        }
    }
    // ===================================================================

    // 駆動の位置[モータの出口 rad](ログ用。帰還の位置は回転子の値なので減速比で割る)
    double drive_pos_out() const { return drive_.position_unwrapped() / kDriveEncRatio; }

    // 「異常」が hold_sec 続いたら true を返す見張りの共通部分
    static bool held_for(
        bool abnormal, bool &watching, std::chrono::steady_clock::time_point &since,
        std::chrono::steady_clock::time_point now, double hold_sec) {
        if (!abnormal) {
            watching = false;
            return false;
        }
        if (!watching) {
            watching = true;
            since = now;
        }
        return now - since >= std::chrono::duration<double>(hold_sec);
    }

    // 駆動(DM3519)の暴走の見張り。速すぎるのは脱力中も見る。逆向きは駆動が有効な間だけ見る。
    bool runaway_detected(std::chrono::steady_clock::time_point now) {
        const double v = drive_.velocity();
        const bool opposite = drive_enabled_ &&
                              std::abs(v - drive_out_) >= kRunawayDiffRadps && v * drive_out_ <= 0.0;
        const bool too_fast = std::abs(v) > kDriveMaxRadps + kRunawayOverRadps;
        return held_for(opposite || too_fast, runaway_watch_, runaway_since_, now, kRunawayHoldSec);
    }

    // 駆動(DM3519)の固まりの見張り。駆動が有効なのに、大きな力を出したまま回っていない。
    bool stall_detected(std::chrono::steady_clock::time_point now) {
        const bool stalled = drive_enabled_ &&
                             std::abs(drive_.torque()) >= kStallTorqueNm && std::abs(drive_.velocity()) < kStallVelRadps;
        return held_for(stalled, stall_watch_, stall_since_, now, kStallHoldSec);
    }

    // 速度制御の時の見張り。止めた(finish した)ら true
    bool steer_velocity_watch(std::chrono::steady_clock::time_point now) {
        if (steer_.pos_jump()) {
            RCLCPP_ERROR(get_logger(), "操舵の位置を数え間違えました(帰還の抜け・速すぎる回転)。"
                                       "原点が分からなくなったので止めます");
            finish();
            return true;
        }
        if (now - stuck_check_time_ >= std::chrono::duration<double>(kPosStuckCheckSec)) {
            const double dv = steer_.vel_integral() - stuck_check_vi_; // 速度の読みから見た動き
            const double dp = steer_pos() - stuck_check_pos_;          // 位置の読みの動き
            if (std::abs(dv) > kPosStuckMinRad && std::abs(dp) < kPosStuckRatio * std::abs(dv)) {
                RCLCPP_ERROR(get_logger(), "操舵の位置の読みが、速度の読みと合いません(速度から%+.2frad、"
                                           "位置は%+.2frad)。ESCの中の位置の数え方が粗くなっている恐れがあるので止めます。"
                                           "ESCの電源を入れ直してください(内部の位置 %+.1frad)",
                             dv, dp, steer_.internal_pos());
                finish();
                return true;
            }
            stuck_check_time_ = now;
            stuck_check_pos_ = steer_pos();
            stuck_check_vi_ = steer_.vel_integral();
        }
        if (!std::isnan(steer_.internal_pos()) && std::abs(steer_.internal_pos()) > kInternalPosWarnRad) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000,
                                 "操舵の内部の位置が%+.0fradです(±%.0fradを超えた)。このまま増えると、ゆっくり回した時に"
                                 "位置の読みが変わらなくなります。止めてESCの電源を入れ直してください",
                                 steer_.internal_pos(), kInternalPosWarnRad);
        }
        return false;
    }

    void publisher_timer_callback() {
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        if (finished_) {
            return;
        }
        const auto now = std::chrono::steady_clock::now();

        if (!started_) {
            // 始める前の確認: 使うモータの返事が届くまで待つ(5秒待っても来なければやめる)
            // 操舵だけの時は、駆動の返事は待たない(駆動のESCの電源が切れていてもよい)
            // 24Vを入れた直後の値を避けるため、帰還が0.5秒続けて届いてから始める
            if (steer_.settled() && (!use_drive_ || drive_.settled())) {
                if (!kSteerVel && !steer_.angle_ok()) {
                    RCLCPP_ERROR(get_logger(), "操舵(モータ1)の内部の位置が±12radを超えている(または不明)"
                                               "ので、角度制御が使えません。ESCの電源を入れ直してください");
                    finish();
                    return;
                }
                steer_.lock_position();
                drive_.lock_position();
                steer_zero_ = steer_pos(); // 起動時の向きを「スティック上」にする
                steer_target_ = steer_zero_;
                started_ = true;
                stuck_check_time_ = now;
                stuck_check_pos_ = steer_pos();
                stuck_check_vi_ = steer_.vel_integral();
                auto_homing_pending_ = true; // 最初にR1を押した時に1回だけ原点合わせをする
                RCLCPP_INFO(get_logger(), "操舵は%s", kSteerVel ? "速度制御(何周でも回れる。120°を超える向きは車輪を逆転)" : "角度制御(起動時から約±175°。届かない向きは車輪を逆転)");
                RCLCPP_INFO(get_logger(), "準備できました(操舵の始めの向き %+.3frad、%s)。"
                                          "R1を押している間だけ動きます。最初にR1を押すと原点合わせをします",
                            steer_zero_,
                            use_drive_ ? "駆動も使う" : "操舵だけ。駆動には脱力だけを送る");
                if (use_drive_) {
                    RCLCPP_INFO(get_logger(), "駆動の最高速度 %.2fm/s(モータ%.1frad/s)、加速度 %.2fm/s^2%s",
                                kDriveMaxRadps / kDriveGear * kWheelRadiusM, kDriveMaxRadps, kDriveAccelMps2,
                                kDriveMaxRadps < kDriveMaxMps / kWheelRadiusM * kDriveGear ? "(ドライバの速度の上限で、決めた最高速度より低くなっています)" : "");
                }
            } else if (now - created_ > 5s) {
                RCLCPP_ERROR(get_logger(), "5秒待ってもモータから返事が来ません"
                                           "(ros2can・damiao_driver_nodeの起動、モータの電源、ROS_AUTOMATIC_DISCOVERY_RANGEを確認)");
                finish();
                return;
            }
            steer_.send();
            drive_.send(); // 待っている間も脱力を送り続ける(止めたままの解除にもなる)
            return;
        }

        // 故障したら全部止める(状態15 = ESP32・駆動ノードが止めたまま、も含む)
        // 操舵だけの時は、駆動の故障・返事の途絶では止めない(使っていないため)
        const bool drive_bad = use_drive_ && (drive_.state() > 1 || !drive_.ready());
        if (steer_.state() > 1 || !steer_.ready() || drive_bad) {
            RCLCPP_ERROR(get_logger(), "故障または返事の途絶を検出したので止めます(状態 操舵%d 駆動%d)",
                         steer_.state(), drive_.state());
            finish();
            return;
        }

        // 速度制御の時の見張り: 位置の数え間違い・位置の読みの止まり・内部の位置の大きさ
        if (kSteerVel && steer_velocity_watch(now)) {
            return;
        }

        // PS4の入力が使えるか
        bool input_ok = joy_received_ &&
                        now - last_joy_time_ < std::chrono::duration<double>(kJoyTimeoutSec);
        if (count_publishers("joy") > 1) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "/joy の送り手が2つ以上います。入力を無視します(ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST を確認)");
            input_ok = false;
        }
        const bool active = input_ok && r1_ && armed_ && !estop_;

        if (!active) {
            // R1を離した・E-STOP・入力が途絶えた → 2台とも脱力
            steer_.set_disable();
            drive_.set_disable();
            drive_cmd_ = 0.0;
            drive_out_ = 0.0;
            coupling_cmd_ = 0.0;
            coupling_until_ = {};
            drive_enabled_ = false;
            drive_waiting_ = false;
            stall_watch_ = false;
            was_active_ = false;
            steer_vel_cmd_ = 0.0;
            if (homing_ != Homing::kIdle) {
                RCLCPP_WARN(get_logger(), "原点合わせを中断しました(R1を離した・E-STOP・入力の途絶)");
                homing_ = Homing::kIdle;
            }
            homing_requested_ = false;
        } else {
            if (!was_active_) {
                // 脱力中に手で回されているかもしれないので、今の向きから始める
                if (!kSteerVel && (!steer_.angle_ok() || std::abs(steer_pos()) > kSteerLimitRad)) {
                    RCLCPP_ERROR(get_logger(), "操舵の位置が範囲(±%.1frad)の外です。ESCの電源を入れ直してください",
                                 kSteerLimitRad);
                    finish();
                    return;
                }
                steer_target_ = steer_pos();
                steer_reverse_ = false;
                was_active_ = true;
            }
            if (auto_homing_pending_ && homing_ == Homing::kIdle) {
                homing_requested_ = true; // 起動後の1回目の原点合わせ
            }
            if (homing_requested_) {
                homing_requested_ = false;
                if (homing_ == Homing::kIdle) {
                    if (!origin_ok(now)) {
                        RCLCPP_ERROR(get_logger(), "原点センサーの値が来ていないので、原点合わせを始められません"
                                                   "(serial_rx_%d の%zu番。ESP32のファームが原点センサーに対応しているか確認)",
                                     RX_DEVICE_ID, kOriginSlot);
                        auto_homing_pending_ = false; // 自動ではもうやらない(△で手動ならできる)
                    } else {
                        homing_ = origin_on_ ? Homing::kBackoff : Homing::kSeek;
                        homing_fine_ = false; // まず速く探す
                        backoff_to_target_ = false;
                        settle_started_ = false;
                        homing_started_ = now;
                        homing_start_pos_ = steer_pos();
                        steer_target_ = steer_pos();
                        RCLCPP_INFO(get_logger(), "原点合わせを始めます(R1を押したまま。離すと中断)");
                    }
                }
            }
            if (homing_ != Homing::kIdle) {
                homing_step(now);
            } else {
                control_step();
            }
            if (stall_detected(now)) {
                RCLCPP_ERROR(get_logger(), "駆動(DM3519)が大きな力を出したまま回っていないので止めます"
                                           "(指令%+.2frad/s 実速度%+.2frad/s トルク%+.2fN・m)。モータが熱くなっていないか、"
                                           "動力線・エンコーダの校正を確かめてください",
                             drive_out_, drive_.velocity(), drive_.torque());
                RCLCPP_ERROR(get_logger(), "  止まった時の駆動の位置: モータ%+.3frad(帰還の値 %+.3frad 回転子)"
                                           "  回し始めてから %+.3frad(車輪 %+.1f°)",
                             drive_pos_out(), drive_.position(), drive_pos_out() - drive_start_pos_,
                             (drive_pos_out() - drive_start_pos_) / kDriveGear * 180.0 / M_PI);
                finish();
                return;
            }
        }
        // 暴走の見張りは、動作中・脱力中どちらでも行う(駆動の返事が来ている時だけ)
        if (drive_.ready() && runaway_detected(now)) {
            RCLCPP_ERROR(get_logger(), "駆動(DM3519)の暴走を検出したので止めます"
                                       "(指令%+.2frad/s 実速度%+.2frad/s トルク%+.2fN・m)",
                         drive_out_, drive_.velocity(), drive_.torque());
            finish();
            return;
        }
        steer_.send();
        drive_.send();

        // 駆動を回している間は、0.1秒ごとに駆動の様子を出す(固まりは0.6秒ほどで止まるため)
        if (drive_enabled_ && !drive_was_enabled_) {
            drive_start_pos_ = drive_pos_out();
            last_drive_print_ = now - 1s;
            RCLCPP_INFO(get_logger(), "[駆動] 回し始め: 位置 モータ%+.3frad(帰還の値 %+.3frad 回転子)",
                        drive_pos_out(), drive_.position());
        }
        drive_was_enabled_ = drive_enabled_;
        if (drive_enabled_ && now - last_drive_print_ >= 100ms) {
            last_drive_print_ = now;
            const double moved = drive_pos_out() - drive_start_pos_;
            RCLCPP_INFO(get_logger(), "[駆動] 指令%+5.2f(うち打ち消し%+5.2f) 実%+6.2frad/s  トルク%+5.2fN・m  "
                                      "位置 モータ%+8.3frad(帰還の値 %+.3f 回転子)  回し始めから%+.3frad(車輪%+6.1f°)  状態%d",
                        drive_out_, drive_out_ - drive_cmd_, drive_.velocity(), drive_.torque(),
                        drive_pos_out(), drive_.position(),
                        moved, moved / kDriveGear * 180.0 / M_PI, drive_.state());
        }

        // 1秒ごとに様子を表示
        if (now - last_print_ >= 1s) {
            last_print_ = now;
            // 速度制御の時は、操舵へ送っている速度と内部の位置も出す
            char extra[192] = "";
            int n = 0;
            if (kSteerVel) {
                n = std::snprintf(extra, sizeof(extra), "  操舵の速度指令%+5.2frad/s 内部の位置%+.1frad",
                                  steer_vel_cmd_, steer_.internal_pos());
            }
            if (n >= 0 && static_cast<size_t>(n) < sizeof(extra)) {
                std::snprintf(extra + n, sizeof(extra) - n, "  駆動の位置 モータ%+.3frad(帰還の値 %+.3f 回転子)",
                              drive_pos_out(), drive_.position());
            }
            // 車輪の値で表示する(かっこ内はモータの値)
            RCLCPP_INFO(
                get_logger(), "%s 車輪の向き: 目標%+7.1f°%s 今%+7.1f°(モータ%+6.2frad)  "
                              "車輪の速さ: 指令%+5.2f 実%+5.2frad/s(モータ%+5.2frad/s)  "
                              "状態 操舵%d 駆動%d  駆動トルク%+5.2fN・m  原点センサー %s%s%s",
                active ? "[動作]" : "[脱力]",
                motor_to_wheel(steer_target_) * 180.0 / M_PI, steer_reverse_ ? "(逆転)" : "",
                motor_to_wheel(steer_pos()) * 180.0 / M_PI, steer_pos(), drive_cmd_ / kDriveGear, drive_.velocity() / kDriveGear, drive_.velocity(),
                steer_.state(), drive_.state(), drive_.torque(),
                !origin_ok(now) ? "なし" : (origin_on_ ? "ON" : "OFF"),
                homing_ != Homing::kIdle ? "(原点合わせ中)" : (homed_ ? "(合わせ済み)" : ""),
                extra);
        }
    }

    // 2台とも脱力を送ってからノードを終える
    void finish() {
        finished_ = true;
        steer_.set_disable();
        drive_.set_disable();
        for (int i = 0; i < 5; ++i) {
            steer_.send();
            drive_.send();
            std::this_thread::sleep_for(20ms);
        }
        RCLCPP_INFO(get_logger(), "終わりました(2台とも脱力)");
        rclcpp::shutdown();
    }

    MotorHandle steer_; // モータ1: 操舵 DM2325
    MotorHandle drive_; // モータ2: 駆動 DM-S3519
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr origin_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::PreShutdownCallbackHandle shutdown_cb_;
    // タイマー・joy・Ctrl+C時の処理が同時に動かないようにする鍵。finish()の中の
    // rclcpp::shutdown()が同じスレッドで終了処理(鍵を取る)を呼ぶので、同じスレッドなら
    // 何度でも取れるrecursive_mutexにしている(普通のmutexだと固まる)。
    std::recursive_mutex mutex_;
    bool started_{false};
    bool finished_{false};
    // PS4の入力
    double ls_x_{0.0};
    double ls_y_{0.0};
    double r2_{0.0};
    bool r1_{false};
    bool r2_seen_released_{false};
    bool armed_{false}; // R1を一度離したか(起動直後・E-STOP後は押し直しが必要)
    bool estop_{false};
    bool joy_received_{false};
    bool triangle_prev_{false};
    bool homing_requested_{false};    // △が押された(次の publisher_timer_callback() で原点合わせを始める)
    bool auto_homing_pending_{false}; // 起動後の自動の原点合わせがまだ終わっていない
    // 原点センサー・原点合わせ
    bool origin_valid_{false};
    bool origin_on_{false};
    std::chrono::steady_clock::time_point last_origin_time_{};
    Homing homing_{Homing::kIdle};
    bool homing_fine_{false};      // true = 見つけた後のゆっくり合わせ直し中
    double homing_start_pos_{0.0}; // 原点合わせを始めた所(操舵モータ[rad])
    double steer_vel_cmd_{0.0};    // 速度制御の時に操舵へ送っている速度[rad/s]
    std::chrono::steady_clock::time_point stuck_check_time_{};
    double stuck_check_pos_{0.0};
    double stuck_check_vi_{0.0};
    bool backoff_to_target_{false}; // true = backoff_target_ まで戻ってから探し直す
    double backoff_target_{0.0};    // 見つけた後に戻る所(操舵モータの位置[rad])
    bool settle_started_{false};
    std::chrono::steady_clock::time_point settle_since_{};
    std::chrono::steady_clock::time_point homing_started_{};
    bool homed_{false};
    double steer_origin_pos_{0.0};  // 原点(センサーが+向きでONになった所)の操舵モータの位置[rad]
    double origin_offset_rad_{0.0}; // パラメータ origin_offset_deg[rad]
    std::chrono::steady_clock::time_point last_joy_time_{};
    // 操舵・駆動の状態
    bool was_active_{false};
    double steer_zero_{0.0};                                 // 起動時の操舵の向き(スティック上 = この向き)
    double steer_target_{0.0};                               // 今の操舵の目標[rad]
    bool steer_reverse_{false};                              // 車輪を逆転させているか
    double drive_cmd_{0.0};                                  // R2による駆動の速度[rad/s](加速度制限後。打ち消しは含まない)
    double drive_out_{0.0};                                  // 実際に駆動へ送っている速度[rad/s](drive_cmd_ + 打ち消し)
    double drive_coupling_{-1.0};                            // パラメータ drive_coupling(操舵と駆動のつながりの打ち消し)
    double coupling_cmd_{0.0};                               // 打ち消しの量[駆動モータ rad/s]
    std::chrono::steady_clock::time_point coupling_until_{}; // この時刻まで打ち消しのため駆動を有効にする
    bool use_drive_{true};                                   // パラメータ use_drive(false = 操舵だけ)
    bool drive_enabled_{false};                              // 今、駆動に速度の指令を送っているか(false = 脱力を送っている)
    bool runaway_watch_{false};
    std::chrono::steady_clock::time_point runaway_since_{};
    bool stall_watch_{false};
    std::chrono::steady_clock::time_point stall_since_{};
    std::chrono::steady_clock::time_point created_;
    std::chrono::steady_clock::time_point last_print_{};
    std::chrono::steady_clock::time_point last_drive_print_{};
    bool drive_was_enabled_{false};                            // 前の周期に駆動を回していたか
    bool drive_waiting_{false};                                // 駆動のESCが有効になるのを待っているか
    std::chrono::steady_clock::time_point drive_wait_since_{}; // 待ち始めた時刻
    double drive_start_pos_{0.0};                              // 駆動を回し始めた時の位置[モータの出口 rad]
};

} // namespace Infinite_Zakisute

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    // figletでノード名を表示
    std::string figletout = "figlet BR Swerve Drive";
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

    // ノードのコールバックは同じグループ(同時には動かない)なので、MultiThreadedExecutorでも
    // MotorHandle の帰還とタイマーが同時に走ることはない
    rclcpp::executors::MultiThreadedExecutor exec;

    auto swerve_drive = std::make_shared<Infinite_Zakisute::SwerveDriveControl>();
    exec.add_node(swerve_drive);
    exec.spin();

    rclcpp::shutdown();
    return 0;
}
