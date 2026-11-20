#pragma once
// =====================================================================
// infinite_ctrl_params.hpp : infinite_ctrl_node の定数をまとめたもの
//   変えたら colcon build し直す(ROSのパラメータではない)。
//   機構で決まる値・安全のための値(見張りのしきい値・ドライバの上限)は、打ち間違いで
//   見張りが外れないように、yaml ではなくここに置く。
//   動き方の説明は src/infinite_ctrl_node.cpp の先頭のコメント。
// =====================================================================
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>

namespace Infinite_Zakisute {

// ---------------------------------------------------------------------
// 調整する値(まずは低速のまま試すこと)
// ---------------------------------------------------------------------
// 指令を送る周期(PUBLISH_RATE_MS)・スティックのデッドゾーン(DEADZONE_L)・ESP32のDEVICE_ID(RX_DEVICE_ID)・
// PS4のボタンの番号は、チームのテンプレートに合わせて infinite_ctrl_node.cpp の先頭の #define にある。

// 歯車の比(モータ側の歯数 : 車輪側の歯数)。モータの値 = 車輪の値 × 比
constexpr double kSteerGear = 49.0 / 13.0; // 操舵 13:49。モータが約3.77回転で車輪の向きが1回転
constexpr double kDriveGear = 33.0 / 23.0; // 駆動 23:33。モータが約1.43回転で車輪が1回転
// 駆動モータ DM-S3519-1EC は中に減速機(3591/187 ≒ 19.2)がある。速度は出口の値で返ってくるが、
// 位置は中の回転子の値で返ってくる(車輪1回転で位置が約178rad変わる)。
// ログに出す駆動の位置は、これで割って出口の値に直す(制御には位置を使っていない)。
constexpr double kDriveEncRatio = 3591.0 / 187.0;
constexpr double kSteerSign = 1.0; // 右に倒して車輪が左を向くなら -1.0 にする
constexpr double kDriveSign = 1.0; // 車輪の前後が逆なら -1.0 にする

// 駆動(モータの出力軸の値。暴走の見張りもこの値で行う)
// 走る速さ・加速度は m/s で決め、中でモータの rad/s に直す。
constexpr double kWheelRadiusM = 0.040; // 車輪の半径[m](直径80mm)
constexpr double kDriveMaxMps = 1.5;    // 最高速度[m/s]
constexpr double kDriveAccelMps2 = 1.5; // 加速度の上限[m/s^2]
// damiao_driver_node のモータ2の速度の上限[rad/s]。ノードの上限はこれを超えないようにする
// (超えた分はドライバで切られ、R2の奥の方が効かなくなるため)。ドライバの上限を上げたら、ここも変える。
// ドライバ(damiao_ctrl.yaml の max_vel_per_motor)とESP32のファームの上限は40。それを超えないこと。
constexpr double kDriverLimitRadps = 25.0;
// 1.5m/s → 車輪 37.5rad/s → モータ 53.8rad/s だが、今はドライバの上限 25.0rad/s(≒0.70m/s)で止まる
constexpr double kDriveMaxRadps =
    std::min(kDriveMaxMps / kWheelRadiusM * kDriveGear, kDriverLimitRadps);        // 駆動モータの最高速度[rad/s]
constexpr double kDriveAccelRadps2 = kDriveAccelMps2 / kWheelRadiusM * kDriveGear; // ≒53.8rad/s^2
// 操舵(モータの出力軸の値)
constexpr double kSteerLimitRad = 11.5; // 操舵モータの目標の範囲[rad](±12radより少し内側)
                                        // → 車輪の向きでは ±11.5 ÷ 3.77 ≒ ±3.05rad(±175°)
// 操舵(車輪の向きの値)
// 車輪の向きを変える速さの上限は、damiao_driver_node のモータ1の上限(45.6rad/s)の何割かで決める
// 50% = モータ22.8rad/s = 車輪の向き約6.0rad/s(約347°/秒)
constexpr double kSteerDriverLimitRadps = 45.6; // damiao_driver_node のモータ1の速度の上限[rad/s]
constexpr double kSteerSpeedRatio = 0.50;       // その何割で回すか
constexpr double kSteerSpeedRadps =
    kSteerDriverLimitRadps * kSteerSpeedRatio / kSteerGear; // 車輪の向きを変える速さの上限[rad/s]
constexpr double kAlignTolRad = 0.3;                        // 車輪の向きがこれ以上ずれている間は駆動しない[rad]
constexpr double kReverseHysteresisRad = 0.2;               // 車輪の逆転を切り替えにくくする余裕[rad](角度制御の時)
// 速度制御の時: 進む向きからこれを超えて離れた向きを指令されたら、旋回せずに車輪を逆転する[rad]
constexpr double kReverseThresholdRad = 120.0 * M_PI / 180.0;
constexpr double kJoyTimeoutSec = 0.5;  // joyがこれ以上来なければ止める
constexpr double kR2OnThreshold = 0.05; // R2がこれより押されている時だけ駆動する(0〜1)
// 暴走の見張り(9.1節 調べ方4)
constexpr double kRunawayDiffRadps = 10.0; // 指令と逆向きで、これ以上ずれたら異常
constexpr double kRunawayOverRadps = 5.0;  // |実速度| > 最高速度 + これ なら異常
constexpr double kRunawayHoldSec = 0.1;    // 異常がこれだけ続いたら止める
// 固まりの見張り(回らないまま大きな電流を流し続けてモータを焼かないため。駆動モータの値)
constexpr double kStallTorqueNm = 3.0; // トルクがこれ以上なのに
constexpr double kStallVelRadps = 0.5; // 速度がこれ未満のまま
constexpr double kStallHoldSec = 0.5;  // これだけ続いたら止める
// 操舵と駆動のつながりの打ち消し(駆動モータの値)
constexpr double kCouplingMinRadps = 0.05; // 打ち消しの量がこれ以上の時に駆動を有効にする
constexpr double kCouplingHoldSec = 0.3;   // 操舵が止まってから、これだけたったら駆動を脱力に戻す

// ---------------------------------------------------------------------
// 操舵の制御のしかた【ここで切り替える】(説明はファイルの先頭)
//   SteerMode::kAngle    : 角度制御。正確。起動時から約±175°まで、届かない向きは車輪を逆転
//   SteerMode::kVelocity : 速度制御 + ノードの中で位置制御。何周でも回れる(120°を超えたら車輪を逆転)
// ---------------------------------------------------------------------
enum class SteerMode { kAngle,
                       kVelocity };
constexpr SteerMode kSteerMode = SteerMode::kVelocity;
constexpr bool kSteerVel = (kSteerMode == SteerMode::kVelocity);

// 速度制御(kVelocity)の時だけ使う値(モータの出力軸の値)
constexpr double kSteerKp = 6.0;              // 位置のずれ1radあたりの速度指令[rad/s]
constexpr double kSteerHoldTolRad = 0.01;     // ずれがこれ未満なら速度0にする[rad]
constexpr double kPosWrapRad = 12.5;          // 帰還の位置は±これで一周する(ESCのP_MAX)
constexpr double kPosJumpRad = 3.0;           // 1回の帰還でこれ以上動いたら数え間違いとして止める
constexpr double kPosStuckCheckSec = 0.3;     // 位置の読みが止まっていないか見る間隔[s]
constexpr double kPosStuckMinRad = 0.3;       // 速度の読みからこれ以上動いたはずなのに
constexpr double kPosStuckRatio = 0.5;        // 位置の読みがその半分も変わっていなければ止める
constexpr double kInternalPosWarnRad = 100.0; // 内部の位置がこれを超えたら警告(電源を入れ直すと0)
constexpr double kHomingMaxTurns = 1.3;       // 原点合わせで車輪を何周まで回して探すか

// 原点合わせ(原点センサー)
constexpr size_t kOriginSlot = 23;        // 原点センサーの値が入る番号(damiao.cppと同じ)
constexpr int16_t kOriginEmpty = 1;       // 1 = 空
constexpr int16_t kOriginDetected = 2;    // 2 = 原点(黒線がON)
constexpr double kOriginTimeoutSec = 0.5; // センサーの値がこれ以上古ければ使わない
constexpr double kHomingFastRadps = 3.5;  // 原点を探す時の車輪の向きの速さ[rad/s](約200°/s)
                                          // → モータでは約13.2rad/s(上限45.6の約3割)
constexpr double kHomingSlowRadps = 0.3;  // 見つけた後、合わせ直す時の速さ[rad/s](約17°/s)
// 速く探して見つけた後に戻る量[rad](車輪の向き)。
// 速く回している時の行き過ぎがこれより大きくても、戻った所で止まってセンサーの値を待つので原点はずれない
// (まだONならさらに戻る)。ただ、行き過ぎがセンサーの幅+この量より大きいと、端まで
// 行ってから戻って見つけ直すので時間がかかる → その時はこの値を大きくする。
constexpr double kHomingBackoffRad = 1.0 * M_PI / 180.0; // 1°
constexpr double kHomingSettleSec = 0.15;                // 戻った所で止まって、センサーの値が追いつくのを待つ時間
constexpr double kHomingTimeoutSec = 20.0;               // これを超えたら失敗(端から端まで約2.5秒)

} // namespace Infinite_Zakisute
