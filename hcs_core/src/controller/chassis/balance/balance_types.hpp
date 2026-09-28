#pragma once

#include <cstdint>

namespace hcs_core::controller::chassis::balance {

/// 平衡模式（对应 Helios mode_e 的活状态；TANK/TOUCH_DOWN 是死状态不搬）。
/// 通过 /chassis/balance/mode 接口传递，枚举值即线上值，顺序不可改。
enum class Mode : std::uint8_t {
    kDisabled = 0,   ///< 操作手关（拨杆下）
    kNormal = 1,     ///< 平衡站立/行驶
    kSlowStart = 2,  ///< 起身（收腿 + 转腿）
    kUpStair = 3,    ///< 上台阶
    kSelfHeal = 4,   ///< 自救
    kSpin = 6,       ///< 小陀螺
    kJump = 7,       ///< 跳跃（五阶段，见 JumpPhase）
    kFly = 8,        ///< 飞坡（离地检测默认禁用时不可达，同 Helios）
    kFallen = 9,     ///< 摔倒（Helios 的 fatal 等待期，2 s 后转自救）
};

/// 控制器选择（对应 Helios process_e）。
enum class Process : std::uint8_t {
    kLqrOn = 0,     ///< NLMPC（回退 LQR 表）+ 腿长 PID
    kDisabled = 1,  ///< 输出清零
    kPidOnly = 2,   ///< 摆动腿角度 PID（起身/自救/腾空）
    kHealing = 3,   ///< 自救转腿
};

/// 跳跃五阶段（对应 Helios jump_phase_e）。
enum class JumpPhase : std::uint8_t {
    kPress = 0,    ///< 压腿
    kTakeOff = 1,  ///< 起跳蹬伸
    kFlying = 2,   ///< 腾空
    kLanding = 3,  ///< 落地缓冲
    kComplete = 4, ///< 完成
};

/// 腿长三档（对应 Helios LL_Discribe_e）。
enum class LegLengthLevel : std::uint8_t {
    kLow = 0,  ///< 0.17 m
    kMid = 1,  ///< 0.25 m
    kHigh = 2, ///< 0.36 m
};

/// 离地检测三态（对应 Helios Gnd_Off_Detect 的返回；检测默认禁用，恒 kGrounded）。
enum class GroundState : std::uint8_t {
    kGrounded = 0,
    kAirborne = 1,
    kImpact = 2,
};

} // namespace hcs_core::controller::chassis::balance
