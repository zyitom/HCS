#pragma once

#include <cstdint>

namespace hcs_utility {

/// 按拍计数的看门狗(冷却计数器):收到数据 reset,每周期 tick,断流时报警一次。
///
/// 状态机(旧实现把 Latched 编码在 counter_ 的奇偶性里 —— 触发后 counter_=1,
/// 靠 1-2 的无符号下溢进入永不到 0 的奇数环。数学上成立,但零文档、零保护,
/// 任何人改一行就碎。这里是同一个行为的显式版本):
///
///   kCounting ──counter 归零──▶ kLatched(报警一次,静默)
///       ▲                          │
///       └──────── reset(n>0) ──────┘
///   kDue ──下一次 tick 报警──▶ kLatched   (reset(0):立即到期)
///
/// 三条语义,全部与旧实现一致(时序上比旧实现早一拍报警,见 reset):
///   - reset(cooldown):喂狗。cooldown = 再过多少个 tick() 没喂就报警。
///   - tick():计数减到 0 的那一拍返回 true;之后 Latched,直到下一次 reset ——
///     报警只报一次,不会每拍刷屏,安全值由调用方在报警时设置一次。
///   - 从未 reset 过(默认构造)tick() 恒返回 false:看门狗只负责"来过、然后断了"
///     的数据流;从没来过的数据,初值安全是调用方初始化的职责。
class TickTimer {
public:
    /// @param cooldown 冷却拍数。0 表示下一次 tick() 立即报警。
    void reset(unsigned int cooldown) {
        counter_ = cooldown;
        state_ = cooldown == 0 ? State::kDue : State::kCounting;
    }

    /// @return 本拍是否报警(断流超时)。
    [[nodiscard]] bool tick() {
        switch (state_) {
        case State::kCounting:
            // reset 保证 counter_ >= 1 才进 kCounting,这里永不下溢。
            if (--counter_ == 0) {
                state_ = State::kLatched;
                return true;
            }
            return false;
        case State::kDue:
            state_ = State::kLatched;
            return true;
        case State::kLatched:
            return false;
        }
        return false; // 不可达,照顾不认枚举穷举的编译器
    }

private:
    enum class State : std::uint8_t { kCounting, kDue, kLatched };

    /// 默认 kLatched = 未武装,永不触发 —— 与旧实现的默认行为一致。
    State state_ = State::kLatched;
    unsigned int counter_ = 0;
};

} // namespace hcs_utility
