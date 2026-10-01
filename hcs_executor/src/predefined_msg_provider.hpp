#pragma once

// /predefined/* 是遗留接口，新代码一律用 update(const Tick&) 的参数拿时间和拍号。
// 改造前 /predefined/update_count 是"执行次数"：PredefinedMsgProvider::update 每跑一拍加一，
// 跳拍时不增。组件拿它乘 dt 去推参考轨迹相位，于是跳一拍整条轨迹就被静默拉长，
// 日志上只看得到 skipped 涨了一点。换成 tick.sequence（按拍数递增，含被跳过的拍）
// 之后这个洞才补上——这就是 RT-1 的修法。

#include <chrono>
#include <cstddef>

#include <hcs_base/channel/tick.hpp>
#include <hcs_base/thread/rt_attributes.hpp>

#include "hcs_executor/component.hpp"

class PredefinedMsgProvider : public hcs_executor::Component {
public:
    // 显式命名，不经过 NameScope：它由 Executor 直接 new，不是 pluginlib 加载进来的。
    PredefinedMsgProvider()
        : Component{"predefined_msg_provider"} {
        register_output("/predefined/update_rate", update_rate_);
        register_output("/predefined/update_count", update_count_, static_cast<size_t>(-1));
        register_output("/predefined/timestamp", timestamp_);
        register_output("/predefined/safe_mode", safe_mode_, false);
    }

    /// 启动时调一次，仅供显示。
    void set_update_rate(double frame_rate) { *update_rate_ = frame_rate; }

    /// executor 捕获组件异常后调用。写者与 update() 是同一条线程，普通 bool 就够。
    void set_safe_mode(bool safe_mode) noexcept { *safe_mode_ = safe_mode; }

    void update(const hcs_sync::Tick& tick) HCS_NONBLOCKING override {
        *update_count_ = tick.sequence;
        *timestamp_ = tick.scheduled;
    }

private:
    /// 仅供显示，禁止用来算 dt——dt 只许来自 Tick::dt。
    OutputInterface<double> update_rate_;
    /// == tick.sequence，跳拍时照样递增。
    OutputInterface<size_t> update_count_;
    /// == tick.scheduled，是本拍的名义起点而不是实际起拍时刻。
    OutputInterface<std::chrono::steady_clock::time_point> timestamp_;
    /// 有组件失效时为 true。
    OutputInterface<bool> safe_mode_;
};
