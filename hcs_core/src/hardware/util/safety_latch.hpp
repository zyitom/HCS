#pragma once

#include <cstddef>
#include <cstdint>

#include <hcs_msgs/switch.hpp>

namespace hcs_core::hardware::util {

// ============================================================================
// 关键设备失效 → 锁存全失能。
//
// 驱动的 online() / faulted() 只是观测；没人读，它们就只是话题。这里是读的那一方：
// 硬件组件每拍把关键设备的健康状况交进来，任何一个"上过线之后坏了"就锁存，
// 锁存期间 Command 伙伴只发安全批次（DM 0xFD、LK 0 电流、DJI 0 电流）。
//
// 为什么锁存而不是跟着 online 走：接触不良的线会让设备反复掉线又上线。
// DM 驱动在反馈恢复后会自己使能、清错，跟着 online 走就等于允许一条腿在
// 没人看着的时候自己重新出力。所以恢复必须由人来确认。
//
// 复位：遥控左拨杆先拨到 DOWN（失能挡），再拨离 DOWN。平衡控制器本来就把
// DOWN 当失能，所以复位的那一刻，控制器也是从失能态重新起来的。拨杆拨下去
// 的时候如果还有设备不健康，复位作废，继续锁存。
//
// 从未上线的设备不算失效：台架上缺板、电机没上电，车本来就平衡不起来，
// 由平衡控制器自己的"未就绪"判断去管，这里不因此锁死整车。
//
// 纯逻辑，不碰硬件，gtest 直接覆盖。
// ============================================================================

/// 一个关键设备这一拍的健康快照。
struct DeviceHealth {
    bool received; ///< 是否收到过至少一帧反馈
    bool online;   ///< 离线看门狗
    bool faulted;  ///< 设备自报的、需要人处理的故障
};

class SafetyLatch {
public:
    /// 失效原因，按首次发生记录，锁存期间不再改写。
    enum class Reason : std::uint8_t { kNone, kOffline, kFaulted };

    /// 周期域每拍调用一次。返回这一拍是否必须发安全批次。
    /// @param devices 关键设备，顺序即 tripped_device() 的下标
    /// @param switch_left 遥控左拨杆
    template <std::size_t N>
    bool update(const DeviceHealth (&devices)[N], hcs_msgs::Switch switch_left) noexcept {
        const auto [index, reason] = first_failure(devices);

        if (!latched_ && reason != Reason::kNone) {
            latched_ = true;
            reason_ = reason;
            device_ = index;
            ++trip_count_;
        }

        // 复位分两步：先看到 DOWN 并且此刻全部健康才"预备"，再看到离开 DOWN 才解除。
        // DOWN 期间设备又坏了，预备作废。
        if (latched_) {
            if (switch_left == hcs_msgs::Switch::DOWN)
                armed_for_reset_ = reason == Reason::kNone;
            else if (armed_for_reset_) {
                latched_ = false;
                armed_for_reset_ = false;
                reason_ = Reason::kNone;
            }
        }

        return latched_;
    }

    [[nodiscard]] bool latched() const noexcept { return latched_; }
    [[nodiscard]] Reason reason() const noexcept { return reason_; }
    /// 首个失效设备在 devices 里的下标；未锁存时无意义。
    [[nodiscard]] std::size_t tripped_device() const noexcept { return device_; }
    /// 累计锁存次数，尽力域据此判断"有新的一次"再打日志。
    [[nodiscard]] std::uint32_t trip_count() const noexcept { return trip_count_; }

private:
    struct Failure {
        std::size_t index;
        Reason reason;
    };

    template <std::size_t N>
    static Failure first_failure(const DeviceHealth (&devices)[N]) noexcept {
        for (std::size_t index = 0; index < N; ++index) {
            const auto& device = devices[index];
            if (!device.received)
                continue;
            if (device.faulted)
                return {index, Reason::kFaulted};
            if (!device.online)
                return {index, Reason::kOffline};
        }
        return {0, Reason::kNone};
    }

    bool latched_ = false;
    bool armed_for_reset_ = false;
    Reason reason_ = Reason::kNone;
    std::size_t device_ = 0;
    std::uint32_t trip_count_ = 0;
};

} // namespace hcs_core::hardware::util
