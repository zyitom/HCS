// libhcs（板卡 SDK）自己的诊断日志 → hcs_log。
//
// 分工：SDK 只管"发生了什么"，并留了一个出口（libhcs/logging.hpp 的 Sink）；日志
// "怎么送、送到哪"是用它的程序的事。SDK 单独用的时候自己写 stderr（写不进去就丢、不等）；
// 在 HCS 里，由这个文件把出口接到 hcs_log 上，SDK 的日志就和组件的日志走同一条路：
// 无锁队列 → 日志线程 → rclcpp / stderr。两边互相不认识——SDK 不知道 hcs_log，
// hcs_log 也不知道 SDK，认识双方的只有这里。
//
// 进来的线程是控制环依赖的那几条：USB 事件线程、会话保活线程、调 transmit() 的发送线程。
// 所以这里只许做"拷贝一次、入队一次"。
#include <algorithm>
#include <array>
#include <cstddef>
#include <cstring>
#include <string_view>

#include <hcs_base/logging/logger.hpp>
#include <hcs_base/thread/rt_attributes.hpp>
#include <hcs_executor/component.hpp>
#include <libhcs/logging.hpp>

namespace hcs_core::hardware {
namespace {

namespace sdk = libhcs::host::logging;

[[nodiscard]] constexpr hcs_log::Level to_hcs_level(sdk::Level level) noexcept {
    switch (level) {
    case sdk::Level::kTrace:
    case sdk::Level::kDebug: return hcs_log::Level::kDebug;
    case sdk::Level::kInfo: return hcs_log::Level::kInfo;
    case sdk::Level::kWarn: return hcs_log::Level::kWarn;
    case sdk::Level::kErr: return hcs_log::Level::kError;
    case sdk::Level::kCritical:
    case sdk::Level::kOff: return hcs_log::Level::kFatal;
    }
    return hcs_log::Level::kError;
}

/// SDK 的日志出口在 HCS 这边的实现。
class LibhcsLogRoute final : public sdk::Sink {
public:
    explicit LibhcsLogRoute(hcs_log::Backend& backend) noexcept
        : backend_(backend) {}

    /// SDK 的各条线程都会进来，可能同时。文本在 SDK 那边已经格式化好了（在它自己的栈上，
    /// 只在这次调用期间有效），这里不格式化、不分配、不等谁。
    ///
    /// 日志口的名字带上板的序列号（"libhcs.AF-90A7"）：一个进程里接着三块板，
    /// "USB link faulted" 不说是哪一块就等于没说。不针对某块板的行（设备扫描）就叫 "libhcs"。
    void write(const sdk::Record& record) noexcept HCS_NONBLOCKING override {
        std::array<char, hcs_log::Record::kNameCapacity> name;
        std::size_t size = append(name, 0, kName);
        if (!record.source.empty()) {
            size = append(name, size, ".");
            size = append(name, size, record.source);
        }

        // 现造一个 Logger：它只是一个指针加这个名字，造它就是一次 memcpy。
        const hcs_log::Logger logger{backend_, std::string_view{name.data(), size}};
        logger.rt().write(to_hcs_level(record.level), record.message);
    }

private:
    static constexpr std::string_view kName = "libhcs";

    /// 往定长的名字里接一段，装不下的截掉。返回接完之后的长度。
    template <std::size_t Capacity>
    [[nodiscard]] static std::size_t append(
        std::array<char, Capacity>& out, std::size_t size, std::string_view text) noexcept
        HCS_NONBLOCKING {
        const std::size_t count = std::min(text.size(), Capacity - size);
        if (count != 0)
            std::memcpy(out.data() + size, text.data(), count);
        return size + count;
    }

    hcs_log::Backend& backend_;
};

// 这个库被加载的那一刻（pluginlib 的 dlopen，主线程；单测里是进程启动）接管，卸载时交还。
//
// 不放在某个板卡组件的构造函数里：SDK 的日志出口是全进程一个，板卡组件却有好几种
// （Board、CanRttProbe、HcsLinkProbe），以后还会有新的。接没接上不该取决于哪个组件先构造、
// 新写的组件记没记得调——和 pluginlib 自己登记组件类用的是同一个办法。
//
// 顺序是有意的：同一个文件里的静态对象按声明顺序构造、逆序析构。route 先于 installed 构造，
// 所以接管时它已经可用；installed 先析构，所以 route 销毁时 SDK 已经不再指着它。
// 取后端也发生在这里而不是第一条日志上：那一条多半出在 USB 事件线程，
// "第一次用到才建后端、顺带起一条线程"不能落到它头上。
LibhcsLogRoute route{hcs_executor::process_log_backend()};
const sdk::ScopedSink installed{route};

} // namespace
} // namespace hcs_core::hardware
