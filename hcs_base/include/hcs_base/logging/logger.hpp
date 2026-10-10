#pragma once

#include <algorithm>
#include <array>
#include <bit>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <format>
#include <memory>
#include <optional>
#include <string_view>
#include <type_traits>
#include <utility>

#include "hcs_base/logging/backend.hpp"
#include "hcs_base/logging/record.hpp"
#include "hcs_base/thread/rt_attributes.hpp"

namespace hcs_log {

/// 日志的前半段：一个名字 + 往哪个 Backend 送。值类型，随便拷。
///
/// 两套入口，按**调用线程属于哪个域**选，写在调用点上一眼看得出来：
///
///   logger.warn("accept: {}", error.message());       // 尽力域：当场格式化，实参随意
///   logger.rt().warn("crc invalid, total {}", count); // 周期域 / IO 线程：延迟格式化
///
/// rt() 那一套标了 nonblocking，只收平凡可拷贝、非指针的实参（见 DeferredArgument），
/// 传一个 std::string 进去是编译错误而不是运行时的一次 malloc。手上已经有一段现成的文本
/// 要在那边打，用 rt().write()——它拷贝，装不下就截断。
///
/// 两套的格式串都在编译期按实参类型检查，而且**必须是字符串字面量**：
/// rt() 那一套只存它的地址，到日志线程才去读。
///
/// 高频事件别直接打：1 kHz 的回路里每拍一条，队列几百毫秒就满。用 Backoff 稀释，
/// 或者照旧"周期域计数、尽力域汇总"。
class Logger {
public:
    Logger(Backend& backend, std::string_view name) noexcept
        : backend_(&backend)
        , name_size_(static_cast<std::uint8_t>(std::min(name.size(), name_.size()))) {
        std::memcpy(name_.data(), name.data(), name_size_);
    }

    [[nodiscard]] std::string_view name() const noexcept { return {name_.data(), name_size_}; }

    // ── 尽力域 ────────────────────────────────────────────────────────────

    template <typename... Args>
    void debug(std::format_string<Args...> format, Args&&... args) const {
        log(Level::kDebug, format, std::forward<Args>(args)...);
    }
    template <typename... Args>
    void info(std::format_string<Args...> format, Args&&... args) const {
        log(Level::kInfo, format, std::forward<Args>(args)...);
    }
    template <typename... Args>
    void warn(std::format_string<Args...> format, Args&&... args) const {
        log(Level::kWarn, format, std::forward<Args>(args)...);
    }
    template <typename... Args>
    void error(std::format_string<Args...> format, Args&&... args) const {
        log(Level::kError, format, std::forward<Args>(args)...);
    }
    template <typename... Args>
    void fatal(std::format_string<Args...> format, Args&&... args) const {
        log(Level::kFatal, format, std::forward<Args>(args)...);
    }

    /// 当场格式化。绝大多数日志直接落在记录的定长载荷里，不分配；
    /// 装不下的（长的汇总行）再格式化一遍到堆上，不截断。
    template <typename... Args>
    void log(Level level, std::format_string<Args...> format, Args&&... args) const {
        if (level < backend_->threshold())
            return;

        // 实参可能被 forward 两次（装不下时还要再格式化一遍）。这是安全的：std::format 一族
        // 只把实参绑成引用去读，从不移走它们。
        Record record = header(level);
        const auto result = std::format_to_n(
            reinterpret_cast<char*>(record.payload.data()), record.payload.size(), format,
            std::forward<Args>(args)...);
        const auto size = static_cast<std::size_t>(result.size);
        if (size <= record.payload.size()) {
            record.payload_size = static_cast<std::uint16_t>(size);
            backend_->submit(record);
            return;
        }

        auto text = std::make_unique_for_overwrite<char[]>(size);
        std::format_to_n(text.get(), size, format, std::forward<Args>(args)...);
        submit_spilled(record, std::move(text), size);
    }

    /// 文本已经在手上时用（异常的 what()、别处生成好的报告），不经过格式化。
    void write(Level level, std::string_view text) const {
        if (level < backend_->threshold())
            return;

        Record record = header(level);
        if (text.size() <= record.payload.size()) {
            record.payload_size = static_cast<std::uint16_t>(text.size());
            if (!text.empty()) // 空视图的 data() 可以是空指针，而 memcpy 不收空指针
                std::memcpy(record.payload.data(), text.data(), text.size());
            backend_->submit(record);
            return;
        }

        auto copy = std::make_unique_for_overwrite<char[]>(text.size());
        std::memcpy(copy.get(), text.data(), text.size());
        submit_spilled(record, std::move(copy), text.size());
    }

    // ── 周期域 / IO 线程 ──────────────────────────────────────────────────

    class Realtime {
    public:
        template <DeferredArgument... Args>
        void debug(DeferredFormat<Args...> format, const Args&... args) const noexcept
            HCS_NONBLOCKING {
            logger_.defer(Level::kDebug, format.get(), args...);
        }
        template <DeferredArgument... Args>
        void info(DeferredFormat<Args...> format, const Args&... args) const noexcept
            HCS_NONBLOCKING {
            logger_.defer(Level::kInfo, format.get(), args...);
        }
        template <DeferredArgument... Args>
        void warn(DeferredFormat<Args...> format, const Args&... args) const noexcept
            HCS_NONBLOCKING {
            logger_.defer(Level::kWarn, format.get(), args...);
        }
        template <DeferredArgument... Args>
        void error(DeferredFormat<Args...> format, const Args&... args) const noexcept
            HCS_NONBLOCKING {
            logger_.defer(Level::kError, format.get(), args...);
        }
        template <DeferredArgument... Args>
        void fatal(DeferredFormat<Args...> format, const Args&... args) const noexcept
            HCS_NONBLOCKING {
            logger_.defer(Level::kFatal, format.get(), args...);
        }

        /// 文本已经在手上时用（别的库递过来的一行、定长缓冲里拼好的）：只有一次 memcpy。
        ///
        /// 和尽力域的 write() 不同，这里**会截断**：装不下的那一段上不了堆（这条线程不许分配），
        /// 所以只留下开头，结尾换成 "..." 表明后面还有。
        void write(Level level, std::string_view text) const noexcept HCS_NONBLOCKING {
            logger_.copy_truncated(level, text);
        }

    private:
        friend class Logger;
        explicit Realtime(const Logger& logger) noexcept
            : logger_(logger) {}

        const Logger& logger_;
    };

    [[nodiscard]] Realtime rt() const noexcept HCS_NONBLOCKING { return Realtime{*this}; }

private:
    static constexpr std::string_view kEllipsis = "..."; ///< 截断过的文本以它结尾

    [[nodiscard]] Record header(Level level) const noexcept HCS_NONBLOCKING {
        Record record;
        record.level = level;
        record.name = name_;
        record.name_size = name_size_;
        return record;
    }

    /// 堆上那段文本的所有权：入队成功就交给日志线程（它写完后释放），
    /// 队列满了则留在这里，随 text 一起释放。
    void submit_spilled(Record& record, std::unique_ptr<char[]> text, std::size_t size) const {
        record.set_spill(Record::Spill{.data = text.get(), .size = size});
        if (backend_->submit(record))
            (void)text.release();
    }

    /// 文本原样拷进记录的定长载荷，装不下就截断。
    void copy_truncated(Level level, std::string_view text) const noexcept HCS_NONBLOCKING {
        if (level < backend_->threshold())
            return;

        Record record = header(level);
        std::size_t size = text.size();
        const bool truncated = size > record.payload.size();
        if (truncated) {
            size = record.payload.size() - kEllipsis.size();
            // 别把一个 UTF-8 字符劈成两半：落在后续字节上就退到它的首字节之前。
            while (size > 0 && (static_cast<unsigned char>(text[size]) & 0xC0U) == 0x80U)
                --size;
        }

        if (size != 0)
            std::memcpy(record.payload.data(), text.data(), size);
        if (truncated) {
            std::memcpy(record.payload.data() + size, kEllipsis.data(), kEllipsis.size());
            size += kEllipsis.size();
        }
        record.payload_size = static_cast<std::uint16_t>(size);
        backend_->submit(record);
    }

    /// 只把实参按字节装进记录；文本由日志线程生成。
    template <DeferredArgument... Args>
    void defer(Level level, std::string_view format, const Args&... args) const noexcept
        HCS_NONBLOCKING {
        if (level < backend_->threshold())
            return;

        Record record = header(level);
        record.render = &Deferred<Args...>::render;
        record.format = format;
        record.payload_size = static_cast<std::uint16_t>(Deferred<Args...>::kSize);
        Deferred<Args...>::pack(record.payload.data(), args...);
        backend_->submit(record);
    }

    Backend* backend_;
    std::array<char, Record::kNameCapacity> name_{};
    std::uint8_t name_size_;
};

/// 给高频事件稀释用：第 1、2、4、8、… 次命中时返回累计次数，其余返回空。
///
///     if (const auto count = crc_errors_.hit())
///         logger.rt().warn("crc invalid, {} so far", *count);
///
/// 第一次立刻报，之后越来越稀，但不会完全沉默——线路一直坏着的话日志里一直有新的一行。
/// 不取时钟、不分配，周期域里可用。不是线程安全的：一个实例只归一条线程。
class Backoff {
public:
    [[nodiscard]] std::optional<std::uint64_t> hit() noexcept HCS_NONBLOCKING {
        ++count_;
        return std::has_single_bit(count_) ? std::optional{count_} : std::nullopt;
    }

    [[nodiscard]] std::uint64_t count() const noexcept { return count_; }

private:
    std::uint64_t count_ = 0;
};

/// 尽力域的按时间限流：距上次放行不足 period 就拦下。第一次总是放行。
///
///     if (link_fault_report_.ready())
///         logger().error("board faulted (link lost); its devices are offline");
///
/// 要取时钟，所以**不要**在周期域里用——那里用 Backoff。一个实例只归一条线程。
class Throttle {
public:
    explicit Throttle(std::chrono::steady_clock::duration period) noexcept
        : period_(period) {}

    [[nodiscard]] bool ready() noexcept {
        const auto now = std::chrono::steady_clock::now();
        if (last_ && now - *last_ < period_)
            return false;
        last_ = now;
        return true;
    }

private:
    std::chrono::steady_clock::duration period_;
    std::optional<std::chrono::steady_clock::time_point> last_;
};

} // namespace hcs_log
