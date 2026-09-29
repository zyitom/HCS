#pragma once

#include <array>
#include <cstdint>
#include <optional>

#include <hcs_link/autoaim.hpp>
#include <hcs_link/channel.hpp>
#include <hcs_utility/rt_attributes.hpp>

namespace hcs_core::controller::auto_aim {

/// 每拍一次：从视觉的命令通道取最新一条，校验、判陈旧、外推到本拍。
///
/// 纯逻辑，不碰图也不碰线程：单测直接喂一个 Reader 和时刻。
/// 陈旧只按本端自己的拍数判——对端写的时间戳只用来外推，不用来判断对端还活着没有。
class AimFollower {
public:
    using Command = hcs_link::autoaim::AimCommand;
    using CommandReader = hcs_link::Reader<Command>;

    struct Config {
        std::uint32_t stale_ticks = 50;                  ///< 连续这么多拍没有新命令就停用
        std::int64_t max_extrapolation_ns = 60'000'000; ///< 外推最多这么远
        hcs_link::autoaim::Limits limits{};
    };

    struct Decision {
        bool control = false;
        bool shoot = false;
        std::array<double, 3> direction{}; ///< OdomImu 世界系单位向量，control 为 true 时有效
    };

    explicit AimFollower(const Config& config) noexcept
        : config_{config}
        , ticks_since_fresh_{config.stale_ticks} {}

    /// reader 可以为空（没有视觉连着）。换了一个 reader 就是换了会话，旧命令作废。
    [[nodiscard]] Decision update(const CommandReader* reader, std::int64_t now_ns) noexcept
        HCS_NONBLOCKING {
        if (reader != reader_) {
            reader_ = reader;
            last_index_.reset();
            command_.reset();
            ticks_since_fresh_ = config_.stale_ticks;
        }

        std::optional<hcs_link::Sample<Command>> sample;
        if (reader_ != nullptr)
            sample = reader_->latest();

        if (sample && sample->index != last_index_) {
            last_index_ = sample->index;
            ticks_since_fresh_ = 0;
            if (hcs_link::autoaim::is_valid(sample->value, now_ns, config_.limits)) {
                command_ = sample->value;
            } else {
                command_.reset();
                ++rejected_;
            }
        } else if (ticks_since_fresh_ < config_.stale_ticks) {
            ++ticks_since_fresh_;
        }

        if (!command_ || ticks_since_fresh_ >= config_.stale_ticks
            || (command_->flags & Command::kHasTarget) == 0
            || now_ns - command_->t_ref_ns > config_.limits.max_age_ns)
            return {};

        const auto aim =
            hcs_link::autoaim::extrapolate(*command_, now_ns, config_.max_extrapolation_ns);
        return Decision{
            .control = true,
            .shoot = hcs_link::autoaim::fire_allowed(*command_, now_ns),
            .direction = hcs_link::autoaim::direction(aim),
        };
    }

    /// 被拒收的命令条数，累计值。
    [[nodiscard]] std::uint64_t rejected() const noexcept { return rejected_; }

private:
    Config config_;
    const CommandReader* reader_ = nullptr;
    std::optional<std::uint64_t> last_index_;
    std::optional<Command> command_;
    std::uint32_t ticks_since_fresh_;
    std::uint64_t rejected_ = 0;
};

} // namespace hcs_core::controller::auto_aim
