// 接线器 + 组件语义的单测。
//
// 这个文件里没有 rclcpp::init、没有 pluginlib、没有参数服务器、没有线程 ——
// 直接 new 几个 Component、link 一次、手动跑几拍。改造前这些一条都做不到：
// 组件的名字来自一个静态全局，建图算法长在 Executor（一个 rclcpp::Node）身上，
// 接线跑完图就被吃掉了。「零测试」不是懒，是结构上写不了。

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include <hcs_sync/tick.hpp>

#include "hcs_executor/component.hpp"
#include "hcs_executor/wiring.hpp"
#include <hcs_utility/doorbell.hpp>

using hcs_executor::Component;
using hcs_executor::Linker;
using hcs_executor::Wiring;

namespace {

constexpr auto kPeriod = std::chrono::milliseconds{1};

/// 最小调度器：把组件收进来（含伙伴组件），link 一次，然后手动推拍。
/// 拍内的两步顺序刻意和 Executor::execute_update_iteration 一致：
/// **先闩存所有 1/z，再按拓扑序 update**。
class Harness {
public:
    template <typename T, typename... Args>
    std::shared_ptr<T> add(Args&&... args) {
        auto component = std::make_shared<T>(std::forward<Args>(args)...);
        collect(component);
        return component;
    }

    [[nodiscard]] std::expected<Wiring, hcs_executor::graph::BuildError> link() {
        auto wiring = Linker::link(components_);
        if (wiring)
            wiring_ = *wiring;
        return wiring;
    }

    void tick() {
        for (const auto& entry : wiring_.latches)
            entry.latch(entry.interface);

        const hcs_sync::Tick tick{
            .scheduled = start_ + kPeriod * sequence_,
            .dt = std::chrono::duration_cast<hcs_sync::Duration>(kPeriod),
            .sequence = sequence_};
        ++sequence_;

        for (auto* component : wiring_.updating_order)
            if (!component->failed())
                component->update(tick);
    }

    [[nodiscard]] std::vector<std::string> order() const {
        std::vector<std::string> names;
        names.reserve(wiring_.updating_order.size());
        for (const auto* component : wiring_.updating_order)
            names.push_back(component->get_component_name());
        return names;
    }

    [[nodiscard]] const std::vector<std::shared_ptr<Component>>& components() const {
        return components_;
    }

    /// 拍尾振铃，顺序和 Executor::thread_main 一致（在所有 update 之后）。
    void ring_tick_end() {
        for (auto* doorbell : wiring_.tick_end_doorbells)
            doorbell->ring();
    }

    [[nodiscard]] std::size_t doorbell_count() const {
        return wiring_.tick_end_doorbells.size();
    }

private:
    void collect(const std::shared_ptr<Component>& component) {
        components_.push_back(component);
        for (const auto& partner : component->partner_components())
            collect(partner);
    }

    std::vector<std::shared_ptr<Component>> components_;
    Wiring wiring_;
    hcs_sync::Timestamp start_ = hcs_sync::Clock::now();
    std::uint64_t sequence_ = 0;
};

class Source : public Component {
public:
    explicit Source(std::string name)
        : Component{std::move(name)} {
        register_output("/x", x_, 100.0);
    }

    void update(const hcs_sync::Tick&) override { *x_ = next; }

    double next = 0.0;
    OutputInterface<double> x_;
};

class Sink : public Component {
public:
    explicit Sink(std::string name)
        : Component{std::move(name)} {
        register_input("/x", x_);
    }

    void update(const hcs_sync::Tick&) override { seen = *x_; }

    double seen = -1.0;
    InputInterface<double> x_;
};

/// 读 /x 的 1/z 版本。
class DelayedSink : public Component {
public:
    explicit DelayedSink(std::string name)
        : Component{std::move(name)} {
        register_delayed_input("/x", x_);
    }

    void update(const hcs_sync::Tick&) override { seen = *x_; }

    double seen = -1.0;
    DelayedInput<double> x_{-42.0};
};

} // namespace

TEST(Wiring, InputSeesTheSameTicksValue) {
    Harness harness;
    auto source = harness.add<Source>("source");
    auto sink = harness.add<Sink>("sink");

    ASSERT_TRUE(harness.link().has_value());
    EXPECT_EQ(harness.order(), (std::vector<std::string>{"source", "sink"}));

    source->next = 3.0;
    harness.tick();
    // 普通输入是零拷贝直读上游存储：同一拍里上游写完，下游立刻看得见。
    EXPECT_DOUBLE_EQ(sink->seen, 3.0);
}

/// 1/z 的语义必须**与拓扑序无关**。这里 source 排在 delayed sink 前面、
/// 本拍早些时候就写了新值，sink 仍然只能看到上一拍的 —— 因为闩存发生在拍首。
/// 如果实现成「直接指向上游存储」，这个测试会读到本拍的新值而挂掉。
TEST(Wiring, DelayedInputIsExactlyOneTickBehind) {
    Harness harness;
    auto source = harness.add<Source>("source");
    auto sink = harness.add<DelayedSink>("sink");

    ASSERT_TRUE(harness.link().has_value());

    source->next = 1.0;
    harness.tick();
    // 首拍闩到的是 /x 注册时的初值（100.0），也就是「第 -1 拍的状态」。
    EXPECT_DOUBLE_EQ(sink->seen, 100.0);

    source->next = 2.0;
    harness.tick();
    EXPECT_DOUBLE_EQ(sink->seen, 1.0);

    source->next = 3.0;
    harness.tick();
    EXPECT_DOUBLE_EQ(sink->seen, 2.0);
}

TEST(Wiring, UnboundDelayedInputKeepsItsInitialValue) {
    class OptionalDelayedSink : public Component {
    public:
        OptionalDelayedSink()
            : Component{"lone"} {
            register_delayed_input("/nobody", x_, /*required=*/false);
        }
        void update(const hcs_sync::Tick&) override { seen = *x_; }
        double seen = 0.0;
        DelayedInput<double> x_{-42.0};
    };

    Harness lone;
    auto component = lone.add<OptionalDelayedSink>();

    ASSERT_TRUE(lone.link().has_value());
    EXPECT_FALSE(component->x_.has_provider());
    lone.tick();
    EXPECT_DOUBLE_EQ(component->seen, -42.0);
}

/// 代数环：hardware 读 pid 的输出，pid 读 hardware 的输出。
/// 改造前唯一的解法是把 hardware 拆成 Status / Command 两个 Component；
/// 这里用一条显式的 1/z 边把环打开，硬件包保持一个组件。
TEST(Wiring, DelayedInputBreaksAlgebraicLoopWithoutSplittingTheComponent) {
    class Hardware : public Component {
    public:
        Hardware()
            : Component{"hardware"} {
            register_output("/velocity", velocity_, 0.0);
            register_delayed_input("/torque", torque_);
        }
        void update(const hcs_sync::Tick&) override {
            // 一阶积分：这一拍的速度由**上一拍**发出去的力矩决定，物理上本来就是这样。
            *velocity_ = *velocity_ + *torque_;
            applied = *torque_;
        }
        double applied = 0.0;
        OutputInterface<double> velocity_;
        DelayedInput<double> torque_{0.0};
    };

    class Pid : public Component {
    public:
        Pid()
            : Component{"pid"} {
            register_input("/velocity", velocity_);
            register_output("/torque", torque_, 0.0);
        }
        void update(const hcs_sync::Tick&) override { *torque_ = 10.0 - *velocity_; }
        InputInterface<double> velocity_;
        OutputInterface<double> torque_;
    };

    Harness harness;
    auto hardware = harness.add<Hardware>();
    auto pid = harness.add<Pid>();

    ASSERT_TRUE(harness.link().has_value()) << "1/z 应当把环打开";
    EXPECT_EQ(harness.order(), (std::vector<std::string>{"hardware", "pid"}));

    harness.tick();
    EXPECT_DOUBLE_EQ(hardware->applied, 0.0); // 首拍：上一拍没有力矩
    EXPECT_DOUBLE_EQ(*pid->torque_, 10.0);

    harness.tick();
    EXPECT_DOUBLE_EQ(hardware->applied, 10.0); // 第二拍才收到第一拍算出的力矩
    EXPECT_DOUBLE_EQ(*hardware->velocity_, 10.0);
}

/// ARCH-2：可选输入没人提供时必须真的绑上一个默认值。
/// 改造前这里留 nullptr，而 operator* 是无条件解引用 —— 读一下就是 UB。
TEST(Wiring, OptionalInputWithoutProviderIsBoundToADefault) {
    class OptionalSink : public Component {
    public:
        OptionalSink()
            : Component{"sink"} {
            register_input("/nobody", x_, /*required=*/false);
        }
        void update(const hcs_sync::Tick&) override { seen = *x_; }
        double seen = -1.0;
        InputInterface<double> x_;
    };

    Harness harness;
    auto sink = harness.add<OptionalSink>();
    ASSERT_TRUE(harness.link().has_value());

    EXPECT_TRUE(sink->x_.ready());
    // ready() 是「有存储可读」，has_provider() 是「真的接上了上游」。
    // 组件想据此选自己的 fallback 时，要看的是后者。
    EXPECT_FALSE(sink->x_.has_provider());
    harness.tick();
    EXPECT_DOUBLE_EQ(sink->seen, 0.0);
}

/// 可重入：同一组组件 link 两次得到同样的结果，而且第二次不会因为
/// 「这个接口已经绑过了」而抛。确定性回放要重新接线，单测更是每个用例都要来一遍。
TEST(Wiring, LinkIsReentrant) {
    Harness harness;
    auto source = harness.add<Source>("source");
    auto sink = harness.add<Sink>("sink");
    // 可选输入那条路径会分配存储，重复绑定最容易在这里炸。
    class OptionalSink : public Component {
    public:
        OptionalSink()
            : Component{"optional"} {
            register_input("/nobody", x_, /*required=*/false);
        }
        void update(const hcs_sync::Tick&) override {}
        InputInterface<double> x_;
    };
    harness.add<OptionalSink>();

    ASSERT_TRUE(harness.link().has_value());
    const auto first = harness.order();

    ASSERT_TRUE(harness.link().has_value()) << "第二次 link 不该失败";
    EXPECT_EQ(harness.order(), first);

    source->next = 5.0;
    harness.tick();
    EXPECT_DOUBLE_EQ(sink->seen, 5.0);
}

namespace {

/// 硬件包的经典形状：Status / Command 两个 Component 共享同一份成员。
class Board : public Component {
public:
    Board()
        : Component{"board"}
        , command_(create_partner_component<Command>("board_command", *this)) {
        register_output("/torque", torque_, 0.0);
    }

    void update(const hcs_sync::Tick&) override {
        ++status_ticks;
        *torque_ = 7.0;
    }

    class Command : public Component {
    public:
        explicit Command(Board& board)
            : board_(board) {}
        void update(const hcs_sync::Tick&) override { ++board_.command_ticks; }

    private:
        Board& board_;
    };

    int status_ticks = 0;
    int command_ticks = 0;
    std::shared_ptr<Command> command_;
    OutputInterface<double> torque_;
};

} // namespace

/// 伙伴组件的名字来自 create_partner_component 里的 NameScope。
TEST(Wiring, PartnerComponentGetsItsScopedName) {
    Harness harness;
    auto board = harness.add<Board>();
    EXPECT_EQ(board->get_component_name(), "board");
    EXPECT_EQ(board->command_->get_component_name(), "board_command");
    // RAII 还原：造完伙伴之后，"待命名"那一格必须回到进入前的样子，
    // 而不是停在 "board_command" 上。改造前那个静态全局只设不还原。
    EXPECT_TRUE(hcs_executor::detail::pending_component_name().empty());
}

/// 失效隔离的两条硬要求：输出复位回注册时的初值、整个伙伴组一起停。
///
/// 只停一半是假隔离 —— Command::update() 直接回调 Status 那半的方法，
/// 留着它就等于那个已经出问题的组件每拍照样在跑自己的代码。
TEST(Wiring, IsolateResetsOutputsAndStopsTheWholePartnerGroup) {
    Harness harness;
    auto board = harness.add<Board>();
    ASSERT_TRUE(harness.link().has_value());

    harness.tick();
    ASSERT_EQ(board->status_ticks, 1);
    ASSERT_EQ(board->command_ticks, 1);
    ASSERT_DOUBLE_EQ(*board->torque_, 7.0);

    std::vector<std::uint32_t> newly_failed;
    newly_failed.reserve(harness.components().size());
    std::uint32_t failed_count = 0;

    // 从 command 那一半触发，验证隔离沿 partner_root_ 往上找到组根。
    Linker::isolate(board->command_.get(), newly_failed, failed_count);

    EXPECT_TRUE(board->failed());
    EXPECT_TRUE(board->command_->failed());
    EXPECT_EQ(failed_count, 2u);
    EXPECT_EQ(newly_failed.size(), 2u);
    // 电机指令必须回到安全值，而不是停在最后一条指令上继续转。
    EXPECT_DOUBLE_EQ(*board->torque_, 0.0);

    harness.tick();
    EXPECT_EQ(board->status_ticks, 1) << "失效组件不该再被调度";
    EXPECT_EQ(board->command_ticks, 1);
}

TEST(Wiring, IsolateIsIdempotent) {
    Harness harness;
    auto board = harness.add<Board>();
    ASSERT_TRUE(harness.link().has_value());

    std::vector<std::uint32_t> newly_failed;
    newly_failed.reserve(harness.components().size());
    std::uint32_t failed_count = 0;

    Linker::isolate(board.get(), newly_failed, failed_count);
    Linker::isolate(board.get(), newly_failed, failed_count);
    // 同一个组件每拍都可能再抛一次，计数不能跟着涨，名字也不该重复报。
    EXPECT_EQ(failed_count, 2u);
    EXPECT_EQ(newly_failed.size(), 2u);
}

TEST(Wiring, NameScopeRestoresThePreviousName) {
    using NameScope = Component::NameScope;
    ASSERT_TRUE(hcs_executor::detail::pending_component_name().empty());
    {
        NameScope outer{"outer"};
        EXPECT_EQ(hcs_executor::detail::pending_component_name(), "outer");
        {
            NameScope inner{"inner"};
            EXPECT_EQ(hcs_executor::detail::pending_component_name(), "inner");
        }
        EXPECT_EQ(hcs_executor::detail::pending_component_name(), "outer");
    }
    EXPECT_TRUE(hcs_executor::detail::pending_component_name().empty());
}

TEST(Wiring, DuplicateRegistrationInOneComponentThrows) {
    class Doubled : public Component {
    public:
        Doubled()
            : Component{"doubled"} {
            register_output("/x", a_, 0.0);
            register_output("/x", b_, 0.0); // 同一个组件里重名
        }
        void update(const hcs_sync::Tick&) override {}
        OutputInterface<double> a_;
        OutputInterface<double> b_;
    };

    EXPECT_THROW({ Doubled{}; }, std::runtime_error);
}

/// 拍尾门铃：只有重写了 tick_end_doorbell() 的组件才会被收进列表，
/// 而且列表在 link() 时一次定型 —— RT 线程不许在运行期改它。
TEST(Wiring, OnlyComponentsThatWantADoorbellGetOne) {
    class Ringer : public Component {
    public:
        Ringer()
            : Component{"ringer"} {}
        void update(const hcs_sync::Tick&) override {}
        hcs_utility::Doorbell* tick_end_doorbell() override { return &bell_; }
        hcs_utility::Doorbell bell_;
    };

    Harness harness;
    harness.add<Source>("source"); // 不要门铃
    auto ringer = harness.add<Ringer>();
    ASSERT_TRUE(harness.link().has_value());

    EXPECT_EQ(harness.doorbell_count(), 1u);
    EXPECT_EQ(ringer->bell_.rings(), 0u);

    harness.tick();
    harness.ring_tick_end();
    harness.tick();
    harness.ring_tick_end();
    EXPECT_EQ(ringer->bell_.rings(), 2u);
    // 拍尾振的铃在下一次 wait 时必须拿得到，且三次并成一次（见 Doorbell 的单测）。
    EXPECT_TRUE(ringer->bell_.wait_for(std::chrono::milliseconds{0}));
}
