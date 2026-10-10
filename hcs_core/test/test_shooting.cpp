// 射击控制器（controller/shooting）：热量、摩擦轮、拨弹、推杆、弹速记录。
//
// 这些组件从 RMCS 移植过来时没有任何测试，逻辑又全是状态机和计时——正是"读代码看不出来、
// 上车才发现"的那一类。这里把每个组件的主要行为钉住：
//
//   - 安全态（拨杆 UNKNOWN 或双下）下所有给定都是 NaN；
//   - 计时按 Tick::dt 走（测试里一拍 1 ms，所以"200 ms"就是 200 拍）；
//   - 各个状态机在该转的时候转。
//
// 做法：每个被测组件接进一张小图，它要读的东西由 Source<T> 提供（测试直接写），它写出来的
// 东西由 Probe<T> 读。参数经 rclcpp 的参数文件给，和真车一样按组件名查。

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <functional>
#include <memory>
#include <numbers>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>

#include <hcs_base/channel/tick.hpp>
#include <hcs_executor/component.hpp>
#include <hcs_executor/wiring.hpp>
#include <hcs_msgs/keyboard.hpp>
#include <hcs_msgs/mouse.hpp>
#include <hcs_msgs/shoot_mode.hpp>
#include <hcs_msgs/switch.hpp>

#include "controller/shooting/shooting_statistics.hpp"

// 组件本体：包含进来就能直接构造。
#include "controller/shooting/bullet_feeder_controller_17mm.cpp"
#include "controller/shooting/bullet_feeder_controller_42mm.cpp"
#include "controller/shooting/friction_wheel_controller.cpp"
#include "controller/shooting/heat_controller.cpp"
#include "controller/shooting/hero_friction_wheel_controller.cpp"
#include "controller/shooting/hero_heat_controller.cpp"
#include "controller/shooting/putter_controller.cpp"
#include "controller/shooting/shooting_recorder.cpp"

namespace {

using namespace hcs_core::controller::shooting; // NOLINT(google-build-using-namespace)
using hcs_executor::Component;
using hcs_msgs::Keyboard;
using hcs_msgs::Mouse;
using hcs_msgs::ShootMode;
using hcs_msgs::Switch;

constexpr double kPi = std::numbers::pi;

std::string unique_name(const char* kind) {
    static int next = 0;
    return std::string{kind} + "_" + std::to_string(next++);
}

/// 按名字提供一个输出。测试在接线之后直接写 *value。
template <class T>
class Source : public Component {
public:
    Source(const std::string& name, T initial)
        : Component{unique_name("source")} {
        register_output(name, value, std::move(initial));
    }
    void update(const hcs_sync::Tick&) override {}

    OutputInterface<T> value;
};

/// 按名字读一个输出：*value。
template <class T>
class Probe : public Component {
public:
    explicit Probe(const std::string& name)
        : Component{unique_name("probe")} {
        register_input(name, value);
    }
    void update(const hcs_sync::Tick&) override {}

    InputInterface<T> value;
};

/// 一张小图：被测组件加上它的输入输出。
class Graph {
public:
    template <class T>
    std::shared_ptr<Source<T>> source(const std::string& name, T initial) {
        auto component = std::make_shared<Source<T>>(name, std::move(initial));
        components_.push_back(component);
        return component;
    }

    template <class T>
    std::shared_ptr<Probe<T>> probe(const std::string& name) {
        auto component = std::make_shared<Probe<T>>(name);
        components_.push_back(component);
        return component;
    }

    /// 被测组件：按名字构造（参数按这个名字从参数文件里查）。
    template <class T>
    std::shared_ptr<T> add(const std::string& name) {
        const Component::NameScope scope{name};
        auto component = std::make_shared<T>();
        components_.push_back(component);
        under_test_.push_back(component.get());
        return component;
    }

    void link() {
        const auto linked = hcs_executor::Linker::link(components_);
        ASSERT_TRUE(linked.has_value()) << linked.error().message;
        for (const auto& component : components_)
            component->before_updating();
    }

    /// 走 n 拍，一拍 1 ms。只跑被测组件：别的都是不动的替身。
    /// each_tick 在每一拍之后调，给"被控对象跟着给定走"这种事用。
    void tick(int n = 1, const std::function<void()>& each_tick = {}) {
        for (int i = 0; i < n; ++i) {
            now_ += std::chrono::milliseconds{1};
            const hcs_sync::Tick tick{
                .scheduled = now_, .dt = std::chrono::milliseconds{1}, .sequence = sequence_++};
            for (auto* component : under_test_)
                component->update(tick);
            if (each_tick)
                each_tick();
        }
    }

    /// 一拍一拍地走，直到 done() 为真；返回走了几拍。最多走 limit 拍（走满说明没等到）。
    int tick_until(const std::function<bool()>& done, int limit,
                   const std::function<void()>& each_tick = {}) {
        for (int ticks = 1; ticks <= limit; ++ticks) {
            tick(1, each_tick);
            if (done())
                return ticks;
        }
        ADD_FAILURE() << "condition not reached within " << limit << " ticks";
        return limit;
    }

private:
    std::vector<std::shared_ptr<Component>> components_;
    std::vector<Component*> under_test_;
    hcs_sync::Timestamp now_{std::chrono::seconds{100}};
    std::uint64_t sequence_ = 0;
};

/// 遥控器那几路：两个拨杆、鼠标、键盘。初值是"失联"（拨杆 UNKNOWN）。
struct Remote {
    explicit Remote(Graph& graph)
        : right{graph.source<Switch>("/remote/switch/right", Switch::UNKNOWN)}
        , left{graph.source<Switch>("/remote/switch/left", Switch::UNKNOWN)}
        , mouse{graph.source<Mouse>("/remote/mouse", Mouse::zero())}
        , keyboard{graph.source<Keyboard>("/remote/keyboard", Keyboard::zero())} {}

    void switches(Switch left_value, Switch right_value) {
        *left->value = left_value;
        *right->value = right_value;
    }

    std::shared_ptr<Source<Switch>> right, left;
    std::shared_ptr<Source<Mouse>> mouse;
    std::shared_ptr<Source<Keyboard>> keyboard;
};

} // namespace

// ── 热量 ───────────────────────────────────────────────────────────────────

// 参数：heat_per_shot = 10，reserved_heat = 20。
TEST(HeatController, AllowanceFollowsHeatAndCooling) {
    Graph graph;
    auto cooling = graph.source<std::int64_t>("/referee/shooter/cooling", 1);
    auto limit = graph.source<std::int64_t>("/referee/shooter/heat_limit", 100);
    auto fired = graph.source<bool>("/gimbal/bullet_fired", false);
    graph.add<HeatController>("heat");
    auto allowance =
        graph.probe<std::int64_t>("/gimbal/control_bullet_allowance/limited_by_heat");
    graph.link();

    graph.tick();
    EXPECT_EQ(*allowance->value, 8); // (100 - 0 - 20) / 10

    // 打一发：热量 +10，另外多记 10。
    *fired->value = true;
    graph.tick();
    *fired->value = false;
    EXPECT_EQ(*allowance->value, 6); // (100 - 20 - 20) / 10

    // 每拍冷却 1：10 拍之后多出一发的余量。
    graph.tick(10);
    EXPECT_EQ(*allowance->value, 7);

    // 上限掉下来（比如被扣了等级）：余量不会是负数。
    *limit->value = 10;
    graph.tick();
    EXPECT_EQ(*allowance->value, 0);
}

// 参数：heat_per_shot = 100，reserved_heat = 0。
TEST(HeroHeatController, CountsEachShotOnceHoweverLongTheSignalStaysHigh) {
    Graph graph;
    graph.source<std::int64_t>("/referee/shooter/cooling", 0);
    graph.source<std::int64_t>("/referee/shooter/heat_limit", 200);
    auto fired = graph.source<bool>("/gimbal/bullet_fired", false);
    graph.add<HeroHeatController>("hero_heat");
    auto allowance =
        graph.probe<std::int64_t>("/gimbal/control_bullet_allowance/limited_by_heat");
    auto heat = graph.probe<double>("/shoot/heat");
    graph.link();

    graph.tick();
    EXPECT_EQ(*allowance->value, 2);

    *fired->value = true;
    graph.tick(5); // 信号保持了 5 拍：只算一发
    EXPECT_DOUBLE_EQ(*heat->value, 100.0);
    EXPECT_EQ(*allowance->value, 1);

    *fired->value = false;
    graph.tick();
    *fired->value = true;
    graph.tick();
    EXPECT_DOUBLE_EQ(*heat->value, 200.0);
    EXPECT_EQ(*allowance->value, 0);
}

// ── 摩擦轮 ─────────────────────────────────────────────────────────────────

namespace {

/// 两个摩擦轮（工作转速 600，缓启停 0.1 s），以及"摩擦轮跟着给定走"的理想被控对象。
struct FrictionRig {
    explicit FrictionRig(const std::string& name)
        : remote{graph}
        , mouse_wheel{graph.source<double>("/remote/mouse/mouse_wheel", 0.0)}
        , knob{graph.source<double>("/remote/rotary_knob", 0.0)}
        , left_velocity{graph.source<double>("/gimbal/left_friction_wheel/velocity", 0.0)}
        , right_velocity{graph.source<double>("/gimbal/right_friction_wheel/velocity", 0.0)}
        , controller{graph.add<FrictionWheelController>(name)}
        , control{graph.probe<double>("/gimbal/left_friction_wheel/control_velocity")}
        , working{graph.probe<double>("/gimbal/left_friction_wheel/working_velocity")}
        , ready{graph.probe<bool>("/gimbal/friction_ready")}
        , jammed{graph.probe<bool>("/gimbal/friction_jammed")}
        , fired{graph.probe<bool>("/gimbal/bullet_fired")} {
        graph.link();
    }

    /// 摩擦轮的转速就是上一拍的给定。
    std::function<void()> follow() {
        return [this] {
            if (std::isfinite(*control->value))
                *left_velocity->value = *right_velocity->value = *control->value;
        };
    }

    /// 离开安全态，再用"左拨杆 中 → 上"把摩擦轮打开，等到转速升满。
    void spin_up() {
        remote.switches(Switch::MIDDLE, Switch::MIDDLE);
        graph.tick(1, follow());
        remote.switches(Switch::UP, Switch::MIDDLE);
        graph.tick_until([this] { return *ready->value; }, 200, follow());
    }

    Graph graph;
    Remote remote;
    std::shared_ptr<Source<double>> mouse_wheel, knob, left_velocity, right_velocity;
    std::shared_ptr<FrictionWheelController> controller;
    std::shared_ptr<Probe<double>> control, working;
    std::shared_ptr<Probe<bool>> ready, jammed, fired;
};

} // namespace

TEST(FrictionWheelController, SafeStateCommandsNothing) {
    FrictionRig rig{"friction"};
    rig.graph.tick(5);
    EXPECT_TRUE(std::isnan(*rig.control->value));
    EXPECT_FALSE(*rig.ready->value);

    // 双下也是安全态，哪怕之前已经转起来了。
    rig.spin_up();
    ASSERT_TRUE(*rig.ready->value);
    rig.remote.switches(Switch::DOWN, Switch::DOWN);
    rig.graph.tick();
    EXPECT_TRUE(std::isnan(*rig.control->value));
    EXPECT_FALSE(*rig.ready->value);
}

// 缓启动：0.1 s 从 0 升到工作转速，按 Tick::dt 走。升满之前不算就绪。
TEST(FrictionWheelController, SoftStartsOverTheConfiguredTime) {
    FrictionRig rig{"friction"};
    rig.remote.switches(Switch::MIDDLE, Switch::MIDDLE);
    rig.graph.tick(1, rig.follow());
    EXPECT_DOUBLE_EQ(*rig.control->value, 0.0) << "left the safe state, still switched off";

    rig.remote.switches(Switch::UP, Switch::MIDDLE); // 中 → 上：打开
    rig.graph.tick(50, rig.follow());
    EXPECT_NEAR(*rig.control->value, 300.0, 1e-6);
    EXPECT_FALSE(*rig.ready->value);

    rig.graph.tick(60, rig.follow());
    EXPECT_DOUBLE_EQ(*rig.control->value, 600.0);
    EXPECT_TRUE(*rig.ready->value);

    // 再拨一次：关掉，缓停。
    rig.remote.switches(Switch::MIDDLE, Switch::MIDDLE);
    rig.graph.tick(1, rig.follow());
    rig.remote.switches(Switch::UP, Switch::MIDDLE);
    rig.graph.tick(50, rig.follow());
    EXPECT_NEAR(*rig.control->value, 300.0, 6.0 + 1e-6);
    EXPECT_FALSE(*rig.ready->value);
}

// 出弹：主轮掉速再回升。掉得够多、而且确实低于工作转速一截，回升的那一拍报一发。
TEST(FrictionWheelController, DetectsAShotFromTheSpeedDipOfThePrimaryWheel) {
    FrictionRig rig{"friction"};
    rig.spin_up();
    rig.graph.tick(5, rig.follow());
    ASSERT_FALSE(*rig.fired->value);

    int shots = 0;
    for (const double velocity : {590.0, 580.0, 575.0, 600.0, 600.0}) {
        *rig.left_velocity->value = velocity;
        rig.graph.tick();
        shots += *rig.fired->value;
    }
    EXPECT_EQ(shots, 1);

    // 只抖了一下、没掉够：不算。
    shots = 0;
    for (const double velocity : {597.0, 595.0, 600.0, 600.0}) {
        *rig.left_velocity->value = velocity;
        rig.graph.tick();
        shots += *rig.fired->value;
    }
    EXPECT_EQ(shots, 0);
}

// 卡住：转速不到给定的一半，累计 200 ms 之后报卡住一拍、把摩擦轮关掉。
TEST(FrictionWheelController, ReportsAJamAndSwitchesOff) {
    FrictionRig rig{"friction"};
    rig.spin_up();

    *rig.left_velocity->value = 100.0; // 给定 600
    rig.graph.tick(200);
    EXPECT_FALSE(*rig.jammed->value);
    EXPECT_TRUE(*rig.ready->value) << "still counted as ready while the fault is being timed";

    rig.graph.tick();
    EXPECT_TRUE(*rig.jammed->value);
    EXPECT_FALSE(*rig.ready->value);

    rig.graph.tick();
    EXPECT_FALSE(*rig.jammed->value) << "reported for one cycle only";
    EXPECT_LT(*rig.control->value, 600.0) << "switched off: ramping down";
}

// 低速档（参数里给了 friction_velocities_low_mode = 400）：左拨杆 下、右拨杆 中 时把拨轮拨到顶切换。
TEST(FrictionWheelController, KnobTogglesTheLowSpeedProfile) {
    FrictionRig rig{"friction_low"};
    rig.remote.switches(Switch::DOWN, Switch::MIDDLE);
    rig.graph.tick();
    EXPECT_DOUBLE_EQ(*rig.working->value, 600.0);

    *rig.knob->value = -1.0;
    rig.graph.tick(3); // 拨轮一直顶着：只切一次
    EXPECT_DOUBLE_EQ(*rig.working->value, 400.0);

    *rig.knob->value = 0.0;
    rig.graph.tick();
    *rig.knob->value = -1.0;
    rig.graph.tick();
    EXPECT_DOUBLE_EQ(*rig.working->value, 600.0);
}

// Ctrl + F 的同时滚滚轮：每格 ±5，夹在 [低速档, 配置值] 之间；滚轮停下来之前只算一格。
TEST(FrictionWheelController, MouseWheelTrimsTheWorkingSpeed) {
    FrictionRig rig{"friction_low"};
    rig.remote.switches(Switch::MIDDLE, Switch::MIDDLE);
    Keyboard keyboard = Keyboard::zero();
    keyboard.ctrl = keyboard.f = true;
    *rig.remote.keyboard->value = keyboard;

    *rig.mouse_wheel->value = -0.01;
    rig.graph.tick(5);
    EXPECT_DOUBLE_EQ(*rig.working->value, 595.0) << "one notch, however long the wheel keeps moving";

    *rig.mouse_wheel->value = 0.0; // 停一下
    rig.graph.tick();
    *rig.mouse_wheel->value = -0.01;
    rig.graph.tick(2);
    EXPECT_DOUBLE_EQ(*rig.working->value, 590.0);

    // 往上调不超过配置值。
    for (int i = 0; i < 5; ++i) {
        *rig.mouse_wheel->value = 0.0;
        rig.graph.tick();
        *rig.mouse_wheel->value = 0.01;
        rig.graph.tick(2);
    }
    EXPECT_DOUBLE_EQ(*rig.working->value, 600.0);
}

// 英雄：F 键在两套转速之间切。参数：三个轮，第 0 档 500，第 1 档 300。
TEST(HeroFrictionWheelController, FKeySwitchesBetweenTheTwoProfiles) {
    Graph graph;
    Remote remote{graph};
    std::vector<std::shared_ptr<Source<double>>> velocities;
    for (const char* wheel : {"/gimbal/wheel0", "/gimbal/wheel1", "/gimbal/wheel2"})
        velocities.push_back(graph.source<double>(std::string{wheel} + "/velocity", 0.0));
    graph.add<HeroFrictionWheelController>("hero_friction");
    auto control = graph.probe<double>("/gimbal/wheel0/control_velocity");
    auto ready = graph.probe<bool>("/gimbal/friction_ready");
    auto profile_1 = graph.probe<bool>("/gimbal/friction_profile_1_active");
    graph.link();

    const auto follow = [&] {
        if (std::isfinite(*control->value))
            for (auto& velocity : velocities)
                *velocity->value = *control->value;
    };

    graph.tick();
    EXPECT_TRUE(std::isnan(*control->value)) << "safe state";

    remote.switches(Switch::MIDDLE, Switch::MIDDLE);
    graph.tick(1, follow);
    EXPECT_TRUE(std::isnan(*control->value)) << "not switched on: no setpoint at all";

    remote.switches(Switch::UP, Switch::MIDDLE);
    graph.tick_until([&] { return *ready->value; }, 200, follow);
    EXPECT_DOUBLE_EQ(*control->value, 500.0);
    EXPECT_FALSE(*profile_1->value);

    Keyboard keyboard = Keyboard::zero();
    keyboard.f = true;
    *remote.keyboard->value = keyboard;
    graph.tick(1, follow);
    EXPECT_TRUE(*profile_1->value);
    graph.tick(200, follow);
    EXPECT_DOUBLE_EQ(*control->value, 300.0);
}

// ── 17 mm 拨弹 ─────────────────────────────────────────────────────────────

namespace {

/// 参数：一圈 8 发，射频 20 / 10 发每秒，退弹 10 发每秒 50 ms，深退 15 发每秒 200 ms，
/// 单发最多转 2 s。
struct Feeder17Rig {
    static constexpr double kAnglePerBullet = 2 * kPi / 8;
    static constexpr double kWorkingVelocity = kAnglePerBullet * 20;
    static constexpr double kSafeVelocity = kAnglePerBullet * 10;
    static constexpr double kEjectVelocity = -kAnglePerBullet * 10;
    static constexpr double kDeepEjectVelocity = -kAnglePerBullet * 15;

    Feeder17Rig()
        : remote{graph}
        , friction_ready{graph.source<bool>("/gimbal/friction_ready", true)}
        , bullet_fired{graph.source<bool>("/gimbal/bullet_fired", false)}
        , allowance{graph.source<std::int64_t>(
              "/gimbal/control_bullet_allowance/limited_by_heat", 5)}
        , velocity{graph.source<double>("/gimbal/bullet_feeder/velocity", 0.0)}
        , controller{graph.add<BulletFeederController17mm>("feeder17")}
        , control{graph.probe<double>("/gimbal/bullet_feeder/control_velocity")}
        , mode{graph.probe<ShootMode>("/gimbal/shooter/mode")} {
        graph.link();
    }

    void hold_left_button(bool down) {
        Mouse mouse = Mouse::zero();
        mouse.left = down;
        *remote.mouse->value = mouse;
    }

    /// 拨弹盘跟着给定走（正转时）。
    std::function<void()> follow() {
        return [this] { *velocity->value = std::max(0.0, *control->value); };
    }

    Graph graph;
    Remote remote;
    std::shared_ptr<Source<bool>> friction_ready, bullet_fired;
    std::shared_ptr<Source<std::int64_t>> allowance;
    std::shared_ptr<Source<double>> velocity;
    std::shared_ptr<BulletFeederController17mm> controller;
    std::shared_ptr<Probe<double>> control;
    std::shared_ptr<Probe<ShootMode>> mode;
};

} // namespace

TEST(BulletFeederController17mm, SafeStateCommandsNothing) {
    Feeder17Rig rig;
    rig.hold_left_button(true);
    rig.graph.tick(3);
    EXPECT_TRUE(std::isnan(*rig.control->value));

    rig.remote.switches(Switch::DOWN, Switch::DOWN);
    rig.graph.tick();
    EXPECT_TRUE(std::isnan(*rig.control->value));
}

// 连发：按着左键就拨。余量只剩一发时用慢的那个射频；没有余量、摩擦轮没就绪都不拨。
TEST(BulletFeederController17mm, FeedsWhileTheTriggerIsHeldAndHeatAllows) {
    Feeder17Rig rig;
    rig.remote.switches(Switch::MIDDLE, Switch::MIDDLE);
    rig.graph.tick(1, rig.follow());
    EXPECT_DOUBLE_EQ(*rig.control->value, 0.0);
    EXPECT_EQ(*rig.mode->value, ShootMode::AUTOMATIC);

    // 按下的那一拍有一个"单发"的触发，但连发模式下看的是按没按着。
    rig.hold_left_button(true);
    rig.graph.tick(10, rig.follow());
    EXPECT_DOUBLE_EQ(*rig.control->value, Feeder17Rig::kWorkingVelocity);

    *rig.allowance->value = 1;
    rig.graph.tick(1, rig.follow());
    EXPECT_DOUBLE_EQ(*rig.control->value, Feeder17Rig::kSafeVelocity);

    *rig.allowance->value = 0;
    rig.graph.tick(1, rig.follow());
    EXPECT_DOUBLE_EQ(*rig.control->value, 0.0);

    *rig.allowance->value = 5;
    *rig.friction_ready->value = false;
    rig.graph.tick(1, rig.follow());
    EXPECT_DOUBLE_EQ(*rig.control->value, 0.0);

    *rig.friction_ready->value = true;
    rig.hold_left_button(false);
    rig.graph.tick(1, rig.follow());
    EXPECT_DOUBLE_EQ(*rig.control->value, 0.0);
}

// 自瞄开着（鼠标右键）时，打不打听自瞄的，不听左键。
TEST(BulletFeederController17mm, AutoAimDecidesWhenAimingIsEnabled) {
    Graph graph;
    Remote remote{graph};
    graph.source<bool>("/gimbal/friction_ready", true);
    graph.source<bool>("/gimbal/bullet_fired", false);
    graph.source<std::int64_t>("/gimbal/control_bullet_allowance/limited_by_heat", 5);
    graph.source<double>("/gimbal/bullet_feeder/velocity", 0.0);
    auto should_shoot = graph.source<bool>("/auto_aim/should_shoot", false);
    graph.add<BulletFeederController17mm>("feeder17");
    auto control = graph.probe<double>("/gimbal/bullet_feeder/control_velocity");
    graph.link();

    remote.switches(Switch::MIDDLE, Switch::MIDDLE);
    Mouse mouse = Mouse::zero();
    mouse.left = mouse.right = true;
    *remote.mouse->value = mouse;
    graph.tick(3);
    EXPECT_DOUBLE_EQ(*control->value, 0.0) << "left button held, but auto aim says no";

    *should_shoot->value = true;
    graph.tick();
    EXPECT_DOUBLE_EQ(*control->value, Feeder17Rig::kWorkingVelocity);
}

// 卡弹：给着转速却转不到一半，持续 500 ms 判卡弹，反转退弹；连着卡第三次起退得更狠。
TEST(BulletFeederController17mm, EjectsWhenJammedAndHarderFromTheThirdTime) {
    Feeder17Rig rig;
    rig.remote.switches(Switch::MIDDLE, Switch::MIDDLE);
    rig.hold_left_button(true);
    rig.graph.tick(); // 给定从 0 变成工作转速

    // 拨弹盘一直不转。
    const int until_first_jam =
        rig.graph.tick_until([&] { return *rig.control->value < 0.0; }, 600);
    EXPECT_EQ(until_first_jam, 501);
    EXPECT_DOUBLE_EQ(*rig.control->value, Feeder17Rig::kEjectVelocity);

    // 退弹 50 ms（判卡弹的那一拍算第 1 拍），然后重新正转。
    const int eject_ticks = rig.graph.tick_until([&] { return *rig.control->value > 0.0; }, 100);
    EXPECT_EQ(eject_ticks, 50);

    // 第二次卡：还是普通退弹。
    rig.graph.tick_until([&] { return *rig.control->value < 0.0; }, 600);
    EXPECT_DOUBLE_EQ(*rig.control->value, Feeder17Rig::kEjectVelocity);
    rig.graph.tick_until([&] { return *rig.control->value > 0.0; }, 100);

    // 第三次卡：深退，200 ms。
    rig.graph.tick_until([&] { return *rig.control->value < 0.0; }, 600);
    EXPECT_DOUBLE_EQ(*rig.control->value, Feeder17Rig::kDeepEjectVelocity);
    const int deep_eject_ticks =
        rig.graph.tick_until([&] { return *rig.control->value > 0.0; }, 300);
    EXPECT_EQ(deep_eject_ticks, 200);
}

// 稳定转了 500 ms 之后突然转不动：不用再等 500 ms，立刻判卡弹。
TEST(BulletFeederController17mm, AStallAfterRunningSmoothlyIsAJamAtOnce) {
    Feeder17Rig rig;
    rig.remote.switches(Switch::MIDDLE, Switch::MIDDLE);
    rig.hold_left_button(true);
    rig.graph.tick(600, rig.follow());
    ASSERT_DOUBLE_EQ(*rig.control->value, Feeder17Rig::kWorkingVelocity);

    *rig.velocity->value = 0.0;
    rig.graph.tick();
    EXPECT_DOUBLE_EQ(*rig.control->value, Feeder17Rig::kEjectVelocity);
}

// 左拨杆拨到 下：打一发。转到出弹信号来了为止；等不到的话最多转 2 s。
TEST(BulletFeederController17mm, SwitchEdgeFiresOneShot) {
    Feeder17Rig rig;
    rig.remote.switches(Switch::MIDDLE, Switch::MIDDLE);
    rig.graph.tick(1, rig.follow());

    rig.remote.switches(Switch::DOWN, Switch::MIDDLE);
    rig.graph.tick(1, rig.follow());
    EXPECT_EQ(*rig.mode->value, ShootMode::SINGLE);
    EXPECT_DOUBLE_EQ(*rig.control->value, Feeder17Rig::kSafeVelocity) << "one shot: slow feed";

    rig.graph.tick(20, rig.follow());
    EXPECT_GT(*rig.control->value, 0.0);

    *rig.bullet_fired->value = true; // 打出去了
    rig.graph.tick(1, rig.follow());
    *rig.bullet_fired->value = false;
    EXPECT_DOUBLE_EQ(*rig.control->value, 0.0);

    // 单发模式只持续 500 ms，之后拨杆还在 下 就是连发的"按着"。
    rig.graph.tick(500, rig.follow());
    EXPECT_EQ(*rig.mode->value, ShootMode::AUTOMATIC);
    EXPECT_DOUBLE_EQ(*rig.control->value, Feeder17Rig::kWorkingVelocity);
}

// ── 42 mm 拨弹 ─────────────────────────────────────────────────────────────

namespace {

/// 六格拨盘，以及一个"力矩让转速变、转速让角度变"的被控对象。
struct Feeder42Rig {
    static constexpr double kAnglePerBullet = 2 * kPi / 6;
    static constexpr double kZeroPoint = 0.58;

    Feeder42Rig()
        : remote{graph}
        , friction_ready{graph.source<bool>("/gimbal/friction_ready", true)}
        , allowance{graph.source<std::int64_t>(
              "/gimbal/control_bullet_allowance/limited_by_heat", 5)}
        , angle{graph.source<double>("/gimbal/bullet_feeder/angle", kZeroPoint + 0.5 * kAnglePerBullet)}
        , velocity{graph.source<double>("/gimbal/bullet_feeder/velocity", 0.0)}
        , controller{graph.add<BulletFeederController42mm>("feeder42")}
        , torque{graph.probe<double>("/gimbal/bullet_feeder/control_torque")} {
        graph.source<bool>("/gimbal/bullet_fired", false);
        graph.link();
    }

    /// 转速一拍之内跟上速度环的给定（速度环 kp = 50，这里取它的倒数），角度是转速的积分。
    std::function<void()> plant() {
        return [this] {
            if (!std::isfinite(*torque->value))
                return;
            *velocity->value += *torque->value / 50.0;
            *angle->value += *velocity->value * 0.001;
        };
    }

    void click() {
        Mouse mouse = Mouse::zero();
        mouse.left = true;
        *remote.mouse->value = mouse;
        graph.tick(1, plant());
        *remote.mouse->value = Mouse::zero();
    }

    Graph graph;
    Remote remote;
    std::shared_ptr<Source<bool>> friction_ready;
    std::shared_ptr<Source<std::int64_t>> allowance;
    std::shared_ptr<Source<double>> angle, velocity;
    std::shared_ptr<BulletFeederController42mm> controller;
    std::shared_ptr<Probe<double>> torque;
};

} // namespace

TEST(BulletFeederController42mm, SafeStateCommandsNothing) {
    Feeder42Rig rig;
    rig.graph.tick(3);
    EXPECT_TRUE(std::isnan(*rig.torque->value));
}

// 点一下打一发：拨盘先推过整格多一点把弹丸送出去，再停到下一格的半格位置等着。
// 一发下来正好走过一格。
TEST(BulletFeederController42mm, OneClickAdvancesExactlyOneSlot) {
    Feeder42Rig rig;
    rig.remote.switches(Switch::MIDDLE, Switch::MIDDLE);
    rig.graph.tick(50, rig.plant());
    const double start = *rig.angle->value;
    EXPECT_NEAR(start, Feeder42Rig::kZeroPoint + 0.5 * Feeder42Rig::kAnglePerBullet, 1e-6)
        << "holds its position until fired";

    rig.click();
    rig.graph.tick(4000, rig.plant());
    EXPECT_NEAR(*rig.angle->value - start, Feeder42Rig::kAnglePerBullet, 0.02);
    EXPECT_NEAR(*rig.velocity->value, 0.0, 0.01);

    rig.click();
    rig.graph.tick(4000, rig.plant());
    EXPECT_NEAR(*rig.angle->value - start, 2 * Feeder42Rig::kAnglePerBullet, 0.02);
}

// 没有热量余量、或者摩擦轮没就绪：点了也不动。
TEST(BulletFeederController42mm, DoesNotFireWithoutHeatAllowanceOrFriction) {
    Feeder42Rig rig;
    rig.remote.switches(Switch::MIDDLE, Switch::MIDDLE);
    rig.graph.tick(10, rig.plant());
    const double start = *rig.angle->value;

    *rig.allowance->value = 0;
    rig.click();
    rig.graph.tick(1000, rig.plant());
    EXPECT_NEAR(*rig.angle->value, start, 1e-3);

    *rig.allowance->value = 5;
    *rig.friction_ready->value = false;
    rig.click();
    rig.graph.tick(1000, rig.plant());
    EXPECT_NEAR(*rig.angle->value, start, 1e-3);
}

// 卡弹：拨盘顶死，力矩在 300 以上顶满 1 s，然后反转 500 ms、松开 500 ms。
TEST(BulletFeederController42mm, BacksOffWhenJammed) {
    Feeder42Rig rig;
    rig.remote.switches(Switch::MIDDLE, Switch::MIDDLE);
    rig.graph.tick(10, rig.plant());
    rig.click();

    // 拨盘不动：角度环饱和在 1.0，速度环的积分一路攒上去，力矩越来越大。
    *rig.velocity->value = 0.0;
    rig.graph.tick_until([&] { return *rig.torque->value >= 300.0; }, 200);

    const int until_backoff = rig.graph.tick_until([&] { return *rig.torque->value < 0.0; }, 1200);
    EXPECT_EQ(until_backoff, 1001) << "1 s of sustained stall, then the back-off begins";

    const int reversing = rig.graph.tick_until([&] { return *rig.torque->value == 0.0; }, 600);
    EXPECT_EQ(reversing, 499);
    rig.graph.tick(499);
    EXPECT_DOUBLE_EQ(*rig.torque->value, 0.0) << "released for the second half of the cool-down";
}

// ── 推杆 ───────────────────────────────────────────────────────────────────

namespace {

/// 参数：两个速度环都是纯比例，kp = 1。于是力矩 = 目标速度 - 实际速度，一眼看得出在干什么。
struct PutterRig {
    PutterRig()
        : remote{graph}
        , friction_ready{graph.source<bool>("/gimbal/friction_ready", true)}
        , allowance{graph.source<std::int64_t>(
              "/gimbal/control_bullet_allowance/limited_by_heat", 5)}
        , photoelectric{graph.source<bool>("/gimbal/photoelectric_sensor", true)}
        , feeder_velocity{graph.source<double>("/gimbal/bullet_feeder/velocity", 0.0)}
        , putter_velocity{graph.source<double>("/gimbal/putter/velocity", 0.0)}
        , controller{graph.add<PutterController>("putter")}
        , feeder_torque{graph.probe<double>("/gimbal/bullet_feeder/control_torque")}
        , putter_torque{graph.probe<double>("/gimbal/putter/control_torque")}
        , preloaded_ready{graph.probe<bool>("/gimbal/shooter/preloaded_ready")} {
        graph.source<double>("/gimbal/bullet_feeder/angle", 0.0);
        graph.source<bool>("/gimbal/grayscale_sensor", false);
        graph.source<bool>("/gimbal/bullet_fired", false);
        graph.source<double>("/gimbal/putter/angle", 0.0);
        graph.link();
    }

    void click() {
        Mouse mouse = Mouse::zero();
        mouse.left = true;
        *remote.mouse->value = mouse;
        graph.tick();
        *remote.mouse->value = Mouse::zero();
    }

    /// 走完"推杆退到底"和"弹丸上膛"，停在等开火的状态。
    void preload() {
        remote.switches(Switch::MIDDLE, Switch::MIDDLE);
        graph.tick_until([this] { return *preloaded_ready->value; }, 1000);
    }

    Graph graph;
    Remote remote;
    std::shared_ptr<Source<bool>> friction_ready;
    std::shared_ptr<Source<std::int64_t>> allowance;
    std::shared_ptr<Source<bool>> photoelectric;
    std::shared_ptr<Source<double>> feeder_velocity, putter_velocity;
    std::shared_ptr<PutterController> controller;
    std::shared_ptr<Probe<double>> feeder_torque, putter_torque;
    std::shared_ptr<Probe<bool>> preloaded_ready;
};

} // namespace

TEST(PutterController, SafeStateCommandsNothing) {
    PutterRig rig;
    rig.graph.tick(3);
    EXPECT_TRUE(std::isnan(*rig.feeder_torque->value));
    EXPECT_TRUE(std::isnan(*rig.putter_torque->value));
}

// 离开安全态：推杆先退到底（堵转 50 ms 判定到位），之后只留一个很小的回拉力。
TEST(PutterController, RetractsThePutterUntilItStallsThenHoldsIt) {
    PutterRig rig;
    *rig.friction_ready->value = false; // 只看推杆
    rig.remote.switches(Switch::MIDDLE, Switch::MIDDLE);

    rig.graph.tick();
    EXPECT_DOUBLE_EQ(*rig.putter_torque->value, -80.0) << "driving back at the retract speed";

    // 推杆还在动的时候不算堵转。
    *rig.putter_velocity->value = -30.0;
    rig.graph.tick(100);
    EXPECT_DOUBLE_EQ(*rig.putter_torque->value, -50.0);

    *rig.putter_velocity->value = 0.0; // 到底了
    const int stalled_for =
        rig.graph.tick_until([&] { return *rig.putter_torque->value == -0.02; }, 100);
    EXPECT_EQ(stalled_for, 51);
    EXPECT_DOUBLE_EQ(*rig.feeder_torque->value, 0.0) << "friction wheels not ready: feeder idle";
}

// 上膛：拨弹盘转到顶住（堵转 150 ms）。光电被挡住 → 弹丸到位；然后反转一小段松开。
TEST(PutterController, PreloadsUntilTheFeederLocksAgainstTheBullet) {
    PutterRig rig;
    rig.remote.switches(Switch::MIDDLE, Switch::MIDDLE);
    rig.graph.tick(50); // 推杆初始化：堵转 50 ms
    rig.graph.tick();
    EXPECT_DOUBLE_EQ(*rig.feeder_torque->value, 5.0) << "feeding at the preload speed";

    // 顶住超过 150 ms（第 151 拍）判定到位，下一拍开始反转。
    const int until_reverse =
        rig.graph.tick_until([&] { return *rig.feeder_torque->value < 0.0; }, 400);
    EXPECT_EQ(until_reverse, 151);
    EXPECT_DOUBLE_EQ(*rig.feeder_torque->value, -2.5);

    // 400 ms 的保护里，剩余超过 300 ms 时反转：一共 99 拍，这是第 1 拍之后的 98 拍加上松开的那一拍。
    const int reversing =
        rig.graph.tick_until([&] { return *rig.feeder_torque->value == 0.0; }, 400);
    EXPECT_EQ(reversing, 99);
    EXPECT_FALSE(*rig.preloaded_ready->value);

    const int releasing = rig.graph.tick_until([&] { return *rig.preloaded_ready->value; }, 400);
    EXPECT_EQ(releasing, 300);
}

// 光电没被挡住就顶住了：只是卡了一下，反转之后接着转，不算上膛。
TEST(PutterController, ALockWithoutTheSensorIsJustAJam) {
    PutterRig rig;
    *rig.photoelectric->value = false;
    rig.remote.switches(Switch::MIDDLE, Switch::MIDDLE);
    rig.graph.tick(51);

    rig.graph.tick_until([&] { return *rig.feeder_torque->value < 0.0; }, 400);
    rig.graph.tick(400);
    EXPECT_FALSE(*rig.preloaded_ready->value);
    rig.graph.tick();
    EXPECT_DOUBLE_EQ(*rig.feeder_torque->value, 5.0) << "feeding again";
}

// 开火：推杆前推到堵转（弹丸出去了），退回 400 ms，然后开始给下一发上膛。
TEST(PutterController, FiresThenReturnsAndPreloadsAgain) {
    PutterRig rig;
    rig.preload();
    rig.graph.tick();
    ASSERT_DOUBLE_EQ(*rig.putter_torque->value, -0.02);

    rig.click();
    EXPECT_DOUBLE_EQ(*rig.putter_torque->value, 120.0) << "pushing the bullet forward";

    const int pushing =
        rig.graph.tick_until([&] { return *rig.putter_torque->value == -50.0; }, 200);
    EXPECT_EQ(pushing, 50) << "the stall that means the bullet has left";

    const int returning =
        rig.graph.tick_until([&] { return *rig.putter_torque->value == -0.02; }, 600);
    EXPECT_EQ(returning, 400);
    rig.graph.tick();
    EXPECT_DOUBLE_EQ(*rig.feeder_torque->value, 5.0) << "preloading the next bullet";
}

// 还没上膛（或者热量不够）时点了不发射。开机后的第一发例外：不要求已经上膛。
TEST(PutterController, RefusesToFireUntilPreloadedExceptForTheVeryFirstShot) {
    PutterRig rig;
    rig.preload();
    rig.click(); // 第一发
    ASSERT_DOUBLE_EQ(*rig.putter_torque->value, 120.0);
    rig.graph.tick_until([&] { return *rig.putter_torque->value == -0.02; }, 1000);

    // 现在在给第二发上膛：还没到位，点了不算。
    rig.click();
    EXPECT_DOUBLE_EQ(*rig.putter_torque->value, -0.02);

    // "上膛完成"这个输出只在反转保护期间才被改写：上一发留下的 true 会一直保持到下一次顶住。
    // 所以要等它先落下、再升起，才是这一发真的到位了。
    rig.graph.tick_until([&] { return !*rig.preloaded_ready->value; }, 1000);
    rig.graph.tick_until([&] { return *rig.preloaded_ready->value; }, 1000);
    *rig.allowance->value = 0;
    rig.click();
    EXPECT_DOUBLE_EQ(*rig.putter_torque->value, -0.02) << "no heat allowance";

    *rig.allowance->value = 5;
    rig.graph.tick(); // 松开的那一拍：触发看的是按下的沿
    rig.click();
    EXPECT_DOUBLE_EQ(*rig.putter_torque->value, 120.0);
}

// ── 弹速统计与记录 ─────────────────────────────────────────────────────────

TEST(ShootingStatistics, SummarisesABatchOfSpeeds) {
    std::vector<double> speeds{15.00, 15.02, 14.98, 15.10, 14.90};
    const auto statistics = analyse(speeds);

    EXPECT_NEAR(statistics.mean, 15.0, 1e-12);
    EXPECT_DOUBLE_EQ(statistics.max, 15.10);
    EXPECT_DOUBLE_EQ(statistics.min, 14.90);
    EXPECT_NEAR(statistics.range, 0.20, 1e-12);
    EXPECT_NEAR(statistics.trimmed_range, 0.04, 1e-12) << "without the one highest and lowest";
    EXPECT_DOUBLE_EQ(statistics.excellence_rate, 0.6); // ±0.025：15.00、15.02、14.98
    EXPECT_DOUBLE_EQ(statistics.pass_rate, 0.6);       // ±0.05：同样三个

    std::vector<double> two{15.0, 15.2};
    EXPECT_DOUBLE_EQ(analyse(two).trimmed_range, analyse(two).range) << "fewer than three";

    std::vector<double> none;
    EXPECT_DOUBLE_EQ(analyse(none).mean, 0.0);
}

// 每打一发记一行。控制线程只把样本放进队列；写文件发生在记录线程上（这里手动按拍尾门铃）。
TEST(ShootingRecorder, WritesOneLinePerShotFromTheRecorderThread) {
    const auto directory = std::filesystem::path{testing::TempDir()} / "hcs_shoot_records";
    const auto velocity_file = directory / "velocities";
    std::filesystem::remove(velocity_file);

    Graph graph;
    auto speed = graph.source<float>("/referee/shooter/initial_speed", 0.0F);
    auto timestamp = graph.source<double>("/referee/shooter/shoot_timestamp", 0.0);
    auto recorder = graph.add<ShootingRecorder>("recorder");
    graph.link();

    const auto recorded_lines = [&] {
        std::vector<std::string> lines;
        std::ifstream file{velocity_file};
        for (std::string line; std::getline(file, line);)
            lines.push_back(line);
        return lines;
    };
    const auto wait_for_lines = [&](std::size_t count) {
        for (int i = 0; i < 400 && recorded_lines().size() < count; ++i)
            std::this_thread::sleep_for(std::chrono::milliseconds{5});
        return recorded_lines();
    };
    const auto end_of_tick = [&] {
        graph.tick();
        recorder->tick_end_doorbell()->ring(); // executor 在拍尾做的事
    };

    for (int i = 0; i < 20; ++i) // 时间戳没变：没有发射，不记
        end_of_tick();

    *speed->value = 15.5F;
    *timestamp->value = 1.0;
    end_of_tick();
    for (int i = 0; i < 20; ++i) // 同一发不记第二次
        end_of_tick();
    EXPECT_EQ(wait_for_lines(1), (std::vector<std::string>{"15.500"}));

    *speed->value = 15.25F;
    *timestamp->value = 2.0;
    end_of_tick();
    EXPECT_EQ(wait_for_lines(2), (std::vector<std::string>{"15.500", "15.250"}));
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    setenv("ROS_LOG_DIR", "/tmp/hcs_test_ros_logs", 0);

    const auto records = std::filesystem::path{testing::TempDir()} / "hcs_shoot_records";
    std::filesystem::create_directories(records);

    const std::string params = testing::TempDir() + "test_shooting_params.yaml";
    std::ofstream{params} << R"(
heat:
  ros__parameters: {heat_per_shot: 10, reserved_heat: 20}
hero_heat:
  ros__parameters: {heat_per_shot: 100, reserved_heat: 0}
friction:
  ros__parameters:
    friction_wheels: [/gimbal/left_friction_wheel, /gimbal/right_friction_wheel]
    friction_velocities: [600.0, 600.0]
    friction_soft_start_stop_time: 0.1
friction_low:
  ros__parameters:
    friction_wheels: [/gimbal/left_friction_wheel, /gimbal/right_friction_wheel]
    friction_velocities: [600.0, 600.0]
    friction_velocities_low_mode: [400.0, 400.0]
    friction_soft_start_stop_time: 0.1
hero_friction:
  ros__parameters:
    friction_wheels: [/gimbal/wheel0, /gimbal/wheel1, /gimbal/wheel2]
    friction_velocities_profile_0: [500.0, 500.0, 500.0]
    friction_velocities_profile_1: [300.0, 300.0, 300.0]
    friction_soft_start_stop_time: 0.1
feeder17:
  ros__parameters:
    bullets_per_feeder_turn: 8.0
    shot_frequency: 20.0
    safe_shot_frequency: 10.0
    eject_frequency: 10.0
    eject_time: 0.05
    deep_eject_frequency: 15.0
    deep_eject_time: 0.2
    single_shot_max_stop_delay: 2.0
putter:
  ros__parameters:
    bullet_feeder_velocity_kp: 1.0
    bullet_feeder_velocity_ki: 0.0
    bullet_feeder_velocity_kd: 0.0
    putter_return_velocity_kp: 1.0
    putter_return_velocity_ki: 0.0
    putter_return_velocity_kd: 0.0
recorder:
  ros__parameters:
    log_mode: 1
    log_directory: )" << records.string() << R"(
    velocity_file: )" << (records / "velocities").string() << "\n";

    std::vector<char*> arguments{argv, argv + argc};
    for (const char* argument : {"--ros-args", "--params-file", params.c_str()})
        arguments.push_back(const_cast<char*>(argument));
    rclcpp::init(static_cast<int>(arguments.size()), arguments.data());
    const int result = RUN_ALL_TESTS();
    rclcpp::shutdown();
    return result;
}
