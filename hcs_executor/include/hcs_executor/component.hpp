#pragma once

// 组件的全部抽象。这个头**不依赖 rclcpp**——Event 族删掉之后，唯一那处 RCLCPP_WARN
// 也跟着走了。于是单独 new 一个 Component 子类、注册几个接口、调一次 update()，
// 不需要 rclcpp::init、不需要 pluginlib、不需要 Executor。这是能写单测的前提。

#include <cstddef>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <new>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <typeinfo>
#include <utility>
#include <vector>

#include <hcs_base/channel/tick.hpp>
#include <hcs_base/raw_storage.hpp>
#include <hcs_base/thread/rt_attributes.hpp>

#include "hcs_executor/graph.hpp"

namespace hcs_utility {
/// 只用到指针，前向声明就够 —— 别把 futex / syscall 那堆头拖进每个组件。
class Doorbell;
} // namespace hcs_utility

namespace hcs_executor {

namespace detail {
/// 见 Component::NameScope。定义在 component.cpp 里。
std::string& pending_component_name();
} // namespace detail

class Component {
public:
    /// Executor 在热路径上要读 failed_、要递归 partner_component_list_。
    friend class Executor;
    /// 接线与失效隔离的全部私有访问都收在 Linker 里（见 wiring.hpp）。
    friend struct Linker;

    using OutputInfoMap = std::map<std::string, std::reference_wrapper<const std::type_info>>;

    Component(const Component&) = delete;
    Component& operator=(const Component&) = delete;
    Component(Component&&) = delete;
    Component& operator=(Component&&) = delete;

    virtual ~Component() = default;

    virtual void before_pairing(const OutputInfoMap& output_map) { (void)output_map; }
    virtual void before_updating() {}

    /// 标 HCS_NONBLOCKING 的理由：clang 的 function effect analysis 规定
    /// override 一个 nonblocking 虚函数的实现必须也是 nonblocking，于是
    /// "update() 里不许分配 / 加锁 / 进内核"从 README 里的一句约定变成编译器管的事，
    /// 以后新来的人写的组件也一样被管住。gcc 下宏为空，只影响 clang-20+ 的构建。
    virtual void update(const hcs_sync::Tick& tick) HCS_NONBLOCKING = 0;

    /// pluginlib 只能默认构造组件，名字没法从构造参数进来，而组件在构造期就要用它
    /// （register_* 的报错要带名字，继承 rclcpp::Node 的还要拿它当节点名）。
    ///
    /// 改造前这件事靠一个**静态全局** `initializing_component_name` 完成，两个毛病：
    ///   - 全局：两个 Executor 没法在各自线程里同时建图，单测里更是互相踩；
    ///   - 只设不还原：create_partner_component 嵌套时，子组件把名字顶掉之后
    ///     父组件那一格再也拿不回自己的名字。
    /// 换成 thread_local + RAII 还原，两条一起修掉。想完全绕开它的（比如单测），
    /// 直接用 `Component(std::string)` 那个构造函数。
    class NameScope {
    public:
        explicit NameScope(std::string name)
            : previous_(std::exchange(detail::pending_component_name(), std::move(name))) {}

        ~NameScope() { detail::pending_component_name() = std::move(previous_); }

        NameScope(const NameScope&) = delete;
        NameScope& operator=(const NameScope&) = delete;
        NameScope(NameScope&&) = delete;
        NameScope& operator=(NameScope&&) = delete;

    private:
        std::string previous_;
    };

    template <typename T>
    requires(!std::is_reference_v<T> && !std::is_unbounded_array_v<T>) class InputInterface {
    public:
        friend class Component;

        InputInterface() = default;

        InputInterface(const InputInterface&) = delete;
        InputInterface& operator=(const InputInterface&) = delete;
        InputInterface(InputInterface&&) = delete;
        InputInterface& operator=(InputInterface&&) = delete;

        ~InputInterface() {
            if (delete_data_when_deconstruct) {
                if constexpr (std::is_array_v<T>) {
                    delete[] data_pointer_;
                } else {
                    delete data_pointer_;
                }
            }
        }

        [[nodiscard]] bool active() const { return activated; }

        /// 有没有可读的存储。ARCH-2 修好之后，接线跑完它就恒为 true ——
        /// 这正是那个修复的意义：不再有"指针是 nullptr 但 operator* 照样解引用"的 UB。
        [[nodiscard]] bool ready() const { return data_pointer_ != nullptr; }

        /// 这个输入背后到底有没有人在写。
        ///
        /// 可选输入没人提供时，框架会给它绑一个默认构造的值，于是 ready() 变成 true。
        /// 想区分"真的接上了上游"和"只是拿到了一个静止的默认值"，用这个，别用 ready()。
        [[nodiscard]] bool has_provider() const { return ready() && !defaulted_; }

        template <typename... Args>
        void make_and_bind_directly(Args&&... args) {
            if (ready())
                throw std::runtime_error("The interface has already been bound to somewhere");

            data_pointer_ = new T(std::forward<Args>(args)...);
            activated = true;

            delete_data_when_deconstruct = true;
        }

        void bind_directly(const T& destination) {
            if (ready())
                throw std::runtime_error("The interface has already been bound to somewhere");

            data_pointer_ = const_cast<T*>(&destination);
            activated = true;
        }

        const T* operator->() const { return data_pointer_; }
        const T& operator*() const { return *data_pointer_; }

    private:
        void* activate() {
            activated = true;
            return this;
        }

        /// 接线时由 Executor 调用。**幂等**——这是 init() 可重入的一半：
        /// 第二次接线要么改指向新的上游，要么原地留着上一次补的那个默认值。
        void bind(T* source) {
            if (delete_data_when_deconstruct && data_pointer_ != nullptr) {
                if constexpr (std::is_array_v<T>) {
                    delete[] data_pointer_;
                } else {
                    delete data_pointer_;
                }
                delete_data_when_deconstruct = false;
            }
            data_pointer_ = source;
            defaulted_ = false;
        }

        /// 可选输入没人提供时由 Executor 调用：就地造一个默认值，并记下"这是补的"。
        /// 留着 data_pointer_ == nullptr 是 UB——operator* 是无条件解引用，
        /// 而不是 README 里写的"读到默认构造值"。同样幂等。
        void bind_default() {
            if (defaulted_ && ready())
                return;
            data_pointer_ = nullptr;
            delete_data_when_deconstruct = false;
            make_and_bind_directly();
            defaulted_ = true;
        }

        T* data_pointer_ = nullptr;
        bool activated = false;
        bool defaulted_ = false;

        bool delete_data_when_deconstruct = false;
    };

    /// 显式单位延迟（1/z）。读到的**永远是上一拍**上游写完的值。
    ///
    /// 存在的理由是代数环。"电机反馈 → PID → 电机指令"在单线程顺序执行下是个环，
    /// 改造前的解法是把硬件包拆成 Status / Command 两个 Component（见 README 第四节）——
    /// 那是**代偿**，不是解法：它把一个时序问题伪装成了组件划分问题，代价是所有硬件包
    /// 都得写成两半、共享成员、失效隔离还得按"伙伴组"整组停。
    ///
    /// Simulink 的标准解法是插一个**显式可见**的 Unit Delay，这个类就是它。
    /// 它接线，但**不产生依赖边**（graph::InputDesc::delayed），所以环自然打开。
    ///
    /// 语义靠一次 latch 保证，而不是靠拓扑序碰运气：Executor 在**每拍最开头、任何
    /// update() 之前**，把所有 DelayedInput 从上游存储各拷一次。于是"读到的是上一拍的值"
    /// 与上下游谁先跑完全无关——上游在本拍晚些时候写的新值，要下一拍才看得见。
    ///
    /// 因此 T 必须平凡可拷贝：latch 跑在周期域里，不许分配、不许抛。
    template <typename T>
    requires std::is_trivially_copyable_v<T>
    class DelayedInput {
    public:
        friend class Component;

        /// @param initial **上游缺席时**（可选输入没人提供）一直用的值。
        ///        接上了上游的话，首拍闩到的是那个输出注册时的初值，
        ///        也就是"第 -1 拍的状态"——这才是 1/z 在 n=0 处的正确取值，
        ///        所以这个参数在有上游时不参与任何计算。
        explicit DelayedInput(const T& initial = T{})
            : value_(initial) {}

        DelayedInput(const DelayedInput&) = delete;
        DelayedInput& operator=(const DelayedInput&) = delete;
        DelayedInput(DelayedInput&&) = delete;
        DelayedInput& operator=(DelayedInput&&) = delete;

        [[nodiscard]] bool active() const { return activated_; }
        [[nodiscard]] bool has_provider() const { return source_ != nullptr; }

        const T* operator->() const { return &value_; }
        const T& operator*() const { return value_; }

    private:
        void* activate() {
            activated_ = true;
            return this;
        }

        void bind(const T* source) { source_ = source; }

        /// 周期域，每拍一次。平凡拷贝，无分配无锁。
        void latch() noexcept {
            if (source_ != nullptr)
                value_ = *source_;
        }

        const T* source_ = nullptr;
        T value_;
        bool activated_ = false;
    };

    template <typename T>
    requires(!std::is_reference_v<T> && !std::is_unbounded_array_v<T>) class OutputInterface {
    public:
        friend class Component;

        OutputInterface() = default;

        OutputInterface(const OutputInterface&) = delete;
        OutputInterface& operator=(const OutputInterface&) = delete;
        OutputInterface(OutputInterface&&) = delete;
        OutputInterface& operator=(OutputInterface&&) = delete;

        ~OutputInterface() {
            if (active())
                std::destroy_at(storage_pointer());
        };

        [[nodiscard]] bool active() const { return activated; }

        T* operator->() { return storage_pointer(); }
        const T* operator->() const { return storage_pointer(); }
        T& operator*() { return *storage_pointer(); }
        const T& operator*() const { return *storage_pointer(); }

    private:
        template <typename... Args>
        void* activate(Args&&... args) {
            // placement new 走 raw():对象此刻还不存在于这块存储里,launder 是
            // 给"已构造之后"的访问用的。这一对区别由 RawStorage 的两个访问器承载。
            std::construct_at(storage_.raw(), std::forward<Args>(args)...);
            activated = true;
            return storage_.raw();
        }

        [[nodiscard]] T* storage_pointer() { return storage_.ptr(); }
        [[nodiscard]] const T* storage_pointer() const { return storage_.ptr(); }

        hcs_utility::RawStorage<T> storage_;
        bool activated = false;
    };

    [[nodiscard]] const std::string& get_component_name() const { return component_name_; }

    /// 由 create_partner_component 创建的伙伴组件。调度器要递归把它们也纳入管理，
    /// 失效隔离要按整组停。公开出来是为了让"收集组件"这件事不必是 friend。
    [[nodiscard]] const std::vector<std::shared_ptr<Component>>& partner_components() const {
        return partner_component_list_;
    }

    /// 需要**拍尾唤醒**的组件重写它，返回非空时 Executor 会在每拍最后 ring 一次。
    ///
    /// 典型用户是持有传输发送线程的硬件组件：命令在 update() 里写进 Snapshot，
    /// 发送线程要被叫起来才会去取。
    ///
    /// 为什么是拍尾而不是 update() 里面：ring() 在有等待者时会发一次 FUTEX_WAKE，
    /// 那是系统调用，update() 上的 HCS_NONBLOCKING 会直接拦住。拍尾那个位置紧贴着
    /// sleep_until_precise —— 本来就要进内核 —— 所以这一下叠在已经要付的开销上，
    /// 不引入新的抖动源。这是周期域里**唯一**被允许的额外系统调用位置。
    [[nodiscard]] virtual hcs_utility::Doorbell* tick_end_doorbell() { return nullptr; }

    /// 组件是否已被 executor 隔离（update() 抛过异常）。失效后它不再被调度，
    /// 它的输出停在注册时的默认值上。
    [[nodiscard]] bool failed() const noexcept { return failed_; }

    template <typename T>
    void register_input(
        const std::string& name, InputInterface<T>& interface, bool required = true) {
        if (interface.active())
            throw std::runtime_error("The interface has been activated");

        ensure_registration_name_is_available(name, RegistrationDirection::Input);

        // T 不可默认构造时只能留 nullptr，由 graph::build 在没人提供该输出时报致命错。
        BindDefaultFunction default_bind = nullptr;
        if constexpr (std::is_default_constructible_v<T>)
            default_bind = &bind_default_input_interface<T>;

        input_list_.emplace_back(
            InputDeclaration{
                .type = typeid(T),
                .name = name,
                .required = required,
                .delayed = false,
                .interface = interface.activate(),
                .bind = &bind_input_interface<T>,
                .default_bind = default_bind,
                .latch = nullptr});
    }

    /// 注册一个显式单位延迟输入（1/z）。见 DelayedInput 的注释。
    ///
    /// 它不产生依赖边，所以**可以安全地读下游、甚至读自己的输出**——那正是它的用途。
    /// 代价是那份数据晚一拍，而这个代价现在写在类型上，谁都看得见。
    template <typename T>
    void register_delayed_input(
        const std::string& name, DelayedInput<T>& interface, bool required = true) {
        if (interface.active())
            throw std::runtime_error("The interface has been activated");

        ensure_registration_name_is_available(name, RegistrationDirection::Input);

        input_list_.emplace_back(
            InputDeclaration{
                .type = typeid(T),
                .name = name,
                .required = required,
                .delayed = true,
                .interface = interface.activate(),
                .bind = &bind_delayed_input_interface<T>,
                // 上游缺席时影子存储里就是构造时给的初值，什么都不用补，
                // 但"可默认"这一位必须是真，否则 graph::build 会误报
                // OptionalWithoutDefault。所以给一个真的空实现，而不是 nullptr。
                .default_bind = &no_default_needed,
                .latch = &latch_delayed_input<T>});
    }

    template <typename T, typename... Args>
    requires std::constructible_from<T, Args...>
    void register_output(const std::string& name, OutputInterface<T>& interface, Args&&... args) {
        if (interface.active())
            throw std::runtime_error("The interface has been activated");

        ensure_registration_name_is_available(name, RegistrationDirection::Output);

        // 先按左值拷一份初值，再把 args forward 给 activate()：反过来（或同一条语句里两用）
        // 会把同一批实参 move 两次。构造失败就直接抛，不会留下半个注册项。
        //
        // 复位钩子只给**平凡可拷贝**的类型装。它是在控制线程里、组件已经抛了异常之后
        // 才被调用的，那条路径上不许分配也不许再抛：
        // std::vector / std::string 这类的拷贝赋值会 malloc，还可能抛 bad_alloc，
        // 而调用它的 mark_component_failed 是 noexcept —— 抛出去就是 std::terminate，
        // 本来只想隔离一个组件，结果整机没了，比不复位还糟。
        // 非平凡类型因此没有复位钩子：它们的输出会停在最后一个值。这是有意的取舍，
        // 也是"跨域/热路径上的输出应当是平凡可拷贝的"这条约定的又一处体现。
        std::shared_ptr<void> default_value;
        ResetFunction reset = nullptr;
        if constexpr (
            std::is_trivially_copyable_v<T> && std::is_copy_assignable_v<T>
            && std::is_copy_constructible_v<T> && std::constructible_from<T, Args&...>) {
            default_value = std::make_shared<T>(args...);
            reset = &reset_output_interface<T>;
        }

        output_list_.emplace_back(
            OutputDeclaration{
                .type = typeid(T),
                .name = name,
                .binding = interface.activate(std::forward<Args>(args)...),
                .component = this,
                .default_value = std::move(default_value),
                .reset = reset});
    }

    template <typename T, typename... Args>
    requires std::constructible_from<T, Args...>
    std::shared_ptr<T> create_partner_component(const std::string& name, Args&&... args) {
        auto scope = NameScope{name};

        auto component = std::make_shared<T>(std::forward<Args>(args)...);
        // 伙伴组共用一个根。失效隔离要靠它把整组一起停掉：
        // DemoBoard::Command::update() 里就是直接回调 board_.command_update()，
        // 只隔离 DemoBoard 而放着它的 command partner 继续跑，等于没隔离。
        component->partner_root_ = partner_root_ != nullptr ? partner_root_ : this;
        partner_component_list_.emplace_back(component);

        return component;
    }

    /// 交给 graph::build 的那份纯数据。这是建图算法看得到的**全部**——
    /// 它不再需要 friend 进来摸 input_list_ / output_list_ / dependency_count_，
    /// 所以它也就能被单独拿出去测。
    [[nodiscard]] graph::NodeDesc describe() const {
        graph::NodeDesc desc;
        desc.name = component_name_;

        desc.outputs.reserve(output_list_.size());
        for (const auto& output : output_list_)
            desc.outputs.push_back(graph::OutputDesc{.name = output.name, .type = &output.type});

        desc.inputs.reserve(input_list_.size());
        for (const auto& input : input_list_)
            desc.inputs.push_back(
                graph::InputDesc{
                    .name = input.name,
                    .type = &input.type,
                    .required = input.required,
                    .delayed = input.delayed,
                    .can_default = input.default_bind != nullptr});

        return desc;
    }

protected:
    /// pluginlib 路径：名字从当前 NameScope 取。
    Component()
        : component_name_(detail::pending_component_name()) {}

    /// 显式命名。单测和任何直接 new 组件的地方都该走这条——它不碰任何全局状态。
    explicit Component(std::string name)
        : component_name_(std::move(name)) {}

private:
    enum class RegistrationDirection : std::uint8_t {
        Input,
        Output,
    };

    using BindFunction = void (*)(void* interface, void* output_storage);
    using BindDefaultFunction = void (*)(void* interface);
    using LatchFunction = void (*)(void* interface) noexcept;
    using ResetFunction = void (*)(void* storage, const void* default_value);

    static const char* registration_direction_name(RegistrationDirection direction) {
        switch (direction) {
        case RegistrationDirection::Input: return "input";
        case RegistrationDirection::Output: return "output";
        }

        return "interface";
    }

    void ensure_registration_name_is_available(
        const std::string& name, RegistrationDirection direction) const {
        const auto check = [&](const auto& declarations, RegistrationDirection existing) {
            for (const auto& declaration : declarations) {
                if (declaration.name != name)
                    continue;

                if (existing != direction)
                    throw std::runtime_error(
                        "Component [" + component_name_ + "] cannot register "
                        + registration_direction_name(direction) + " \"" + name + "\" because \""
                        + name + "\" has already been registered as "
                        + registration_direction_name(existing) + " in this component.");

                throw std::runtime_error(
                    "Component [" + component_name_ + "] registered "
                    + registration_direction_name(direction) + " \"" + name + "\" more than once.");
            }
        };

        check(input_list_, RegistrationDirection::Input);
        check(output_list_, RegistrationDirection::Output);
    }

    template <typename T>
    static void bind_input_interface(void* interface, void* output_storage) {
        static_cast<InputInterface<T>*>(interface)->bind(static_cast<T*>(output_storage));
    }

    template <typename T>
    static void bind_default_input_interface(void* interface) {
        static_cast<InputInterface<T>*>(interface)->bind_default();
    }

    template <typename T>
    static void bind_delayed_input_interface(void* interface, void* output_storage) {
        static_cast<DelayedInput<T>*>(interface)->bind(static_cast<const T*>(output_storage));
    }

    template <typename T>
    static void latch_delayed_input(void* interface) noexcept {
        static_cast<DelayedInput<T>*>(interface)->latch();
    }

    /// DelayedInput 的"补默认值"是空操作：初值在构造时就已经在影子存储里了。
    static void no_default_needed(void* /*interface*/) {}

    // storage 是 OutputInterface<T>::activate() 返回的那块存储，对象已经构造好。
    template <typename T>
    static void reset_output_interface(void* storage, const void* default_value) {
        static_assert(
            std::is_trivially_copyable_v<T>,
            "输出复位跑在控制线程的失效隔离路径上，那里不许分配也不许抛");
        *static_cast<T*>(storage) = *static_cast<const T*>(default_value);
    }

    std::string component_name_;

    struct InputDeclaration {
        const std::type_info& type;
        std::string name;
        bool required;
        bool delayed;

        void* interface; ///< InputInterface<T>* 或 DelayedInput<T>*
        BindFunction bind;
        BindDefaultFunction default_bind; ///< nullptr 表示 T 不可默认构造
        LatchFunction latch;              ///< 非 nullptr 当且仅当 delayed
    };

    struct OutputDeclaration {
        const std::type_info& type;
        std::string name;
        void* binding;

        Component* component;

        // 组件失效隔离用：把这个输出复位回注册时的初值。
        // 两者同时为 nullptr 表示不支持复位（类型不可平凡拷贝）。
        std::shared_ptr<void> default_value;
        ResetFunction reset;
    };

    std::vector<InputDeclaration> input_list_;
    std::vector<OutputDeclaration> output_list_;

    std::vector<std::shared_ptr<Component>> partner_component_list_;
    /// 伙伴组的根；自己就是根时为 nullptr。由 create_partner_component 设，失效隔离用。
    Component* partner_root_ = nullptr;
    /// 在 Executor::updating_order_ 里的下标，接线时填。失效时用来报名字。
    std::uint32_t updating_index_ = 0;

    // 只被 executor 在控制线程里读写，不需要原子。
    bool failed_ = false;
};

} // namespace hcs_executor
