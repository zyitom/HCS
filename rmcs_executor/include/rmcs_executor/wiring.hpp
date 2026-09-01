#pragma once

// 接线器与失效隔离。
//
// 这里收着**全部**需要击穿 Component 私有状态的动作，而且一行都不依赖 rclcpp、
// pluginlib、参数服务器或任何线程。于是：
//
//   - Executor 只剩"起线程、定周期、记样本、打日志"，图和接线不再和 ROS 缠在一起；
//   - 单测能直接 new 几个组件、link 一次、手动 update() 几拍，验证拓扑序、1/z 语义、
//     可选输入的默认绑定、以及失效隔离到底有没有把整组停掉、有没有把输出复位。
//     这些路径改造前只能靠"跑真车看它 fatal 不 fatal"来验证。
//
// Component 里的 `friend struct Linker` 就是这份收敛的记账：私有字段仍然被外面碰，
// 但碰它们的地方从"一整个 670 行的 Executor"缩到了这一个不到 100 行、无依赖的结构。

#include <cstdint>
#include <expected>
#include <memory>
#include <vector>

#include "rmcs_executor/component.hpp"
#include "rmcs_executor/graph.hpp"
#include "rmcs_utility/doorbell.hpp"

namespace rmcs_executor {

/// 一个待闩存的 1/z 输入。见 Component::DelayedInput。
struct LatchEntry {
    void (*latch)(void*) noexcept;
    void* interface;
};

struct Wiring {
    /// 拓扑序。裸指针的所有权在调用方的 shared_ptr 列表里。
    std::vector<Component*> updating_order;
    /// updating_order[i] 在启动日志那棵缩进树里的深度。只影响打印。
    std::vector<std::uint32_t> depth;
    /// 按执行顺序收集，让每拍那趟遍历顺着内存走。
    std::vector<LatchEntry> latches;
    /// 拍尾要振铃的门铃。在这里一次收齐并定型，RT 线程只按这个固定列表遍历。
    std::vector<rmcs_utility::Doorbell*> tick_end_doorbells;
};

struct Linker {
    /// 建图 + 接线。**可重入**：graph::build 是纯函数，落方案的每一步都幂等，
    /// 所以同一组组件 link 两次得到同样的结果。改造前的 init() 原地递减
    /// dependency_count_ 来跑 Kahn，跑完图就被吃掉了，第二次必然报"全是环"。
    [[nodiscard]] static std::expected<Wiring, graph::BuildError> link(
        const std::vector<std::shared_ptr<Component>>& components) {
        // before_pairing 的输入是"目前已注册的输出有哪些、各是什么类型"。
        // 它在接线之前跑，允许组件据此再注册接口（ValueBroadcaster 就是这么给
        // 每个 double 输出各开一个转发器的），所以 describe() 必须排在它后面。
        auto output_map = Component::OutputInfoMap{};
        for (const auto& component : components)
            for (const auto& output : component->output_list_)
                output_map.emplace(output.name, std::cref(output.type));

        for (const auto& component : components)
            component->before_pairing(output_map);

        auto nodes = std::vector<graph::NodeDesc>{};
        nodes.reserve(components.size());
        for (const auto& component : components)
            nodes.push_back(component->describe());

        auto plan = graph::build(nodes);
        if (!plan)
            return std::unexpected(std::move(plan.error()));

        for (const auto& binding : plan->bindings) {
            auto* consumer = components[binding.consumer].get();
            const auto& input = consumer->input_list_[binding.input_index];

            if (binding.producer == graph::kNoProducer) {
                // 可选输入没人提供：绑一个默认值。default_bind 非空这件事
                // 已经由 graph::build 校验过（否则它会返回 OptionalWithoutDefault）。
                input.default_bind(input.interface);
                continue;
            }

            auto* producer = components[binding.producer].get();
            input.bind(input.interface, producer->output_list_[binding.output_index].binding);
        }

        Wiring wiring;
        wiring.updating_order.reserve(plan->order.size());
        wiring.depth = std::move(plan->depth);

        for (const auto node_index : plan->order) {
            auto* component = components[node_index].get();
            component->updating_index_ =
                static_cast<std::uint32_t>(wiring.updating_order.size());
            component->failed_ = false;
            wiring.updating_order.push_back(component);
        }

        for (auto* component : wiring.updating_order)
            for (const auto& input : component->input_list_)
                if (input.latch != nullptr)
                    wiring.latches.push_back(
                        LatchEntry{.latch = input.latch, .interface = input.interface});

        for (auto* component : wiring.updating_order)
            if (auto* doorbell = component->tick_end_doorbell(); doorbell != nullptr)
                wiring.tick_end_doorbells.push_back(doorbell);

        return wiring;
    }

    /// 单点失效不再拉整机。跑在控制线程里 —— 不打日志、不分配、不加锁、不抛。
    ///
    /// 输出必须复位到注册时的初值：改造前组件一抛异常就 rclcpp::shutdown()，
    /// 电机会保持最后一条指令继续转，这是真实的安全问题而不是可观测性问题。
    ///
    /// 隔离的粒度是**整个伙伴组**，不是单个 Component。硬件包的 Status / Command
    /// 两半共享同一份成员（Command::update() 直接回调 Status 那半的方法），
    /// 只停一半、剩下的一半照样每拍执行它的代码，隔离就是假的。
    ///
    /// @param newly_failed 新失效组件的 updating_index 追加于此。**不扩容**：
    ///        这条路径不许分配，写满就丢（名字只需要报一次）。
    static void isolate(
        Component* component, std::vector<std::uint32_t>& newly_failed,
        std::uint32_t& failed_count) noexcept {
        auto* root = component->partner_root_ != nullptr ? component->partner_root_ : component;
        fail_group(root, newly_failed, failed_count);
    }

private:
    static void fail_group(
        Component* component, std::vector<std::uint32_t>& newly_failed,
        std::uint32_t& failed_count) noexcept {
        if (component->failed_)
            return;
        component->failed_ = true;
        ++failed_count;

        for (const auto& output : component->output_list_) {
            if (output.reset == nullptr || output.default_value == nullptr)
                continue;
            output.reset(output.binding, output.default_value.get());
        }

        if (newly_failed.size() < newly_failed.capacity())
            newly_failed.push_back(component->updating_index_);

        for (const auto& partner : component->partner_component_list_)
            fail_group(partner.get(), newly_failed, failed_count);
    }
};

} // namespace rmcs_executor
