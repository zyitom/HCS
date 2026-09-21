#pragma once

// 建图与拓扑排序，从 Executor::init() 里整块剥出来。
//
// 剥出来买到的是三样东西，每一样都是之前做不到的：
//
//   1. **可离线构造**。这里不认识 Component，不认识 rclcpp，不认识 pluginlib——
//      输入就是一组 NodeDesc（名字 + 输入表 + 输出表），输出是一份接线方案。
//      单测里手写几个 NodeDesc 就能打到重名、类型不匹配、成环、可选输入没人提供
//      这四条错误路径，不用起 ROS、不用加载插件、不用真的造出一个组件。
//      改造前这些分支只能靠"跑一次真车看它 fatal 不 fatal"来验证。
//
//   2. **纯函数**。build() 不碰任何组件的状态，同一份输入永远给出同一份输出。
//      于是 init() 天然可重入——之前它靠原地递减 dependency_count_ 来跑 Kahn，
//      跑完图就被吃掉了，第二次调用必然得到"全是环"。
//
//   3. **递归改迭代**。append_updating_order 是递归的，深度等于依赖链长度。
//      这里用显式栈，且**逐位复刻**原来的 DFS 访问次序（见 build() 里的注释），
//      所以启动日志里那棵缩进树和改造前一模一样，执行顺序也没变。
//
// 报错文本在这里拼好、原样返回，调用方只负责打一次日志再抛。
// 这样"错在哪"这件事也进了单测的射程。

#include <cstdint>
#include <expected>
#include <string>
#include <typeinfo>
#include <vector>

namespace hcs_executor::graph {

/// 没有生产者。可选输入没人提供时，Binding::producer 就是它。
inline constexpr std::uint32_t kNoProducer = 0xFFFF'FFFFu;

struct OutputDesc {
    std::string name;
    const std::type_info* type = nullptr;
};

struct InputDesc {
    std::string name;
    const std::type_info* type = nullptr;

    /// false 表示可选：没人提供时不报错，绑一个默认值。
    bool required = true;

    /// 显式单位延迟（1/z）。读的是**上一拍**的值，所以它**不产生依赖边**——
    /// 这正是它能打破代数环的原因，也是 Status/Command 拆分的替代品。
    bool delayed = false;

    /// 类型可默认构造。可选输入没人提供、且这里是 false 时，
    /// 没有任何合法的值能绑给它，只能报错——留 nullptr 是 UB（ARCH-2）。
    bool can_default = true;
};

struct NodeDesc {
    std::string name;
    std::vector<OutputDesc> outputs;
    std::vector<InputDesc> inputs;
};

/// 一条接线：consumer 的第 input_index 个输入，接到 producer 的第 output_index 个输出。
/// producer == kNoProducer 表示"没人提供，用默认值"（此时 output_index 无意义）。
struct Binding {
    std::uint32_t consumer = 0;
    std::uint32_t input_index = 0;
    std::uint32_t producer = kNoProducer;
    std::uint32_t output_index = 0;
};

struct Plan {
    /// 拓扑序，元素是 NodeDesc 的下标。size() 恒等于节点总数（否则就是成环，走 error 路径）。
    std::vector<std::uint32_t> order;
    /// order[i] 在启动日志那棵缩进树里的深度。只影响打印，不影响执行。
    std::vector<std::uint32_t> depth;
    /// 每个节点的每个输入各一条，按 (consumer, input_index) 升序。
    std::vector<Binding> bindings;
};

struct BuildError {
    enum class Kind : std::uint8_t {
        DuplicateOutput,       ///< 两个组件注册了同名输出
        MissingRequiredInput,  ///< 必需输入没人提供
        TypeMismatch,          ///< 输入输出同名但类型不同
        OptionalWithoutDefault ///< 可选输入没人提供，而类型不可默认构造
        ,
        CircularDependency ///< 拓扑排序排不完
    };

    Kind kind;
    /// 已经拼好的完整报错，可能是多行。调用方直接打、直接抛。
    std::string message;
};

namespace detail {

inline std::string describe_output(const NodeDesc& node, const OutputDesc& output) {
    return "Component [" + node.name + "] registered output \"" + output.name + "\" with type \""
         + (output.type != nullptr ? output.type->name() : "?") + "\"";
}

} // namespace detail

/// 纯函数：一组节点描述 → 接线方案，或第一条错误。
///
/// 检查顺序是刻意的，和改造前一致：先把所有输出收进表（重名在这一步就报），
/// 再逐个输入查表（缺失 / 类型不匹配 / 可选无默认），最后才排序（成环）。
/// 换句话说，成环的报错只在接线全部合法之后才可能出现——这样"环"里出现的边
/// 一定都是真实存在的边，诊断不会被前一类错误污染。
[[nodiscard]] inline std::expected<Plan, BuildError> build(const std::vector<NodeDesc>& nodes) {
    const auto node_count = static_cast<std::uint32_t>(nodes.size());

    // ── 1. 收集输出 ───────────────────────────────────────────────────────
    struct OutputLocation {
        std::uint32_t node;
        std::uint32_t index;
    };
    // 启动期跑一次，用 vector + 线性查找足够；换成 unordered_map 只会多一份哈希实现要维护。
    // 组件数是两位数，输出数是三位数量级。
    std::vector<std::pair<std::string, OutputLocation>> outputs;
    for (std::uint32_t node_index = 0; node_index < node_count; ++node_index) {
        const auto& node = nodes[node_index];
        for (std::uint32_t output_index = 0;
             output_index < static_cast<std::uint32_t>(node.outputs.size()); ++output_index) {
            const auto& output = node.outputs[output_index];

            for (const auto& [name, location] : outputs) {
                if (name != output.name)
                    continue;
                return std::unexpected(BuildError{
                    .kind = BuildError::Kind::DuplicateOutput,
                    .message =
                        "Duplicate output name \"" + output.name + "\": "
                        + detail::describe_output(
                            nodes[location.node], nodes[location.node].outputs[location.index])
                        + "; " + detail::describe_output(node, output)
                        + ". Only one output may be registered for each name."});
            }

            outputs.emplace_back(output.name, OutputLocation{node_index, output_index});
        }
    }

    const auto find_output = [&outputs](const std::string& name) -> const OutputLocation* {
        for (const auto& [output_name, location] : outputs)
            if (output_name == name)
                return &location;
        return nullptr;
    };

    // ── 2. 接线 ───────────────────────────────────────────────────────────
    Plan plan;
    plan.bindings.reserve(outputs.size());

    // producers[c] / consumers[p] 都按"去重后的节点"存：一个组件读同一个上游的两个输出，
    // 只算一条依赖边。改造前用 unordered_set<Component*> wanted_by_ 达到同样效果。
    std::vector<std::vector<std::uint32_t>> producers(node_count);
    std::vector<std::vector<std::uint32_t>> consumers(node_count);

    for (std::uint32_t consumer = 0; consumer < node_count; ++consumer) {
        const auto& node = nodes[consumer];
        for (std::uint32_t input_index = 0;
             input_index < static_cast<std::uint32_t>(node.inputs.size()); ++input_index) {
            const auto& input = node.inputs[input_index];
            const auto* location = find_output(input.name);

            if (location == nullptr) {
                if (input.required)
                    return std::unexpected(BuildError{
                        .kind = BuildError::Kind::MissingRequiredInput,
                        .message = "Cannot find output \"" + input.name
                                 + "\" required by component [" + node.name + "]."});

                if (!input.can_default)
                    return std::unexpected(BuildError{
                        .kind = BuildError::Kind::OptionalWithoutDefault,
                        .message =
                            "Optional input \"" + input.name + "\" of component [" + node.name
                            + "] is not provided by any output, and its type \""
                            + (input.type != nullptr ? input.type->name() : "?")
                            + "\" is not default-constructible, so no default value can be bound "
                              "to it. Either provide the output, declare the input as required, "
                              "or give the type a default constructor."});

                plan.bindings.push_back(
                    Binding{.consumer = consumer, .input_index = input_index});
                continue;
            }

            const auto& producer_node = nodes[location->node];
            const auto& output = producer_node.outputs[location->index];
            if (input.type != output.type) {
                return std::unexpected(BuildError{
                    .kind = BuildError::Kind::TypeMismatch,
                    .message =
                        "Type mismatch for interface \"" + input.name + "\": component ["
                        + producer_node.name + "] declared output type \""
                        + (output.type != nullptr ? output.type->name() : "?")
                        + "\", but component [" + node.name + "] requested input type \""
                        + (input.type != nullptr ? input.type->name() : "?") + "\"."});
            }

            plan.bindings.push_back(
                Binding{
                    .consumer = consumer,
                    .input_index = input_index,
                    .producer = location->node,
                    .output_index = location->index});

            // 1/z 照样接线（它要知道从哪儿取值），但**不产生边**。
            if (input.delayed)
                continue;

            auto& producer_list = producers[consumer];
            bool known = false;
            for (const auto existing : producer_list)
                if (existing == location->node) {
                    known = true;
                    break;
                }
            if (!known) {
                producer_list.push_back(location->node);
                consumers[location->node].push_back(consumer);
            }
        }
    }

    // consumers[p] 是按 consumer 递增顺序 push 的（外层循环就是 consumer 递增），
    // 已经等价于改造前"遍历 component_list_ 找 wanted_by_ 成员"的次序，不需要再排。

    // ── 3. 拓扑排序（迭代版 DFS，次序与改造前的递归版逐位一致）─────────────
    std::vector<std::uint32_t> remaining(node_count);
    for (std::uint32_t index = 0; index < node_count; ++index)
        remaining[index] = static_cast<std::uint32_t>(producers[index].size());

    plan.order.reserve(node_count);
    plan.depth.reserve(node_count);

    struct Frame {
        std::uint32_t node;
        std::uint32_t depth;
        std::uint32_t cursor;
    };
    std::vector<Frame> stack;
    stack.reserve(node_count);

    // 根集合必须**在遍历开始前**快照。remaining[] 会在遍历过程中被递减，
    // 边遍历边判 `remaining[i] == 0` 的话，一个在别人的 DFS 里刚被减到 0、
    // 已经发过一次的节点，轮到外层循环时会被当成新的根再发一次。
    std::vector<std::uint32_t> roots;
    for (std::uint32_t index = 0; index < node_count; ++index)
        if (remaining[index] == 0)
            roots.push_back(index);

    for (const auto root : roots) {
        plan.order.push_back(root);
        plan.depth.push_back(0);
        stack.push_back(Frame{.node = root, .depth = 0, .cursor = 0});

        while (!stack.empty()) {
            // 先把要用的值抄出来：下面 push_back 可能让 stack.back() 的引用失效。
            const auto node = stack.back().node;
            const auto depth = stack.back().depth;
            const auto cursor = stack.back().cursor;

            if (cursor == consumers[node].size()) {
                stack.pop_back();
                continue;
            }
            stack.back().cursor = cursor + 1;

            const auto consumer = consumers[node][cursor];
            if (--remaining[consumer] != 0)
                continue;

            plan.order.push_back(consumer);
            plan.depth.push_back(depth + 1);
            stack.push_back(Frame{.node = consumer, .depth = depth + 1, .cursor = 0});
        }
    }

    if (plan.order.size() < node_count) {
        std::string message = "Circular dependency found:";
        for (std::uint32_t index = 0; index < node_count; ++index) {
            if (remaining[index] == 0)
                continue;
            message += "\n  Component [" + nodes[index].name + "]:";
            for (const auto& input : nodes[index].inputs) {
                if (input.delayed)
                    continue;
                const auto* location = find_output(input.name);
                if (location == nullptr || remaining[location->node] == 0)
                    continue;
                message += "\n      Depends on [" + nodes[location->node].name
                         + "] because requesting interface \"" + input.name + "\"";
            }
        }
        return std::unexpected(
            BuildError{.kind = BuildError::Kind::CircularDependency, .message = std::move(message)});
    }

    return plan;
}

} // namespace hcs_executor::graph
