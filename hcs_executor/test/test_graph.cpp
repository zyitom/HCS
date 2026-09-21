// 建图算法的单测。整个文件不 include rclcpp、不 include component.hpp ——
// graph::build 的输入就是几个 NodeDesc 结构体，这正是把它剥出来要买的东西：
// 「重名 / 缺输入 / 类型不匹配 / 可选无默认 / 成环」这五条错误路径，
// 改造前只能靠跑一次真车看它 fatal 不 fatal 来验证。

#include <string>
#include <typeinfo>
#include <vector>

#include <gtest/gtest.h>

#include "hcs_executor/graph.hpp"

using hcs_executor::graph::build;
using hcs_executor::graph::BuildError;
using hcs_executor::graph::InputDesc;
using hcs_executor::graph::kNoProducer;
using hcs_executor::graph::NodeDesc;
using hcs_executor::graph::OutputDesc;

namespace {

OutputDesc out(std::string name, const std::type_info& type = typeid(double)) {
    return OutputDesc{.name = std::move(name), .type = &type};
}

InputDesc in(std::string name, const std::type_info& type = typeid(double)) {
    return InputDesc{
        .name = std::move(name),
        .type = &type,
        .required = true,
        .delayed = false,
        .can_default = true};
}

InputDesc optional_in(std::string name, bool can_default = true) {
    auto desc = in(std::move(name));
    desc.required = false;
    desc.can_default = can_default;
    return desc;
}

InputDesc delayed_in(std::string name) {
    auto desc = in(std::move(name));
    desc.delayed = true;
    return desc;
}

/// order 里的下标 -> 名字，断言起来比记下标直观。
std::vector<std::string> names_in_order(
    const std::vector<NodeDesc>& nodes, const std::vector<std::uint32_t>& order) {
    std::vector<std::string> names;
    names.reserve(order.size());
    for (const auto index : order)
        names.push_back(nodes[index].name);
    return names;
}

} // namespace

TEST(Graph, LinearChainIsSortedAndIndented) {
    const std::vector<NodeDesc> nodes{
        {.name = "a", .outputs = {out("/x")}, .inputs = {}},
        {.name = "c", .outputs = {}, .inputs = {in("/y")}},
        {.name = "b", .outputs = {out("/y")}, .inputs = {in("/x")}},
    };

    const auto plan = build(nodes);
    ASSERT_TRUE(plan.has_value()) << plan.error().message;
    EXPECT_EQ(names_in_order(nodes, plan->order), (std::vector<std::string>{"a", "b", "c"}));
    // 缩进深度只影响启动日志那棵树，但它是「谁依赖谁」最直接的可视化，值得钉住。
    EXPECT_EQ(plan->depth, (std::vector<std::uint32_t>{0, 1, 2}));
}

TEST(Graph, BindingsPointAtTheRightOutput) {
    const std::vector<NodeDesc> nodes{
        {.name = "producer", .outputs = {out("/a"), out("/b")}, .inputs = {}},
        {.name = "consumer", .outputs = {}, .inputs = {in("/b")}},
    };

    const auto plan = build(nodes);
    ASSERT_TRUE(plan.has_value()) << plan.error().message;
    ASSERT_EQ(plan->bindings.size(), 1u);
    EXPECT_EQ(plan->bindings[0].consumer, 1u);
    EXPECT_EQ(plan->bindings[0].input_index, 0u);
    EXPECT_EQ(plan->bindings[0].producer, 0u);
    EXPECT_EQ(plan->bindings[0].output_index, 1u);
}

/// 回归：根集合必须在遍历前快照。不快照的话，一个在别人的 DFS 里刚被减到 0 的节点
/// 会被外层循环当成新的根再发一次 —— order 里出现重复，而 size() >= node_count
/// 让成环检查也发现不了，症状是启动日志里同一个组件打了两遍、每拍被 update 两次。
TEST(Graph, NodeIsEmittedExactlyOnceWithMultipleRoots) {
    const std::vector<NodeDesc> nodes{
        {.name = "root_a", .outputs = {out("/a")}, .inputs = {}},
        {.name = "sink", .outputs = {}, .inputs = {in("/a"), in("/b")}},
        {.name = "root_b", .outputs = {out("/b")}, .inputs = {}},
    };

    const auto plan = build(nodes);
    ASSERT_TRUE(plan.has_value()) << plan.error().message;
    EXPECT_EQ(plan->order.size(), nodes.size());
    EXPECT_EQ(
        names_in_order(nodes, plan->order), (std::vector<std::string>{"root_a", "root_b", "sink"}));
}

TEST(Graph, TwoInputsFromTheSameProducerAreOneEdge) {
    const std::vector<NodeDesc> nodes{
        {.name = "producer", .outputs = {out("/a"), out("/b")}, .inputs = {}},
        {.name = "consumer", .outputs = {}, .inputs = {in("/a"), in("/b")}},
    };

    const auto plan = build(nodes);
    ASSERT_TRUE(plan.has_value()) << plan.error().message;
    // 边算两次的话 remaining 永远减不到 0，consumer 会被误报成环。
    EXPECT_EQ(plan->order.size(), 2u);
    EXPECT_EQ(plan->bindings.size(), 2u);
}

TEST(Graph, BuildIsPureAndRepeatable) {
    const std::vector<NodeDesc> nodes{
        {.name = "a", .outputs = {out("/x")}, .inputs = {}},
        {.name = "b", .outputs = {}, .inputs = {in("/x")}},
    };

    const auto first = build(nodes);
    const auto second = build(nodes);
    ASSERT_TRUE(first.has_value());
    ASSERT_TRUE(second.has_value());
    // 改造前的 init() 靠原地递减 dependency_count_ 跑 Kahn，图跑完就被吃掉，
    // 第二次调用必然得到「全是环」。可重入是这一版才有的性质。
    EXPECT_EQ(first->order, second->order);
    EXPECT_EQ(first->depth, second->depth);
}

TEST(Graph, DuplicateOutputIsRejected) {
    const std::vector<NodeDesc> nodes{
        {.name = "a", .outputs = {out("/x")}, .inputs = {}},
        {.name = "b", .outputs = {out("/x")}, .inputs = {}},
    };

    const auto plan = build(nodes);
    ASSERT_FALSE(plan.has_value());
    EXPECT_EQ(plan.error().kind, BuildError::Kind::DuplicateOutput);
    // 报错必须同时点名两个组件，否则拿着日志找不到是哪两个撞了。
    EXPECT_NE(plan.error().message.find("[a]"), std::string::npos);
    EXPECT_NE(plan.error().message.find("[b]"), std::string::npos);
}

TEST(Graph, MissingRequiredInputIsRejected) {
    const std::vector<NodeDesc> nodes{
        {.name = "consumer", .outputs = {}, .inputs = {in("/nobody")}},
    };

    const auto plan = build(nodes);
    ASSERT_FALSE(plan.has_value());
    EXPECT_EQ(plan.error().kind, BuildError::Kind::MissingRequiredInput);
    EXPECT_NE(plan.error().message.find("/nobody"), std::string::npos);
}

TEST(Graph, TypeMismatchIsRejected) {
    const std::vector<NodeDesc> nodes{
        {.name = "producer", .outputs = {out("/x", typeid(int))}, .inputs = {}},
        {.name = "consumer", .outputs = {}, .inputs = {in("/x", typeid(double))}},
    };

    const auto plan = build(nodes);
    ASSERT_FALSE(plan.has_value());
    EXPECT_EQ(plan.error().kind, BuildError::Kind::TypeMismatch);
    EXPECT_NE(plan.error().message.find("[producer]"), std::string::npos);
    EXPECT_NE(plan.error().message.find("[consumer]"), std::string::npos);
}

TEST(Graph, OptionalInputWithoutProviderGetsNoProducer) {
    const std::vector<NodeDesc> nodes{
        {.name = "consumer", .outputs = {}, .inputs = {optional_in("/nobody")}},
    };

    const auto plan = build(nodes);
    ASSERT_TRUE(plan.has_value()) << plan.error().message;
    ASSERT_EQ(plan->bindings.size(), 1u);
    EXPECT_EQ(plan->bindings[0].producer, kNoProducer);
}

/// ARCH-2：可选输入没人提供、类型又不可默认构造时，没有任何合法的值能绑给它。
/// 改造前这里直接 continue，指针留 nullptr，而 operator* 是无条件解引用 —— 是 UB，
/// 不是 README 里写的「读到默认构造值」。
TEST(Graph, OptionalInputThatCannotDefaultIsRejected) {
    const std::vector<NodeDesc> nodes{
        {.name = "consumer",
         .outputs = {},
         .inputs = {optional_in("/nobody", /*can_default=*/false)}},
    };

    const auto plan = build(nodes);
    ASSERT_FALSE(plan.has_value());
    EXPECT_EQ(plan.error().kind, BuildError::Kind::OptionalWithoutDefault);
}

TEST(Graph, CycleIsRejectedAndDiagnosed) {
    const std::vector<NodeDesc> nodes{
        {.name = "hardware", .outputs = {out("/velocity")}, .inputs = {in("/torque")}},
        {.name = "pid", .outputs = {out("/torque")}, .inputs = {in("/velocity")}},
    };

    const auto plan = build(nodes);
    ASSERT_FALSE(plan.has_value());
    EXPECT_EQ(plan.error().kind, BuildError::Kind::CircularDependency);
    // 诊断要能直接读出「谁卡在哪个接口上」。
    EXPECT_NE(plan.error().message.find("[hardware]"), std::string::npos);
    EXPECT_NE(plan.error().message.find("[pid]"), std::string::npos);
    EXPECT_NE(plan.error().message.find("/velocity"), std::string::npos);
    EXPECT_NE(plan.error().message.find("/torque"), std::string::npos);
}

/// 1/z 的全部意义：同一个环，把其中一条边标成 delayed 就排得开。
/// 这是 Status/Command 拆分的替代品 —— 拆分是把时序问题伪装成组件划分问题，
/// 这里是把它显式地标在那条边上。
TEST(Graph, DelayedInputBreaksTheCycle) {
    const std::vector<NodeDesc> nodes{
        {.name = "hardware", .outputs = {out("/velocity")}, .inputs = {delayed_in("/torque")}},
        {.name = "pid", .outputs = {out("/torque")}, .inputs = {in("/velocity")}},
    };

    const auto plan = build(nodes);
    ASSERT_TRUE(plan.has_value()) << plan.error().message;
    EXPECT_EQ(names_in_order(nodes, plan->order), (std::vector<std::string>{"hardware", "pid"}));
    // 不产生边，但照样接线 —— 它得知道从哪儿取上一拍的值。
    ASSERT_EQ(plan->bindings.size(), 2u);
    for (const auto& binding : plan->bindings)
        EXPECT_NE(binding.producer, kNoProducer);
}

/// 自反馈：组件读自己的输出。普通输入下这是自环（改造前报「循环依赖」），
/// 标成 delayed 之后是完全合法的状态保持器。
TEST(Graph, DelayedInputAllowsSelfFeedback) {
    const std::vector<NodeDesc> self_loop{
        {.name = "integrator", .outputs = {out("/state")}, .inputs = {in("/state")}},
    };
    EXPECT_FALSE(build(self_loop).has_value());

    const std::vector<NodeDesc> delayed{
        {.name = "integrator", .outputs = {out("/state")}, .inputs = {delayed_in("/state")}},
    };
    const auto plan = build(delayed);
    ASSERT_TRUE(plan.has_value()) << plan.error().message;
    EXPECT_EQ(plan->order.size(), 1u);
}

TEST(Graph, EmptyGraphIsValid) {
    const auto plan = build({});
    ASSERT_TRUE(plan.has_value());
    EXPECT_TRUE(plan->order.empty());
    EXPECT_TRUE(plan->bindings.empty());
}
