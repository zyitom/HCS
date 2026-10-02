// RclcppSink：hcs_log 的日志交给 rclcpp 之后，/rosout 上到底收不收得到。
//
// 要钉死的是一条 rcl 不会替你报错的规矩：/rosout 只认"名字和某个节点的 logger 相同"
// 以及"从节点 logger 上 get_child() 出来的"logger。一个凭空的 rclcpp::get_logger("x")
// 打的日志，控制台有、/rosout 上没有——悄无声息。所以不是节点的名字必须挂到 root 下面。
#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <string_view>
#include <thread>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include <rcl_interfaces/msg/log.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>

#include <hcs_base/logging/backend.hpp>
#include <hcs_base/logging/logger.hpp>

#include "../src/rclcpp_log_sink.hpp"

namespace {

using namespace std::chrono_literals;
using hcs_executor::RclcppSink;
using hcs_log::Backend;
using hcs_log::Level;
using hcs_log::Logger;

/// 一个节点（扮演 executor）加一个订阅 /rosout 的旁听者，都在本进程里。
class RosoutBench {
public:
    RosoutBench() {
        listener_subscription_ = listener_->create_subscription<rcl_interfaces::msg::Log>(
            "/rosout", rclcpp::QoS{1000}.reliable().transient_local(),
            [this](const rcl_interfaces::msg::Log& message) {
                // 别的进程（同一个 domain 里别人的节点）的 /rosout 也会进来，只留带标记的。
                if (message.msg.starts_with(kMarker))
                    heard_.emplace_back(message.name, message.msg, message.level);
            });
        executor_.add_node(root_);
        executor_.add_node(component_node_);
        executor_.add_node(listener_);
        spin_for(300ms); // 让 /rosout 的发布端和订阅端先接上
    }

    struct Heard {
        std::string name;
        std::string text;
        std::uint8_t level;
    };

    /// 转到 /rosout 上凑齐 count 条带标记的日志，或者超时。
    std::vector<Heard> wait_for(std::size_t count, std::chrono::milliseconds timeout = 10s) {
        const auto deadline = std::chrono::steady_clock::now() + timeout;
        while (heard_.size() < count && std::chrono::steady_clock::now() < deadline)
            spin_for(20ms);
        return heard_;
    }

    void spin_for(std::chrono::milliseconds duration) {
        const auto deadline = std::chrono::steady_clock::now() + duration;
        while (std::chrono::steady_clock::now() < deadline) {
            executor_.spin_some(10ms);
            std::this_thread::sleep_for(5ms);
        }
    }

    static constexpr std::string_view kMarker = "hcs-sink-test:";

    std::shared_ptr<rclcpp::Node> root_ = std::make_shared<rclcpp::Node>("hcs_sink_test_root");
    std::shared_ptr<rclcpp::Node> component_node_ =
        std::make_shared<rclcpp::Node>("hcs_sink_test_component");

private:
    std::shared_ptr<rclcpp::Node> listener_ =
        std::make_shared<rclcpp::Node>("hcs_sink_test_listener");
    rclcpp::Subscription<rcl_interfaces::msg::Log>::SharedPtr listener_subscription_;
    rclcpp::executors::SingleThreadedExecutor executor_;
    std::vector<Heard> heard_; ///< 只在 spin 的那条线程（测试线程）上读写
};

const RosoutBench::Heard* find(const std::vector<RosoutBench::Heard>& heard, std::string_view text) {
    for (const auto& line : heard)
        if (line.text == text)
            return &line;
    return nullptr;
}

class RclcppSinkTest : public ::testing::Test {
protected:
    static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
    static void TearDownTestSuite() { rclcpp::shutdown(); }
};

} // namespace

// 有 root 的时候：节点的名字原样，其余的挂到 root 下面，两种都上 /rosout，级别对得上。
TEST_F(RclcppSinkTest, EveryNameReachesRosoutOnceAttachedToARoot) {
    RosoutBench bench;
    {
        auto node_loggers = RclcppSink::NodeLoggers{};
        node_loggers.emplace("hcs_sink_test_root", bench.root_->get_logger());
        node_loggers.emplace("hcs_sink_test_component", bench.component_node_->get_logger());
        Backend backend{
            std::make_unique<RclcppSink>(bench.root_->get_logger(), std::move(node_loggers))};

        Logger{backend, "hcs_sink_test_root"}.info("hcs-sink-test: from the root itself");
        Logger{backend, "hcs_sink_test_component"}.warn("hcs-sink-test: from a node component");
        Logger{backend, "not_a_node"}.error("hcs-sink-test: from a plain component {}", 42);
        Logger{backend, "libhcs"}.rt().write(Level::kFatal, "hcs-sink-test: from a library");
        Logger{backend, "not_a_node"}.rt().warn("hcs-sink-test: deferred {}", 7);
        // libhcs 的桥给针对某块板的行起的名字：带点、带板的序列号。
        Logger{backend, "libhcs.AF-90A7"}.rt().write(Level::kError, "hcs-sink-test: from one board");
        backend.flush();

        const auto heard = bench.wait_for(6);
        ASSERT_EQ(heard.size(), 6u);

        using rcl_interfaces::msg::Log;
        const auto* root = find(heard, "hcs-sink-test: from the root itself");
        ASSERT_NE(root, nullptr);
        EXPECT_EQ(root->name, "hcs_sink_test_root");
        EXPECT_EQ(root->level, Log::INFO);

        const auto* component = find(heard, "hcs-sink-test: from a node component");
        ASSERT_NE(component, nullptr);
        EXPECT_EQ(component->name, "hcs_sink_test_component"); // 名字原样，没有被挂到 root 下
        EXPECT_EQ(component->level, Log::WARN);

        const auto* plain = find(heard, "hcs-sink-test: from a plain component 42");
        ASSERT_NE(plain, nullptr);
        EXPECT_EQ(plain->name, "hcs_sink_test_root.not_a_node");
        EXPECT_EQ(plain->level, Log::ERROR);

        const auto* library = find(heard, "hcs-sink-test: from a library");
        ASSERT_NE(library, nullptr);
        EXPECT_EQ(library->name, "hcs_sink_test_root.libhcs");
        EXPECT_EQ(library->level, Log::FATAL);

        const auto* deferred = find(heard, "hcs-sink-test: deferred 7");
        ASSERT_NE(deferred, nullptr);
        EXPECT_EQ(deferred->name, "hcs_sink_test_root.not_a_node");

        const auto* board = find(heard, "hcs-sink-test: from one board");
        ASSERT_NE(board, nullptr);
        EXPECT_EQ(board->name, "hcs_sink_test_root.libhcs.AF-90A7");
        EXPECT_EQ(board->level, Log::ERROR);
    }
}

// 没有 root 的时候（节点建起来之前的那一小段）：日志不丢、不崩，只是凭空的名字上不了 /rosout。
// 这一条同时把"为什么需要 root"这件事钉在测试里——哪天 rcl 改了规矩，这里会先知道。
TEST_F(RclcppSinkTest, WithoutARootOnlyNodeNamesReachRosout) {
    RosoutBench bench;
    {
        Backend backend{std::make_unique<RclcppSink>()};
        Logger{backend, "not_a_node"}.warn("hcs-sink-test: orphan");
        Logger{backend, "hcs_sink_test_component"}.warn("hcs-sink-test: same name as a node");
        backend.flush();

        const auto heard = bench.wait_for(1);
        bench.spin_for(500ms); // 再等一会儿：要证明的是"那一条没有来"
        const auto settled = bench.wait_for(1, 0ms);

        EXPECT_NE(find(settled, "hcs-sink-test: same name as a node"), nullptr);
        EXPECT_EQ(find(settled, "hcs-sink-test: orphan"), nullptr);
    }
}

// RclcppLogScope：活着的时候走 rclcpp 且不拦 DEBUG；restore 之后换回原来的输出端和级别；
// attach 在 restore 之后是空操作（不会把已经换回去的输出端又换成 rclcpp 的）。
TEST_F(RclcppSinkTest, ScopeSwapsTheSinkAndPutsItBack) {
    struct Counting final : hcs_log::Sink {
        explicit Counting(std::shared_ptr<std::atomic<int>> count)
            : count_(std::move(count)) {}
        void write(const hcs_log::Entry&) override { count_->fetch_add(1); }
        std::shared_ptr<std::atomic<int>> count_;
    };

    RosoutBench bench;
    auto to_original = std::make_shared<std::atomic<int>>(0);
    Backend backend{std::make_unique<Counting>(to_original)};
    const Logger logger{backend, "hcs_sink_test_root"};

    {
        hcs_executor::RclcppLogScope scope{backend};
        EXPECT_EQ(backend.threshold(), Level::kDebug);

        scope.attach(bench.root_->get_logger(), {});
        logger.info("hcs-sink-test: while the scope is alive");
        backend.flush();
        EXPECT_EQ(to_original->load(), 0);

        scope.restore();
        EXPECT_EQ(backend.threshold(), Level::kInfo);
        scope.attach(bench.root_->get_logger(), {}); // restore 之后：空操作
        logger.info("after restore");
        backend.flush();
        EXPECT_EQ(to_original->load(), 1);

        scope.restore(); // 幂等
    }
    logger.info("after the scope");
    backend.flush();
    EXPECT_EQ(to_original->load(), 2);

    const auto heard = bench.wait_for(1);
    ASSERT_EQ(heard.size(), 1u);
    EXPECT_EQ(heard[0].text, "hcs-sink-test: while the scope is alive");
}
