// 会合的单测：两个进程互换通道 fd 之后能互相读到；对端退出能察觉；
// 对端发来不合规的消息时拒绝，并且它塞过来的 fd 一个都不漏。

#include <dirent.h>
#include <sys/socket.h>
#include <sys/wait.h>
#include <unistd.h>

#include <array>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <string>
#include <string_view>
#include <thread>

#include <gtest/gtest.h>

#include "hcs_link/channel.hpp"
#include "hcs_link/rendezvous.hpp"

namespace {

using namespace std::chrono_literals;
using hcs_link::ErrorCode;
using hcs_link::UniqueFd;

struct Ping {
    static constexpr std::string_view kName = "test.Ping";
    static constexpr std::uint32_t kVersion = 1;
    std::uint64_t value;
};

struct Pong {
    static constexpr std::string_view kName = "test.Pong";
    static constexpr std::uint32_t kVersion = 1;
    std::uint64_t value;
};

constexpr hcs_link::WriterOptions kWriterOptions{.capacity = 16, .lock_memory = false, .name = "test"};
constexpr hcs_link::ReaderOptions kReaderOptions{.lock_memory = false};

/// 每个用例一个独立的名字，并行跑也不会撞。
std::string unique_name(std::string_view tag) {
    return "hcs-link-test/" + std::string{tag} + "/" + std::to_string(::getpid());
}

std::size_t open_fd_count() {
    std::size_t count = 0;
    if (DIR* directory = ::opendir("/proc/self/fd")) {
        while (::readdir(directory) != nullptr)
            ++count;
        ::closedir(directory);
    }
    return count;
}

TEST(Rendezvous, ProcessesReadEachOthersChannels) {
    const auto name = unique_name("exchange");
    auto listener = hcs_link::Listener::bind(name);
    ASSERT_TRUE(listener.has_value()) << listener.error().message();

    const pid_t child = ::fork();
    ASSERT_GE(child, 0);
    if (child == 0) {
        // 视觉那一头：连上，递出自己写的 Pong，拿到对方的 Ping，读到 Ping 就回一个 +1。
        int code = 1;
        auto socket = hcs_link::connect(name);
        auto pong = hcs_link::Writer<Pong>::create(kWriterOptions);
        if (socket && pong) {
            const std::array fds{pong->fd()};
            if (auto received = hcs_link::exchange(socket->get(), fds, 2s);
                received && received->count == 1) {
                if (auto ping = hcs_link::Reader<Ping>::attach(std::move(received->fds[0]), kReaderOptions)) {
                    const auto deadline = std::chrono::steady_clock::now() + 5s;
                    while (std::chrono::steady_clock::now() < deadline) {
                        if (const auto sample = ping->latest()) {
                            pong->publish(Pong{sample->value.value + 1});
                            code = 0;
                            break;
                        }
                        std::this_thread::sleep_for(1ms);
                    }
                    // 等控制那一头读完再走
                    while (!hcs_link::peer_closed(socket->get())
                           && std::chrono::steady_clock::now() < deadline)
                        std::this_thread::sleep_for(1ms);
                }
            }
        }
        ::_exit(code);
    }

    // 控制那一头
    auto ping = hcs_link::Writer<Ping>::create(kWriterOptions);
    ASSERT_TRUE(ping.has_value());
    auto socket = listener->accept(5s);
    ASSERT_TRUE(socket.has_value()) << socket.error().message();

    const std::array fds{ping->fd()};
    auto received = hcs_link::exchange(socket->get(), fds, 5s);
    ASSERT_TRUE(received.has_value()) << received.error().message();
    ASSERT_EQ(received->count, 1U);

    auto pong = hcs_link::Reader<Pong>::attach(std::move(received->fds[0]), kReaderOptions);
    ASSERT_TRUE(pong.has_value()) << pong.error().message();

    ping->publish(Ping{41});
    std::optional<hcs_link::Sample<Pong>> answer;
    const auto deadline = std::chrono::steady_clock::now() + 5s;
    while (!answer && std::chrono::steady_clock::now() < deadline) {
        answer = pong->latest();
        std::this_thread::sleep_for(1ms);
    }
    ASSERT_TRUE(answer.has_value());
    EXPECT_EQ(answer->value.value, 42U);

    socket->reset(); // 让对端看到 EOF
    int status = 0;
    ASSERT_EQ(::waitpid(child, &status, 0), child);
    EXPECT_TRUE(WIFEXITED(status));
    EXPECT_EQ(WEXITSTATUS(status), 0);
}

TEST(Rendezvous, PeerClosedIsDetected) {
    std::array<int, 2> pair{};
    ASSERT_EQ(::socketpair(AF_UNIX, SOCK_SEQPACKET | SOCK_CLOEXEC, 0, pair.data()), 0);
    UniqueFd ours{pair[0]};
    UniqueFd theirs{pair[1]};

    EXPECT_FALSE(hcs_link::peer_closed(ours.get()));
    theirs.reset();
    EXPECT_TRUE(hcs_link::peer_closed(ours.get()));
}

TEST(Rendezvous, SecondListenerOnTheSameNameFails) {
    const auto name = unique_name("twice");
    const auto first = hcs_link::Listener::bind(name);
    ASSERT_TRUE(first.has_value());
    const auto second = hcs_link::Listener::bind(name);
    ASSERT_FALSE(second.has_value());
    EXPECT_EQ(second.error().system_errno, EADDRINUSE);
}

TEST(Rendezvous, ConnectWithoutListenerFails) {
    const auto socket = hcs_link::connect(unique_name("nobody"));
    ASSERT_FALSE(socket.has_value());
    EXPECT_EQ(socket.error().system_errno, ECONNREFUSED);
}

TEST(Rendezvous, AcceptTimesOut) {
    const auto listener = hcs_link::Listener::bind(unique_name("idle"));
    ASSERT_TRUE(listener.has_value());
    const auto socket = listener->accept(10ms);
    ASSERT_FALSE(socket.has_value());
    EXPECT_EQ(socket.error().code, ErrorCode::kTimeout);
}

TEST(Rendezvous, MalformedHelloIsRejectedWithoutLeakingFds) {
    std::array<int, 2> pair{};
    ASSERT_EQ(::socketpair(AF_UNIX, SOCK_SEQPACKET | SOCK_CLOEXEC, 0, pair.data()), 0);
    UniqueFd ours{pair[0]};
    UniqueFd theirs{pair[1]};

    const auto before = open_fd_count();

    // 对端发一条长度不对的消息，还顺手塞过来一个 fd。
    std::array<char, 3> junk{'b', 'a', 'd'};
    iovec payload{.iov_base = junk.data(), .iov_len = junk.size()};
    alignas(cmsghdr) std::array<std::byte, CMSG_SPACE(sizeof(int))> control{};
    msghdr message{};
    message.msg_iov = &payload;
    message.msg_iovlen = 1;
    message.msg_control = control.data();
    message.msg_controllen = control.size();
    cmsghdr* header = CMSG_FIRSTHDR(&message);
    header->cmsg_level = SOL_SOCKET;
    header->cmsg_type = SCM_RIGHTS;
    header->cmsg_len = CMSG_LEN(sizeof(int));
    const int smuggled = STDIN_FILENO;
    std::memcpy(CMSG_DATA(header), &smuggled, sizeof smuggled);
    ASSERT_EQ(::sendmsg(theirs.get(), &message, MSG_NOSIGNAL), static_cast<ssize_t>(junk.size()));

    const auto received = hcs_link::exchange(ours.get(), {}, 1s);
    ASSERT_FALSE(received.has_value());
    EXPECT_EQ(received.error().code, ErrorCode::kProtocol);

    EXPECT_EQ(open_fd_count(), before);
}

TEST(Rendezvous, ExchangeTimesOutWhenThePeerStaysSilent) {
    std::array<int, 2> pair{};
    ASSERT_EQ(::socketpair(AF_UNIX, SOCK_SEQPACKET | SOCK_CLOEXEC, 0, pair.data()), 0);
    UniqueFd ours{pair[0]};
    UniqueFd theirs{pair[1]};

    const auto received = hcs_link::exchange(ours.get(), {}, 20ms);
    ASSERT_FALSE(received.has_value());
    EXPECT_EQ(received.error().code, ErrorCode::kTimeout);
}

} // namespace
