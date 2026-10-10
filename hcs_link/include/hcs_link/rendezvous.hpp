#pragma once

// 会合：两个进程在一个 abstract unix socket 上各递一次自己写的通道 fd，之后数据全走通道。
//
// socket 只在这一刻有用，连接留着只是为了让对端退出时能被察觉（peer_closed）。
// abstract 名挂在 network namespace 上，不落文件系统，进程崩了也没有残留。
// abstract socket 没有文件权限，所以两端都用 SO_PEERCRED 核对 uid。

#include <poll.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <expected>
#include <span>
#include <string_view>
#include <utility>

#include "hcs_link/channel.hpp"
#include "hcs_link/error.hpp"
#include "hcs_link/unique_fd.hpp"

namespace hcs_link {

inline constexpr std::size_t kMaxExchangedFds = 4;

/// 对端递过来的 fd，全部已经装进 UniqueFd：任何一步出错都不会漏 fd。
struct ReceivedFds {
    std::array<UniqueFd, kMaxExchangedFds> fds{};
    std::size_t count = 0;
};

namespace detail {

struct Hello {
    Word magic;
    std::uint32_t protocol;
    std::uint32_t fd_count;
};
static_assert(sizeof(Hello) == 16);

inline constexpr std::size_t kControlBytes = CMSG_SPACE(sizeof(int) * kMaxExchangedFds);

struct Address {
    sockaddr_un address{};
    socklen_t length = 0;
};

[[nodiscard]] inline std::expected<Address, Error> abstract_address(std::string_view name) {
    Address result;
    if (name.empty() || name.size() + 1 > sizeof(result.address.sun_path))
        return std::unexpected(Error{.code = ErrorCode::kInvalidArgument, .where = "socket name"});
    result.address.sun_family = AF_UNIX;
    result.address.sun_path[0] = '\0'; // abstract：首字节为 0
    std::memcpy(result.address.sun_path + 1, name.data(), name.size());
    result.length = static_cast<socklen_t>(offsetof(sockaddr_un, sun_path) + 1 + name.size());
    return result;
}

[[nodiscard]] inline std::expected<void, Error> check_peer_uid(int socket) {
    ucred credentials{};
    socklen_t length = sizeof credentials;
    if (::getsockopt(socket, SOL_SOCKET, SO_PEERCRED, &credentials, &length) != 0)
        return std::unexpected(Error::from_errno("getsockopt(SO_PEERCRED)"));
    if (credentials.uid != ::geteuid())
        return std::unexpected(Error{.code = ErrorCode::kPeerRejected});
    return {};
}

[[nodiscard]] inline std::expected<void, Error>
    wait_readable(int fd, std::chrono::milliseconds timeout, std::string_view where) {
    pollfd descriptor{.fd = fd, .events = POLLIN, .revents = 0};
    for (;;) {
        const int ready = ::poll(&descriptor, 1, static_cast<int>(timeout.count()));
        if (ready > 0)
            return {};
        if (ready == 0)
            return std::unexpected(Error{.code = ErrorCode::kTimeout, .where = where});
        if (errno != EINTR)
            return std::unexpected(Error::from_errno("poll"));
    }
}

[[nodiscard]] inline std::expected<UniqueFd, Error> make_socket() {
    UniqueFd socket{::socket(AF_UNIX, SOCK_SEQPACKET | SOCK_CLOEXEC, 0)};
    if (!socket)
        return std::unexpected(Error::from_errno("socket"));
    return socket;
}

} // namespace detail

/// 监听端。控制进程在普通线程里用它。
class Listener {
public:
    /// 同一 network namespace 里名字已被占用时失败（errno = EADDRINUSE），顺带防止起两个控制进程。
    [[nodiscard]] static std::expected<Listener, Error> bind(std::string_view name) {
        auto address = detail::abstract_address(name);
        if (!address)
            return std::unexpected(address.error());
        auto socket = detail::make_socket();
        if (!socket)
            return std::unexpected(socket.error());
        if (::bind(socket->get(), reinterpret_cast<const sockaddr*>(&address->address), address->length)
            != 0)
            return std::unexpected(Error::from_errno("bind"));
        if (::listen(socket->get(), 4) != 0)
            return std::unexpected(Error::from_errno("listen"));
        return Listener{std::move(*socket)};
    }

    /// 等一个同 uid 的对端连上来，最多等 timeout。
    [[nodiscard]] std::expected<UniqueFd, Error> accept(std::chrono::milliseconds timeout) const {
        if (auto ready = detail::wait_readable(socket_.get(), timeout, "accept"); !ready)
            return std::unexpected(ready.error());
        UniqueFd peer{::accept4(socket_.get(), nullptr, nullptr, SOCK_CLOEXEC)};
        if (!peer)
            return std::unexpected(Error::from_errno("accept4"));
        if (auto checked = detail::check_peer_uid(peer.get()); !checked)
            return std::unexpected(checked.error());
        return peer;
    }

    /// 给 poll 用。
    [[nodiscard]] int fd() const noexcept { return socket_.get(); }

private:
    explicit Listener(UniqueFd socket) noexcept
        : socket_{std::move(socket)} {}

    UniqueFd socket_;
};

/// 连接端。没人监听时失败（errno = ECONNREFUSED），调用方隔一会儿重试即可。
[[nodiscard]] inline std::expected<UniqueFd, Error> connect(std::string_view name) {
    auto address = detail::abstract_address(name);
    if (!address)
        return std::unexpected(address.error());
    auto socket = detail::make_socket();
    if (!socket)
        return std::unexpected(socket.error());
    if (::connect(socket->get(), reinterpret_cast<const sockaddr*>(&address->address), address->length)
        != 0)
        return std::unexpected(Error::from_errno("connect"));
    if (auto checked = detail::check_peer_uid(socket->get()); !checked)
        return std::unexpected(checked.error());
    return std::move(*socket);
}

/// 双方各发一条 Hello 连同自己的 fd，再收对端的。对称：两端调用方式完全一样，谁先谁后都行。
[[nodiscard]] inline std::expected<ReceivedFds, Error>
    exchange(int socket, std::span<const int> fds, std::chrono::milliseconds timeout) {
    if (fds.size() > kMaxExchangedFds)
        return std::unexpected(Error{.code = ErrorCode::kInvalidArgument, .where = "too many fds"});

    {
        detail::Hello hello{
            .magic = detail::kMagic,
            .protocol = detail::kProtocol,
            .fd_count = static_cast<std::uint32_t>(fds.size()),
        };
        iovec payload{.iov_base = &hello, .iov_len = sizeof hello};
        alignas(cmsghdr) std::array<std::byte, detail::kControlBytes> control{};
        msghdr message{};
        message.msg_iov = &payload;
        message.msg_iovlen = 1;
        if (!fds.empty()) {
            message.msg_control = control.data();
            message.msg_controllen = CMSG_SPACE(sizeof(int) * fds.size());
            cmsghdr* header = CMSG_FIRSTHDR(&message);
            header->cmsg_level = SOL_SOCKET;
            header->cmsg_type = SCM_RIGHTS;
            header->cmsg_len = CMSG_LEN(sizeof(int) * fds.size());
            std::memcpy(CMSG_DATA(header), fds.data(), sizeof(int) * fds.size());
        }
        // MSG_NOSIGNAL：对端中途退出时，SIGPIPE 的默认动作会杀掉整个进程。
        if (::sendmsg(socket, &message, MSG_NOSIGNAL) != static_cast<ssize_t>(sizeof hello))
            return std::unexpected(Error::from_errno("sendmsg"));
    }

    if (auto ready = detail::wait_readable(socket, timeout, "exchange"); !ready)
        return std::unexpected(ready.error());

    detail::Hello hello{};
    iovec payload{.iov_base = &hello, .iov_len = sizeof hello};
    alignas(cmsghdr) std::array<std::byte, detail::kControlBytes> control{};
    msghdr message{};
    message.msg_iov = &payload;
    message.msg_iovlen = 1;
    message.msg_control = control.data();
    message.msg_controllen = control.size();

    const ssize_t received = ::recvmsg(socket, &message, MSG_CMSG_CLOEXEC | MSG_DONTWAIT);
    if (received < 0)
        return std::unexpected(Error::from_errno("recvmsg"));
    if (received == 0)
        return std::unexpected(Error{.code = ErrorCode::kClosed, .where = "exchange"});

    // 先把收到的 fd 全部接管，再做任何校验。
    ReceivedFds result;
    for (cmsghdr* header = CMSG_FIRSTHDR(&message); header != nullptr;
         header = CMSG_NXTHDR(&message, header)) {
        if (header->cmsg_level != SOL_SOCKET || header->cmsg_type != SCM_RIGHTS)
            continue;
        const std::size_t count = (header->cmsg_len - CMSG_LEN(0)) / sizeof(int);
        for (std::size_t i = 0; i < count; ++i) {
            int fd = -1;
            std::memcpy(&fd, CMSG_DATA(header) + i * sizeof(int), sizeof fd);
            if (result.count < kMaxExchangedFds)
                result.fds[result.count++].reset(fd);
            else
                ::close(fd);
        }
    }

    if (received != static_cast<ssize_t>(sizeof hello)
        || (message.msg_flags & (MSG_TRUNC | MSG_CTRUNC)) != 0)
        return std::unexpected(Error{.code = ErrorCode::kProtocol, .where = "hello size"});
    if (hello.magic != detail::kMagic || hello.protocol != detail::kProtocol)
        return std::unexpected(Error{.code = ErrorCode::kProtocol, .where = "hello magic"});
    if (hello.fd_count != result.count)
        return std::unexpected(Error{.code = ErrorCode::kProtocol, .where = "fd count"});
    return result;
}

/// 对端是否已经关闭。不阻塞。
[[nodiscard]] inline bool peer_closed(int socket) noexcept {
    pollfd descriptor{.fd = socket, .events = POLLIN | POLLRDHUP, .revents = 0};
    if (::poll(&descriptor, 1, 0) <= 0)
        return false;
    if ((descriptor.revents & (POLLHUP | POLLRDHUP | POLLERR | POLLNVAL)) != 0)
        return true;
    if ((descriptor.revents & POLLIN) != 0) {
        std::byte probe{};
        return ::recv(socket, &probe, 1, MSG_PEEK | MSG_DONTWAIT) == 0;
    }
    return false;
}

} // namespace hcs_link
