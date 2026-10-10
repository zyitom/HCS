#pragma once

#include <cerrno>
#include <cstdint>
#include <string>
#include <string_view>
#include <system_error>

namespace hcs_link {

enum class ErrorCode : std::uint8_t {
    kSystem,          ///< 系统调用失败，见 Error::system_errno
    kInvalidArgument, ///< 调用方给的参数不合法
    kBadSeals,        ///< 对端给的 memfd 没封好：写者还能截断或打洞，读它可能 SIGBUS 或缺页
    kBadHeader,       ///< 魔数或协议版本不对
    kPayloadMismatch, ///< 载荷的名字、版本或大小与本端编译进来的不一致
    kBadGeometry,     ///< 容量或段大小不自洽
    kProtocol,        ///< 握手消息不合规
    kPeerRejected,    ///< 对端 uid 与本进程不同
    kTimeout,
    kClosed, ///< 对端已关闭
};

[[nodiscard]] constexpr std::string_view to_string(ErrorCode code) noexcept {
    switch (code) {
    case ErrorCode::kSystem: return "system call failed";
    case ErrorCode::kInvalidArgument: return "invalid argument";
    case ErrorCode::kBadSeals: return "memfd is not sealed against shrink/grow/write";
    case ErrorCode::kBadHeader: return "bad magic or protocol version";
    case ErrorCode::kPayloadMismatch: return "payload name, version or size mismatch";
    case ErrorCode::kBadGeometry: return "inconsistent capacity or segment size";
    case ErrorCode::kProtocol: return "malformed handshake";
    case ErrorCode::kPeerRejected: return "peer runs under a different uid";
    case ErrorCode::kTimeout: return "timed out";
    case ErrorCode::kClosed: return "peer closed";
    }
    return "unknown error";
}

struct Error {
    ErrorCode code = ErrorCode::kSystem;
    int system_errno = 0;
    std::string_view where{}; ///< 静态字符串：哪个系统调用或哪一步

    [[nodiscard]] static Error from_errno(std::string_view where) noexcept {
        return Error{.code = ErrorCode::kSystem, .system_errno = errno, .where = where};
    }

    [[nodiscard]] std::string message() const {
        std::string text{to_string(code)};
        if (!where.empty())
            text.append(" (").append(where).append(")");
        if (system_errno != 0)
            text.append(": ").append(std::system_category().message(system_errno));
        return text;
    }
};

} // namespace hcs_link
