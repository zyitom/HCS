#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>

#include <concepts>
#include <new>

namespace hcs_utility {

enum class ReceiveResult : uint8_t {
    SUCCESS = 0,
    TIMEOUT = 1,
    HEADER_INVALID = 2,
    VERIFY_INVALID = 3
};

template <typename T>
concept is_byte =
    std::is_same_v<T, char> || std::is_same_v<T, unsigned char> || std::is_same_v<T, std::byte>;

template <typename SerialT, typename ByteT>
concept is_readable_stream = requires(SerialT& serial, ByteT* pointer, size_t size) {
    { serial.read(pointer, size) } -> std::convertible_to<size_t>;
};

template <typename F, typename PackageT>
concept is_verify_function = requires(const F& f, const PackageT& package) {
    { f(package) } -> std::convertible_to<bool>;
};

/// 逐字节重同步:丢掉缓存头部直到 header_verify 通过(或剩余不足一个头)。
///
/// 两段式 —— 先在原地逐偏移探测、最后一次 memmove 搬移。逐字节逐次搬整个缓存
/// 是 O(n²):这个函数跑在 RT 拍内,线上出现连续垃圾字节时最坏工作量会平方级
/// 膨胀;探测每个偏移不可避免,重复搬移可以避免。
///
/// @param buffer_pointer   缓存首字节(类型双关到 PackageT 的同一段内存)
/// @param cache_size       进出参数:当前有效字节数,返回时为重同步后剩余字节数
/// @param header_size      头部长度;0 表示没有可搜的头,只退一格保证进展
template <size_t header_size, typename PackageT, typename Verify>
static void resynchronize(
    std::byte* buffer_pointer, size_t& cache_size, const Verify& header_verify) {
    if (cache_size == 0)
        return;

    size_t drop = 1; // 无条件先丢一个字节:与流式语义一致,保证每次调用都有进展
    if (header_size != 0) {
        while (drop < cache_size && cache_size - drop >= header_size
               && !header_verify(
                   *std::launder(reinterpret_cast<PackageT*>(buffer_pointer + drop)))) {
            ++drop;
        }
    }

    cache_size -= drop;
    if (cache_size != 0)
        std::memmove(buffer_pointer, buffer_pointer + drop, cache_size);
}

/// @brief 从字节流接收一个定长包,头部不对则逐字节滑动重同步。
///
/// @param stream     满足 is_readable_stream 的流;read 非阻塞,读到多少算多少
/// @param buffer     包对象;头部先落在这里,再补齐整包
/// @param cache_size 跨调用保持的缓存字节数,调用方初始化为 0
/// @param header_verify 头部校验。**只允许触碰 buffer 的前 header_size 个字节** ——
///                      它被调到时包可能还没收满,后面的字节是未初始化的
/// @param verify     整包校验,只在包收满后调用
template <size_t header_size, is_byte ByteT, typename PackageT>
requires std::is_trivially_copyable_v<PackageT> inline auto receive_package(
    is_readable_stream<ByteT> auto& stream, PackageT& buffer, size_t& cache_size,
    const is_verify_function<PackageT> auto& header_verify,
    const is_verify_function<PackageT> auto& verify) -> ReceiveResult {
    if (cache_size == sizeof(PackageT))
        return ReceiveResult::SUCCESS;

    auto* buffer_pointer = reinterpret_cast<ByteT*>(&buffer);
    cache_size += stream.read(buffer_pointer + cache_size, sizeof(PackageT) - cache_size);

    if (cache_size == 0 || cache_size < header_size)
        return ReceiveResult::TIMEOUT;

    ReceiveResult result;
    if (header_size == 0 || header_verify(buffer)) {
        if (cache_size != sizeof(PackageT))
            return ReceiveResult::TIMEOUT;
        if (verify(buffer))
            return ReceiveResult::SUCCESS;
        else
            result = ReceiveResult::VERIFY_INVALID;
    } else {
        result = ReceiveResult::HEADER_INVALID;
    }

    resynchronize<header_size, PackageT>(
        reinterpret_cast<std::byte*>(buffer_pointer), cache_size, header_verify);
    return result;
}

template <is_byte ByteT, typename PackageT, std::integral HeaderT>
requires std::is_trivially_copyable_v<PackageT> inline auto receive_package(
    is_readable_stream<ByteT> auto& stream, PackageT& buffer, size_t& cache_size, HeaderT header,
    const is_verify_function<PackageT> auto& verify) -> ReceiveResult {
    return receive_package<sizeof(HeaderT), ByteT>(
        stream, buffer, cache_size,
        [header](const PackageT& package) {
            HeaderT actual_header;
            std::memcpy(&actual_header, &package, sizeof(HeaderT));
            return actual_header == header;
        },
        verify);
}

} // namespace hcs_utility
