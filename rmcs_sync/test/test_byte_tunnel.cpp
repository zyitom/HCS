// ByteTunnel 不解析任何协议，所以它唯一的职责就是"进去什么字节，出来就是什么字节"。
// 回绕和部分写是这类环最容易写错的两处，压测那条则盯住跨线程的逐字节等价。
#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <span>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include <rmcs_sync/byte_tunnel.hpp>

namespace {

using rmcs_sync::ByteTunnel;

static_assert(ByteTunnel<8>::capacity == 8);
static_assert(ByteTunnel<1024>::capacity == 1024);

/// 位置 i 上应该出现的字节。两端各自算，不用互相传数据就能核对。
[[nodiscard]] constexpr std::byte pattern_at(std::size_t index) noexcept {
    const std::uint32_t mixed =
        static_cast<std::uint32_t>(index) * 1103515245u + 12345u;
    return static_cast<std::byte>((mixed >> 16) & 0xffu);
}

[[nodiscard]] constexpr std::byte byte_of(int value) noexcept {
    return static_cast<std::byte>(static_cast<unsigned char>(value));
}

TEST(ByteTunnel, EmptyOnConstruction) {
    ByteTunnel<8> tunnel;
    std::array<std::byte, 4> out{};

    EXPECT_EQ(tunnel.readable(), std::size_t{0});
    EXPECT_EQ(tunnel.writable(), std::size_t{8});
    EXPECT_EQ(tunnel.overrun_bytes(), std::uint64_t{0});
    EXPECT_EQ(tunnel.read(out), std::size_t{0});
}

TEST(ByteTunnel, WriteThenReadRoundTrip) {
    ByteTunnel<8> tunnel;
    const std::array<std::byte, 5> data{byte_of(1), byte_of(2), byte_of(3), byte_of(4),
                                        byte_of(5)};

    EXPECT_EQ(tunnel.write(data), std::size_t{5});
    EXPECT_EQ(tunnel.readable(), std::size_t{5});
    EXPECT_EQ(tunnel.writable(), std::size_t{3});

    // 部分读：只取走 3 个，剩下的必须原地留着。
    std::array<std::byte, 3> first{};
    EXPECT_EQ(tunnel.read(first), std::size_t{3});
    EXPECT_EQ(first[0], byte_of(1));
    EXPECT_EQ(first[1], byte_of(2));
    EXPECT_EQ(first[2], byte_of(3));
    EXPECT_EQ(tunnel.readable(), std::size_t{2});

    // 读得比有的多：只给已有的那些，不许越界补数。
    std::array<std::byte, 16> rest{};
    EXPECT_EQ(tunnel.read(rest), std::size_t{2});
    EXPECT_EQ(rest[0], byte_of(4));
    EXPECT_EQ(rest[1], byte_of(5));
    EXPECT_EQ(rest[2], std::byte{0}); // 多出来的部分不许被写
    EXPECT_EQ(tunnel.readable(), std::size_t{0});
    EXPECT_EQ(tunnel.overrun_bytes(), std::uint64_t{0});
}

TEST(ByteTunnel, PartialWriteCountsTheRestAsOverrun) {
    ByteTunnel<8> tunnel;
    std::array<std::byte, 10> data{};
    for (std::size_t i = 0; i < data.size(); ++i)
        data[i] = byte_of(static_cast<int>(i));

    // 只写得下 8 个，剩下 2 个计入 overrun 并丢掉 —— 丢弃是显式策略。
    EXPECT_EQ(tunnel.write(data), std::size_t{8});
    EXPECT_EQ(tunnel.overrun_bytes(), std::uint64_t{2});
    EXPECT_EQ(tunnel.writable(), std::size_t{0});

    // 满了以后再写：一个字节都进不去，全部计数。
    EXPECT_EQ(tunnel.write(std::span<const std::byte>{data.data(), 4}), std::size_t{0});
    EXPECT_EQ(tunnel.overrun_bytes(), std::uint64_t{6});

    // 丢掉的是尾巴，已写进去的 8 个必须完好。
    std::array<std::byte, 8> out{};
    EXPECT_EQ(tunnel.read(out), std::size_t{8});
    for (std::size_t i = 0; i < out.size(); ++i)
        EXPECT_EQ(out[i], byte_of(static_cast<int>(i))) << "第 " << i << " 个字节";
}

TEST(ByteTunnel, WrapsAroundWithoutCorruption) {
    // 容量 8，每轮写 6 读 6：第二轮起写入必然跨越回绕点，两段 memcpy 有一段错就露馅。
    ByteTunnel<8> tunnel;

    for (int round = 0; round < 200; ++round) {
        std::array<std::byte, 6> data{};
        for (std::size_t i = 0; i < data.size(); ++i)
            data[i] = byte_of(round * 6 + static_cast<int>(i));

        ASSERT_EQ(tunnel.write(data), std::size_t{6});
        ASSERT_EQ(tunnel.readable(), std::size_t{6});

        std::array<std::byte, 6> out{};
        ASSERT_EQ(tunnel.read(out), std::size_t{6});
        for (std::size_t i = 0; i < out.size(); ++i)
            ASSERT_EQ(out[i], data[i]) << "round=" << round << " index=" << i;
        ASSERT_EQ(tunnel.readable(), std::size_t{0});
    }
    EXPECT_EQ(tunnel.overrun_bytes(), std::uint64_t{0});
}

TEST(ByteTunnel, InterleavedPartialWriteAndReadKeepsStreamOrder) {
    // 写多少读多少完全不对齐，逼着 head/tail 停在各种偏移上。
    ByteTunnel<16> tunnel;
    std::size_t written = 0;
    std::size_t consumed = 0;

    for (int round = 0; round < 500; ++round) {
        const std::size_t chunk = static_cast<std::size_t>(round % 7) + 1;
        std::array<std::byte, 8> data{};
        for (std::size_t i = 0; i < chunk; ++i)
            data[i] = pattern_at(written + i);
        written += tunnel.write(std::span<const std::byte>{data.data(), chunk});

        const std::size_t want = static_cast<std::size_t>(round % 5) + 1;
        std::array<std::byte, 8> out{};
        const std::size_t got = tunnel.read(std::span<std::byte>{out.data(), want});
        ASSERT_LE(got, want);
        for (std::size_t i = 0; i < got; ++i)
            ASSERT_EQ(out[i], pattern_at(consumed + i))
                << "round=" << round << " index=" << consumed + i;
        consumed += got;
    }
    EXPECT_GT(consumed, std::size_t{0});
    EXPECT_EQ(tunnel.readable(), written - consumed);
}

TEST(ByteTunnel, ConcurrentStreamIsByteForByteIdentical) {
    constexpr std::size_t kTotal = 1u << 23; // 8 MiB，单次压测控制在 2s 以内
    ByteTunnel<1024> tunnel;

    std::atomic<bool> writer_done{false};
    std::atomic<bool> writer_stuck{false};
    std::atomic<bool> short_write{false};

    std::thread writer([&] {
        const auto  deadline = std::chrono::steady_clock::now() + std::chrono::seconds{5};
        std::array<std::byte, 128> chunk{};
        std::size_t                produced = 0;
        while (produced < kTotal) {
            const std::size_t size =
                std::min(kTotal - produced, static_cast<std::size_t>(produced % 128) + 1);
            for (std::size_t i = 0; i < size; ++i)
                chunk[i] = pattern_at(produced + i);

            // 先等出足够的空位再整块写：这样任何 overrun 都只可能来自实现的记账错误。
            // （SPSC 下 writable() 只会低估——消费者只会让空位变多——所以这个门是安全的。）
            while (tunnel.writable() < size) {
                if (std::chrono::steady_clock::now() > deadline) {
                    writer_stuck.store(true, std::memory_order_release);
                    writer_done.store(true, std::memory_order_release);
                    return;
                }
            }
            if (tunnel.write(std::span<const std::byte>{chunk.data(), size}) != size)
                short_write.store(true, std::memory_order_release);
            produced += size;
        }
        writer_done.store(true, std::memory_order_release);
    });

    std::vector<std::byte> out(256);
    std::size_t            consumed = 0;
    std::size_t            mismatch = kTotal; // kTotal 表示没有不匹配
    for (;;) {
        const bool        finished = writer_done.load(std::memory_order_acquire);
        const std::size_t got = tunnel.read(std::span<std::byte>{out.data(), out.size()});
        for (std::size_t i = 0; i < got; ++i) {
            if (out[i] != pattern_at(consumed + i) && mismatch == kTotal)
                mismatch = consumed + i;
        }
        consumed += got;
        if (got == 0 && finished)
            break;
    }
    writer.join();

    ASSERT_FALSE(writer_stuck.load(std::memory_order_acquire)) << "写者卡死：隧道不再放行";
    EXPECT_FALSE(short_write.load(std::memory_order_acquire))
        << "writable() 说放得下，write() 却没写完";
    EXPECT_EQ(mismatch, kTotal) << "第 " << mismatch << " 个字节和源不一致";
    EXPECT_EQ(consumed, kTotal);
    EXPECT_EQ(tunnel.overrun_bytes(), std::uint64_t{0});
    EXPECT_EQ(tunnel.readable(), std::size_t{0});
}

} // namespace
