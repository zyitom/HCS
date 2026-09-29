// 通道的单测，钉住这几条：
//   - 语义：序号从 0 连续；没写到的读出 kNotYet，被覆盖的读出 kOverwritten；
//   - 无损：游标在一圈之内一条不漏，落后超过一圈时跳过的条数精确计入 lost()；
//   - 认人：载荷名字 / 版本对不上、fd 没封好，attach 一律拒绝；
//   - 封印：写者交出去的 fd 谁都截不短、打不了洞、拿不到可写映射——包括写者自己；
//   - 一致：多线程、跨进程并发读写时，读者拿到的每一条都是完整的某一次写入。

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/wait.h>
#include <unistd.h>

#include <array>
#include <atomic>
#include <cerrno>
#include <cstdint>
#include <cstring>
#include <string_view>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include "hcs_link/channel.hpp"

namespace {

using hcs_link::ErrorCode;
using hcs_link::ReadStatus;
using hcs_link::Reader;
using hcs_link::UniqueFd;
using hcs_link::Writer;

/// 每个字都等于同一个序号：只要读到的字不全相等，就是撕裂。
struct Pattern {
    static constexpr std::string_view kName = "test.Pattern";
    static constexpr std::uint32_t kVersion = 1;

    std::array<std::uint64_t, 7> words;

    static Pattern of(std::uint64_t value) {
        Pattern pattern;
        pattern.words.fill(value);
        return pattern;
    }

    [[nodiscard]] bool consistent() const {
        for (const auto word : words)
            if (word != words[0])
                return false;
        return true;
    }
};

struct OtherName {
    static constexpr std::string_view kName = "test.Other";
    static constexpr std::uint32_t kVersion = 1;
    std::array<std::uint64_t, 7> words;
};

struct NewerVersion {
    static constexpr std::string_view kName = "test.Pattern";
    static constexpr std::uint32_t kVersion = 2;
    std::array<std::uint64_t, 7> words;
};

// 单测进程不一定有足够的 RLIMIT_MEMLOCK，锁页的路径由周期域真正使用时覆盖。
constexpr hcs_link::WriterOptions kWriterOptions{.capacity = 8, .lock_memory = false, .name = "test"};
constexpr hcs_link::ReaderOptions kReaderOptions{.lock_memory = false};

Writer<Pattern> make_writer(std::uint64_t capacity = 8) {
    auto options = kWriterOptions;
    options.capacity = capacity;
    auto writer = Writer<Pattern>::create(options);
    EXPECT_TRUE(writer.has_value()) << writer.error().message();
    return std::move(*writer);
}

Reader<Pattern> open(const Writer<Pattern>& writer) {
    auto reader = writer.open_reader(kReaderOptions);
    EXPECT_TRUE(reader.has_value()) << reader.error().message();
    return std::move(*reader);
}

TEST(Channel, LatestFollowsTheWriter) {
    auto writer = make_writer();
    const auto reader = open(writer);

    EXPECT_FALSE(reader.latest().has_value());

    for (std::uint64_t i = 0; i < 3; ++i)
        writer.publish(Pattern::of(100 + i));

    const auto latest = reader.latest();
    ASSERT_TRUE(latest.has_value());
    EXPECT_EQ(latest->index, 2U);
    EXPECT_EQ(latest->value.words[0], 102U);
    EXPECT_EQ(reader.published(), 3U);
}

TEST(Channel, ReadReportsNotYetAndOverwritten) {
    auto writer = make_writer(8);
    const auto reader = open(writer);

    EXPECT_EQ(reader.read(0).error(), ReadStatus::kNotYet);

    for (std::uint64_t i = 0; i < 9; ++i) // 第 8 条覆盖了第 0 条的槽
        writer.publish(Pattern::of(i));

    EXPECT_EQ(reader.read(0).error(), ReadStatus::kOverwritten);
    EXPECT_EQ(reader.read(1)->words[0], 1U);
    EXPECT_EQ(reader.read(8)->words[0], 8U);
    EXPECT_EQ(reader.read(9).error(), ReadStatus::kNotYet);
}

TEST(Channel, CursorIsLosslessWithinCapacity) {
    auto writer = make_writer(8);
    const auto reader = open(writer);
    auto cursor = reader.cursor();

    for (std::uint64_t round = 0; round < 5; ++round) {
        for (std::uint64_t i = 0; i < 7; ++i)
            writer.publish(Pattern::of(round * 7 + i));
        for (std::uint64_t i = 0; i < 7; ++i) {
            const auto sample = cursor.next();
            ASSERT_TRUE(sample.has_value());
            EXPECT_EQ(sample->index, round * 7 + i);
            EXPECT_EQ(sample->value.words[0], round * 7 + i);
        }
        EXPECT_FALSE(cursor.next().has_value());
    }
    EXPECT_EQ(cursor.lost(), 0U);
}

TEST(Channel, CursorCountsWhatItMissed) {
    auto writer = make_writer(8);
    const auto reader = open(writer);
    auto cursor = reader.cursor();

    for (std::uint64_t i = 0; i < 20; ++i)
        writer.publish(Pattern::of(i));

    // 20 条里环只留得住 8 条，其中最老那条的槽随时可能被写：安全的最老一条是 13。
    const auto first = cursor.next();
    ASSERT_TRUE(first.has_value());
    EXPECT_EQ(first->index, 13U);
    EXPECT_EQ(cursor.lost(), 13U);

    std::uint64_t expected = 14;
    while (const auto sample = cursor.next())
        EXPECT_EQ(sample->index, expected++);
    EXPECT_EQ(expected, 20U);
}

TEST(Channel, CursorFromOldestStartsAtTheRingTail) {
    auto writer = make_writer(8);
    for (std::uint64_t i = 0; i < 5; ++i)
        writer.publish(Pattern::of(i));
    const auto reader = open(writer);

    auto cursor = reader.cursor_from_oldest();
    EXPECT_EQ(cursor.next()->index, 0U);
}

TEST(Channel, ReaderOutlivesWriter) {
    std::optional<Reader<Pattern>> reader;
    {
        auto writer = make_writer();
        writer.publish(Pattern::of(42));
        reader.emplace(open(writer));
    } // 写者进程"退出"：它的映射和 fd 都没了

    const auto latest = reader->latest();
    ASSERT_TRUE(latest.has_value());
    EXPECT_EQ(latest->value.words[0], 42U);
}

TEST(Channel, AttachRejectsAnotherPayload) {
    const auto writer = make_writer();
    const int copy = ::fcntl(writer.fd(), F_DUPFD_CLOEXEC, 0);
    const auto by_name = Reader<OtherName>::attach(UniqueFd{copy}, kReaderOptions);
    ASSERT_FALSE(by_name.has_value());
    EXPECT_EQ(by_name.error().code, ErrorCode::kPayloadMismatch);

    const int another = ::fcntl(writer.fd(), F_DUPFD_CLOEXEC, 0);
    const auto by_version = Reader<NewerVersion>::attach(UniqueFd{another}, kReaderOptions);
    ASSERT_FALSE(by_version.has_value());
    EXPECT_EQ(by_version.error().code, ErrorCode::kPayloadMismatch);
}

TEST(Channel, AttachRejectsUnsealedMemfd) {
    UniqueFd fd{::memfd_create("unsealed", MFD_CLOEXEC | MFD_ALLOW_SEALING)};
    ASSERT_TRUE(fd);
    ASSERT_EQ(::ftruncate(fd.get(), 1 << 16), 0);

    const auto reader = Reader<Pattern>::attach(std::move(fd), kReaderOptions);
    ASSERT_FALSE(reader.has_value());
    EXPECT_EQ(reader.error().code, ErrorCode::kBadSeals);
}

TEST(Channel, AttachRejectsGarbage) {
    UniqueFd fd{::memfd_create("garbage", MFD_CLOEXEC | MFD_ALLOW_SEALING)};
    ASSERT_TRUE(fd);
    ASSERT_EQ(::ftruncate(fd.get(), 1 << 16), 0);
    std::array<char, 64> junk;
    junk.fill('x');
    ASSERT_EQ(::pwrite(fd.get(), junk.data(), junk.size(), 0), static_cast<ssize_t>(junk.size()));
    ASSERT_EQ(::fcntl(fd.get(), F_ADD_SEALS, F_SEAL_SHRINK | F_SEAL_GROW | F_SEAL_FUTURE_WRITE), 0);

    const auto reader = Reader<Pattern>::attach(std::move(fd), kReaderOptions);
    ASSERT_FALSE(reader.has_value());
    EXPECT_EQ(reader.error().code, ErrorCode::kBadHeader);
}

TEST(Channel, CreateRejectsCapacityThatIsNotAPowerOfTwo) {
    auto options = kWriterOptions;
    options.capacity = 12;
    const auto writer = Writer<Pattern>::create(options);
    ASSERT_FALSE(writer.has_value());
    EXPECT_EQ(writer.error().code, ErrorCode::kInvalidArgument);
}

TEST(Seal, NobodyCanShrinkPunchOrWriteThroughTheFd) {
    const auto writer = make_writer();
    const int fd = writer.fd();

    errno = 0;
    EXPECT_NE(::ftruncate(fd, 0), 0);
    EXPECT_EQ(errno, EPERM);

    errno = 0;
    EXPECT_NE(::fallocate(fd, FALLOC_FL_PUNCH_HOLE | FALLOC_FL_KEEP_SIZE, 0, 4096), 0);
    EXPECT_EQ(errno, EPERM);

    void* writable = ::mmap(nullptr, 4096, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    EXPECT_EQ(writable, MAP_FAILED);
    EXPECT_EQ(errno, EPERM);

    // 只读映射可以拿，但不能再改成可写。
    void* readable = ::mmap(nullptr, 4096, PROT_READ, MAP_SHARED, fd, 0);
    ASSERT_NE(readable, MAP_FAILED);
    EXPECT_NE(::mprotect(readable, 4096, PROT_READ | PROT_WRITE), 0);
    ::munmap(readable, 4096);

    errno = 0;
    EXPECT_NE(::write(fd, "x", 1), 1);
    EXPECT_EQ(errno, EPERM);
}

TEST(Seal, WriterCannotDropPagesUnderTheReader) {
    // Writer 的可写映射是私有成员，这里按 Writer::create 的顺序用裸系统调用重做一遍：
    // 先建可写映射，再封；然后写者对自己那段可写映射做 MADV_REMOVE / MADV_DONTNEED。
    constexpr std::size_t kBytes = 1 << 16;
    UniqueFd fd{::memfd_create("writer", MFD_CLOEXEC | MFD_ALLOW_SEALING)};
    ASSERT_TRUE(fd);
    ASSERT_EQ(::ftruncate(fd.get(), kBytes), 0);
    auto* writable = static_cast<volatile char*>(
        ::mmap(nullptr, kBytes, PROT_READ | PROT_WRITE, MAP_SHARED | MAP_POPULATE, fd.get(), 0));
    ASSERT_NE(writable, MAP_FAILED);
    writable[0] = 7;
    ASSERT_EQ(::fcntl(fd.get(), F_ADD_SEALS, F_SEAL_SHRINK | F_SEAL_GROW | F_SEAL_FUTURE_WRITE | F_SEAL_SEAL),
              0);

    auto* readable = static_cast<const volatile char*>(
        ::mmap(nullptr, kBytes, PROT_READ, MAP_SHARED | MAP_POPULATE, fd.get(), 0));
    ASSERT_NE(readable, MAP_FAILED);

    void* writer_view = const_cast<char*>(writable);
    errno = 0;
    EXPECT_NE(::madvise(writer_view, kBytes, MADV_REMOVE), 0); // 打洞：被 FUTURE_WRITE 挡住
    EXPECT_EQ(errno, EPERM);
    EXPECT_EQ(::madvise(writer_view, kBytes, MADV_DONTNEED), 0); // 只丢写者自己的页表项
    EXPECT_EQ(::munmap(writer_view, kBytes), 0);                // 写者退出

    EXPECT_EQ(readable[0], 7); // 读者的页还在
    ::munmap(const_cast<char*>(readable), kBytes);
}

TEST(Concurrency, ThreadsNeverSeeTornValues) {
    auto writer = make_writer(64);
    const auto reader = open(writer);
    constexpr std::uint64_t kCount = 200'000;

    std::atomic<bool> done{false};
    std::atomic<std::uint64_t> bad{0};

    auto check_cursor = [&] {
        auto cursor = reader.cursor_from_oldest();
        std::uint64_t last = 0;
        bool first = true;
        while (!done.load(std::memory_order_acquire) || cursor.position() < kCount) {
            const auto sample = cursor.next();
            if (!sample)
                continue;
            if (!sample->value.consistent() || sample->value.words[0] != sample->index
                || (!first && sample->index <= last))
                bad.fetch_add(1);
            last = sample->index;
            first = false;
        }
    };
    auto check_latest = [&] {
        while (!done.load(std::memory_order_acquire)) {
            const auto sample = reader.latest();
            if (sample && (!sample->value.consistent() || sample->value.words[0] != sample->index))
                bad.fetch_add(1);
        }
    };

    std::thread cursor_reader{check_cursor};
    std::thread latest_reader{check_latest};
    for (std::uint64_t i = 0; i < kCount; ++i)
        writer.publish(Pattern::of(i));
    done.store(true, std::memory_order_release);
    cursor_reader.join();
    latest_reader.join();

    EXPECT_EQ(bad.load(), 0U);
}

TEST(Concurrency, ProcessesNeverSeeTornValues) {
    auto writer = make_writer(256);
    const auto reader = open(writer);
    constexpr std::uint64_t kCount = 2'000'000;

    const pid_t child = ::fork();
    ASSERT_GE(child, 0);
    if (child == 0) {
        // 子进程是写者：fork 继承的共享映射就是同一段 memfd。
        for (std::uint64_t i = 0; i < kCount; ++i)
            writer.publish(Pattern::of(i));
        ::_exit(0);
    }

    auto cursor = reader.cursor_from_oldest();
    std::uint64_t received = 0;
    std::uint64_t bad = 0;
    while (cursor.position() < kCount) {
        const auto sample = cursor.next();
        if (!sample)
            continue;
        ++received;
        if (!sample->value.consistent() || sample->value.words[0] != sample->index)
            ++bad;
    }

    int status = 0;
    ASSERT_EQ(::waitpid(child, &status, 0), child);
    EXPECT_TRUE(WIFEXITED(status) && WEXITSTATUS(status) == 0);

    EXPECT_EQ(bad, 0U);
    // 每一条要么读到了，要么被精确地算进了 lost。
    EXPECT_EQ(received + cursor.lost(), kCount);
}

} // namespace
