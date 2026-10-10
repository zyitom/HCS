#pragma once

// 跨进程通道：一个写者、任意多个只读读者的广播环，放在由写者自己封好的 memfd 里。
//
// 为什么这样就够安全（本机 6.8.1-rt 实测过每一条，见 test_channel.cpp 的 Seal 用例）：
//   - 写者建段、写满一遍、再加 SHRINK | GROW | FUTURE_WRITE | SEAL。此后**包括写者自己**在内，
//     谁都不能截断、打洞（fallocate / MADV_REMOVE），也拿不到新的可写映射。
//   - 读者 attach 时先查 seal，再只读映射并锁页。于是读者的页不会被任何进程回收，
//     读它不会 SIGBUS、不会缺页——周期域线程也能直接读对端写的通道。
//   - 写者从不等读者；读者从不写共享内存。读者再多、再慢，也碰不到写者。
//
// 一致性靠带代号的 seqlock：第 k 条写进槽 k % capacity，槽的 sequence 在写的过程中是 2k+1，
// 写完是 2k+2。读者问的是"第 k 条"，一次比较就同时查出"还没写到"和"已被覆盖"。
// 数据逐个 64 位字用 release 写、acquire 读，不用独立 fence（gcc 的 TSan 不支持 fence）。

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#include <array>
#include <atomic>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <expected>
#include <new>
#include <optional>
#include <utility>

#include "hcs_link/detail/nonblocking.hpp"
#include "hcs_link/error.hpp"
#include "hcs_link/payload.hpp"
#include "hcs_link/unique_fd.hpp"

namespace hcs_link {

enum class ReadStatus : std::uint8_t {
    kNotYet,      ///< 这一条还没写完（或还没开始写）
    kOverwritten, ///< 这一条已经被后来的覆盖了
};

/// 读出来的一条，连同它的序号。
template <typename T>
struct Sample {
    std::uint64_t index;
    T value;
};

struct WriterOptions {
    std::uint64_t capacity = 1024; ///< 条数，2 的幂；决定读者最多能落后多少条而不丢
    bool lock_memory = true;       ///< mlock 整段；周期域写者必须开
    const char* name = "hcs-link"; ///< 只出现在 /proc/<pid>/fd 里，便于排查
};

struct ReaderOptions {
    bool lock_memory = true; ///< mlock 只读映射；周期域读者必须开
};

namespace detail {

inline constexpr std::uint64_t kMagic = 0x314B4E494C534348ULL; // 小端字节序就是 "HCSLINK1"
inline constexpr std::uint32_t kProtocol = 1;
inline constexpr std::size_t kCacheLine = 64;
inline constexpr int kSeals = F_SEAL_SHRINK | F_SEAL_GROW | F_SEAL_FUTURE_WRITE | F_SEAL_SEAL;
inline constexpr int kRequiredSeals = F_SEAL_SHRINK | F_SEAL_GROW | F_SEAL_FUTURE_WRITE;

using Word = std::uint64_t;
static_assert(std::atomic_ref<Word>::is_always_lock_free);
static_assert(std::atomic_ref<Word>::required_alignment == alignof(Word));

// 只读映射上只许用 8 字节原子：16 字节原子读在部分实现里是 cmpxchg16b / LDXP-STXP，会写内存。
// atomic_ref<const T> 要到 C++26 才有，这里 const_cast 之后只调 load。
[[nodiscard]] inline Word load(const Word& word, std::memory_order order) noexcept {
    return std::atomic_ref<Word>{const_cast<Word&>(word)}.load(order);
}

inline void store(Word& word, Word value, std::memory_order order) noexcept {
    std::atomic_ref<Word>{word}.store(value, order);
}

/// 段头：建段时写一次，seal 之前写完，此后不变。
struct alignas(kCacheLine) Header {
    Word magic;
    std::uint32_t protocol;
    std::uint32_t payload_version;
    Word payload_id;
    std::uint32_t payload_size;
    std::uint32_t slot_size;
    Word capacity;
    Word reserved[3];
};
static_assert(sizeof(Header) == kCacheLine);

/// 已发布的条数，独占一行。
struct alignas(kCacheLine) Head {
    Word published;
};
static_assert(sizeof(Head) == kCacheLine);

template <Payload T>
inline constexpr std::size_t kWords = sizeof(T) / sizeof(Word);

/// 一条记录。sequence：0 = 从没写过；2k+1 = 正在写第 k 条；2k+2 = 装着写完的第 k 条。
template <Payload T>
struct alignas(kCacheLine) Slot {
    Word sequence;
    Word words[kWords<T>];
};

inline constexpr std::size_t kSlotsOffset = sizeof(Header) + sizeof(Head);

[[nodiscard]] inline std::size_t page_size() noexcept {
    return static_cast<std::size_t>(::sysconf(_SC_PAGESIZE));
}

template <Payload T>
[[nodiscard]] std::size_t segment_bytes(std::uint64_t capacity) noexcept {
    const std::size_t raw = kSlotsOffset + static_cast<std::size_t>(capacity) * sizeof(Slot<T>);
    const std::size_t page = page_size();
    return (raw + page - 1) / page * page;
}

/// 一段 mmap。只可移动，析构时 munmap（mlock 随之解除）。
class Mapping {
public:
    Mapping() noexcept = default;

    [[nodiscard]] static std::expected<Mapping, Error>
        map(int fd, std::size_t bytes, bool writable, bool lock) {
        const int protection = writable ? PROT_READ | PROT_WRITE : PROT_READ;
        void* address = ::mmap(nullptr, bytes, protection, MAP_SHARED | MAP_POPULATE, fd, 0);
        if (address == MAP_FAILED)
            return std::unexpected(Error::from_errno("mmap"));

        Mapping mapping{static_cast<std::byte*>(address), bytes};
        // 不用 MAP_LOCKED：它锁页失败不报错。
        if (lock && ::mlock(address, bytes) != 0)
            return std::unexpected(Error::from_errno("mlock"));
        return mapping;
    }

    Mapping(Mapping&& other) noexcept
        : data_{std::exchange(other.data_, nullptr)}
        , size_{std::exchange(other.size_, 0)} {}
    Mapping& operator=(Mapping&& other) noexcept {
        if (this != &other) {
            release();
            data_ = std::exchange(other.data_, nullptr);
            size_ = std::exchange(other.size_, 0);
        }
        return *this;
    }

    Mapping(const Mapping&) = delete;
    Mapping& operator=(const Mapping&) = delete;

    ~Mapping() { release(); }

    [[nodiscard]] std::byte* data() const noexcept { return data_; }
    [[nodiscard]] std::size_t size() const noexcept { return size_; }

private:
    Mapping(std::byte* data, std::size_t size) noexcept
        : data_{data}
        , size_{size} {}

    void release() noexcept {
        if (data_ != nullptr)
            ::munmap(data_, size_);
        data_ = nullptr;
        size_ = 0;
    }

    std::byte* data_ = nullptr;
    std::size_t size_ = 0;
};

[[nodiscard]] inline std::expected<UniqueFd, Error> create_memfd(const char* name) {
#ifdef MFD_NOEXEC_SEAL
    // 6.3+：段不可执行，顺带封上 F_SEAL_EXEC。老内核不认这个标志，回落到下面。
    if (const int fd = ::memfd_create(name, MFD_CLOEXEC | MFD_NOEXEC_SEAL); fd >= 0)
        return UniqueFd{fd};
    if (errno != EINVAL)
        return std::unexpected(Error::from_errno("memfd_create"));
#endif
    const int fd = ::memfd_create(name, MFD_CLOEXEC | MFD_ALLOW_SEALING);
    if (fd < 0)
        return std::unexpected(Error::from_errno("memfd_create"));
    return UniqueFd{fd};
}

/// 读侧对一段环的只读视图。不拥有映射，Reader 和 Cursor 共用。
template <Payload T>
class RingView {
public:
    RingView() noexcept = default;
    RingView(const Slot<T>* slots, const Head* head, std::uint64_t capacity) noexcept
        : slots_{slots}
        , head_{head}
        , capacity_{capacity} {}

    [[nodiscard]] std::uint64_t published() const noexcept HCS_LINK_NONBLOCKING {
        return load(head_->published, std::memory_order_acquire);
    }

    [[nodiscard]] std::uint64_t capacity() const noexcept { return capacity_; }

    [[nodiscard]] std::expected<T, ReadStatus> read(std::uint64_t index) const noexcept
        HCS_LINK_NONBLOCKING {
        const Slot<T>& slot = slots_[index & (capacity_ - 1)];
        const Word ready = 2 * index + 2;

        const Word before = load(slot.sequence, std::memory_order_acquire);
        if (before < ready)
            return std::unexpected(ReadStatus::kNotYet);
        if (before > ready)
            return std::unexpected(ReadStatus::kOverwritten);

        // 先整条拷进局部，只校验、只使用这份副本：原子读不会被编译器重新物化成第二次读。
        std::array<Word, kWords<T>> words;
        for (std::size_t i = 0; i < words.size(); ++i)
            words[i] = load(slot.words[i], std::memory_order_acquire);

        // 读的过程中只要看到过后一次写入的任何一个字，这里的 sequence 就一定已经变了。
        if (load(slot.sequence, std::memory_order_relaxed) != ready)
            return std::unexpected(ReadStatus::kOverwritten);
        return std::bit_cast<T>(words);
    }

    /// 现在还能安全读到的最老一条：再老的那一条的槽，写者可能正在写。
    [[nodiscard]] std::uint64_t oldest_safe(std::uint64_t published) const noexcept {
        return published >= capacity_ ? published - capacity_ + 1 : 0;
    }

private:
    const Slot<T>* slots_ = nullptr;
    const Head* head_ = nullptr;
    std::uint64_t capacity_ = 0;
};

} // namespace detail

/// 读者自己的游标：逐条往后读，一条不漏；落后超过一圈时跳到还能读的最老一条，
/// 跳过的条数计入 lost()——丢了就一定知道丢了多少，不会悄悄丢。
///
/// 不拥有映射：创建它的 Reader 必须活得比它长。
template <Payload T>
class Cursor {
public:
    Cursor(detail::RingView<T> view, std::uint64_t position) noexcept
        : view_{view}
        , position_{position} {}

    /// 下一条；没有新的就返回空。周期域可调。
    [[nodiscard]] std::optional<Sample<T>> next() noexcept HCS_LINK_NONBLOCKING {
        for (int attempt = 0; attempt < kMaxAttempts; ++attempt) {
            const auto value = view_.read(position_);
            if (value)
                return Sample<T>{position_++, *value};
            if (value.error() == ReadStatus::kNotYet)
                return std::nullopt;

            const std::uint64_t oldest = view_.oldest_safe(view_.published());
            const std::uint64_t skip_to = oldest > position_ ? oldest : position_ + 1;
            lost_ += skip_to - position_;
            position_ = skip_to;
        }
        return std::nullopt;
    }

    /// 下一次 next() 要读的序号。
    [[nodiscard]] std::uint64_t position() const noexcept { return position_; }

    /// 因为落后太多而跳过的条数，累计值。
    [[nodiscard]] std::uint64_t lost() const noexcept { return lost_; }

private:
    // 写者每次只能多覆盖一条，重试几次一定追得上；设上限只为让周期域的最坏耗时有界。
    static constexpr int kMaxAttempts = 4;

    detail::RingView<T> view_;
    std::uint64_t position_;
    std::uint64_t lost_ = 0;
};

/// 通道的只读端。从对端递过来的 fd 构造，校验不过就拒绝。
template <Payload T>
class Reader {
public:
    /// 校验顺序：seal → 大小 → 映射并锁页 → 段头（只读一次，拷进本地再比）。
    [[nodiscard]] static std::expected<Reader, Error>
        attach(UniqueFd fd, const ReaderOptions& options = {}) {
        const int seals = ::fcntl(fd.get(), F_GET_SEALS);
        if (seals < 0)
            return std::unexpected(Error::from_errno("fcntl(F_GET_SEALS)"));
        if ((seals & detail::kRequiredSeals) != detail::kRequiredSeals)
            return std::unexpected(Error{.code = ErrorCode::kBadSeals});

        struct stat status {};
        if (::fstat(fd.get(), &status) != 0)
            return std::unexpected(Error::from_errno("fstat"));
        const auto bytes = static_cast<std::size_t>(status.st_size);
        if (bytes < detail::kSlotsOffset)
            return std::unexpected(Error{.code = ErrorCode::kBadGeometry, .where = "segment size"});

        auto mapping = detail::Mapping::map(fd.get(), bytes, false, options.lock_memory);
        if (!mapping)
            return std::unexpected(mapping.error());

        detail::Header header;
        std::memcpy(&header, mapping->data(), sizeof header);

        if (header.magic != detail::kMagic || header.protocol != detail::kProtocol)
            return std::unexpected(Error{.code = ErrorCode::kBadHeader});
        if (header.payload_id != payload_id<T> || header.payload_version != T::kVersion
            || header.payload_size != sizeof(T) || header.slot_size != sizeof(detail::Slot<T>))
            return std::unexpected(Error{.code = ErrorCode::kPayloadMismatch});
        if (header.capacity < 2 || !std::has_single_bit(header.capacity)
            || header.capacity > bytes / sizeof(detail::Slot<T>)
            || detail::segment_bytes<T>(header.capacity) != bytes)
            return std::unexpected(Error{.code = ErrorCode::kBadGeometry, .where = "capacity"});

        const std::byte* base = mapping->data();
        const auto* head =
            std::launder(reinterpret_cast<const detail::Head*>(base + sizeof(detail::Header)));
        const auto* slots =
            std::launder(reinterpret_cast<const detail::Slot<T>*>(base + detail::kSlotsOffset));

        // 映射已经持有这个文件；fd 本身不再需要，随 UniqueFd 关掉。
        return Reader{std::move(*mapping), detail::RingView<T>{slots, head, header.capacity}};
    }

    /// 写者已发布的条数（= 最新一条的序号 + 1）。
    [[nodiscard]] std::uint64_t published() const noexcept HCS_LINK_NONBLOCKING {
        return view_.published();
    }

    [[nodiscard]] std::uint64_t capacity() const noexcept { return view_.capacity(); }

    /// 读第 index 条。
    [[nodiscard]] std::expected<T, ReadStatus> read(std::uint64_t index) const noexcept
        HCS_LINK_NONBLOCKING {
        return view_.read(index);
    }

    /// 最新一条。读的时候恰好被覆盖就换最新的重读，次数有界；周期域可调。
    [[nodiscard]] std::optional<Sample<T>> latest() const noexcept HCS_LINK_NONBLOCKING {
        for (int attempt = 0; attempt < 4; ++attempt) {
            const std::uint64_t published = view_.published();
            if (published == 0)
                return std::nullopt;
            if (const auto value = view_.read(published - 1))
                return Sample<T>{published - 1, *value};
        }
        return std::nullopt;
    }

    /// 从现在起的游标：只读之后新发布的。
    [[nodiscard]] Cursor<T> cursor() const noexcept { return Cursor<T>{view_, published()}; }

    /// 从环里还留着的最老一条开始的游标。
    [[nodiscard]] Cursor<T> cursor_from_oldest() const noexcept {
        return Cursor<T>{view_, view_.oldest_safe(published())};
    }

private:
    Reader(detail::Mapping mapping, detail::RingView<T> view) noexcept
        : mapping_{std::move(mapping)}
        , view_{view} {}

    detail::Mapping mapping_;
    detail::RingView<T> view_; // 指向 mapping_ 里的内存；Reader 移动时映射地址不变
};

/// 通道的写端，也是它的创建者和唯一写者。
template <Payload T>
class Writer {
public:
    [[nodiscard]] static std::expected<Writer, Error> create(const WriterOptions& options = {}) {
        if (options.capacity < 2 || !std::has_single_bit(options.capacity))
            return std::unexpected(
                Error{.code = ErrorCode::kInvalidArgument, .where = "capacity must be a power of two"});

        auto fd = detail::create_memfd(options.name);
        if (!fd)
            return std::unexpected(fd.error());

        const std::size_t bytes = detail::segment_bytes<T>(options.capacity);
        if (::ftruncate(fd->get(), static_cast<off_t>(bytes)) != 0)
            return std::unexpected(Error::from_errno("ftruncate"));

        auto mapping = detail::Mapping::map(fd->get(), bytes, true, options.lock_memory);
        if (!mapping)
            return std::unexpected(mapping.error());

        std::byte* base = mapping->data();
        // 每一页都写一遍：共享可写映射的 MAP_POPULATE 只做读缺页，不写一遍的话
        // publish() 第一次碰到每一页时还要进一次内核。
        std::memset(base, 0, bytes);

        ::new (static_cast<void*>(base)) detail::Header{
            .magic = detail::kMagic,
            .protocol = detail::kProtocol,
            .payload_version = T::kVersion,
            .payload_id = payload_id<T>,
            .payload_size = sizeof(T),
            .slot_size = sizeof(detail::Slot<T>),
            .capacity = options.capacity,
            .reserved = {},
        };
        auto* head = ::new (static_cast<void*>(base + sizeof(detail::Header))) detail::Head{};
        auto* slots = ::new (static_cast<void*>(base + detail::kSlotsOffset))
            detail::Slot<T>[static_cast<std::size_t>(options.capacity)]{};

        // 必须在第一次把 fd 交出去之前封上。
        if (::fcntl(fd->get(), F_ADD_SEALS, detail::kSeals) != 0)
            return std::unexpected(Error::from_errno("fcntl(F_ADD_SEALS)"));

        return Writer{std::move(*fd), std::move(*mapping), slots, head, options.capacity};
    }

    /// 发布一条。不分配、不加锁、不进内核、不等任何读者；周期域可调。
    void publish(const T& value) noexcept HCS_LINK_NONBLOCKING {
        const std::uint64_t index = next_++;
        detail::Slot<T>& slot = slots_[index & mask_];

        // 2k+1 用 relaxed 就够：后面每个数据字都是 release 写，会把它一起带出去。
        detail::store(slot.sequence, 2 * index + 1, std::memory_order_relaxed);
        const auto words = std::bit_cast<std::array<detail::Word, detail::kWords<T>>>(value);
        for (std::size_t i = 0; i < words.size(); ++i)
            detail::store(slot.words[i], words[i], std::memory_order_release);
        detail::store(slot.sequence, 2 * index + 2, std::memory_order_release);
        detail::store(head_->published, index + 1, std::memory_order_release);
    }

    [[nodiscard]] std::uint64_t published() const noexcept { return next_; }
    [[nodiscard]] std::uint64_t capacity() const noexcept { return mask_ + 1; }

    /// 交给对端的 fd（借出，不转移所有权）。
    [[nodiscard]] int fd() const noexcept { return fd_.get(); }

    /// 同一进程里的只读端（进程内部署、测试）。走的是和跨进程完全相同的 attach。
    [[nodiscard]] std::expected<Reader<T>, Error> open_reader(const ReaderOptions& options = {}) const {
        const int duplicate = ::fcntl(fd_.get(), F_DUPFD_CLOEXEC, 0);
        if (duplicate < 0)
            return std::unexpected(Error::from_errno("fcntl(F_DUPFD_CLOEXEC)"));
        return Reader<T>::attach(UniqueFd{duplicate}, options);
    }

private:
    Writer(UniqueFd fd, detail::Mapping mapping, detail::Slot<T>* slots, detail::Head* head,
           std::uint64_t capacity) noexcept
        : fd_{std::move(fd)}
        , mapping_{std::move(mapping)}
        , slots_{slots}
        , head_{head}
        , mask_{capacity - 1} {}

    UniqueFd fd_;
    detail::Mapping mapping_;
    detail::Slot<T>* slots_; // 指向 mapping_ 里的内存；Writer 移动时映射地址不变
    detail::Head* head_;
    std::uint64_t mask_;
    std::uint64_t next_ = 0;
};

} // namespace hcs_link
