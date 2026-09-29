# 自瞄跨进程通道 · 设计定稿（v2，未实现）

> 状态：设计完成，待实现。v1 定稿于 2026-09-29，同日审查后改为 v2。
> 前提：视觉进程独立（崩溃隔离），**不用任何 ROS**；相机由视觉进程独占采集，
> **图像永远不进控制进程**——跨进程载荷只有姿态（64 B/条）、上下文（64 B）和目标（128 B）。
> 带"已实测"的结论是在本机（GCC 14.2 / clang 20 / Linux 7.0）上验证过的，其余来自读代码和文档。

## 0. v1 → v2 改了什么

| v1 | v2 | 原因 |
|---|---|---|
| 姿态走 SPSC FIFO（ByteTunnel 形状） | 覆盖式历史环，按时间查 | FIFO 写满时丢的是**新**数据；控制侧要读视觉写的游标；视觉要的是"某一刻的姿态"而不是"下一条" |
| 按帧号配对 | 按 CLOCK_MONOTONIC 时间戳配对 | 控制进程不知道相机帧号；没有硬触发（libhcs 的 `send_pulse_schedule` 只用于测试） |
| 一个双方都可写的 POSIX shm 段 | 两个 memfd，按写者分段，加 seal | 共写一段就挡不住野写；对端 `ftruncate` 缩段会让控制进程 SIGBUS；memfd 没有 `/dev/shm` 残段 |
| 心跳 + `kill(pid,0)` | socket EOF + 按自己的拍数判陈旧 | PID 会复用、跨 PID namespace 对不上；对端写的时间戳不可信 |
| 固定段名 + 死段接管 | abstract unix socket 握手，SCM_RIGHTS 递 fd | 没有文件系统残留；握手失败能回一条带双方版本的错误 |
| 手动 `layout_version` | 编译期布局哈希 + 金值测试 + 语义版本号 | 手动递增迟早会忘（kSetEndpointMode 的教训） |
| memcpy 版 seqlock | 每 8 字节一次 `std::atomic_ref` | 形式上无数据竞争；TSan 能跑；编译器不能对不可信内存二次读取 |
| AttitudeTap + AutoAimBridge 两个组件 | 一个组件 AutoAimLink | 两半共享同一对段和会话状态，拆开只多一层管道 |
| 载荷 48 B / 64 B | 姿态 64 B，目标 128 B，另加上下文 64 B | 补会话号、帧号、robot_center（UI 在用）、控制→视觉的颜色/弹速 |

保留不变的判断：视觉独立进程；不用 ROS/DDS；不引 iceoryx2 等第三方库；小载荷不做零拷贝 loan；
v1 不用 futex（双方都按自己的节拍读，没有人需要睡着等对方）；目标只保留最新值。

## 1. 范围与前提

- **图像只属于视觉进程**。相机 SDK、检测、跟踪、预测、弹道补偿都在视觉进程里；控制进程只收目标。
- **1:1**：同一时刻只接受一个视觉进程。第二个连接直接拒绝。
- **同机、同内核、同一个 time namespace 和 network namespace**。abstract unix socket 挂在
  network namespace 上，CLOCK_MONOTONIC 挂在 time namespace 上（视觉进容器时的要求见 §9）。
- **平台**：Linux ≥ 5.1（`F_SEAL_FUTURE_WRITE`），x86-64 或 aarch64，小端。
- **视觉侧只要 C++20**：共享头文件只依赖标准库，不依赖 ROS、Eigen 或 hcs 的其他包。
- **威胁模型是"有 bug 的视觉"，不是攻击者**：同 uid 的任何进程都能连上 socket 冒充视觉，这一点不防。
  要防的是视觉崩溃、挂起、野写、写出 NaN 或离谱值、版本不匹配。

## 2. 架构

```
控制进程（hcs_executor）                                      视觉进程（C++20，无 ROS）
┌ hcs-ctrl（FIFO 90，CPU 3）──────────────────┐
│ AutoAimLink::update(tick)                    │   att 段：控制写，视觉只读（内核 seal 保证）
│  ① IMU 有新帧 → 写姿态环 ──────────────────────▶ 姿态环 256 × 64 B ──▶ 推理完成后按曝光时刻二分 + slerp
│  ② 上下文有变 → 写上下文槽 ────────────────────▶ 上下文槽 64 B     ──▶ 敌方颜色、弹速、自瞄模式
│  ③ 读目标槽 → 校验 → 外推到本拍 → 写图边 ◀────── aim 段：视觉写，控制只读 ◀── 每帧发布一次（无目标也发）
└──────────────────────────────────────────────┘
┌ hcs-autoaim-ipc（SCHED_OTHER，非隔离核）──────┐
│ accept → 握手 → SCM_RIGHTS 递两个 fd          │◀── abstract unix socket "@hcs/autoaim" ──▶ connect，断线重连
│ poll 到 EOF → session = 0                     │
└──────────────────────────────────────────────┘
```

- 控制进程里**只有 hcs-ctrl 线程碰共享内存**：写 att 段、读 aim 段。
- IPC 线程只递 fd、只写一个进程内的 `std::atomic<uint32_t> session`，从不碰段的内容。
- 所以两个段各自都是严格的单写者，不需要任何跨进程锁。

## 3. 会话与段

### 3.1 一个会话 = 一对全新的段

每次视觉连上来，控制侧都新建一对 memfd，会话结束就作废，**绝不跨会话复用**。
这样每个段在整个生命周期里只有一个写者进程，这条不变量是后面所有保证的地基：

- 上一个视觉进程哪怕没死透、还在往旧段里写，也写不进新会话；
- 不存在"接手半写的 seqlock"这种特殊情况，每个段的序号都从 0 开始。

| 段 | 大小 | 写者 | 读者 | 控制侧映射 | 视觉侧映射 | seal |
|---|---|---|---|---|---|---|
| att | 20 KiB | 控制 hcs-ctrl | 视觉 | RW | **只读**（内核强制） | SHRINK、GROW、FUTURE_WRITE、SEAL |
| aim | 4 KiB | 视觉 | 控制 hcs-ctrl | **只读** | RW + `MADV_DONTFORK` | SHRINK、GROW、SEAL |

建段顺序（IPC 线程，非 RT）：

```
memfd_create(name, MFD_CLOEXEC | MFD_ALLOW_SEALING) → ftruncate(固定大小)
→ mmap(MAP_SHARED | MAP_POPULATE | MAP_LOCKED)      // 先映射，RT 线程碰到时不会缺页
→ 写段头（只写这一次）
→ fcntl(F_ADD_SEALS, ...)                          // 必须在递 fd 之前
→ 握手里用 SCM_RIGHTS 递 fd
```

- `F_SEAL_SHRINK | F_SEAL_GROW`：对端无法改段大小，控制进程不会因为对端 `ftruncate` 而 SIGBUS。
- `F_SEAL_FUTURE_WRITE`：控制侧已有的 RW 映射不受影响，但之后任何人都拿不到可写映射、也不能 `write()`。
  视觉进程只能只读映射 att 段，野写只会让它自己段错误。
- `F_SEAL_SEAL`：封印本身不能再改。
- aim 段控制侧以只读方式映射，视觉野写只能写坏自己的目标，由 §5.3 的校验兜住。

### 3.2 IPC 线程和 RT 线程之间的交接

控制进程内，段的指针由 IPC 线程建好后交给 RT 线程。交接只用两个进程内 atomic：

```
active_ : atomic<u32>   IPC 写。0 = 无会话，否则 = 当前会话号
ack_    : atomic<u32>   RT 写。RT 每拍结束时写回本拍看到的 active_
bundle_ : {att*, aim*}  普通成员，只在 active_ == 0 且 ack_ == 0 时由 IPC 线程改写
```

- **开会话**：建段 → 握手成功 → 写 `bundle_` → `active_.store(s, release)`。
- **RT 每拍**：`s = active_.load(acquire)`；`s != 0` 就用 `bundle_`；本拍结束 `ack_.store(s, release)`。
- **关会话**（EOF、出错、控制进程退出）：`active_.store(0, release)` → 等 `ack_ == 0` → munmap、close。
  - 等待上限 100 ms；超时说明 RT 线程没在跑拍，**宁可泄漏映射也不 munmap**。
  - munmap 会给本进程跑过的所有核发一次 TLB shootdown IPI，CPU 3 也会被打断几 µs。只在重连时发生；
    进程里 DDS 的大块 malloc/free 本来就在触发同样的 IPI，不是新增的风险类别。
- RT 线程**从不** mmap/munmap，也从不等待 IPC 线程。

### 3.3 握手（abstract unix socket）

- **socket**：`AF_UNIX, SOCK_SEQPACKET | SOCK_CLOEXEC`，abstract 名 `"\0hcs/autoaim"`（组件参数可改）。
  SEQPACKET 保留消息边界，对端关闭时是明确的 EOF，不用自己分帧。
- **身份**：`SO_PEERCRED` 取对端 uid/pid。uid 必须等于本进程 uid（abstract socket 没有文件权限，
  同 netns 的任何用户都能连）；pid 只写日志。
- **1:1**：已有会话时再来的连接，回 `Reject{kBusy}` 后关闭。
- **消息**（全部是 §6 规则下的平凡结构体，SEQPACKET 一条消息一个结构体）：

```
视觉 → 控制   Hello   { magic, protocol_version, layout_hash, pid }
控制 → 视觉   Welcome { magic, protocol_version, layout_hash, session, att_bytes, aim_bytes }
                + SCM_RIGHTS [att_fd, aim_fd]
          或  Reject  { magic, reason, protocol_version, layout_hash }   // 带控制侧的版本，便于排查
```

- `Hello` 1 s 内没到就断开。magic 或版本任何一项不一致都回 `Reject`，**不存在兼容区间**：
  两边用同一个协议头文件编译，不一致就是该重编了。
- 控制侧所有 `send` 都带 `MSG_NOSIGNAL`（对端握手中途退出时，SIGPIPE 的默认动作会直接杀掉控制进程）。
- 视觉侧 `recvmsg` 带 `MSG_CMSG_CLOEXEC`，校验收到恰好两个 fd、`fstat` 大小与 Welcome 一致、
  `F_GET_SEALS` 包含预期的 seal，再映射，再逐字段校验段头。
- **会话号**：控制进程内从 1 递增的 u32，写进 Welcome；视觉写进每一条目标，控制侧只接受当前会话号的目标。
- **断线**：IPC 线程 `poll` 连接 fd，读到 EOF/ERR/HUP 就关会话（§3.2）。
  视觉侧读到 EOF（控制进程重启）就丢掉映射、每 200 ms 重连一次。

## 4. 时间对齐（精度的主要来源）

### 4.1 唯一的公共时钟

- **CLOCK_MONOTONIC 的纳秒数**，存为 `int64_t`。就是 `hcs_sync::Clock`（`steady_clock`），
  libstdc++ 在 Linux 上用的就是它，两个进程天然共享，不需要任何同步。
- 共享内存里**只出现这一种时钟**。板钟、microframe、相机设备时钟、HiPNUC 的 `system_time`
  都必须在各自那一侧换算成 monotonic 以后才能写进来。
- 不用 CLOCK_REALTIME（NTP 会跳）；不用 CLOCK_MONOTONIC_RAW（`steady_clock` 不是它，
  两进程各用一种就对不上）；不用 CLOCK_BOOTTIME（整车不休眠，用不上）。

### 4.2 姿态的时间戳 = IMU 的采样时刻

**不能用 `tick.scheduled`**。姿态是 hcs-ctrl 在拍里写的，但它描述的是 IMU 采样的那一刻：

```
IMU 采样 ─→ UART 传输 ─→ 板卡 ─→ USB ─→ IO 线程 store_status ─→ 下一拍 update_status ─→ 写环
   t_s       921600 波特，82 B ≈ 0.9 ms    ≈ 0.1–1 ms           0–1 ms（等拍）
```

用拍时刻会平白多出 1–3 ms 的滞后，而且带抖动。分三档实现，逐档升级：

| 档 | 戳怎么来 | 需要改什么 | 残差 |
|---|---|---|---|
| A（v1 起步） | IO 线程在 `store_status` 成功时记 `Clock::now()`，存进 Hipnuc；再减一个常数 `imu_latency`（组件参数，默认 1.0 ms） | Hipnuc 加一个 `int64` 字段，改 `hcs_core` 内部 | USB 调度抖动，约 0.1–1 ms |
| B | 固件在 UART 收到帧头时打 microframe 戳，host 用 `libhcs::Timeline` 换算成 monotonic | libhcs 固件 + 协议 | Timeline 拟合相位误差约 10 µs + 模块内部固定延迟 |
| C | 同 B，另外标定 IMU 模块内部的滤波延迟 | 标定台架 | 上限由模块决定 |

- 档 A 的 `imu_latency` 用 §10 的"摆云台"标定来定。
- **IMU 输出率要提上去**：CH040 默认 HI91 输出 100 Hz（`hipnuc.hpp:73`），也就是姿态历史只有 10 ms 一个点，
  云台快速转动时插值误差远大于上面这些。上车前把模块配到 ≥ 400 Hz（921600 波特下 82 B 帧最多约 1100 Hz）。
- 环里**每个 IMU 帧写一条**，靠 `frame_count()` 变化来判断，不是每拍写一条：
  否则 100 Hz 输出、1 kHz 控制时会写 10 条重复姿态，把历史窗口压缩到 1/10。

### 4.3 坐标系：两边都用 OdomImu

- 环里只写云台 CH040 的原始四元数：机体 FLU 相对模块世界系，也就是 `hcs_description::OdomImu`
  的那个旋转。控制侧不做任何变换——出问题时可以直接对照原始日志。
- 相机和 IMU 都装在 pitch 连杆上，所以相机的世界姿态 = IMU 四元数 × 相机相对 IMU 的安装外参，
  不需要编码器角。外参在视觉侧标定、在视觉侧用，和相机内参放在一起管。
- 视觉发回的瞄准方向也表达在同一个 IMU 世界系里，正好是 `SetControlDirection` 要的
  `OdomImu::DirectionVector`。IMU 的 yaw 漂移对两边是同一个漂移，互相抵消。

### 4.4 相机侧（视觉进程的责任，这里只定约束）

- 每帧的 `exposure_mid_ns` = 曝光中点的 monotonic 时刻。
  - 优先：相机设备时间戳 → monotonic 的在线线性拟合（与 libhcs `Timeline` 同一个方法，窗口约 1 分钟）；
  - 保底：SDK 回调时刻 − 标定过的常数延迟 − 曝光时间 / 2。
- 用 `exposure_mid_ns` 去姿态环里查；目标里也写这个时刻，控制侧外推用它。

### 4.5 误差预算

| 来源 | 量级 | 在 5 m 处 |
|---|---|---|
| 通道本身（seqlock 读写） | < 1 µs | 可忽略 |
| 姿态戳，档 A | 0.1–1 ms × 云台角速度 3 rad/s ≈ 0.3–3 mrad | 1.5–15 mm |
| IMU 100 Hz 时的插值 | 取决于角加速度，最坏 ~5 mrad | ~25 mm |
| 相机戳（保底方案） | 1–2 ms × 3 rad/s ≈ 3–6 mrad | 15–30 mm |
| 小装甲板半宽 | — | 67 mm |

结论：v2 要把 IMU 输出率和相机戳做对，通道本身的纳秒级成本不是问题。
