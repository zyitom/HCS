# 自瞄跨进程通道 · v4（已实现）

> 状态：代码和单测已完成，未上车。v1–v3 于 2026-09-29 反复审查后收敛成下面这个简单版本（§8 说明删掉了什么）。
> 代码：`hcs_link/`（纯头文件，只依赖标准库和 POSIX，C++23）、`hcs_core/src/controller/auto_aim/`。
> 前提：视觉是独立进程、不用任何 ROS；图像永远不进控制进程；两个进程同机、同 time/network namespace。
> 带"实测"的数字在本机（i7-1165G7，Linux 6.8.1-rt，`isolcpus=3,7`，GCC 14.2 / clang 20）上测得。

## 1. 设计：一个原语，两个方向各用一次

**通道 `Channel<T>`**：一个写者、任意多个只读读者的广播环，放在一段 memfd 里。
**谁写谁建**：写者建段、写满一遍、自己加 seal，再把 fd 交出去；另一方只能拿到只读视图。

```
hcs_executor（hcs-ctrl，每拍）                         视觉进程（任意框架）
  Writer<GimbalState>.publish(姿态)   ── 通道 ──▶  Reader<GimbalState>：latest() / 游标逐条读
  Reader<AimCommand>.latest()         ◀── 通道 ──  Writer<AimCommand>.publish(命令)
                    ↑
  hcs-autoaim 线程：abstract socket 上互换一次 fd，之后 socket 只用来察觉对端退出
```

- 周期域只做通道的读写：不分配、不加锁、不进内核、不等任何人（clang `-Wfunction-effects` 核实）。
- 读者从不写共享内存，写者从不等读者，所以读者再多、再慢也影响不到写者。
- 视觉以后改成控制进程里的线程时，用 `Writer::open_reader()` 拿同一种只读视图，代码不变。

## 2. 通道（`hcs_link/channel.hpp`）

### 2.1 布局

| 偏移 | 内容 |
|---|---|
| 0 | 段头 64 B：魔数 `HCSLINK1`、协议版本、载荷名字哈希 / 版本 / 大小、槽大小、容量。建段时写一次 |
| 64 | `published`：已发布条数，独占一行 |
| 128 | 槽 × 容量。每槽 = `sequence` + 载荷的 64 位字，按 64 B 对齐 |

### 2.2 一致性：带代号的 seqlock

第 k 条写进槽 `k % 容量`；写的过程中槽的 `sequence = 2k+1`，写完 `= 2k+2`。
读者问的是"第 k 条"：`sequence` 小于 `2k+2` 是还没写到，大于是已被覆盖，读完再比一次不等也是被覆盖。

- 数据逐个 64 位字用 `std::atomic_ref` 做 release 写 / acquire 读，不用独立 fence：
  GCC 14 对 `atomic_thread_fence` 报 `-Wtsan`（实测）。x86-64 上仍是普通 `mov`。
- 读侧先把整条拷进局部再校验、再使用，不对共享内存做第二次读。
- 只读映射上只用 8 字节原子：16 字节原子读在部分实现里会写内存，只读映射上直接 SIGSEGV。

### 2.3 无损

- 每个读者自己的 `Cursor` 逐条往后读，一圈之内一条不漏。
- 落后超过一圈时跳到还能安全读的最老一条，跳过的条数精确计入 `lost()`。不会悄悄丢，但写者也绝不等读者。

### 2.4 为什么读对端写的内存也安全（实测）

写者建段后加 `SHRINK | GROW | FUTURE_WRITE | SEAL`。之后**包括写者自己**：

| 动作 | 结果 |
|---|---|
| `ftruncate` 缩 / 扩 | EPERM |
| `fallocate(PUNCH_HOLE)`、对自己可写映射做 `MADV_REMOVE` | EPERM |
| 新建可写映射、只读映射改可写、`write(fd)` | EPERM / EACCES |
| 对自己映射做 `MADV_DONTNEED`、`munmap`、进程退出 | 允许，但只影响写者自己，读者的页原样还在 |

读者 `attach` 时先查这些 seal，再只读映射并 `mlock`。于是读者的页不会被任何进程回收，
读它不会 SIGBUS、不会缺页——周期域线程可以直接读视觉写的命令通道。

## 3. 会合（`hcs_link/rendezvous.hpp`）

- abstract `SOCK_SEQPACKET`，名字默认 `@hcs/autoaim`：不落文件系统，崩溃不留残段。
  同一 netns 里名字被占用时 `bind` 失败，顺带防止起两个控制进程。
- 两端都用 `SO_PEERCRED` 核对 uid（abstract socket 没有文件权限）。
- `exchange()` 对称：各发一条 `Hello{magic, protocol, fd_count}` 连同自己写的通道 fd，再收对端的。
  收到的 fd 先全部装进 `UniqueFd` 再校验，任何一步失败都不漏 fd；发送带 `MSG_NOSIGNAL`。
- 之后连接只用来察觉对端退出（`peer_closed()`）。

## 4. HCS 侧：`AutoAimLink` 组件

| | |
|---|---|
| 输入 | `/gimbal/imu/quaternion`、`/gimbal/imu/angular_velocity`、`/gimbal/imu/online` |
| 输出 | `/auto_aim/should_control`、`/auto_aim/control_direction`、`/auto_aim/should_shoot`（`SimpleGimbalController` 早已注册为可选输入） |
| 参数 | `endpoint`（默认 `hcs/autoaim`）、`stale_ticks`（50）、`max_extrapolation_ms`（60）、`state_capacity`（1024）、`lock_memory`（true） |

每拍：写一条 `GimbalState` → 从命令通道取最新一条交给 `AimFollower` → 写三个输出。

- `AimFollower`（纯逻辑）：新命令先校验（§6），不合法就作废；按本端拍数判陈旧（对端时间戳只用来外推）；
  外推到本拍（钳在 0–60 ms）；开火只在命令给的时间窗内。
- `hcs-autoaim` 线程：接连接、互换 fd、`attach` 命令通道，**新连接直接顶掉旧的**
  （视觉 fork 过时旧连接的 EOF 可能永远不来）；对端退出就把命令通道撤下。
- 会话切换用 `QuiescentCell`（QSBR）：换指针后等周期域走过一个静止点（拍末）再释放旧的；
  周期域 200 ms 内没走过静止点（没在跑拍）就宁可泄漏也不释放。

## 5. 视觉侧怎么用

```cpp
#include <hcs_link/autoaim.hpp>
#include <hcs_link/rendezvous.hpp>

using namespace hcs_link;
auto socket   = connect(autoaim::kEndpoint);                        // 没连上就隔一会儿重试
auto commands = Writer<autoaim::AimCommand>::create({.capacity = 64});
auto received = exchange(socket->get(), std::array{commands->fd()}, std::chrono::seconds{1});
auto state    = Reader<autoaim::GimbalState>::attach(std::move(received->fds[0]));

auto cursor = state->cursor();                   // 逐拍读姿态，一条不漏；或者 state->latest()
while (auto sample = cursor.next()) { /* sample->value.quaternion, tick_ns … */ }

commands->publish(autoaim::AimCommand{/* t_ref_ns, 方位/俯仰及其导数, 开火窗, flags */});
// 每处理完一帧发一条，没看到目标也发（flags 不带 kHasTarget），当心跳用。
// publish 只许一个线程调用；读可以任意线程并发。
```

## 6. 契约（`hcs_link/autoaim.hpp`）

时间一律 CLOCK_MONOTONIC 纳秒（`steady_clock`）；角度一律在 OdomImu 世界系：方位绕 +z、自 +x 逆时针为正，俯仰向上为正。

`GimbalState`（80 B，控制 → 视觉，每拍一条）：`tick_ns`、`tick_sequence`、`quaternion`（w x y z）、`angular_velocity`（rad/s，FLU）、`imu_online`。

`AimCommand`（88 B，视觉 → 控制）：`t_ref_ns`、`frame_id`、方位 / 俯仰及其角速度、角加速度、`fire_from_ns` / `fire_until_ns`、`flags`（`kHasTarget | kFire`）。

控制侧拒收：未知 flag 或保留字段非零；任何非有限值；|俯仰| > π/2；角速度 > 50 rad/s；角加速度 > 1000 rad/s²；
`t_ref` 早于现在 200 ms 或晚于现在 50 ms；开火窗倒置或长于 200 ms。

载荷规则：可平凡复制、标准布局、大小是 8 的倍数（`Payload` 概念检查）；不用 `bool` / `enum` / 指针；
每个载荷带 `kName` / `kVersion`，两端对不上就拒绝接上；改布局必须升 `kVersion`（紧跟一条 `sizeof` 断言）。

## 7. 测试与实测

单测（2026-09-29 全部通过：GCC 14.2 与 clang 20 各一遍，`hcs_link` + `hcs_demo` 共 104 项；
`hcs_link` 另在 clang + TSan 下通过；端到端用例连跑 10 次 10 次通过；新文件在 clang `-Wfunction-effects` 下 0 告警）：

| 文件 | 钉住什么 |
|---|---|
| `hcs_link/test/test_channel.cpp` | 序号语义、游标无损与丢失计数、载荷 / seal / 段头校验、seal 挡住截断打洞（含写者自己）、多线程和跨进程（200 万条）不撕裂 |
| `hcs_link/test/test_rendezvous.cpp` | 两进程互换 fd 互读、对端退出可察觉、名字占用、畸形 Hello 拒收且不漏 fd、超时 |
| `hcs_link/test/test_quiescent_cell.cpp` | 周期域在用的对象绝不被释放；周期域不跑拍时宁漏不放 |
| `hcs_link/test/test_autoaim.cpp` | 布局偏移、拒收规则、外推与开火窗 |
| `hcs_core/test/test_aim_follower.cpp` | 跟随、陈旧、心跳、坏命令、开火窗、换会话 |
| `hcs_core/test/test_auto_aim_link.cpp` | 端到端：真组件 + 真 socket + 真通道，模拟视觉连上、发命令、读姿态、退出、重连 |

本机实测（写者 CPU 3、FIFO 90、1 kHz，读者在另一进程）：

| 项目 | p50 | p99 | 最大 |
|---|---|---|---|
| RT 写一条 64 B | 16 ns | 18 ns | 10.6 µs（CPU 3 被中断打断） |
| 另一进程看到这一条（读者自旋） | 87 ns | 0.75 µs | 11 µs |
| 跨进程读 512 万次 | 放过坏数据 0 次 | | |

对比过、没用的通知方式：`FUTEX_WAKE` 推送让 RT 每拍多 1.4 µs，读者被叫醒的 p99 反而是 407 µs；所以读者一律自己拉。

## 8. 相对 v1–v3 删掉了什么

| 删掉的 | 为什么 |
|---|---|
| 泵线程、`EventQueue → 泵 → 环`、目标走 socket 消息 | 写者自封的 seal 已经让读对端通道没有缺页风险（§2.4），周期域可以直接读写两条通道 |
| 过程映像、字段表、按名字取字段 | 太重。两个定长载荷加版本号足够；以后要导出别的，再加一种载荷 |
| 布局哈希和金值表 | 载荷名字 + 版本 + 大小校验，加每个载荷的 `sizeof` 断言，足够抓住两端不一致 |
| 角色、kBusy、挤占规则 | 新连接直接顶掉旧的 |
| v2 的 `active_ / ack_` 关会话协议 | 有第一拍竞态且是 Dekker 形状；改成 `QuiescentCell`（全部 seq_cst，按拍末静止点等宽限期） |

## 9. 待定

1. 上车后测：视觉满载时控制拍的耗时分布；`/gimbal/imu/*` 的 `frame_count` 与原始时间戳要不要进 `GimbalState`（见附录 A）。
2. 裁判系统移植后，控制 → 视觉的上下文（敌方颜色、弹速、模式）作为第三种载荷加进来。
3. 已修：提交 58df7c7 清理 `hcs_utility` 时删掉了 `tdigest.hpp`，但它的唯一使用者 `hcs_executor` 的 RtReporter
   还在包含它。现在原样放回到使用者旁边（`hcs_executor/src/tdigest.hpp`，命名空间改为 `hcs_executor`），
   `hcs_utility` 保持清理后的样子。全工作区 GCC / clang 从零构建各 197 项测试通过。

---

## 附录 A. 时间对齐（原 §4，搁置到硬件接上）

> 未复审。待查：`hipnuc.hpp` 已定义 `kSyncOutPulse`（HI91 MAIN_STATUS bit12），CH040 数据手册给出了 SYNC_OUT / SOUT_DIV 引脚；
> 如果相机由 IMU 硬触发，下面"按 monotonic 时间戳配对"的做法要重写。
> v4 的 `GimbalState` 目前每拍一条、只带 `tick_ns`；定下配对方式后按需加 IMU 原始时间和状态字段并升 `kVersion`。

### A.1 唯一的公共时钟

- **CLOCK_MONOTONIC 的纳秒数**，存为 `int64_t`。就是 `hcs_sync::Clock`（`steady_clock`），
  libstdc++ 在 Linux 上用的就是它，两个进程天然共享，不需要任何同步。
- 共享内存里**只出现这一种时钟**。板钟、microframe、相机设备时钟、HiPNUC 的 `system_time`
  都必须在各自那一侧换算成 monotonic 以后才能写进来。
- 不用 CLOCK_REALTIME（NTP 会跳）；不用 CLOCK_MONOTONIC_RAW（`steady_clock` 不是它，
  两进程各用一种就对不上）；不用 CLOCK_BOOTTIME（整车不休眠，用不上）。

### A.2 姿态的时间戳 = IMU 的采样时刻

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

- 档 A 的 `imu_latency` 用"摆云台"标定来定。
- **IMU 输出率要提上去**：CH040 默认 HI91 输出 100 Hz（`hipnuc.hpp:73`），也就是姿态历史只有 10 ms 一个点，
  云台快速转动时插值误差远大于上面这些。上车前把模块配到 ≥ 400 Hz（921600 波特下 82 B 帧最多约 1100 Hz）。
- 环里**每个 IMU 帧写一条**，靠 `frame_count()` 变化来判断，不是每拍写一条：
  否则 100 Hz 输出、1 kHz 控制时会写 10 条重复姿态，把历史窗口压缩到 1/10。

### A.3 坐标系：两边都用 OdomImu

- 环里只写云台 CH040 的原始四元数：机体 FLU 相对模块世界系，也就是 `hcs_description::OdomImu`
  的那个旋转。控制侧不做任何变换——出问题时可以直接对照原始日志。
- 相机和 IMU 都装在 pitch 连杆上，所以相机的世界姿态 = IMU 四元数 × 相机相对 IMU 的安装外参，
  不需要编码器角。外参在视觉侧标定、在视觉侧用，和相机内参放在一起管。
- 视觉发回的瞄准方向也表达在同一个 IMU 世界系里，正好是 `SetControlDirection` 要的
  `OdomImu::DirectionVector`。IMU 的 yaw 漂移对两边是同一个漂移，互相抵消。

### A.4 相机侧（视觉进程的责任，这里只定约束）

- 每帧的 `exposure_mid_ns` = 曝光中点的 monotonic 时刻。
  - 优先：相机设备时间戳 → monotonic 的在线线性拟合（与 libhcs `Timeline` 同一个方法，窗口约 1 分钟）；
  - 保底：SDK 回调时刻 − 标定过的常数延迟 − 曝光时间 / 2。
- 用 `exposure_mid_ns` 去姿态环里查；目标里也写这个时刻，控制侧外推用它。

### A.5 误差预算

| 来源 | 量级 | 在 5 m 处 |
|---|---|---|
| 通道本身（seqlock 读写） | < 1 µs | 可忽略 |
| 姿态戳，档 A | 0.1–1 ms × 云台角速度 3 rad/s ≈ 0.3–3 mrad | 1.5–15 mm |
| IMU 100 Hz 时的插值 | 取决于角加速度，最坏 ~5 mrad | ~25 mm |
| 相机戳（保底方案） | 1–2 ms × 3 rad/s ≈ 3–6 mrad | 15–30 mm |
| 小装甲板半宽 | — | 67 mm |

结论：要把 IMU 输出率和相机戳做对，通道本身的纳秒级成本不是问题。
