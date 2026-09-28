# HCS 框架现状与下一步（2026-09-29）

> 记录这一轮的结论：非停机调试工具、实时 I/O 路径的代码级审查、libusb 的取舍、包结构重构计划。
> 除非特别标注"实测"，这里的判断都来自读代码，**尚未接板子验证**。

## 1. 本轮状态

- **上层重写**：按 [balance-infantry-rewrite.md](balance-infantry-rewrite.md) 完成了阶段 0–3（ZCode 执行）：
  - `hcs_core/src/controller/chassis/balance/` 下的平衡底盘组件；
  - `hardware/balance_infantry.cpp`；
  - RMCS 通用云台链；
  - `hcs_description` 包。
- **libhcs 已更新到 `8400254`**（v3.3.0-5），嵌套的 tinyusb 也同步到了 `7d168b65`。
- **验证（实测）**：在独立构建目录里，7 个包 gcc 编译 0 警告，`colcon test` **93 个测试全部通过**。
- **调试信息**：`hcs_core` 和 `hcs_executor` 都加上了 `-g`。`-g` 不改变生成的代码，只增大文件。
- **没做的**：任何上板测试。

## 2. 非停机调试（像 Ozone 一样看运行中的变量）

工具在 `hcs_bringup/tools/`，用法和 VS Code 配置写在 `hcs_gdbstub.py` 的文件头里。

- **`hcs_gdbstub.py`**：gdb 远程服务端，只从 `/proc/PID/mem` 读内存，不用 ptrace，也从不停止进程。
- **`hcs_gdb.py`**：
  - `hcs-components`：把每个组件按 yaml 实例名绑定成 gdb 变量（`$demo_hardware`）；
  - `$hcs("实例名")`：延迟查找的 gdb 函数，供 Cortex-Debug 的 Live Watch 使用；
  - 输入输出接口的 pretty printer，直接显示值。
- **前端**：
  - VS Code 装 Cortex-Debug，用 `servertype: external` + `request: attach` + Live Watch，可以自动刷新；
  - 或者用自带的 cpptools，按 F5 刷新。
- **实测（demo，FIFO 90，隔离核 7）**：
  - 连续读整棵对象树 1904 次（每秒 63 次），再加 10 Hz 刷新 200 次，start_late 最大值一直等于空闲基线（15–16 µs），p99 4 µs，跳拍 0；
  - 在线改值有效；
  - MI 变量对象不用 continue 也能刷新。
- **安全**：写入只允许落在可写数据段。gdb 在继续运行前会往代码段写 `int3` 插断点（包括它自己放在 ld.so 里的内部断点），实测这 4 次写入全部被拒，代码字节不变。
- **前提**：目标进程启动时带 `HCS_ALLOW_DEBUG_ATTACH=1`；读取期间保持 `mlock=on`。
- **局限**：
  - 看不到局部变量；
  - 读到的是两拍之间的值，偶尔会一半属于上一拍；
  - 写入和 RT 线程构成数据竞争，只适合调参；
  - VS Code 界面本身还没实际点过。

## 3. 实时 I/O 路径审查

每一拍经过的路径：

```
控制线程 (CPU3, FIFO90) → 门铃 futex
发送线程 (CPU3, FIFO85) → libhcs 取缓冲（PI 锁）→ libusb_submit_transfer → ioctl(SUBMITURB)
xHCI 中断 (CPU4) → IPI → IO 线程 (CPU7, FIFO80) → poll → REAPURB → libhcs 解包 → store_status → 重新提交接收缓冲
```

**控制线程是干净的**：新控制器在 clang `-Wfunction-effects` 下 0 警告，调度器路径已经审过。
问题集中在控制线程之外：

| # | 问题 | 证据 | 风险 |
|---|---|---|---|
| 1 | libusb 每次提交 bulk 传输都 `calloc` 一个 URB 数组，完成时释放。发送线程和 IO 线程每拍都会走到 | libusb 1.0.27 `linux_usbfs.c:1977`；最新 master 也没改 | **高**：见下一行 |
| 1a | `RealtimeArm` 设了 `M_ARENA_MAX=1`，全进程只有一把 malloc 锁，DDS、ROS 的普通线程也在抢。发送线程只分配、IO 线程只释放，线程本地缓存（tcache）帮不上忙 | `hcs_utility/.../realtime_arm.hpp:439` | 普通线程持锁时被抢占，FIFO 85 的发送线程就要等 |
| 2 | libusb 内部锁（`flying_transfers_lock` 等）不是优先级继承锁 | `threads_posix.h` 用的是 `pthread_mutex_init(mutex, NULL)` | **低**：每块板各自 `libusb_init`，锁只在同一块板的发送线程和 IO 线程之间争，两边都是隔离核上的 RT 线程 |
| 3 | 实时线程上的日志是同步写的 | libhcs `logging.hpp:178` 在 IO/发送线程上直接 `fwrite(stderr)`（注释自认 stderr 一堵链路就死）；`vt13.hpp:50` 在 IO 线程里 `RCLCPP_WARN` | 中：平时不触发，一旦触发就是长时间阻塞 |
| 4 | 实时检查只覆盖控制线程；发送线程和 IO 回调不在 `RealtimeScope` / RTSan 覆盖范围内 | `executor.hpp` 只包住了拍循环 | 中：上面这些问题都是靠人读代码发现的 |
| 5 | 所有 RT 线程在同一个物理核上（CPU 3 和 7 是超线程兄弟） | `balance-infantry.yaml` 的线程布局 | 待测：NLMPC 计算和三块板的 IO 回调抢同一套执行单元 |

**确认不需要做的**：
- 控制路径上的内存池和自定义分配器：武装之后零分配，这条规则已经解决了；
- 大页：这个 PREEMPT_RT 内核本身没有透明大页；
- RT 限流：已经关掉（`sched_rt_runtime_us = -1`），MachineGuard 也会检查。

## 4. libusb 的取舍

- **libusb 的 API 解决不了第 1、2 条**：`libusb_set_option` 只有 LOG_LEVEL、USE_USBDK、NO_DEVICE_DISCOVERY、LOG_CB 四个选项，没有分配器接口，也没有锁属性的配置。
- **libusb 已经解决了的**：事件锁的问题（控制传输单开上下文、专用事件线程）、零拷贝（`libusb_dev_mem_alloc`）。
- **真正要紧的是第 1a 条，在分配器层面就能解决，不需要离开 libusb。**
- **结论：目前不抛弃 libusb。** 升级路径如下，每一步都要有上一步的实测证据才往下走：
  1. 接板子，用 `hcs_link` 探针在 DDS 满负载下 A/B 三种配置：`M_ARENA_MAX=1`（现状）/ 去掉这一项 / `LD_PRELOAD` mimalloc。看发送耗时和 rtt 的 p99.9 与最大值；
  2. 如果尾延迟还不好，用 RTSan 加 trace 定位；
  3. 只有证据指向 libusb 的数据通路本身时，才做下面二选一：
     - 给 libusb 打补丁并内置：URB 数组缓存复用，锁改成优先级继承，约 50 行；
     - 数据通路直接用 usbfs（预分配 URB，`SUBMITURB` / `REAPURBNDELAY`），libusb 只保留给枚举和 EP0，约 500–800 行。
- **完全抛弃 libusb、写内核驱动：都不做。**

## 5. 包结构重构计划（3 层）

```
libhcs          板卡 SDK（子模块）
hcs_base        所有不依赖 ROS 的底层
  thread/         线程配置、实时武装、精确睡眠、门铃、门铃线程、RealtimeScope、RTSan 宏
  channel/        Snapshot、EventQueue、ByteTunnel、时钟类型、（新增）异步日志队列
  protocol/       CRC、字节序、环形缓冲、拆包、Eigen 结构化绑定
  check/          MachineGuard
hcs_executor    组件模型、Tick 与时基、1 kHz 主循环、启动检查、运行统计（rt_sampler / tdigest 收成私有）
hcs_core        device/ hardware/ controller/（包名从 hcs_demo 改成 hcs_core）
hcs_msgs / hcs_description / fast_tf
hcs_bringup     配置、launch、工具
```

规则：
1. 下层不认识上层；
2. ROS 从 `hcs_executor` 才开始出现；
3. 只有一个包在用的东西，就放进那个包。

要删的死代码（没有任何引用）：`csv_writer`、`fps_counter`、`pooled_shared_factory`、`memory_pool`、`thread_assert`、`tick_timer`、`rclcpp/node_mixin`。
`hcs_sync` 和 `hcs_utility` 合并成 `hcs_base`，大约 30 个文件、93 行 include 要改，全是机械替换。

## 6. 待办（按顺序）

**不用板子：**
1. 删死代码，`hcs_demo` 包名改成 `hcs_core`；
2. 合并出 `hcs_base`；
3. 调度器启动时统一跑 MachineGuard，Critical 默认拒绝启动；核号和优先级集中到一份机器配置里（比如 `tl101.yaml`）；
4. `RealtimeScope` 覆盖发送线程和 IO 回调（第 3 节第 4 条）；
5. 异步日志队列，改掉 libhcs Logger 和 `Vt13` 的同步日志（第 3 节第 3 条）；
6. 把 malloc 策略（`M_ARENA_MAX`）做成可配置项，默认先保持原样。

**接板子之后：**

7. RTSan 版本跑一遍 demo 和平衡车空载；
8. 第 4 节的 malloc A/B；
9. 开着组件计时量平衡车的 NLMPC 和 IO 回调，决定要不要隔离第二个物理核；
10. 版本门禁：给 libhcs 定一个打 tag 的发布流程，然后关掉 `dangerously_skip_version_checks`。

**上车前要用户确认的**（来自重写任务书）：
- 遥控接收机型号（VT13 / VT03 / DR16）；
- 腿关节前后的对应关系；
- 三块板的接线表；
- DM 电机里实际存的 PMAX/VMAX/TMAX。
