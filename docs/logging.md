# 日志（hcs_log）

> 状态：代码和单测已完成（gcc 14 / clang 20，ASan + UBSan + TSan 全过），**未上车**。
> 代码：`hcs_base/include/hcs_base/logging/`（纯头文件，不依赖 ROS）、`hcs_executor/src/rclcpp_log_sink.hpp`、
> `hcs_core/src/hardware/libhcs_log_bridge.cpp`、libhcs 的 `host/include/libhcs/logging.hpp`。
> 带"实测"的数字在本机（i9-13900H，Linux 7.0 非 RT 内核，GCC 14.2 `-O3 -march=x86-64-v3`）上测得。

## 1. 为什么自己做

输出会阻塞。stderr 接的是管道（launch、journald），读的一方跟不上时 `write` 就卡住；rclcpp 的日志
是在**调用线程**上同步做这件事的，还拿着一把全进程的锁。卡住的如果是控制线程或 USB 事件线程，
链路就跟着死。原先的对策是"周期域只计数、尽力域去打印"，每个地方都要手写一遍，而 libhcs（板卡
SDK）的日志在库的私有实现里，根本换不掉。

所以把"输出会阻塞"这件事关进一条线程：

```
任何线程 ── Logger ──▶ MpscQueue<Record>（无锁，1024 条）──▶ 日志线程 hcs-log ──▶ Sink
                                                                        ├ StderrSink（默认）
libhcs 的各条线程 ── set_sink ── 桥 ──┘                                 └ RclcppSink（executor 起来之后）
```

打日志的线程只做一次无锁入队：不阻塞、不进内核。队列满了丢一条、记个数，日志线程最多每秒
报一次丢了多少。输出端堵多久，堵住的都只是 `hcs-log`。

## 2. 用哪个入口

按**调用线程属于哪个域**选，写在调用点上一眼看得出来：

| 入口 | 给谁用 | 做什么 | 实测 p50 / p99 |
|---|---|---|---|
| `logger().warn("accept: {}", e.message())` | 尽力域：构造、定时器、工作线程 | 当场格式化，实参随意；超过 442 字节的放到堆上，**不截断** | 61 / 96 ns（一个整数）；232 / 300 ns（三个浮点） |
| `logger().rt().warn("crc invalid, {} so far", n)` | `update()`、板卡 IO 回调 | 只把实参按字节拷进记录，格式化挪到日志线程。标了 `nonblocking` | 33–53 / 76 ns |
| `logger().rt().write(level, text)` | 同上，文本已经在手上 | 一次拷贝；超过 442 字节**截断**，结尾换成 `...` | 32 / 70 ns |
| `logger().write(level, text)` | 尽力域，文本已经在手上 | 一次拷贝，不截断 | — |

（上面的数字含两次读时钟约 17 ns。被级别拦下的调用 12 ns。）

- 组件里直接用 `logger()`（`Component` 自带，名字就是组件名），**不需要继承 `rclcpp::Node`**。
- 两套入口的格式串都在编译期按实参类型检查，而且必须是字符串字面量。
- `rt()` 只收平凡可拷贝、非指针的实参：传 `std::string`、`const char*`、`std::string_view` 是**编译错误**，
  不是运行时的一次 malloc。枚举先转成整数。
- clang 下 `-Wfunction-effects` 会拦住在 `HCS_NONBLOCKING` 函数里用尽力域入口；gcc 不检查，
  所以提交前要过一遍 clang 构建。

**高频事件别直接打。** 1 kHz 的回路里每拍一条，队列一秒就满。两个现成的稀释器：

```cpp
hcs_log::Backoff crc_errors_;                       // 周期域：第 1、2、4、8… 次才报，不取时钟
if (const auto count = crc_errors_.hit())
    logger().rt().warn("crc invalid ({} so far)", *count);

hcs_log::Throttle link_fault_report_{5s};           // 尽力域：按时间限流，要取时钟
if (link_fault_report_.ready())
    logger().error("board faulted (link lost)");
```

## 3. 进程里的接线（hcs_executor）

- **后端只有一个**：`hcs_executor::process_log_backend()`，定义在 `libhcs_executor.so` 里，executor 和每个
  被 dlopen 进来的组件库拿到的是同一个。第一次调用发生在主线程（所以 `hcs-log` 继承的是
  `HCS_NON_RT_CPUS` 设好的进程亲和、`SCHED_OTHER`），不会落到周期域里。
- **输出端的切换**（`RclcppLogScope`，在 `main` 里）：rclcpp 起来之前和关掉之后走 stderr，中间走 rclcpp
  ——控制台、`~/.ros/log`、`/rosout`、按 logger 调级别都照常能用。
- **`/rosout` 认的是名字**（实测，jazzy）：rcl 只替"名字和某个节点相同"以及"从节点 logger 上
  `get_child()` 出来"的 logger 往 `/rosout` 上发；一个凭空的 `rclcpp::get_logger("x")` 控制台有、
  `/rosout` 上没有，而且不报错。所以组件到齐后 `main` 调一次 `attach()`：
  - 本身是节点的组件：用节点自己的 logger，名字原样（`[value_broadcaster]`）；
  - 其余的（不是节点的组件、`libhcs`）：挂到 executor 下面，显示成 `[hcs_executor.libhcs]`；
    libhcs 针对某块板的行带着序列号，显示成 `[hcs_executor.libhcs.AF-90A7]`。

  `attach()` 之前（组件还在一个个构造的那一段）哪些名字是节点还定不下来，那一段里"其余的"
  仍然是凭空的名字：控制台和文件里有，`/rosout` 上没有。受影响的主要是板卡构造时 SDK 打的那几行
  （绑核、提优先级、版本不匹配）；运行期的断连 / 重连都在 `attach()` 之后。
- **卸库之前必须排空**：`rt()` 的记录里存着组件库里的函数地址和格式串地址。`RclcppLogScope` 声明在
  `pluginlib::ClassLoader` 之后，所以先析构、先 `flush()`，库后卸。
- **`std::terminate`**：处理函数先 `flush_for(500ms)` 再交回原来的处理函数。只等半秒——输出端堵着的话
  这几行就不要了：进程必须死得掉，它不死，板子等不到断连，电机停不下来。
  光有超时还不够：原来的处理函数往 stderr 打 `what()` 是一次没有超时的 `fputs`。实测 stderr 接一根
  写满了没人读的管道时进程就挂在那里，`SIGTERM` 也收不走。所以处理函数第一句是 `alarm(2)`，
  两秒后由 `SIGALRM` 的默认动作收场（实测：2.0 s 退出，退出码 142）。

## 4. libhcs 的日志

libhcs 是可以单独使用的库，所以分工是：**库只管"发生了什么"，并留一个出口；日志"怎么送、送到哪"
由用它的程序决定。** 两边互相不认识——libhcs 不依赖 `hcs_log`，`hcs_log` 不知道 libhcs，认识双方的
只有 `hcs_core` 里的那个桥。判断这件事做对没有，看三条：

| | 改之前 | 现在 |
|---|---|---|
| 单独拿出来能编、能跑 | 能 | 能（脱离 ROS / HCS 单独构建，含全部示例，SDK 源码零告警） |
| 单独跑的时候自己不出事 | **不能**：stderr 堵住，USB 事件线程跟着卡死 | 能：写不进去就丢、计数，不等 |
| 放进大系统里听统一安排 | **不能**：输出在库的私有实现里，换不掉 | 能：出口接到 `hcs_log` |

**出口**（`libhcs/logging.hpp`，只依赖标准库）：

```cpp
struct Record { Level level; std::string_view source; std::string_view message; };

class Sink {                                  // 用它的程序来实现
public:
    virtual void write(const Record& record) noexcept = 0;
};

Sink* set_sink(Sink* sink) noexcept;          // nullptr = 回到 stderr；返回原来的那个
class ScopedSink;                             // RAII：活着时接管，析构时把原来的放回去
std::uint64_t stderr_lines_dropped() noexcept;
```

- 用接口而不是"回调 + `void*`"：对象本身就是上下文，接收方不必靠全局变量；SDK 要换的只有一个指针，
  接管仍然是一次原子写。
- `source` 是板的 USB 序列号。SDK 里的 logger 不再是全进程单例，而是每条链路各带一个：
  一个进程接三块板，`USB link faulted` 不说是哪一块就等于没说。不针对某块板的行（设备扫描）为空。

**默认输出**（没人接出口时）：格式和以前一样（针对某块板的行多一段 `[序列号]`），但不再等 stderr。
`pwritev2(RWF_NOWAIT)` 写不进去就丢掉并计数，恢复后第一行之前先报丢了多少；终端不认这个标志，
退回"先 `poll` 再写"。实测同一根写满的管道、读端停 3 秒、连打 200 行：

| | 耗时 | 结果 |
|---|---|---|
| 改之前 | 2999 ms（整段停顿都卡在里面） | 200 行在读端恢复后全部写出 |
| 现在 | 0.16 ms，单次最坏 10 µs | 丢 200 行；恢复后先出 `200 log line(s) dropped`，再出新的一行 |

文件、`/dev/null`、读端正常的管道、终端下 200 行一行不丢。

**HCS 这边的桥**（`hcs_core/src/hardware/libhcs_log_bridge.cpp`）：一个 `Sink` 的实现，
`write()` 里拼出日志口的名字（`libhcs` 或 `libhcs.AF-90A7`）再 `rt().write()`——一次拷贝加一次无锁入队，
标了 `nonblocking`（clang 下往里塞一句 `fprintf` 当场报错，已做过 A/B）。它由一个静态的 `ScopedSink`
在 `libhcs_core.so` 被加载时装上、卸载时摘下，不靠哪个板卡组件记得去调：出口是全进程一个，
板卡组件却有好几种，这和 pluginlib 自己登记组件类是同一个办法。SDK 一行最长约 1 KiB，
长于一条记录的 442 字节，超出的截断。

没有改的：`usb.cpp` 里靠环境变量打开的那几段诊断输出（直方图之类）仍然直接 `fwrite(stderr)`，
它们只在主动调试时出现。非 Linux 平台上默认输出仍是原来的阻塞 `fwrite`（没有等价的系统调用，未验证）。

libhcs 是子模块：这部分改动要**在 libhcs 仓库里单独提交**，再更新 HCS 里的子模块指针。

## 5. 保证与不保证

保证（都有单测钉着，见 `hcs_base/test/test_log.cpp`、`hcs_executor/test/test_rclcpp_log_sink.cpp`、
`hcs_core/test/test_libhcs_log_bridge.cpp`）：

- 输出端卡死时，打日志的线程不等它；丢了多少会被报出来。
- 同一条线程的日志顺序不乱（两套入口混用也不乱）。
- `flush()` / 换输出端 / 析构时，已经入队的日志一条不少；堆上的溢出文本不泄漏。
- 输出端抛异常不带走进程，只计数（`Backend::sink_failures()`）。
- `flush_for()` 在输出端堵死时按时放弃；从日志线程自己身上调 flush 不死锁。

不保证，要知道的：

- **时间戳是输出的时刻，不是事件的时刻。** 记录里不带时间（周期域不取时钟；rclcpp 也不收外来的
  时间戳）。平时晚不超过一个轮询周期（5 ms），输出端堵过的话会晚得更多。跨线程的先后看行的顺序。
- **不同线程之间的顺序是入队顺序**，不是严格的事件顺序。
- 一个生产者在"抢到格子"和"写完格子"之间被抢占时，排在它后面的日志要等它回来才出得去
  （没有人因此被阻塞，只是输出晚一点）。
- `SIGSEGV` 的处理函数不排空队列（信号处理函数里不能等锁）：崩溃前最后几毫秒的日志可能没出来。
