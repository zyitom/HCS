# PORTING.md — Helios 平衡步兵（轮腿）→ HCS 移植盘点（阶段 0）

> 状态：**阶段 0 产出，等确认后才进入阶段 1**。
> 本文档回答任务书阶段 0 要求的全部七项：硬件对照、状态变量归类、阶段转移表、
> 拍数换算表、每拍执行顺序映射、组件划分与接口、风险与待决问题。
>
> 源码行号基准（以今日读到的代码为准，CLAUDE.md 的行号/数值已多处过期，本文一律以代码为准）：
> - Helios（只读，未改动）：`/home/helios/Desktop/Helios_cpp_powerful_framework`
>   - `User_Code/app/chassis_app/balance/chassis_balance_v2.{hpp,cpp}`（下称 **v2**，cpp 共 2498 行）
>   - `User_Code/app/chassis_app/balance/balance_nlmpc.{hpp,cpp}`
>   - `User_Code/user/balance/balance_tasks.cpp`、`balance_def.{hpp,cpp}`
>   - `User_Code/app/chassis_app/chassis_ctrl.{hpp,cpp}`、`User_Code/module/software/fsm/fsm.{hpp,cpp}`
>   - `User_Code/module/def/robot_def.hpp`、`User_Code/module/algorithm/balance_alg/balance_algorithm.{hpp,cpp}`
> - HCS：`/home/helios/Desktop/HCS`（分支 `port/helios-balance`）
> 注意：Helios 部分 `balance_def.hpp` 为 GBK 编码且文件尾部含 NUL 填充；行号以转换后内容为准。

---

## 1. 一页结论

1. **移植范围**：轮腿底盘全套（估计 → 安全 → 阶段状态机 → 控制器 → 电机输出），
   按任务书架构决策 A/B/C 执行：无全局 FSM、阶段用 `std::variant`、不用状态机库。
   云台/发射**不能**按原任务书"复用 RMCS 现有组件"——HCS 里没有那批组件
   （见 §10 待决问题 1）。
2. **HCS 与任务书里 RMCS 事实的差异已逐条核实**（§2）：组件 API 变成
   `update(const hcs_sync::Tick&)`、时间直接取 `tick.dt_seconds()`、新增显式单位延迟
   `DelayedInput`（1/z）、硬件发送必须走拍尾 Doorbell 唤醒的独立线程、组件失效按伙伴组隔离。
   任务书的"单线程 1kHz、拓扑排序、禁函数内 static、时间用秒"四条全部仍然成立。
3. **硬件**：Helios 是两块 STM32H723（云台板/底盘板）+ IBC 板间 CAN。HCS 是纯上位机 +
   libhcs USB-CAN 板（HPM5321 双 CAN）。**HCS 目前没有任何真实设备驱动**
   （`hcs_demo` 里只有 `fake_motor.hpp`），DM/LK/DJI/超级电容/遥控/CH100 全部要新写，
   协议资料来源已逐项列出（§4），没有凭猜测写协议。
4. **重要行为基线（对拍测试必须按这个来）**：Helios 当前代码里
   **离地检测被一行 `return 0;` 禁用**（v2:1047）、**LESO 观测器从未被调用**
   （`leso()` 无调用点，且 LQR_ON 分支每拍 `reset_leso()`，v2:925/2349）、
   **功率 QP/RLS 路径恒不激活**（`pwr_ctrl` 无赋值点，恒 false，v2:927）、
   **TOUCH_DOWN 与 TANK 无入口**（死状态）、**跳跃键恒为大跳**（`jump_cmd` 恒 2，v2:1328）、
   **`pitch_preset_used` 被强制清零**（v2:826）。这些"当前真实行为"全部保留为对拍基线，
   是否在移植中"修复"，逐条列在 §9 由你决定，不顺手改。
5. 组件划分提案与接口（§8）：新增 1 个硬件组件（含 Command 伙伴）+ 1 个平衡底盘控制组件；
   "按键 → 操作意图"做成底盘组件内部的小类（决策 A）。依赖图无环（§8.3）。

---

## 2. HCS 框架事实（已核实，替代任务书里的 RMCS 版本）

| 任务书里的 RMCS 事实 | HCS 实际情况（已读源码核实） |
|---|---|
| 组件继承 `rmcs_executor::Component`，`update()` 无参 | `hcs_executor::Component::update(const hcs_sync::Tick&) HCS_NONBLOCKING override`（component.hpp:60）。Tick 携带 `scheduled`（本拍名义起点）、`dt`（跳拍时为 N×period）、`sequence`；`tick.dt_seconds()` 即秒制 dt |
| 时间从 `/predefined/update_rate`、`/predefined/timestamp` 换算 | `/predefined/*` 是**遗留接口**（predefined_msg_provider.hpp 头注："新代码一律用 update(const Tick&) 的参数拿时间和拍号"）。dt 只许来自 `tick.dt`；新代码不用 `/predefined/*` |
| 所有组件同一线程 1kHz 依次执行，拓扑排序，环即启动报错 | 成立。`Linker::link()` → `graph::build()`（wiring.hpp/graph.hpp），顺序与接线在启动时定型；执行在唯一 RT 线程（executor.hpp:205-225） |
| `/chassis/xxx` 是组件间内存指针绑定，不是 topic | 成立（OutputInterface/InputInterface 直接绑存储） |
| 同一组件类可实例化多次，禁止函数内 static | 成立。全组件状态必须是成员变量或阶段自带数据 |
| 参数用 `get_parameter_or` | 组件可双继承 `rclcpp::Node` 拿参数（DemoBoard/PidController 的写法），或用 `hcs_utility::NodeMixin` 的 `param()/param_or()`（node_mixin.hpp）。yaml 按实例名分段（demo.yaml 同构） |
| 硬件组件通过 librmcs 板子收发 CAN | **改为 libhcs**：`libhcs::board::Hpm5321`（双 CAN，HPM5321 板）等。接收在事件域回调（`can_receive`/`uart0_receive_callback`），经 `hcs_sync::Snapshot<T>` 跨域发布给周期域；发送必须走 `board_->start_transmit()`——**它内部有锁、可能阻塞，禁止在 update() 里调**，标准做法是拍尾 Doorbell 唤醒 `DoorbellWorker` 线程发送（can_rtt_probe.cpp:276-288、component.hpp:293-302） |
| 失效隔离：组件抛异常 → 停调度 | 更细：按**伙伴组**整组停（`create_partner_component` 组），输出复位到注册初值，`/predefined/safe_mode=true`（wiring.hpp:121-148）。这是组件级安全网，与"电机侧超时保护"是两回事（见 §10 待决问题 2） |
| 代数环处理：硬件包拆 Status/Command 两半 | HCS 保留伙伴拆分（DemoBoard 范本），另备了显式单位延迟 `DelayedInput<T>`——它只解决**接口级环**（组件 A 的输入经图又依赖 A 自己的输出，拓扑排序启动即 fatal），是断环备胎，与延迟精度无关。**本移植预计 0 处使用**：所有"读上一拍"的状态（Helios 的 `output_last`/`dphi0_last`/`P_last` 同款）都是组件内部成员变量，不进接口图；反馈→控制→指令的跨组件环由 Status/Command 拆分解开（指令路径零延迟） |
| 编译 `-Wall -Wextra -Wpedantic`，不得有新警告 | 成立，且 `-O3 -march=x86-64-v3`；Clang 下另有 `-Wfunction-effects`：**update() 里的堆分配/加锁/日志会被编译器直接报错**（hcs_demo/CMakeLists.txt:67-72）——任务书"update() 三不"从约定升级为编译器检查 |
| 构建脚本 `.script/build-rmcs`；`set-robot` + `launch-rmcs` | 宿主机直接 `colcon build`（工作区已有 build/install）；启动 `ros2 launch hcs_bringup hcs.launch.py robot:=<名字>`（respawn=true） |
| 插件注册 `rmcs_core/plugins.xml`，源文件 CMake GLOB | HCS **不用 GLOB**：`hcs_demo/CMakeLists.txt` 显式列源文件（"档 0"注释，GLOB 会把子模块/缺依赖文件抓进编译），插件注册在包根 `plugins.xml`，库名须与 `<library path>` 一致 |

其他与移植直接相关的框架事实：
- `DelayedInput<T>` 要求 `T` trivially copyable；周期域每拍拍首闩存（executor.hpp:238-245）。
- RT 线程里"封盘点"之后（RealtimeArm 构造成功）禁止 malloc/new/日志/锁——动态内存只许在构造期做。
- 逐组件计时统计已内置：`HCS_EXECUTOR_COMPONENT_TIMING` 编译开关 + `component_timing_stride`
  参数 + `RtReporter` 周期打印（executor.hpp:43-45、129-134、247-279）——阶段 3 直接用它。

---

## 3. Helios 硬件清单与 HCS 驱动对照

Helios 平衡步兵按 `balance_tasks.cpp` 实例化（行号均指该文件），两块板由
`#ifdef BALANCE_GIMBAL / BALANCE_CHASSIS` 区分，当前默认编 **BALANCE_CHASSIS**（L34-36），
同时保留云台板代码路径。

### 3.1 底盘板（BALANCE_CHASSIS，INF_4）

| 设备 | 实例化（行号） | 总线/ID | 用途 | HCS 驱动现状 |
|---|---|---|---|---|
| DM 腿关节 ×4（bl1/bl0/br1/br0） | `DM_MOTOR(0x05..0x08, 0x01..0x04, &hfdcan1, 0, 6.283185, 45, 1, 1, 40, 零点)`（L127-130，零点依次 1.938694 / 0.717242718 / -1.37379646 / 1.39220476） | CAN，发送 0x05-0x08、反馈 0x01-0x04 | 五连杆腿驱动，MIT 模式 kp=kd=0 纯力矩；**力矩为 0 时改发 0xFD 失能帧**（RefereeTask L170-185） | **缺**。协议资料：达妙 DM-MC-Board02 手册（桌面 PDF）+ `dm_linkx_4c.xml` + Helios `module/.../dm_motor.cpp`（Unpack/Motor_MIT_CTL/Send_CMD 是协议事实的权威来源） |
| DJI 3508 轮毂 ×2（wl id2 / wr id1） | `DJI_motor(2/1, 3508, &hfdcan3, 0)`（L135-136） | CAN 0x200 电流帧 / 0x201-0x205 反馈 | 轮驱 + 电流环（C620 电调，819.2 LSB/A，减速比 13.94） | **缺**。C610/C620 协议公开且 RMCS `dji_motor` 有现成解析可对照 |
| CH100 IMU（底盘） | `CH100::getInstance(&huart7, 1, 1, 1)`（L117） | UART 7，方向映射 (1,1,1) | 姿态（roll/pitch/yaw 及速率）+ 加速度（单位 g，v2 里乘 G） | **缺独立驱动**，但 `hcs_demo/src/hardware/can_rtt_probe.cpp:318-380` 已实现同协议（超核 HI91 定长帧：0x5A A5 + len76 + CRC16/XMODEM，921600bps）的完整解析，可提炼成设备类 |
| BMI088（板载） | `BMI088::getInstance()` + `insPack()`（L120、L229） | 板载 SPI | **底盘控制不用**（v2 只吃 ch100）；云台板才用 | 不移植 |
| IBC 板间通信 | `Chassis_Ctrl(USE_IBC_BOTH, &hfdcan2, 0x3f7, 0x3f6)`（L143） | CAN 0x3f6/0x3f7 | 云台→底盘命令 + 底盘→云台遥测 | 不移植（同进程，直接组件输出），见 §7 |
| 超级电容 | 仅云台板 `SupCap::getInstance(&hfdcan1)`（L88）；底盘功率 QP 路径当前恒不激活（§6 注 5） | — | 功率缓冲 | **缺**（本期可缓），协议见 Helios `supcap.cpp` |

### 3.2 云台板（BALANCE_GIMBAL）

| 设备 | 实例化（行号） | 总线/ID | HCS 驱动现状 |
|---|---|---|---|
| yaw LK 5010 | `LK_motor(5, 5010, &hfdcan1, 0.867753744)`（L101） | LK 协议 | **缺**，协议见 Helios `lk_motor.{hpp,cpp}` |
| 拨弹 LK 5010 | `LK_motor(4, 5010, &hfdcan1, 0)`（L94） | LK 协议 | **缺**（同上） |
| pitch DM | `DM_MOTOR(0x03, 0x04, &hfdcan2, 0, 12.566, 30, 1, 1, 40, 0.0285701752)`（L102） | MIT | **缺**（同 3.1） |
| 摩擦轮 DJI 3508 ×2 | `DJI_motor(1/2, 3508, &hfdcan2, 0)`（L92-93） | C620 | **缺** |
| CH100（云台） | `CH100::getInstance(&huart7, -1, -1, 1)`（L86） | HI91 | 同 3.1，可复用解析器 |
| 超级电容 | `SupCap::getInstance(&hfdcan1)`（L88） | — | **缺** |
| 裁判系统 + UI | `RmReferee(&huart10)`、`RmRefereeUI`（L89、109） | UART | **缺**（HCS 另需串口接入方案，见 §10-6） |
| 遥控器 | `module::RC()`（L85）→ `module::FSM` 翻译 | — | **缺**（DR16/VT03；旧 hcs_balance 里有 libhcs 版解码实现可对照，见 §3.3） |
| 视觉 | `vision_app(1)`（L110） | — | **缺**（不在本期范围） |

### 3.3 核实结论与删除说明

- **`Unitree_Group`：未使用。** 仅声明（balance_tasks.cpp:61-62），从未 `new`，不移植。
- **轮毂电机：使用中**（motordata_wl/wr 直接进入速度估计、电流环、功率 RLS），上述"轮子电机
  是否实际用到"的疑问答案是：用。
- 任务书提到的"CH100 是超核（HiPNUC）产品"——正确（HI91 协议，超核 IMU）。
- **旧 `hcs_balance` 包已按你的确认删除**（该包从未进 git，删除即永久；已留临时备份
  `/tmp/hcs_balance_deleted_20260918.tgz`，不需要可随时删）。其中**设备协议实现**
  （`DmMotor`（MIT+0xFD 失能）、`LkMotor`、`DjiMotor`、`SupCap`、CH040/HI91、VT03/DR16 遥控，
  均已适配 libhcs）是当前唯一一份"libhcs 版驱动先例"——架构虽被否定，协议字节序/帧格式
  仍是有效参考。阶段 2 写驱动时会**以 Helios 原始驱动 + 手册为准重新实现**，不直接照搬。
- HCS 现有 `hcs_demo` 可复用的只有：组件写法范式（DemoBoard/PidController）、HI91 解析、
  `hcs_sync::Snapshot`/Doorbell 跨域范式。**没有可复用的真实电机/IMU/遥控驱动。**

---

## 4. 板卡与链路映射提案

Helios：云台板（遥控/裁判/云台/发射/视觉）+ 底盘板（腿/轮/IMU），IBC CAN 互通。
HCS：单进程组件图 + N 块 USB-CAN 板。暂沿用已删除包的 yaml 中记录的接线事实
（该接线是对着实车定的，电机 ID 与旧车一致），**上车前逐项核对**（§10-3）：

| 板（serial_filter 区分） | CAN1 | CAN2 | UART0 |
|---|---|---|---|
| 5321-A "gimbal" | LK yaw、LK 拨盘 | 3508 摩擦轮 ×2、DM pitch | 云台 CH100（921600） |
| 5321-B "chassis" | 左腿 DM bl1(0x05)/bl0(0x06) | 右腿 DM br1(0x07)/br0(0x08) | 底盘 CH100（921600） |
| 5321-C "aux" | 3508 轮毂 ×2（wl id2 / wr id1） | 超级电容 | 遥控接收机（VT03 直连；DR16 需反相器） |

- 裁判系统：5321 无空闲 UART → 主机 USB 转串口（或走 mc02 板，待定）。
- 每板 1kHz 指令帧预算：chassis 板 4×DM MIT（0x05-0x08，各 1 帧）≈ 8 帧/ms 含反馈，
  在 hcs_link.yaml 实测的 USB 链路预算内（HOST_TUNING 数据）；
  DM 零力矩发 0xFD 失能帧的语义照搬（RefereeTask L170-185）。
- 板数是**待决问题**（3 板沿用现状 vs 收成 2 板，§10-4）。

---

## 5. Helios 全部状态变量清单与归类

归类口径：**按键边沿 / 去抖计数 / 阶段自有数据 / 真正的阶段 / 操作手开关 / 连续状态量 /
控制器内部状态 / 估计器内部状态 / 死代码**。"去处"列给出在 RMCS/HCS 里的落点：
`估计器`、`阶段X`（std::variant 分支成员）、`操作开关`、`控制器`、`输入层`
（BalanceOperatorInput）、`硬件` 或 **删**。

### 5.1 `FSM()` 内 static（v2.cpp，全部 25 个；行号为声明处）

| # | 变量（行号） | 归类 | 去处 |
|---|---|---|---|
| 1 | `fly_init_swing_single`（1238） | Fly 阶段自有数据（起飞瞬间锁定的腿摆角） | `phase::Fly::locked_swing_angle`；Jump::TakeOff 出口也锁它（2093-2095）→ 同时存入 `phase::Jump` |
| 2 | `save_key_last`（1314） | 按键边沿 | `BalanceOperatorInput`（成员 last 值） |
| 3 | `is_healing_active`（1318） | 藏在枚举外的状态：Fallen 等待中 vs 已进入自救 | 由 `phase::Fallen`/`phase::SelfHeal` 本身表达，删除 |
| 4 | `fatal_error_timer`（1319） | Fallen 阶段自有数据（2s 计时） | `phase::Fallen::elapsed_s` |
| 5 | `jump_key_last`（1325） | 按键边沿 | `BalanceOperatorInput` |
| 6 | `fly_enter_cnt`（1390） | 去抖计数 | 估计器（离地确认计数） |
| 7 | `fly_time_cnt`（1391） | Fly 阶段计时 | `phase::Fly::elapsed_s` |
| 8 | `up_stair_cnt`（1392） | 去抖计数（UpStair 进入判定，+3/-1/清零的加权计数） | 估计器（折叠趋势计数），或 `Standing` 自带计数（阶段 1 定，倾向后者：它只在 NORMAL 下累） |
| 9 | `touch_down_stable_cnt`（1393） | 死代码（TOUCH_DOWN 无入口） | **删**（§9-1） |
| 10 | `fly_exit_short_leg_cnt`（1425） | 去抖计数（双腿 <0.25m 计 10 拍） | 估计器 |
| 11 | `fatal_err_cnt`（1504） | 去抖计数（衰减 -2/拍） | 安全检查（翻车累积判定） |
| 12 | `crash_cnt`（1578） | 去抖计数（Path C，>100） | 安全检查（摔跤判定 → 转回 SlowStart） |
| 13 | `slow_start_pose_target`（1706） | SlowStart 阶段自有数据（1.39 / 俯仰<0 时 1.23） | `phase::SlowStart` |
| 14 | `slow_start_phase`（1707） | 藏在枚举外的状态（0=收腿 1=转腿） | `phase::SlowStart::step`（Retract/Rotate） |
| 15 | `cnt`（1739，SlowStart 稳定计数） | SlowStart 阶段自有数据 | `phase::SlowStart::stable_s` |
| 16 | `last_ll_state`（1625，SPIN 分支） | 按键边沿衍生（腿长档位变化检测） | `BalanceOperatorInput` |
| 17 | `last_ll_state`（1790，NORMAL 分支） | 同上（与 16 同名不同实例） | 同上（合并成一个） |
| 18 | `heal_dir`（1826） | SelfHeal 阶段自有数据 | `phase::SelfHeal` |
| 19 | `phi0_total_last[2]`（1827） | SelfHeal 阶段自有数据（堵转检测） | `phase::SelfHeal` |
| 20 | `heal_stall_cnt`（1828） | SelfHeal 阶段自有数据（去抖 800） | `phase::SelfHeal` |
| 21 | `heal_flip_cooldown`（1829） | SelfHeal 阶段自有数据（1500 拍冷却） | `phase::SelfHeal` |
| 22 | `fly_touch_cnt`（2156，JUMP FLYING） | Jump 阶段自有数据（触地确认 10 拍） | `phase::Jump` |
| 23 | `land_short_cnt`（2182，JUMP LANDING） | Jump 阶段自有数据（50 拍） | `phase::Jump` |
| 24 | `phase_timeout_cnt`（2053，JUMP） | Jump 阶段自有数据（各相超时） | `phase::Jump::step_elapsed_s` |
| 25 | `cnt`（2284，IBC 发送节流 11 拍） | 通信实现细节 | **删**（IBC 消失） |

（任务书说 24 个，实测 25 个——两处同名 `last_ll_state` 与两处 `cnt` 计法不同所致。）

### 5.2 其他函数里的 static

| 变量（行号） | 归类 | 去处 |
|---|---|---|
| `yaw_rate_cmd_limited`（524，Controller_Update） | 控制器内部状态（yaw 斜坡限幅） | 控制器成员 |
| `smoothed_yaw_error`（681） | 控制器内部状态（卡死惩罚平滑） | 控制器成员 |
| `landing_recovery_factor`（721） | 控制器内部状态（着陆恢复 15%→100%） | 控制器成员 |
| `adapt_leg_angle_preset_latched` + `_valid`（765-766） | 控制器内部状态（Jump/Fly 窗口锁存） | 控制器成员 |
| `compliance_weight[2]`（440，Chassis_Controller） | 控制器内部状态（柔顺权重低通） | 控制器成员 |
| `takeoff_acc_err_last`（281，TAKE_OFF） | Jump::TakeOff 阶段自有数据 | `phase::Jump`（TakeOff 步数据） |
| `jump_phase_last`（264） | 相位变化检测（可由 phase 本身替换） | **删**（std::visit 天然知道阶段切换） |
| `gnd_state/impact_timer/P_last/dP_init/dP_buf/dP_idx/LL_want_last`（1050-1057，Gnd_Off_Detect） | 估计器内部状态（**当前整段被 `return 0;` 跳过**，v2:1047） | 估计器（保留算法、默认禁用，§9-3） |
| `slow_target`（1188，rotate_handle） | SlowStart 阶段自有数据 | `phase::SlowStart` |
| `leg_key_last`（2299，leg_state_key_check） | 按键边沿（**该函数仅 NO_HEAD 路径调用，死代码**） | 删；功能由 FSM 内联边沿检测（1378-1380）承担 |
| 文件级 static：`balance_nlmpc`、`balance_roll_mpc`（33-34）、`ramp_leg_rotate[2]`、`ramp_leg_len[2]`（42-43）、`filter_Fn_L/R`、`filter_dx`、`filter_roll1`、`filter_roll`、`filter_takeoff_acc_err_d`、`filter_ddphi0_L/R`（45-52） | 控制器/估计器内部状态（**违反多实例原则**） | 全部收进组件成员；`filter_dx` 未使用 → 删 |

### 5.3 `robot_state_s` 成员（v2.hpp:103-134）

| 成员 | 归类 | 去处 |
|---|---|---|
| `if_off_gnd[2]` | 估计器输出（0着地/1离地/2冲击） | 估计器输出（当前恒 0，§9-3） |
| `if_slip` | 死代码（Slip_Detect 无定义无调用） | **删** |
| `if_free` | 操作手开关（自由/小陀螺开关输入） | `BalanceOperatorInput`（ spin 请求） |
| `if_follow` | 派生开关（FSM L2212-2216：NORMAL 时 1） | 控制器由阶段推导（Standing 且未 spin） |
| `if_rotate_move[2]` | 未使用（grep 无写读点） | **删** |
| `mode[2]`（PRESENT/PAST） | 真正的阶段（双缓冲） | `Phase` variant 本身；PAST 语义 = `std::visit` 前保留一份旧 variant |
| `state_flag[2]` | 控制器选择（LQR_ON/DISABLED/PID_ONLY/HEALING） | `controller_for(phase)` 纯函数推导（骨架已给），不再独立存储 |
| `v[3]`、`acc[3]`、`v_level[3]`、`v_raw_target` | 连续状态量/操作输入（v_level 仅 NO_HEAD 用） | `v[1]/v[2]`：输入层 TD 输出（连续量，控制器成员）；`acc[1]`：TD 微分×0.1；`v_level` 删 |
| `follow_angle` | 连续状态量（yaw 跟随目标） | 控制器成员（FSM L2219-2235：NORMAL 时 = phi + real_angle + 掉头偏置；Disabled/SPIN 退出时锁定） |
| `Fn[2]` | 估计器输出镜像（无人读） | 删（估计器直接输出） |
| `dial_level` | 操作输入（spin 转速档 ±8/±14） | `BalanceOperatorInput` |
| `s_want` | 未使用（仅 `Ctrl::s_want{}` 重复声明且不读写） | **删** |
| `LL_FSM_Want[2]` | 阶段输出（目标腿长） | 各阶段自有数据（每阶段写自己的目标） |
| `LL_STATE` | 操作手开关（腿长三档 LOW/MID/HIGH） | `BalanceOperatorInput`（leg_key 边沿翻档，FSM L1378-1386） |
| `rotate_FSM_Want[2]` | 阶段输出（目标摆角） | 各阶段自有数据 |
| `rotate_speed`、`L0_speed` | 阶段参数表（v2:2237-2273 按 mode 选斜坡速率） | 变成 `phase` → 斜坡速率的查表函数（秒制） |
| `v_lift_FSM_Want` | 未使用 | **删** |
| `jump_cnt[2]` | 按键长按计时（仅 `jump_state_key_check` 用，NO_HEAD 死路径） | **删** |
| `up_stair_phase` | UpStair 阶段自有数据 | `phase::UpStair::step` |
| `is_inversed` | 连续开关（掉头方向，SlowStart 完成时按 follow_angle 判，L1771-1774） | `BalanceOperatorInput`/组件成员（影响速度符号与 follow_angle 头向偏置 L2231-2233） |

### 5.4 `fsm_state_s` 与 `Chassis_Balance_Status_Handle` 成员（v2.hpp:136-140、274-311）

| 成员 | 归类 | 去处 |
|---|---|---|
| `fsm_state.init_flag` | 藏在枚举外的状态（0=未站立完成） | 由阶段本身表达：`Standing` 存在即 1 |
| `fsm_state.fail_flag` | **未使用**（无写读点） | **删** |
| `fsm_state.fail_wait_cnt` | SlowStart 阶段自有数据（超时累计 500 拍） | `phase::SlowStart::elapsed_s` |
| `jump_leg_cnt[2]` | Jump 阶段自有数据（PRESS 80 / TAKE_OFF 12 衰减） | `phase::Jump` |
| `jump_phase_cnt` | Jump 的子阶段 | `phase::Jump::step` |
| `jump_cmd` | 每拍命令（键边沿 → 2） | `BalanceOperatorInput` 输出 |
| `jump_level_latched` | Jump 阶段自有数据（当前构建恒 2） | `phase::Jump::level` |
| `jump_duration` | **未使用**（仅清零） | **删** |
| `is_saved` | **未使用**（仅置 false） | **删** |
| `is_fatal_error` | 藏在枚举外的状态（Fallen 的标志） | 由 `phase::Fallen` 表达，删除 |
| `up_stair_phase`/`up_stair_timer` | UpStair 阶段自有数据 | `phase::UpStair` |
| `normal_init_lock_cnt` | Standing 阶段自带计时（起身 3s 内不判离地 + 门控写回） | `phase::Standing::elapsed_s`（任务书骨架已定）；注意常量是 **3000 拍 = 3.0s**（L1395，CLAUDE.md 写 500 已过期），且 SelfHeal 成功时也会重置（L1950） |
| `leg_change_lock_cnt` | 估计器门控（换腿长档后 0.5s 不判离地） | 估计器成员（秒制） |
| `ll = 0.20f` | **未使用** | **删** |
| `td_input[2]` | 输入处理（TD 跟踪微分器：速度指令平滑 + spin 转速平滑） | `BalanceOperatorInput`/控制器成员（纯数学，随 user_lib 移植） |
| `landig_angle` | **未使用** | **删** |

### 5.5 `Chassis_Balance` / `Chassis_Balance_Ctrl` 关键成员

- `leg_state[2]`（phi[4]、L0/dL0/phi0/dphi0、wheel_spd、Fn/Tp/P、spring_force、CoM、I_l、
  round_count/phi0_total 多圈量）→ **估计器输出**（连续状态量）。
- `wbc_state`（s/s1/phi/…/thetab/thetab1、roll/roll1、a_x/y/z、v_x/y/z、LL_want、rotate_angle）
  → s/s1/roll 等为估计+控制器混合：s（位移积分）、v_x/v_y/v_z（加速度积分）是**控制器内部
  状态**（积分器），thetall/r、thetab、phi 是**估计器输出**。
- `lqr_matrix`、`output[2]`、`output_last[2]`、`fn[2]`、`v_err`、`if_jump_controller_enable`、
  `pwr_ctrl`（恒 false）、LESO 全套缓冲（`d_hat_leso`、`x_hat_leso`、`last_u_control`、
  `leso_*_buf`、`pinv_*`）、RLS（`rls_L/R`、`params.k*`）、`park_brake_*`、`td_yaw`（无调用点）、
  `massive_yaw_error_flag`（无读写点）、`delta_u_w`（无读写点）、`stab_roll` →
  控制器内部状态；其中 **LESO 全套、RLS/QP、`pwr_ctrl`、`td_yaw`、`massive_yaw_error_flag`、
  `delta_u_w` 为死代码**（§6 注 4/5），移植时删（§9-4/5）。
- `leg_rotate_ramp/leg_len_ramp`（v2.hpp:173-174 成员版）**未使用**（FSM 用的是文件级
  `ramp_leg_rotate/ramp_leg_len`）→ 成员版删，文件级收进组件。

---

## 6. 阶段转移表（当前真实行为，时间已换算成秒 @1kHz）

`PAST` 保持语义（v2:1361-1374）：`init_flag==1` 且上一拍是 JUMP/FLY/UP_STAIR/TOUCH_DOWN 时，
顶层分发不覆盖 PRESENT（这些阶段"自带粘性"）。下表条件均已把拍数换成秒。

| 当前阶段 | 转移条件（秒制） | 下一阶段 | 行号（v2.cpp） |
|---|---|---|---|
| 任意 | `if_enable==0`（操作手关） | Disabled（清 fatal/heal/init） | 1333-1340 |
| 任意（非 SelfHeal/Disabled） | \|roll\|>1.2 或 \|thetab\|>1.2（灾难，瞬时） | Fallen（DISABLED+fatal） | 1494-1501 |
| 任意（检查模式，非 Disabled/SelfHeal） | 累积：\|roll\|>0.90 或 \|thetab\|>0.82 持续（计数 >50ms，衰减 2ms/拍） | Fallen | 1504-1526 |
| Fallen | 持续 ≥2.0s，或 save_key 边沿/电平 | SelfHeal | 1341-1351 |
| SelfHeal | 完成条件：双腿 phi0≈PI/4（±0.2）、L0<0.22 | （清标志）→ 下拍进 SlowStart | 1944-1952 |
| Disabled/任意 | `if_enable==1` 且未站立（init_flag=0） | SlowStart | 1357-1360 |
| SlowStart | 收腿到位（双腿 L0<0.24）后转腿，姿态稳定计 0.2s | Standing（带 3.0s 离地锁 + LL=LOW + is_inversed 判定） | 1704-1782 |
| SlowStart | 超时 0.5s 且 \|thetab\|<60° 且姿态稳定 | Standing（同上） | 1758-1781 |
| Standing(NORMAL) | dial/自由开关置位（spin_cmd） | SPIN（操作手开法，非独立阶段） | 1540-1541 |
| SPIN | spin_cmd 撤销 | Standing（锁 follow_angle、清 s/s1/KF） | 1609-1617 |
| Standing(NORMAL) | LL_STATE==HIGH 且双腿 phi0<1.23 持续约 27ms（加权 +3/拍） | UpStair（自动检测） | 1543-1572 |
| UpStair | 摆角转到目标（phi0<0.5 双腿）持续 0.1s | SlowStart（phase2 直转，L2035-2047） | 2001-2048 |
| Standing(NORMAL) | 跳跃键边沿 + 姿态稳（\|thetab\|,|roll|<0.20，\|s1\|<2.4） | Jump（level 恒 2） | 1325-1328, 1479-1487 |
| Jump::Press | 双腿 L0<0.20 计 80ms，或相超时 0.45s | Jump::TakeOff（锁 fly_init_swing_single） | 2069-2098 |
| Jump::TakeOff | 双腿离地且 L0>0.33 计 12ms（衰减制，最短蹬伸 18ms）——**当前离地检测禁用，实际只会走超时** 0.18s | Jump::Flying（重置腿长斜坡） | 2099-2135 |
| Jump::Flying | 双腿 L0<0.20 计 10ms，或相超时 1.2s | Jump::Landing | 2136-2169 |
| Jump::Landing | 双腿 L0<0.25 计 50ms，或相超时 0.7s | Standing（COMPLETE→NORMAL） | 2170-2204 |
| Standing(NORMAL) | 双腿"离地"计 3ms + 无跳令 + 无冲击锁（0.1s）+ 无 3s 锁 | Fly —— **当前不可达**（离地检测禁用，§9-3） | 1433-1461 |
| Fly | 双腿 L0<0.25 计 10ms（可靠触地）或超时 1.0s（最短 0.1s） | Standing（LL=LOW） | 1468-1477 |
| 任意（活动模式） | 摔跤：pitch>0.45（UpStair 中 0.65）且触地，或双腿 phi0_total 差>0.69，持续 0.1s | SlowStart（不算 fatal） | 1574-1607 |
| Disabled/其他 | 模式变化瞬间 state_flag 强制 DISABLED 一拍（输出清零过渡） | —— | 1658-1671 |

控制器选择映射（`controller_for` 的现值）：Disabled/Fallen→DISABLED；SlowStart→PID_ONLY；
Standing（含 SPIN）→LQR_ON；Jump::Press/TakeOff/Landing→LQR_ON、Jump::Flying→PID_ONLY；
Fly→LQR_ON；SelfHeal→PID_ONLY，双腿 L0>0.32 后→HEALING（1856-1859）。

---

## 7. 拍数常量换算表与每拍执行顺序映射

### 7.1 拍数常量（1kHz）→ 秒/秒制速率

| 常量（位置） | 原值 | 换算 |
|---|---|---|
| NORMAL_INIT_LOCK_TICKS（v2:1395） | 3000 拍 | 3.0 s（Standing 计时；SelfHeal 成功也重置，L1950） |
| leg_change_lock_cnt（v2:1385） | 500 拍 | 0.5 s |
| IMPACT_LOCK_TIME（v2:1399，阈值 \|a_x\|>6.3 m/s²） | 100 拍 | 0.10 s |
| FLY_ENTER_HYS（v2:1394） | 3 拍 | 3 ms |
| FLY_EXIT_LL_HYS（v2:1421，L0<0.25） | 10 拍 | 10 ms |
| FLY 最短滞空/超时（v2:1468-1469） | 100 / 1000 拍 | 0.1 / 1.0 s |
| fatal 计数（v2:1520-1517，衰减 2/拍） | >50 拍 | 50 ms |
| crash_cnt（v2:1602） | >100 拍 | 0.10 s |
| Fallen 自动自救（v2:1344） | ≥2000 拍 | 2.0 s |
| SlowStart 稳定/超时（v2:1757-1758） | >200 / >500 拍 | 0.2 / 0.5 s |
| up_stair_cnt（v2:1549-1567，+3/拍 上限 120） | >80 | 约 27ms（趋势判定，移植按"加权计数"原样保留并参数化） |
| up_stair_timer（v2:2029） | >100 拍 | 0.10 s |
| SelfHeal 堵转计数/冷却（v2:1903-1904） | 800 / 1500 拍 | 0.8 / 1.5 s |
| Jump：Press 确认/超时（v2:2088） | 80 / 450 拍 | 80 ms / 0.45 s |
| Jump：TakeOff 最短蹬伸/确认/超时（v2:2119-2121） | 18 / 12 / 180 拍 | 18 / 12 / 180 ms |
| Jump：Flying 触地确认/超时（v2:2162） | 10 / 1200 拍 | 10 ms / 1.2 s |
| Jump：Landing 确认/超时（v2:2188） | 50 / 700 拍 | 50 ms / 0.70 s |
| 腿长斜坡速率 L0_speed（v2:2241-2270） | 0.00015~0.003 m/拍 | 0.15 ~ 3.0 m/s（NORMAL 低档 0.2、高档 0.4、JUMP 3.0、FLY 1.0、SLOW_START 0.5、UP_STAIR/SELF_HEAL 0.4、TANK 0.15） |
| 摆角斜坡速率 rotate_speed（同上） | 0.001~0.007 rad/拍 | 1.0 ~ 7.0 rad/s |
| 位移积分/速度积分 dt（v2:108、135-137、625） | ×0.001 | 用 `tick.dt_seconds()` |
| 各一阶滤波器（v2:45-52、444、564） | 1kHz 假设的 (a,b) 系数 | 系数原样保留为参数，注明"假定 1kHz"；若 update_rate 变更需重算 |
| TD 参数 r=4310,h=0.001,h0=0.003；r_input=25,h0_input=0.01（balance_def.hpp:239-244） | 含 h=1ms | h 改为 `tick.dt_seconds()`，r/h0 原样参数化 |
| IBC 发送节流（v2:2285） | 每 11 拍 | 消失（同进程） |
| NO_HEAD 路径全部常量 | — | 死代码，不移植 |

### 7.2 每拍执行顺序映射

Helios（两板并行，各 1kHz FreeRTOS）：

| Helios（底盘板 ChassisTask，balance_tasks.cpp:254-278） | HCS（单进程组件图，拓扑排序后每拍） |
|---|---|
| `RefereeTask`（并行）：RC 更新、DM MIT/0xFD、DJI 全发 | 硬件组件 Status 半（无输入，先跑）：读板卡 Snapshot → 写反馈输出 |
| `Chassis_Data_Update()`（IMU→欧拉角/加速度，电机→腿运动学、接触力、多圈、KF 速度） | **估计器层**（底盘组件内）：同左，全部去抖计数在这层 |
| `FSM()`（模式分发、跳跃/飞行序列、fatal、自救、斜坡目标） | **安全检查 → 阶段状态机（variant）**层 |
| `chassis_ctrl_update()`（IBC RX：云台命令） | 组件输入绑定替代：`/remote/*`（遥控解码）与云台数据直接是组件输入，无打包/量化 |
| `Chassis_Controller_Update()`（LQR A/K 插值、状态向量、NLMPC、限幅） | **控制器层** |
| `Chassis_Controller()`（F_want 分发、roll MPC/PID、VMC） | 同上 |
| `Motor_Send()`（VMC→关节力矩、轮转矩→电流环） | 控制器层尾 |
| （RefereeTask 下拍）CAN TX | 硬件组件 Command 伙伴（依赖全部控制器输出，最后跑）把指令写进 Snapshot → **拍尾 Doorbell** 唤醒 TX 线程 `start_transmit()` |
| 云台板 `FSM_handle()`（fsm.cpp：遥控→speed/keys 开关） | `BalanceOperatorInput`（底盘组件内部小类，决策 A）：`/remote/*` + 键边沿/档位 |

IBC 数据 → 组件输出对照（`Chassis_Ctrl_Data_s`，robot_def.hpp:123-147）：

| IBC 字段（原通道） | HCS 来源 |
|---|---|
| `speed[0..2]`（x/y/w 速度档，Pack1 ×200 量化） | 遥控/操作输入组件输出 `/remote/*`（无量化损失——这是行为差异，§9-8） |
| `if_enable`、`if_free`、`jump_key`、`save_key`、`leg_key`、`if_turn` | 同上（fsm.cpp 的"开关"语义：save/leg/jump 是**电平翻转**，由 `BalanceOperatorInput` 做边沿检测，fsm.cpp:224-230） |
| `real_angle`、`follow_angle`、`yaw_speed`（云台 yaw 电机角/速度） | 云台组件输出（**HCS 暂无云台组件**，§10-1：本期接口留桩，输入可缺席——`register_input(required=false)` 或可选输入默认值） |
| 底盘→云台 Pack4（L0、phi0、phi、thetab、mode、is_fatal_error） | 底盘组件输出 `/chassis/telemetry/*` 或聚合 struct（云台/UI 以后直接读） |

---

## 8. 组件划分与接口

### 8.1 组件清单

新包 `hcs_balance`（沿用包名；旧包已删）。阶段 2 落地，源文件显式列入 CMakeLists
（HCS 不用 GLOB），插件注册进 `hcs_balance/plugins.xml`。

**组件 1：`hcs_balance::hardware::BalanceInfantryHardware`**（+ `…HardwareCommand` 伙伴）
- 职责：持有 3 块 libhcs 板（§4），事件域回调解码 DM/LK/DJI/CH100/遥控/超电，`Snapshot` 跨域；
  周期域 `update()` 把反馈发到输出；Command 伙伴从输入读指令、拍尾 Doorbell 线程发送。
  DM 零力矩 → 0xFD 失能帧语义保留。
- 输出（反馈，trivially copyable struct/float）：
  `/balance/joint/{bl1,bl0,br1,br0}/{angle,velocity,torque}`、
  `/balance/wheel/{left,right}/{angle,velocity,current}`、
  `/balance/imu/{pitch,roll,yaw,pitch_rate,roll_rate,yaw_rate,acc_x,acc_y,acc_z}`、
  `/remote/{rx,ry,lx,ly,dial,sw_left,sw_right,enabled}`（解码后的原始遥控）。
- 输入（指令）：`/balance/joint/{bl1,bl0,br1,br0}/torque_cmd`（float，N·m）、
  `/balance/wheel/{left,right}/current_cmd`（float，LSB 或 A——阶段 2 定）。

**组件 2：`hcs_balance::BalanceChassisController`**（底盘+平衡核心，内部分层照任务书）：
1. `BalanceEstimator`（腿运动学 `module::leg_pos`、接触力 `inverse_contact_force`、弹簧力、
   多圈、速度 KF `spd_estimation`、离地/触地判定（含滞回去抖，默认禁用=现状，§9-3）、
   翻车/摔跤判定、冲击锁）
2. `BalanceSafetyChecker`（Cheetah 式：无力条件（if_enable==0、伙伴组 safe_mode）、
   翻车瞬时/累积、摔跤计数；直接可把输出压到无力）
3. 阶段状态机：`std::variant`（骨架照任务书；`step()` 唯一转移函数，全局规则置顶）
4. `BalanceController`（LQR 插值表 `K_out/A_out/B_out` + `lqr_K`、NLMPC `BalanceNLMPC`、
   roll MPC/PID、腿长/摆角 PID、VMC、摩擦补偿、park-brake/启动助力、力矩→电流）
5. `BalanceOperatorInput`（决策 A：`/remote/*` → 速度指令 TD、档位、键边沿、spin 开关、
   掉头方向；独立成类便于以后上移）
- 输入：上述硬件全部输出（`required=false` 允许台架缺板缺席）+ `/predefined/safe_mode`（可选）。
- 输出：指令（上面硬件的输入）+ 对外发布：
  `/chassis/phase`（`PhaseInfo` struct：`uint8 phase; uint8 jump_step; bool fatal_error; bool spinning; …`）、
  `/chassis/leg_length/{left,right}`、`/chassis/velocity`、`/chassis/follow_angle` 等
  （云台/发射/UI 以后只读这些，不再自行判断——任务书第 5 层）。

**不再建的组件**：全局 FSM（决策 A）；IBC；NO_HEAD 路径；TANK/TOUCH_DOWN。

### 8.2 阶段 variant（按任务书骨架 + 本次盘点补全）

```cpp
namespace phase {
struct Disabled {};
struct SlowStart { enum class Step : uint8_t { Retract, Rotate } step = Step::Retract;
                   double stable_s = 0.0; double elapsed_s = 0.0; double pose_target = 0.0; };
struct Standing  { double elapsed_s = 0.0; };            // 替代 normal_init_lock_cnt(3.0s) 与 init_flag
struct UpStair   { enum class Step : uint8_t { Rotate, Done } step = Step::Rotate;
                   double stable_s = 0.0; };             // 自动检测进入；Done→SlowStart
struct Jump      { enum class Step : uint8_t { Press, TakeOff, Flying, Landing } step = Step::Press;
                   double step_elapsed_s = 0.0; double confirm_s = 0.0; int level = 2;
                   double locked_swing_angle = 0.0; double takeoff_acc_err_last = 0.0; };
struct Fly       { double elapsed_s = 0.0; double locked_swing_angle = 0.0; };
struct Fallen    { double elapsed_s = 0.0; };            // 替代 is_fatal_error + fatal_error_timer
struct SelfHeal  { double elapsed_s = 0.0; int dir = 1; double stall_s = 0.0; double cooldown_s = 0.0;
                   double phi0_total_last[2] = {0.0, 0.0}; };
}
using Phase = std::variant<phase::Disabled, phase::SlowStart, phase::Standing, phase::UpStair,
                           phase::Jump, phase::Fly, phase::Fallen, phase::SelfHeal>;
```

（任务书骨架将 SPIN 列为开法而非阶段，与决策 B 一致；UpStair 按 §6 确认为"自动检测进入"
的阶段，保留为阶段；TOUCH_DOWN 按死状态处理，见 §9-1。）

### 8.3 无环论证

`HardwareStatus`（无输入）→ `BalanceChassisController`（只吃 Status 输出与 `/remote/*`）→
`HardwareCommand`（只吃控制器输出）。控制器若需自己的上拍输出（如 s 积分、NLMPC 内部状态）
一律为组件成员变量（Helios 的 `output_last`、`dphi0_last` 就是这个写法），不进接口图、
不构成接口环；`DelayedInput` 本移植预计 0 处使用。`/chassis/phase` 只出不动 → 无环。
（RMCS 时代"电机反馈→PID→电机指令"的环由 Status/Command 拆分解开，HCS 沿用同一结构。）

---

## 9. 与 Helios 当前真实行为的差异清单（必须逐条决断，不顺手改）

1. **TOUCH_DOWN（死状态）**：无任何赋值点（grep 证实）；但 5 处代码在处理它：
   控制器 switch 分支（342-345）、PAST 保持（1366）、退出块（1967-1999）、init_flag 豁免
   （1697）、L0_speed 表项（2263-2266）。
   **建议：全部删除。** 理由：语义已被"Fly 退出→Standing"与"Jump::Landing"覆盖；
   若将来要独立落地缓冲阶段，应基于实测数据重新设计入口条件，而不是搬一个进不去的状态。
2. **DISABLED 双义拆分**：按任务书拆 `Disabled`（操作手关闭，v2:1333-1340）与
   `Fallen`（fatal，v2:1341-1356、1494-1531）。行为等价性由对拍验证（Fallen 期间输出无力、
   2s 后自救）。
3. **离地检测被禁用**（v2:1047 `return 0;`）：影响面——FLY 不可达（1433-1461 永假）、
   Jump::TakeOff→Flying 只剩 0.18s 超时路径（2108-2125）、轮力矩离地清零（947-955）与
   LESO 离地衰减（2439-2446，反正 LESO 已死）永不触发、柔顺权重恒 1.0（440-475）、
   摔跤判定 is_touching_ground 恒真（1582）。
   **建议**：估计器实现完整三态检测（含 CLAUDE.md 与代码不一致处以代码为准：
   P_AIR=80N、P_RECOVER=60N、dP 冲击 2000 N/s、冲击 10ms），加 yaml 开关
   `gnd_detection_enabled`，**默认 false（=当前行为）**；对拍按 false 跑，另跑一组
   强制注入离地序列的用例覆盖 FLY/Jump 转移逻辑（并注明这是注入）。上车前是否启用 → 你定。
4. **LESO 全死**：`leso()` 无调用点；且 LQR_ON 每拍 `reset_leso()`（925）。**建议：不移植**
   （`reset_kf` 语义保留——它属于速度 KF，是活的）。
5. **功率 QP/RLS 死**：`pwr_ctrl` 恒 false（927），`Power_RLS_Update` 仅在死分支内调用。
   **建议：不移植 QP/RLS**，保留 `T_MAX_WHEEL/T_MAX_LEG` 限幅路径。超级电容设备驱动照写
   （硬件层），控制策略暂不接。
6. **跳跃恒大跳**：`jump_cmd = edge ? 2 : 0`（1328），`jump_level_latched` 恒 2
   （az_ref=20.0、TAKE_OFF_LL_HIGH+0.02）。小跳参数（az_ref=6.2、LOW）保留在配置里，
   对拍场景"小跳"单独标注为不可达分支验证。
7. **`pitch_preset_used = 0` 强制清零**（826，注释"验证 NLMPC 期间临时禁用"）与
   `adapt_leg_angle_preset = 0`（757）：按现状移植（清零），参数保留，注释原样带走。
8. **IBC 量化损失消失**：speed 过 Pack1 `floatToSignedFixed16(…,5)+×200`（chassis_ctrl.cpp:228）
   有量化误差；HCS 直连后更精确。对拍时**输入侧统一取"解码后"的值**，两边一致即可对齐。
9. **NO_HEAD / TANK / `jump_state_key_check` / `leg_state_key_check`**：死代码，不移植
   （jump/leg 边沿在真实路径里各有内联实现：1325-1328、1378-1380）。
10. **原值修正**（CLAUDE.md 与代码不一致，以代码为准，报告备案）：az_ref 大跳 20.0 非 14.2；
    NORMAL_INIT_LOCK 3000 拍非 500；离地滞回 80/60 非 ClinB 60/130；COMPLIANCE_AIR=0.3、
    IMPACT=0.5、WEIGHT_FILTER=0.1；飞坡 LL=0.24 非 0.37。
11. **50+ 处函数内 static 全部消除**：计数器归估计器/安全层，阶段数据归 variant 分支，
    开关归 `BalanceOperatorInput`，控制器内部状态归控制器成员（§5 逐条落位）。
12. **安全性语义变化（增强，需知悉）**：HCS 下任一组件抛异常 → 伙伴组隔离 + 输出复位默认值
    （无力）+ safe_mode；原 STM32 上组件崩了就是整机崩。此外"主机崩溃/USB 断开"的保护
    仍是未决问题（§10-2）。

---

## 10. 风险与待决问题清单（需要你拍板/核实）

1. **云台/发射范围**：原任务书"复用 RMCS 现有组件（SimpleGimbalController 等）"在 HCS 不成立
   ——`hcs_demo` 只有 PidController/DemoBoard/探针/裁判系统骨架。选项：
   (a) 本期只做底盘+硬件层，云台设备不发指令（全部 0xFD 失能），接口留桩；
   (b) 本期把云台/发射控制也一并新写。
   **建议 (a)**，范围可控且不动底盘行为对拍。
2. **延迟与失控保护（最高优先，上车前必须有答案）**：原闭环在 STM32 本地（µs 级）；现在
   传感器→板→USB（实测链路：URB 往返 p50≈113µs，主机拍 1kHz）→ 上位机 → 回板 → 电机。
   主机崩溃、USB 掉线（kFaulted）、进程被杀时谁停车？需核实：
   (a) libhcs 固件是否有"会话超时自动失能/发 0xFD"（hcs_link.yaml 提到 keepalive 会话保活，
   暗示有超时机制，**需读 libhcs 固件源码确认**）；
   (b) DM 电机本身有无指令超时失能（查 DM 手册）；DJI C620 无超时会保持最后电流 →
   板卡侧兜底必须存在。
   在确认前**不上电**。
3. **板卡接线/ID 清单**：§4 表来自被删包的 yaml 注释，非 Helios 源码直接可证（Helios 是
   双 STM32 板载外设）。上车前按实车逐项核对 CAN 分配与 LK 电机 CAN ID（LK_motor(4/5) →
   0x144/0x145 的映射需对 lk_motor.hpp 核实）。
4. **板数**：3 板（现状）vs 2 板（aux 合并）。3 板 USB 带宽/端口占用 vs 2 板 CAN 总线负载
   （chassis 板 CAN2 若并进轮+超电会超预算）。**建议先 3 板**，阶段 2 实测后可再收敛。
5. **NMPC 1kHz 实时性**：STM32 上限 440000 周期（550MHz 下 ≈0.8ms）且带 fallback；
   x86 上预计算力充裕但需实测（阶段 3 用 `HCS_EXECUTOR_COMPONENT_TIMING` + RtReporter）。
   fallback 路径（`calculate_status_vector_fly`）必须保留。
6. **裁判系统接入**：HCS 无裁判系统组件；底盘行为对拍不依赖裁判系统（Helios 底盘 FSM 只吃
   IBC 命令，功率限制路径是死代码）。本期建议：不做裁判系统，`P_MAX_CHASSIS` 等参数不接。
7. **速度指令语义**：`v_want = clip(speed[0]*2.0, 4.2)`（1291）里 speed[0] 的原始档位值由
   云台板 fsm.cpp 产生（RC 摇杆 → ±1/±2 档，fsm.cpp:145-150 + ×200 量化）。HCS 的
   `BalanceOperatorInput` 需要复刻"档位→目标速度"这一段（阶段 1 读 fsm.cpp 全文后定），
   或直接定义秒制 m/s 档位（行为差异，需你确认接受哪种）。
8. **需要改 hcs_executor / 现有组件吗**：目前盘点结论是**不需要**（DelayedInput、伙伴组、
   Doorbell、可选输入已覆盖全部需求）。若阶段 1/2 发现缺口，单独报告。
9. **`update_rate` 假设**：所有滤波器/斜坡参数原按 1kHz 折算；HCS yaml 固定 1000.0，
   不构成短期问题，但参数命名将写明"@1kHz"。

---

## 11. 阶段 1 计划（对拍测试，先立此存照）

- **纯逻辑层**：估计器/安全检查/阶段机/控制律写成不依赖 rclcpp 的普通 C++（HCS 组件基类
  本身已不依赖 rclcpp，天然满足）；`module::leg_pos/lqr_K/leg_VMC/inverse_contact_force/
  spd_estimation/TD/RampFunction/PID/FirstOrderFilter` 从 Helios `module/` 直接拷贝编译
  （纯 float 数学，无 HAL 依赖；arm_math 只出现在死的 LESO/QP 里，不拷）。
- **参照实现**：把 Helios `Chassis_Data_Update + FSM + Chassis_Controller(_Update) +
  Motor_Send` 判定逻辑原样抽成 harness（static 在单实例 harness 中语义等价，允许保留），
  与新实现喂同一组脚本化输入（每拍：enable/speed/keys/IMU/电机反馈），逐拍比较：
  阶段序列、jump step、LL_FSM_Want、rotate_FSM_Want、state_flag、final_Tl0/Tl1/Tw。
- **场景**（§6 表逐行覆盖）：上电起身 / 起身超时 / 起身中途无力 / 正常行驶（含驻车刹车与
  启动助力）/ 飞坡（注入离地序列，标注）/ 大跳全程 / 小跳（不可达分支，单独标注）/
  跳跃中途翻车 / 翻车 2s 自动自救 / 按键自救 / 自救中途无力 / 上台阶 / SPIN 进出 / 摔跤→SlowStart。
- **预期内差异**（§9）逐条列出；**预期外差异停下来报告**，不顺手修。

---

## 12. 本次已做的仓库动作

- 新建分支 `port/helios-balance`（不 push）。
- 删除旧 `hcs_balance` 单包重构（从未入库；临时备份 `/tmp/hcs_balance_deleted_20260918.tgz`，
  不需要可删）。
- 本文件 `hcs_balance/PORTING.md` 为阶段 0 唯一代码产出；包骨架（CMakeLists/plugins.xml/源码）
  等阶段 1/2 落地。

**等待确认后才进入阶段 1。**
