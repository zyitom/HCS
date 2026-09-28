# 平衡步兵上层重写任务书（交接给执行 agent）

> 写于 2026-09-28，分支 `port/helios-balance`。读者：接手实现的 agent。
> 本文件是**唯一的任务说明**。`hcs_balance/PORTING.md` 里的方案（单独建包、单体控制器、逐拍对拍）已作废，
> 只能当 Helios 行为事实的参考来读（见 §2）。

## 0. 一句话目标

按桌面 **RMCS**（`~/Desktop/RMCS/rmcs_ws/src/rmcs_core`）的架构和写法，在 **`hcs_core` 包内**重写平衡步兵的上层：
一个整车硬件组件 + 一组 RMCS 风格的平衡底盘组件 + RMCS 通用云台控制链。
**Helios**（`~/Desktop/Helios_cpp_powerful_framework`）只回答"车要做什么、参数是多少"，
**不提供代码结构**——它的写法（2500 行单文件、50+ 个函数内 static、按拍计数的常量）正是这次要摆脱的。

## 1. 用户已经拍板的决定（不要再讨论，也不要绕开）

1. **不单独建包**。平衡底盘不是一个"包"，是 `hcs_core` 里 `controller/chassis/balance/` 下的几个组件，
   和 RMCS 里 `rmcs_core/src/controller/chassis/deformable_*.cpp` 同一个地位。
2. **架构、命名、写法全部学 RMCS**。不要求与 Helios v2 逐拍对齐，**功能等价**即可；
   可读、规范优先于"原样复刻"。有意的行为差异要列进最终汇报（§8）。
3. **云台用 RMCS 通用控制器**（`SimpleGimbalController` + `ErrorPidController` + `PidController` 串级），
   yaml 里用 Helios 的参数起步。
4. **发射、裁判系统（含超电、UI、底盘功率限制）这次不做。** 做到那一步时**停下来问用户**（§6 阶段 4）。
5. `hcs_core` 里这几个目录是用户故意留着的 rmcs 遗留，**一个字都不许改**：
   `controller/shooting/`、`referee/`、`broadcaster/`，以及 `hardware/device/fake_motor.hpp`、
   `hardware/deprecated_reference_rmcs_real_car.cpp`。
6. **`hardware/device/` 下的驱动是 HCS 已有的驱动，直接用**（DM/LK/DJI/Hipnuc/VT13/CanPacket 等）。
   它们只是 include 和命名空间还写成 rmcs 的，所以现在编不进来。**只允许做这一处机械替换**，驱动逻辑不改：
   - `rmcs_executor/component.hpp` → `hcs_executor/component.hpp`，`rmcs_executor::` → `hcs_executor::`
   - `rmcs_utility/*` → `hcs_utility/*`（`crc/dji_crc`、`double_buffer`、`endian_promise`、`package_receive`、`ring_buffer` 在 HCS 里都有同名头）
   - `rmcs_msgs/{keyboard,mouse,switch}.hpp` → `hcs_msgs/...`
   - `namespace rmcs_core::hardware::device` → `namespace hcs_core::hardware::device`
   - 用不到的（`bmi088*.hpp`、`board_clock_lifter.hpp`）不碰，也不进构建。
7. **IMU 只用 CH040**（HiPNUC HI91，UART 921600），底盘、云台各一颗。BMI088 不用，不要提。
8. 不做：微帧锁相、IMU 数据年龄补偿、板端时间戳、两相拍。这些都评估过了，这次不要碰。

## 2. 必读材料（按这个顺序读）

**RMCS（架构样板）**，路径前缀 `~/Desktop/RMCS/rmcs_ws/src/`：
- `rmcs_core/src/hardware/omni_infantry.cpp`：整车硬件组件怎么写。**这是你写 `balance_infantry.cpp` 的模板**。
- `rmcs_core/src/hardware/device/remote_control.hpp`：遥控仲裁，输出 `/remote/*`。
- `rmcs_core/src/controller/chassis/chassis_controller.cpp`：操作意图组件（遥控/键盘 → 模式 + 速度指令）。
- `rmcs_core/src/controller/chassis/omni_wheel_controller.cpp`、`deformable_chassis.cpp`、
  `deformable_suspension.cpp`、`deformable_joint_controller.cpp`、`deformable_mode.hpp`：
  **复杂底盘怎么拆成多个组件**。平衡底盘照这个拆。
- `rmcs_core/src/controller/gimbal/simple_gimbal_controller.cpp`、`two_axis_gimbal_solver.hpp`。
- `rmcs_core/src/controller/pid/{pid_controller,error_pid_controller}.cpp`、`pid_calculator.hpp`、`smart_input.hpp`。
- `rmcs_bringup/config/omni-infantry.yaml`、`deformable-infantry-omni-b.yaml`：组件怎么在 yaml 里接线。
- `rmcs_description/include/rmcs_description/tf_description.hpp`：云台解算用的 tf 描述。

**HCS 框架（规则来源）**，路径前缀 `~/Desktop/HCS/`：
- `hcs_executor/include/hcs_executor/component.hpp`：API 与 rmcs_executor 的差别都在这里（见 §4.3）。
- `hcs_executor/src/executor.hpp`：一拍怎么跑、拍尾门铃、组件隔离。
- `hcs_sync/include/hcs_sync/{snapshot,event_queue,byte_tunnel,tick}.hpp`、
  `hcs_utility/include/hcs_utility/{doorbell,doorbell_worker}.hpp`。
- `hcs_core/src/hardware/hcs_link_probe.cpp`：**真实 libhcs 板卡 + 门铃发送线程的现成范例**。
- `hcs_core/src/controller/pid/pid_controller.cpp`：已经移植到 HCS API 的 RMCS PidController。
- `hcs_core/src/libhcs/host/include/libhcs/board/hpm5321.hpp`：板卡 API（`can_receive`、`uart0_receive_callback`、`start_transmit`、`link_state`）。
- `hcs_bringup/config/hcs_link.yaml`：TL101 上的线程布局与实测依据，照抄。

**Helios（行为与参数来源）**，路径前缀 `~/Desktop/Helios_cpp_powerful_framework/User_Code/`：
- `user/balance/balance_tasks.cpp`：设备清单、CAN ID、零点、方向。
- `app/chassis_app/balance/CLAUDE.md`：**先读这个**，是 `chassis_balance_v2.cpp` 的导读（模式表、阈值、跳跃五阶段、摔倒三条判据）。
- `app/chassis_app/balance/{chassis_balance_v2,balance_nlmpc}.{hpp,cpp}`、`user/balance/balance_def.{hpp,cpp}`（PID、物理参数、LQR 拟合表）、
  `module/algorithm/balance_alg/balance_algorithm.{hpp,cpp}`（腿运动学、VMC、接触力、LQR 查表）。
- `module/software/fsm/fsm.cpp`：键位映射（C 小陀螺、X 掉头、B 自救、G 腿长档……）。
- `app/gimbal_app/gimbal.cpp`：云台 PID 参数与限位。
- 部分文件是 GBK 编码，读之前先转码，按行混合编码时用 Python 逐行 `utf-8` → `gbk` 回退解码。

**`hcs_balance/PORTING.md`** 只读这几节：§5（Helios 状态变量）、§6（阶段转移表）、§7（按拍常量换算）、
§9（**Helios 里的死代码**：LESO 从未调用、功率 QP 恒不激活、TOUCH_DOWN/TANK 无入口、离地检测被 `return 0` 禁用、跳跃恒为大跳）。
死代码一律不搬。

## 3. 删除清单（阶段 0 第一件事）

| 删除 | 理由 |
|---|---|
| `hcs_core/src/controller/balance/`（整个目录，含 `port/`、`helios_ref/`） | 上一版单体实现；用户判定不可读、不规范。其中的对拍工具在 HEAD 已编译失败（`phase_machine.cpp:684` 引用了未声明的 `debug_tick_`） |
| `hcs_core/src/hardware/balance.cpp` | 空文件 |
| `hcs_bringup/config/balance_infantry.yaml` | 引用的 8 个 `hcs_balance::*` 组件都不存在；按 §5 重写为 `balance-infantry.yaml`（RMCS 命名用连字符） |
| `build/hcs_balance`、`install/hcs_balance`、`build/rmcs_*`、`install/rmcs_*` | 旧构建产物，会在 overlay 里残留旧插件 |
| `hcs_balance/`（含 PORTING.md） | **阶段 2 结束后**再删，在那之前它是 Helios 行为事实的参考 |

**不要删**：`hcs_executor/`、`hcs_sync/`、`hcs_utility/`、`hcs_msgs/`、`fast_tf/`、`hcs_core/src/libhcs/`（子模块）、
两个探针（`can_rtt_probe.cpp`、`hcs_link_probe.cpp`）及其 yaml、`demo_board.cpp` + `demo.yaml`、`controller/pid/`，以及 §1-5 列出的遗留。

## 4. 目标架构

### 4.1 目录（对照 rmcs_core）

```
hcs_core/src/
  hardware/
    balance_infantry.cpp            # 新：整车唯一硬件组件 + Command 伙伴（模板 = RMCS omni_infantry.cpp）
    device/                         # 已有驱动，按 §1-6 只改 include/命名空间
    device/remote_control.hpp       # 新：从 RMCS 移植的遥控仲裁（HCS 里还没有）
    util/board_transmitter.hpp      # 新：HCS 专有的"一拍的帧 → 发送线程"工具，见 §4.4
  controller/
    chassis/balance/                # 新：平衡底盘组件，见 §4.2
    gimbal/simple_gimbal_controller.cpp, two_axis_gimbal_solver.hpp   # 新：从 RMCS 移植
    pid/error_pid_controller.cpp, smart_input.hpp                     # 新：从 RMCS 移植
hcs_description/                    # 新包：从 rmcs_description 移植（tf 描述，依赖已有的 fast_tf）
hcs_msgs/include/hcs_msgs/          # 按需补枚举（例如平衡模式），照 rmcs_msgs 的写法
hcs_bringup/config/balance-infantry.yaml
```

新代码统一放在 `namespace hcs_core::...`。`hcs_core` 的 CMake 包名目前还叫 `hcs_demo`，**这次不改**，改名要先问用户。
每个新 `.cpp` 都要在 `hcs_core/CMakeLists.txt` 的 `PROJECT_SOURCE` 里显式加一行（HCS 不用 GLOB），
每个新组件都要在 `hcs_core/plugins.xml` 里注册。

### 4.2 平衡底盘：组件划分与接口

原则与 RMCS 一致：**一个组件做一件事，组件之间只靠具名接口通信，可复用的部分做成可以在 yaml 里多次实例化的组件**。
下表是推荐拆法。组件名、接口名可以微调，但**分层不许合并回一个大组件**。

| 组件（`controller/chassis/balance/`） | 对应 Helios | 输入 | 输出 |
|---|---|---|---|
| `BalanceChassisController`：操作意图 | `fsm.cpp` 里底盘相关键位 + 遥控拨杆 | `/remote/*`、`/gimbal/yaw/angle`（跟随） | `/chassis/control_mode`、`/chassis/control_velocity`、`/chassis/balance/leg_length_level`、`/chassis/balance/jump_count`、`/chassis/balance/recover_count` |
| `BalanceStateEstimator`：状态估计 | `Chassis_Data_Update`、`Gnd_Off_Detect`、`SpeedEstimation`、`Multi_Turn_Detection` | 4 个腿关节、2 个轮、`/chassis/imu/*` | 每条腿 `/chassis/{left,right}_leg/{length,length_velocity,angle,angle_velocity,support_force,grounded}`；机体 `/chassis/balance/{pitch,pitch_rate,roll,roll_rate,yaw,yaw_rate,velocity,distance}` |
| `BalanceModeManager`：模式状态机 | `Chassis_Balance_Status_Handle::FSM`、`rotate_handle`、斜坡 | 意图 + 估计 | `/chassis/balance/mode`、`/chassis/{left,right}_leg/control_length`（已斜坡）、`.../control_angle` |
| `BalanceLqrController`：LQR / NLMPC | `Chassis_Controller_Update` + LQR 部分 | 估计 + 模式 + 速度指令 | `/chassis/{left,right}_wheel/control_torque`、`/chassis/{left,right}_leg/control_hip_torque` |
| `LegForceController`：腿长 / 横滚 / 起跳力 | `Chassis_Controller` 里的 `F_want`（腿长 PID、roll PID、重力前馈、起跳加速度 PD、柔顺权重） | 估计 + 模式 + 腿长目标 | `/chassis/{left,right}_leg/control_force` |
| `LegJointController`：VMC 映射，**yaml 里实例化两次**（左、右） | `leg_VMC` + `Motor_Send` 的关节部分 | 本腿 `control_force`、`control_hip_torque`、两个关节角 | 本腿两个关节的 `/control_torque` |

要点：
- **边沿请求用计数器**（`jump_count`、`recover_count` 每次按键加一，下游比较变化），照 RMCS `/chassis/deformable/reset_count` 的做法，不传"单拍脉冲"。
- **NaN 表示"不控制"**（RMCS 约定）。模式为失能或摔倒时，控制器输出 NaN；由硬件组件把 NaN 翻译成失能帧（§4.4）。
  这替代了 Helios 里"`send_data == 0` 就发 0xFD"的写法。
- 数学放进同目录的纯 C++ 头文件（例如 `leg_kinematics.hpp`、`lqr_gain_table.hpp`、`nlmpc_solver.hpp`），不依赖 rclcpp，
  照 RMCS 的 `two_axis_gimbal_solver.hpp`、`qcp_solver.hpp`。LQR 拟合表来自 `balance_def.hpp`，在表头注释里写明出处行号。
- 模式状态机的写法参考 RMCS `deformable_mode.hpp`（`enum class` + `switch`）。模式集合按 Helios 的**活**模式来：
  失能、起身（SLOW_START）、平衡（NORMAL/SPIN）、跳跃（压腿/起跳/腾空/落地）、飞坡、摔倒、自救、上台阶。
  §2 里列出的死状态不搬。离地检测（Helios 里被禁用）写成 yaml 开关，默认关。
- **Helios 按拍的常量一律换成秒制**（例：`L0_speed = 0.0002 m/tick` → `0.2 m/s`，计数 2000 拍 → 2.0 s），dt 只用 `tick.dt_seconds()`。
- 物理参数、PID、限幅、阈值全部进 yaml（`balance-infantry.yaml` 按组件名分段）。
- 如果某个组件确实需要读下游的上一拍输出，用 `DelayedInput`（显式 1/z），不要为了断环把组件合并。

### 4.3 RMCS 代码搬到 HCS 要改的地方

| RMCS | HCS |
|---|---|
| `void update() override` | `void update(const hcs_sync::Tick& tick) HCS_NONBLOCKING override` |
| `/predefined/update_rate`、计数 × dt | `tick.dt_seconds()`；和时间有关的逻辑用 `tick.scheduled` 推 |
| `EventOutputInterface` / `EventInputInterface` | **HCS 已删除**。事件域 → 周期域用 `hcs_sync::Snapshot`，周期域 → 事件域用 `hcs_sync::EventQueue` |
| `command_update()` 里直接 `board_->start_transmit()` | **禁止**：libhcs 发送路径里有锁。见 §4.4 |
| `update()` 里 `RCLCPP_WARN` / 分配 / 加锁 | 禁止。clang-20 的 `-Wfunction-effects` 会报；要打日志就用 Snapshot 把数据交给尽力域的定时器 |
| `ready()` 判断可选输入 | 用 `has_provider()`：没人提供的可选输入会被绑定到默认值，`ready()` 恒为真 |
| 函数内 `static`、全局变量 | 禁止，一律改成成员 |

### 4.4 硬件组件 `BalanceInfantry`（HCS 专有的部分）

外形照抄 RMCS `OmniInfantry`：一个 `Component + rclcpp::Node`，构造时建板、建设备、注册输出，
用 `create_partner_component` 建 Command 伙伴，在 `can_receive` 里按 CAN ID 分发到 `device.store_status()`，
`update()` 里调 `update_status()`。**和 RMCS 不同的只有发送和安全这两块**：

1. **发送**：Command 伙伴的 `update()` 调各设备的 `generate_*()`，把这一拍所有板的帧写进一个定长批次
   （例如 `std::array<Frame, N>` + 计数，平凡可拷贝），用 `hcs_sync::Snapshot` 发布；
   `tick_end_doorbell()` 返回**一个** `hcs_utility::DoorbellWorker` 的门铃。
   这个线程醒来后读 Snapshot，只在 `fresh` 时对三块板依次 `start_transmit()`。
   **三块板共用一个发送线程**：实测不同板的发送线程在同一个核上本来就是串行的，第二块板要多等约 5 µs。
   这部分写成 `hardware/util/board_transmitter.hpp`，范例是 `hcs_link_probe.cpp`。
2. **NaN → 失能帧**：DM 关节（腿、pitch）在 `control_torque` 为 NaN 时发 `DmMotor::generate_disable_command()`，
   否则发 `generate_command()`（驱动自己负责使能和清错）。LK 驱动遇到 NaN 本来就会发失能帧；DJI 驱动遇到 NaN 发 0 电流。
3. **只发新批次**：发送线程只在 Snapshot 读到 `fresh` 时发送，**不重发最后一帧**。
   组件被隔离后，拍尾仍然会振铃（`hcs_executor/src/executor.hpp:218`），但批次不再更新，发送线程自然保持静默。
   不做额外的"安全批次"逻辑。
4. **板卡**：三块 `libhcs::board::Hpm5321`（gimbal / chassis / aux），每块都有 `enabled`、`serial_filter`、
   `io_thread_cpu`、`io_thread_rt_priority` 参数。`enabled: false` 时上面的设备保持离线，这样台架上缺板也能跑。
   `link_state()` 为 `kFaulted` 时的处理由 `exit_on_board_fault` 参数决定。
   构造时调一次 `hcs_utility::MachineGuard::run()` 并打印结果（照 `hcs_link_probe.cpp:256`）。
5. **总线负载**：构造时按每路总线每拍的帧数估算负载（经典 CAN 1 Mbit/s，8 字节帧约 110–130 µs），打印出来，超过 70% 时打 WARN。
6. **tf**：云台 CH040 的四元数写进 `/tf`（照 RMCS 用 BMI088 的写法，换成 Hipnuc 的 `/quaternion`），供 `TwoAxisGimbalSolver` 使用。

**接线（来自已删除包的 yaml，上车前必须由用户核对）**：

| 板 | CAN1 | CAN2 | UART0 |
|---|---|---|---|
| gimbal | LK yaw（Helios id 5）、LK 拨盘（id 4） | 3508 摩擦轮 id1/id2、DM pitch（Helios `DM_MOTOR(0x03, 0x04, ...)`） | 云台 CH040 |
| chassis | 左腿 DM（0x05/0x01、0x06/0x02） | 右腿 DM（0x07/0x03、0x08/0x04） | 底盘 CH040 |
| aux | 3508 轮毂（左 id2、右 id1） | 超级电容（本次不做） | 遥控接收机 |

- 板卡固件必须是经典 CAN（DJI/DM/LK 都是 CAN 2.0）。构造时用 `can1_is_fd()` / `can2_is_fd()` 校验，不一致就抛。
- 腿关节命名用 `/chassis/{left,right}_{front,back}_joint`。Helios 的 `motor_leg[0..3]` = bl1、bl0、br1、br0，
  哪个是 front、哪个是 back 由 `balance_algorithm.cpp` 的 `leg_pos()` 里 phi1/phi4 的几何来定。
  把映射表写在 `balance_infantry.cpp` 头注释里。
- 零点、方向、DM 的 PMAX/VMAX/TMAX 从 `balance_tasks.cpp` 和 Helios 的 `dm_motor.cpp` 读，进 yaml。
  DM 驱动注释里说明了：映射范围与电机里存的值不一致时，所有数都会被**静默缩放**（`device/dm_motor.hpp:46`）。
- 发射相关电机（摩擦轮、拨盘）这次也要在硬件组件里建出来（总线上本来就有），但不接控制器，保持失能或 0 电流。

### 4.5 云台（RMCS 通用链）

yaml 接线照 RMCS `omni-infantry.yaml`：
`SimpleGimbalController`（`/remote/*` → `/gimbal/{yaw,pitch}/control_angle_error`）
→ `ErrorPidController`（角度环 → `control_velocity`）
→ `PidController`（速度环，测量值用 `/gimbal/{yaw,pitch}/velocity_imu` → `control_torque`）。
yaw 是 LK MG5010（力矩命令），pitch 是 DM（MIT 纯力矩）。限位用 Helios 的 `PITCH_UP = 0.45`、`PITCH_DN = -0.32`，
方向取 Helios 的 `dir_yaw = -1`、`dir_pitch = -1`，PID 从 Helios `gimbal.cpp` 起步。
Helios 的 FFC/DOB/yaw 堵转保护这次不做，在汇报里列为"以后按需加组件"。
自瞄输入（`/auto_aim/*`）保持可选且不接线：视觉桥这次不做。

## 5. 编码规范

- **RMCS 的写法**：
  - 组件继承 `Component + rclcpp::Node`（需要时加 `NodeMixin`）；参数、`register_*` 都在构造函数里完成，`update()` 保持短小；
  - 物理量用 `double` 和国际单位；NaN 表示不控制；
  - 成员名带尾下划线，常量用 `kCamelCase` 的 `constexpr`；
  - 单文件以 RMCS 为尺度（最大的约 700 行），超过就拆。
- **HCS 的硬规矩**（违反即 bug）：
  - `update()` 里不分配、不加锁、不打日志、不做系统调用；
  - 跨线程只走 `Snapshot`、`EventQueue`、`ByteTunnel`、`Doorbell`；
  - 输出类型尽量平凡可拷贝（隔离复位只对平凡可拷贝的输出生效）；
  - 不用 `/predefined/*`。
- 注释可以写中文，标识符和字符串保持 ASCII。
- 不写兼容旧结构的适配层，不留"以后可能用到"的抽象。

## 6. 分阶段计划与验收

每个阶段结束都要编译通过，再进入下一阶段。**没有硬件：不许连板、不许上电。**所有实机测试由用户来做。

**阶段 0：清理与地基**
- 构建前先 `source /opt/ros/jazzy/setup.bash`，在 `~/Desktop/HCS` 下用 `colcon build`。
- 执行 §3 删除清单（`hcs_balance/` 除外）。
- 按 §1-6 让 `device/` 下要用的驱动编译通过。
- 移植 `remote_control.hpp`、`error_pid_controller.cpp`、`smart_input.hpp`，新建 `hcs_description` 包。
- 验收：`colcon build` 在 gcc 下 0 警告；另用 clang-20 单独构建一次，新代码在 `-Wfunction-effects` 下 0 警告：
  `colcon build --build-base build-clang --install-base install-clang --cmake-args -DCMAKE_CXX_COMPILER=clang++-20`。

**阶段 1：硬件组件**
- 实现 `balance_infantry.cpp` 和 `util/board_transmitter.hpp`。
- 新建 `balance-infantry.yaml`，线程布局抄 `hcs_link.yaml`。
- 验收：三块板全部 `enabled: false` 时，`ros2 launch hcs_bringup hcs.launch.py robot:=balance-infantry` 能跑 60 秒，
  RtReporter 显示 skipped = 0，启动日志里有组件树和总线负载估算。
  发送工具（批次打包、安全批次逻辑）有 gtest 覆盖。

**阶段 2：平衡底盘**
- 实现 §4.2 的六个组件和它们用到的数学头文件。
- 验收：
  - 数学函数有 gtest，与 Helios `balance_algorithm.cpp` 的同名函数在同一组输入下对比（容差 1e-5）。
    覆盖 `leg_pos`、VMC 雅可比、接触力反解、LQR 增益查表。这是**函数级**对比，不是逐拍行为对拍。
  - 有一个图构建测试：用假硬件组件和全部平衡组件跑一次 `Linker::link`，确认无环、必需输入都有提供者。
  - 阶段 1 的空载启动验收仍然通过。
- 阶段结束后删除 `hcs_balance/`。

**阶段 3：云台**
- 按 §4.5 接线。验收：图构建通过，空载启动通过。

**阶段 4：停下来问用户**
- 发射、裁判系统、超级电容、UI、视觉桥都涉及 §1-5 里不许动的遗留，**开始任何一项之前先问用户怎么处理**。

## 7. 边界

- 不改框架：`hcs_executor/`、`hcs_sync/`、`hcs_utility/`、libhcs 子模块。确实需要改时，先停下来报告原因。
- 不 commit、不 push，改动留在工作区给用户审。
- 遇到必须由用户回答的问题（接线、电机参数、遥控型号）就记下来继续做别的，不要猜着写死。

## 8. 最终汇报要包含

1. 改动文件清单：新增 / 删除 / 修改，每条附一句理由。
2. Helios → HCS 映射表：Helios 的函数或段落对应到哪个组件的哪个方法。
3. **有意的行为差异清单**：死代码没搬、单位换算、NaN 失能语义、没做的 FFC/DOB/堵转保护……
4. 各阶段验收的实际输出（构建警告数、gtest 结果、空载运行的 RtReporter 摘要）。
5. 待用户确认的问题：
   - 遥控接收机型号（HCS 驱动是 VT13；旧 yaml 写的是 VT03，DR16 需要外接反相器）；
   - 腿关节 front/back 的映射；
   - 接线表；
   - DM 电机里实际存的 PMAX/VMAX/TMAX。
