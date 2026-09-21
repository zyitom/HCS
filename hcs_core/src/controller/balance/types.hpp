#pragma once

// 平衡底盘逻辑层的输入输出类型。单位语义与 Helios 逐项对应：
//   - IMU：ch100 方向映射后的 euler（rad，±π 缠绕）/ 角速度（rad/s）/ 加速度（g），
//     total_yaw 为连续 yaw；
//   - 腿电机角：解码角 − org_pos（Helios offset_angle 语义）；
//   - 轮电机：speed 为转子 rad/s，current 为母线电流原始 LSB；
//   - 操作命令：fsm.cpp 翻译后的档位与电平。

#include <array>
#include <cstdint>

namespace hcs_core::controller::balance {

struct ImuSample {
    float roll{}, pitch{}, yaw{};
    float droll{}, dpitch{}, dyaw{};
    float total_yaw{};
    float acc_x{}, acc_y{}, acc_z{}; // g
    uint32_t ts{};
};

struct LegMotorSample {
    float angle{};  // rad
    float speed{};  // rad/s
    float torque{}; // N·m
};

struct WheelSample {
    float speed{};  // 转子 rad/s
    float current{}; // 原始 LSB
};

/// BalanceOperatorInput 的每拍输出（替代 Helios 的 IBC Chassis_Ctrl_Data_s 读侧
/// 与 FSM() 里的按键边沿检测）。
struct OperatorCommand {
    float speed_level[3]{};   // fsm.cpp setSpeed 的档位输出（x, y, w）
    bool if_enable{};
    bool if_free{};
    bool if_turn{};
    bool jump_rising_edge{};  // jump_key 上升沿 → jump_cmd=2
    bool save_changed{};      // save_key 电平变化（任意方向）
    bool save_level{};
    bool leg_changed{};       // leg_key 电平变化（任意方向）
    float follow_angle_cmd{}; // data_to_chassis.follow_angle
    float real_angle{};       // 云台 yaw 电机多圈角（IBC real_angle）
};

struct LogicInput {
    ImuSample imu{};
    /// [bl1, bl0, br1, br0]，balance_tasks.cpp 的 motor_leg[] 顺序。
    std::array<LegMotorSample, 4> leg_motor{};
    std::array<WheelSample, 2> wheel{}; // [left, right]
    OperatorCommand command{};
};

struct LegState { // v2 leg_state_s
    std::array<float, 4> phi{};
    std::array<float, 4> dphi{};
    float phi0{}, dphi0{}, dphi0_last{}, ddphi0{};
    float L0{}, dL0{};
    float round_count{}; // 多圈计数（估计器维护，自救/摆角目标要用）
    float phi0_total{};
    float wheel_spd{};
    float Fn{}, Tp{}, P{}, spring_force{};
    float CoM_x{}, CoM_y{}, I_l{};
};

struct WbcState { // v2 wbc_state_s；LL_want/rotate_angle 由阶段机写入
    float s{}, s1{}, phi{}, phi1{}, thetall{}, thetall1{}, thetalr{}, thetalr1{}, thetab{},
        thetab1{};
    float roll{}, roll1{};
    float a_x{}, a_y{}, a_z{};
    float LL_want[2]{};
    float rotate_angle[2]{};
    float v_x{}, v_y{}, v_z{};
};

/// 0 着地 / 1 离地 / 2 冲击。Helios 当前禁用检测时恒为 0。
struct GroundState {
    std::array<int, 2> if_off_gnd{};
};

struct Estimate {
    std::array<LegState, 2> leg{};
    WbcState wbc{};
    GroundState ground{};
    /// 轮电流反馈（原始 LSB，电流环 PID 的测量值；Helios Moto_Measure.real_current）。
    std::array<float, 2> input_wheel_current{};
};

/// 阶段 → 控制器结构（任务书骨架的 controller_for，唯一映射、穷举编译期检查）。
enum class ProcessFlag : uint8_t { LqrOn, Disabled, PidOnly, Healing };

/// 阶段机一拍对控制器的暴露（替代 robot_state 的读侧）。
struct MachineView {
    ProcessFlag process_flag{};
    ProcessFlag process_flag_past{};
    bool phase_changed{};
    bool if_follow{};
    bool spinning{};
    float v_target{};      // robot_state.v[1]（TD 后）
    float v_raw_target{};  // 未平滑原始指令
    float acc_fwd{};       // robot_state.acc[1]
    float yaw_rate_cmd{};  // robot_state.v[2]
    float dial_level{};
    float follow_angle{};
    bool is_inversed{};
};

/// 控制器一拍输出（Motor_Send 的语义）。
struct MotorCommands {
    float leg_torque[4]{};   // [bl1, bl0, br1, br0]，N·m，零 → 硬件发 0xFD 失能帧
    float wheel_current[2]{}; // [left, right]，原始 LSB
};

} // namespace hcs_core::controller::balance
