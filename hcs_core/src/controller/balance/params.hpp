#pragma once

// 平衡底盘全部参数。数值逐项取自 Helios User_Code/user/balance/balance_def.hpp 与
// chassis_balance_v2.cpp 内联常量；拍数一律按 1 kHz 换算成秒（见 PORTING.md §7）。
//
// 结构体只做"常量搬家"，不引入新数值。带 "@1kHz" 注释的参数是拍数常量的秒制换算，
// update_rate 不是 1000 时需要重新整定，而不是改这里。

#include <array>
#include <cstdint>

namespace hcs_core::controller::balance {

struct PidConfig {
    float max_output{};
    float integral_limit{};
    float kp{};
    float ki{};
    float kd{};
    float integral_max{};
    float integral_min{};
    float output_lpf{};
    float d_lpf{};
    float gama{};
    uint8_t improve{};
    uint8_t pid_mode{}; // 0 = POSITION_PID
    float max_err{};
    float deadband{};
};

struct Stribeck {
    float sigma; // 粘滞系数
    float fc;    // 库仑摩擦
    float fs;    // 静摩擦
    float vs;    // Stribeck 速度
};

struct Params {
    // ── 物理（balance_def.hpp） ────────────────────────────────────────────
    float G = 9.8f;
    float L1 = 0.21f;
    float L2 = 0.25f;
    float L5 = 0.0f;
    float wheel_radius = 0.06f;
    float M = 18.0f;
    float I_b = 0.353848333f;
    float I_z = 0.2657f;
    float I_w = 4.98e-4f;
    float M_w = 0.25867f;
    float M_l = 2.6f;
    float L_b = 0.13f;
    float pitch_preset = 0.11f;
    float wheel_reduction = 13.94f;
    float leg_angle_offset = 1.163f;
    float k_t_total = 0.02f * 13.94f * 1.2f;

    // ── 气弹簧解算 ─────────────────────────────────────────────────────────
    float k_spring = 1875.0f;
    float f_prime = 250.0f;
    float spring_length = 0.24f;
    float spring_s1 = 0.165f;

    // ── 减速箱摩擦补偿（斯特里克模型） ─────────────────────────────────────
    std::array<Stribeck, 2> friction_data = {{
        {0.5756f, 207.4161f, 184.1962f, 0.0124f},
        {0.6527f, 210.0311f, 170.1155f, 0.0115f},
    }};

    // ── 限幅与功率 ─────────────────────────────────────────────────────────
    float t_max_wheel = 5.0f;    // N·m
    float t_max_leg = 40.0f;     // N·m
    float motor_send_leg_clip = 40.0f;
    float motor_send_wheel_clip = 6.0f;
    float wheel_current_clip = 16000.0f;
    float roll_inertia = 0.35f;
    float roll_force_arm = 0.20f;
    float roll_df_max = 80.0f;
    float p_max_chassis = 120.0f;
    float p_static_const = 10.0f;

    // ── 跳跃 ───────────────────────────────────────────────────────────────
    float k_v_x = 0.03f;
    float prime_ll = 0.18f;
    float take_off_ll_high = 0.35f;
    float take_off_ll_low = 0.28f;
    float landing_ll = 0.23f;

    // ── 输入 TD（跟踪微分器） ──────────────────────────────────────────────
    float td_r_input = 25.0f;
    float td_h_input = 0.001f;      // "@1kHz：h 折进增益，改 update_rate 需重整定"
    float td_h0_input = 0.01f;      // 10 * h_input
    float td_r_spin_scale = 0.9f;

    // ── PID（balance_def.hpp 原值） ────────────────────────────────────────
    PidConfig leg_l_pid = {
        .max_output = 600,
        .integral_limit = 5,
        .kp = 2800,
        .ki = 0,
        .kd = 60,
        .integral_max = 2,
        .integral_min = 0,
        .output_lpf = 0.001f,
        .d_lpf = 0.001f,
        .gama = 0,
        .improve = 0x3f,
        .pid_mode = 0,
    };
    PidConfig leg_l_pid_soft = {
        .max_output = 200,
        .integral_limit = 5,
        .kp = 800,
        .ki = 0,
        .kd = 250,
        .integral_max = 2,
        .integral_min = 0,
        .output_lpf = 0.001f,
        .d_lpf = 0.001f,
        .gama = 0,
        .improve = 0x3f,
        .pid_mode = 0,
    };
    PidConfig turn_pid = {
        .max_output = 4,
        .integral_limit = 5,
        .kp = 7,
        .ki = 0,
        .kd = 2,
        .integral_max = 2,
        .integral_min = 0,
        .output_lpf = 0.001f,
        .d_lpf = 0.001f,
        .gama = 0,
        .improve = 0x3f,
        .pid_mode = 0,
    };
    PidConfig leg_roll_pid = {
        .max_output = 30000,
        .integral_limit = 8000,
        .kp = 3500,
        .ki = 9000,
        .kd = 0.1f,
        .integral_max = 20000,
        .integral_min = 0,
        .output_lpf = 0.001f,
        .d_lpf = 0.001f,
        .gama = 0,
        .improve = 0x3f,
        .pid_mode = 0,
    };
    PidConfig leg_rotate_pid = {
        .max_output = 30,
        .integral_limit = 20,
        .kp = 250,
        .ki = 0,
        .kd = 7,
        .integral_max = 2,
        .integral_min = 0,
        .output_lpf = 0.001f,
        .d_lpf = 0.001f,
        .gama = 0,
        .improve = 0x3f,
        .pid_mode = 0,
    };
    PidConfig wheel_current_pid = {
        .max_output = 16384,
        .integral_limit = 8000,
        .kp = 1.3f,
        .ki = 5.0f,
        .kd = 0,
        .integral_max = 5,
        .integral_min = -5,
        .output_lpf = 0.001f,
        .d_lpf = 0.001f,
        .gama = 0,
        .improve = 0x00,
        .pid_mode = 0,
    };

    // ── 一阶滤波器（(旧输出权重, 新输入权重)，假定 1 kHz） ─────────────────
    struct FilterCoeffs {
        float past;
        float now;
    };
    FilterCoeffs filter_fn = {0.69f, 0.31f};
    FilterCoeffs filter_roll = {0.5672f, 0.4328f};
    FilterCoeffs filter_roll1 = {0.5672f, 0.4328f};
    FilterCoeffs filter_takeoff_acc_err_d = {0.4700f, 0.5300f}; // fc = 120 Hz @1kHz
    FilterCoeffs filter_ddphi0 = {0.15f, 0.85f};                // fc = 30 Hz @1kHz

    // ── 离地/触地检测（v2 Gnd_Off_Detect；当前 Helios 以 return 0 禁用，
    //    gnd_detection_enabled 默认 false 复刻该行为，true 启用算法本体） ──
    bool gnd_detection_enabled = false;
    float gnd_p_air_thres = 80.0f;        // N
    float gnd_p_ground_recover = 60.0f;   // N
    float gnd_dp_impact_thres = 2000.0f;  // N/s
    float gnd_impact_duration_s = 0.010f; // 10 拍
    float gnd_l_extended_thres = 0.20f;   // m
    float m_l_g = 2.6f * 9.8f;            // Gnd_Off_Detect 的 M_l*G 偏置

    // ── 阶段计时（v2 拍数常量的秒制换算，见 PORTING.md §7） ────────────────
    float standing_lock_s = 3.0f;          // NORMAL_INIT_LOCK_TICKS 3000
    float leg_change_lock_s = 0.5f;        // 500
    float impact_lock_s = 0.10f;           // 100，阈值 |a_x|>6.3
    float impact_ax_thres = 6.3f;          // m/s^2
    float fly_enter_hysteresis_s = 0.003f; // 3
    float fly_exit_short_s = 0.010f;       // 10，L0 < 0.25
    float fly_exit_ll_thres = 0.25f;
    float fly_min_s = 0.10f;               // 100
    float fly_timeout_s = 1.0f;            // 1000
    float fatal_accum_s = 0.050f;          // 50，衰减 0.002/拍
    float fatal_decay_per_tick_s = 0.002f;
    float fatal_roll_threshold = 0.90f;
    float fatal_pitch_threshold = 0.82f;
    float disaster_roll_threshold = 1.2f;
    float disaster_pitch_threshold = 1.2f;
    float fallen_wait_s = 2.0f;            // 2000
    float slow_start_stable_s = 0.2f;      // 200
    float slow_start_timeout_s = 0.5f;     // 500
    float slow_start_ll_want = 0.18f;
    float slow_start_retract_thres = 0.24f;
    float slow_start_pose_window_up = 0.2f;
    float slow_start_pose_window_down = 0.3f;
    float up_stair_enter_count = 80.0f;    // 加权计数：折叠时 +3/拍，上限 120
    float up_stair_enter_bonus = 3.0f;
    float up_stair_enter_decay = 1.0f;
    float up_stair_enter_cap = 120.0f;
    float up_stair_phi_fold = 1.23f;
    float up_stair_phi_fold_release = 1.32f;
    float up_stair_stable_s = 0.10f;       // 100
    float crash_s = 0.10f;                 // 100
    float crash_pitch_threshold = 0.45f;
    float crash_pitch_threshold_up_stair = 0.65f;
    float fatal_split_threshold = 0.69f;
    float heal_ll_want = 0.34f;
    float heal_extend_thres = 0.32f;
    float heal_stall_s = 0.8f;             // 800
    float heal_stall_motion_thres = 0.003f;
    float heal_flip_cooldown_s = 1.5f;     // 1500
    float heal_flip_allow_thetab = 1.05f;
    float heal_ll_short = 0.18f;
    float heal_phi_target = 0.25f * 3.14159265f; // PI/4
    float heal_phi_window = 0.2f;
    float heal_ll_done = 0.22f;
    float fly_ll_want = 0.24f;
    float spin_ll_want[3] = {0.16f, 0.22f, 0.32f};
    float stand_ll_want[3] = {0.17f, 0.25f, 0.36f};
    float jump_press_ll_want = 0.20f;
    float jump_press_confirm_s = 0.08f;    // 80
    float jump_press_timeout_s = 0.45f;    // 450
    float jump_takeoff_ll_margin = 0.02f;
    float jump_takeoff_min_push_s = 0.018f; // 18
    float jump_takeoff_confirm_s = 0.012f;  // 12，衰减 0.002/拍
    float jump_takeoff_decay_per_tick_s = 0.002f;
    float jump_takeoff_timeout_s = 0.18f;   // 180
    float jump_takeoff_air_ll_thres = 0.33f;
    float jump_flying_ll_want = 0.23f;
    float jump_flying_touch_ll = 0.20f;
    float jump_flying_confirm_s = 0.010f;   // 10
    float jump_flying_timeout_s = 1.2f;     // 1200
    float jump_landing_ll_want = 0.17f;
    float jump_landing_confirm_s = 0.050f;  // 50
    float jump_landing_ll_thres = 0.25f;
    float jump_landing_timeout_s = 0.7f;    // 700

    // ── 斜坡速率（m/s、rad/s；v2 L0_speed/rotate_speed 拍值 ×1000） ─────────
    float l0_speed_normal = 0.2f;
    float l0_speed_slow_start = 0.5f;
    float l0_speed_jump = 3.0f;
    float l0_speed_self_heal = 0.4f;
    float l0_speed_fly = 1.0f;
    float l0_speed_up_stair = 0.4f;
    float l0_speed_tank = 0.15f;
    float rotate_speed_slow_start = 7.0f;
    float rotate_speed_tank = 1.0f;
    float rotate_speed_jump = 3.0f;
    float rotate_speed_self_heal = 3.0f;
    float rotate_speed_fly = 6.0f;
    float rotate_speed_touch_down = 6.0f;
    float rotate_speed_up_stair = 5.5f;
    float rotate_speed_disabled = 1.0f;

    // ── 输入命令（v2:1291-1312 + fsm.cpp setSpeed） ─────────────────────────
    float speed_command_clip = 4.2f;
    float speed_command_gain = 2.0f;
    float stick_low_ratio = 100.0f / 660.0f;   // setSpeed 阈值 100（满幅 660）
    float stick_high_ratio = 400.0f / 660.0f;  // setSpeed 阈值 400
    float dial_level_low = 8.0f;               // set_speed ±1 → ±8
    float dial_level_high = 14.0f;             // set_speed ±2 → ±14

    // ── 控制器杂项 ─────────────────────────────────────────────────────────
    float v_err_clip = 2.5f;
    float v_err_deadband = 0.005f;
    float park_cmd_thresh = 0.05f;
    float park_spd_thresh = 0.20f;
    float park_pos_gain = 5.0f;
    float park_max_pos_err = 0.30f;
    float park_err_deadband = 0.005f;
    float launch_ratio = 0.50f;
    float launch_min_raw = 0.10f;
    float launch_pos_gain = 0.35f;
    float launch_max_offset = 0.50f;
    float pos_filter_alpha = 0.314f; // 60 Hz 一阶低通 @1kHz
    float yaw_error_out_limit = 0.85f;
    float yaw_err_slope = 0.6f;
    float yaw_max_step = 0.5f;
    float wheel_speed_takeoff_ll_threshold = 0.3f; // LL_want>0.3 分支
    float pitch_clip = 0.52f;
    float jump_press_thetab_ref = 0.1f;
    float az_ref_big = 20.0f;   // v2:284（CLAUDE.md 的 14.2 已过期）
    float az_ref_small = 6.2f;
    float takeoff_kp_acc = 5.60f;
    float takeoff_kd_acc = 0.4f;
    float takeoff_pitch_comp = 0.85f;
    float flying_kp_pitch = 0.35f;
    float flying_kd_pitch = 0.10f;
    float flying_pitch_clip = 0.30f;
    float compliance_air = 0.3f;
    float compliance_impact = 0.50f;
    float compliance_gnd = 1.0f;
    float compliance_filter = 0.1f; // WEIGHT_FILTER @1kHz
    float torque_to_current_threshold = 0.02f;
    float torque_to_current_k = 45.4049f;
    float torque_to_current_coulomb_k = 0.3703f;
    float torque_to_current_coulomb_v = 0.08f;
    float wheel_current_lsb_per_amp = 819.2f;

    // ── NLMPC（balance_nlmpc.cpp，内部 DT 固定 1 ms） ───────────────────────
    uint32_t nlmpc_cycle_limit = 440000; // STM32 550 MHz 基准；x86 上仅作记录
};

} // namespace hcs_core::controller::balance
