#include "chassis_balance_v2.hpp"
#include "robot_def.hpp"
#include "module_def.hpp"
#include "IBC.hpp"
#include "fsm.hpp"
#include "message_center.hpp"
#include "balance_algorithm.hpp"
#include "balance_def.hpp"
#include "ffc.hpp"
#include "rc.hpp"

#include "user_lib.hpp"
#include "bmi088.hpp"
#include "balance_nlmpc.hpp"
#include "bsp_dwt.h"

extern "C"
{
#include "arm_math.h"
#include "kalman_filter.h"
}
// #define NO_HEAD = 1
// #define INF_3
#ifndef INF_3
#define INF_4
#endif

extern module::PID_Init_Config_s leg_L_pid, leg_L_pid_soft, leg_roll_pid, leg_rotate_pid, turn_pid, wheel_current_pid_l, wheel_current_pid_r;

namespace app
{

    static BalanceNLMPC balance_nlmpc;
    static BalanceRollMPC balance_roll_mpc;
    static constexpr uint32_t NLMPC_CYCLE_LIMIT = 440000U;

    const BalanceNLMPC::Diagnostics &GetBalanceNLMPCDiagnostics()
    {
        return balance_nlmpc.diagnostics();
    }

    module::RampFunction ramp_leg_rotate[2];
    module::RampFunction ramp_leg_len[2];

    module::FirstOrderFilter filter_Fn_L(0.69, 0.31);
    module::FirstOrderFilter filter_Fn_R(0.69, 0.31);
    module::FirstOrderFilter filter_dx(0.001, 0.2); // 简单滤波
    module::FirstOrderFilter filter_roll1(0.5672f, 0.4328f);
    module::FirstOrderFilter filter_roll(0.5672f, 0.4328f);
    module::FirstOrderFilter filter_takeoff_acc_err_d(0.4700f, 0.5300f); // dt=1ms, fc=120Hz
    module::FirstOrderFilter filter_ddphi0_L(0.15f, 0.85f); // fc = 30Hz
    module::FirstOrderFilter filter_ddphi0_R(0.15f, 0.85f);

    Chassis_Balance::Chassis_Balance(module::moto_data_receive_s *motor_bl0,
                                     module::moto_data_receive_s *motor_bl1,
                                     module::moto_data_receive_s *motor_br0,
                                     module::moto_data_receive_s *motor_br1,
                                     module::moto_data_receive_s *motor_wl,
                                     module::moto_data_receive_s *motor_wr,
                                     module::imu_data_t *imu_data_chassis)
    {
        motordata_bl0 = motor_bl0;
        motordata_bl1 = motor_bl1;
        motordata_br0 = motor_br0;
        motordata_br1 = motor_br1;
        motordata_wl = motor_wl;
        motordata_wr = motor_wr;
        this->imu_data = imu_data_chassis;
    }

    Chassis_Balance_Ctrl::Chassis_Balance_Ctrl(Chassis_Balance &balance_, Chassis_Balance_Status_Handle &status_handle)
        : balance_(balance_), status_handle_(status_handle)
    {
        const leg_state_s *leg_state = balance_.leg_state;
        const wbc_state_s &wbc_state = balance_.wbc_state;
        pid_LL = new module::PID(leg_L_pid, const_cast<float *>(&leg_state[LEFT].L0));
        pid_LR = new module::PID(leg_L_pid, const_cast<float *>(&leg_state[RIGHT].L0));
        pid_LL_soft = new module::PID(leg_L_pid_soft, const_cast<float *>(&leg_state[LEFT].L0));
        pid_LR_soft = new module::PID(leg_L_pid_soft, const_cast<float *>(&leg_state[RIGHT].L0));
        pid_roll = new module::PID(leg_roll_pid, const_cast<float *>(&wbc_state.roll));// unused
        pid_leg_rotateL = new module::PID(leg_rotate_pid, const_cast<float *>(&leg_state[LEFT].phi0_total));
        pid_leg_rotateR = new module::PID(leg_rotate_pid, const_cast<float *>(&leg_state[RIGHT].phi0_total));
        pid_rotate = new module::PID(turn_pid, const_cast<float *>(&wbc_state.phi));
        pid_wheel_current_l = new module::PID(wheel_current_pid_l, &balance_.motordata_wl->real_current);
        pid_wheel_current_r = new module::PID(wheel_current_pid_r, &balance_.motordata_wr->real_current);
        rls_L = new RLS<2>(1000.0f, 0.995f, 0.1f);
        rls_R = new RLS<2>(1000.0f, 0.995f, 0.1f);
    }

    Chassis_Balance_Status_Handle::Chassis_Balance_Status_Handle(Chassis_Balance &balance, Chassis_Ctrl *chassis_ctrl)
        : balance_(balance), chassis_ctrl_(chassis_ctrl)
    {
        fsm_state.init_flag = 0;
        robot_state.mode[PRESENT] = app::mode_e::DISABLED;
        robot_state.state_flag[PRESENT] = app::process_e::DISABLED;
        jump_cmd = 0;
        jump_level_latched = 1;
        jump_phase_cnt = app::jump_phase_e::COMPLETE;
    }

    // static float thetab_bias = 0.0f;   // IMU 俯仰零点偏置，用于慢速自校准（当前未启用自动调整）
    // static float adapt_leg_angle_preset = app::pitch_preset; // 溜车时动态调整的 pitch_preset

    void Chassis_Balance::Chassis_Data_Update()

    {
        wbc_state.s1 = -SpeedEstimation();
        wbc_state.s += wbc_state.s1 * 0.001f;
        #ifdef INF_3
        wbc_state.thetab = imu_data->roll + 0.010801021f;
        wbc_state.thetab1 = imu_data->droll;

        wbc_state.phi = imu_data->total_imu_yaw;
        wbc_state.phi1 = imu_data->dyaw;
        wbc_state.roll = -imu_data->pitch - 0.00508242287 ;
        wbc_state.roll1 = -imu_data->dpitch;
        wbc_state.a_x = imu_data->acc_y * G;
        float r_imu = (leg_state[LEFT].L0 * cosf(leg_state[LEFT].phi0) + leg_state[RIGHT].L0 * cosf(leg_state[RIGHT].phi0)) / 2.0f;
        wbc_state.a_y = (imu_data->acc_x * G + sin(wbc_state.thetab) * G) * cos(wbc_state.thetab) - (wbc_state.phi1 * wbc_state.phi1 * r_imu);
        wbc_state.a_z = (imu_data->acc_z * G - cos(wbc_state.thetab) * G) * cos(wbc_state.thetab);
        #endif
        #ifdef INF_4
        wbc_state.thetab = -imu_data->roll + 0.0111782672f;
        wbc_state.thetab1 = -imu_data->droll;

        wbc_state.phi = imu_data->total_imu_yaw;
        wbc_state.phi1 = imu_data->dyaw;
        wbc_state.roll = imu_data->pitch + 0.00654419884f;
        wbc_state.roll1 = imu_data->dpitch;
        wbc_state.a_x = imu_data->acc_y * G;
        float r_imu = (leg_state[LEFT].L0 * cosf(leg_state[LEFT].phi0) + leg_state[RIGHT].L0 * cosf(leg_state[RIGHT].phi0)) / 2.0f;
        wbc_state.a_y = (imu_data->acc_x * G + sin(wbc_state.thetab) * G) * cos(wbc_state.thetab) - (wbc_state.phi1 * wbc_state.phi1 * r_imu);
        wbc_state.a_z = (imu_data->acc_z * G - cos(wbc_state.thetab) * G) * cos(wbc_state.thetab);
        #endif
        wbc_state.v_x += (wbc_state.a_x > 0.02 ? wbc_state.a_x : 0.0f) * 0.001f;
        wbc_state.v_y += wbc_state.a_y * 0.001f;
        wbc_state.v_z += wbc_state.a_z * 0.001f;

        leg_state[LEFT].wheel_spd = motordata_wl->speed * Wheel_R / REDUCTION_RATIO_WHEEL;
        leg_state[RIGHT].wheel_spd = -motordata_wr->speed * Wheel_R / REDUCTION_RATIO_WHEEL;
        leg_state[LEFT].phi[3] = -motordata_bl1->offset_angle - leg_angle_offset / 2.0f + PI / 2.0f;
        leg_state[LEFT].phi[0] = -motordata_bl0->offset_angle + leg_angle_offset / 2.0f + PI / 2.0f;
        leg_state[LEFT].dphi[3] = -motordata_bl1->speed;
        leg_state[LEFT].dphi[0] = -motordata_bl0->speed;
        leg_state[RIGHT].phi[3] = motordata_br1->offset_angle - leg_angle_offset / 2.0f + PI / 2.0f;
        leg_state[RIGHT].phi[0] = motordata_br0->offset_angle + leg_angle_offset / 2.0f + PI / 2.0f;
        leg_state[RIGHT].dphi[3] = motordata_br1->speed;
        leg_state[RIGHT].dphi[0] = motordata_br0->speed;
        leg_state[LEFT].phi0_total = Multi_Turn_Detection(leg_state[LEFT].round_count, leg_state[LEFT].phi0, leg_state[LEFT].phi0_last, leg_state[LEFT].first_time);
        leg_state[RIGHT].phi0_total = Multi_Turn_Detection(leg_state[RIGHT].round_count, leg_state[RIGHT].phi0, leg_state[RIGHT].phi0_last, leg_state[RIGHT].first_time);
        leg_state[LEFT].spring_force = spring_force_estimation(LEFT);
        leg_state[RIGHT].spring_force = spring_force_estimation(RIGHT);

        module::leg_pos(leg_state[LEFT].phi[0], leg_state[LEFT].phi[3], L1, L2, L5, leg_state[LEFT].dphi[0], leg_state[LEFT].dphi[3],
                        leg_state[LEFT].L0, leg_state[LEFT].phi0, leg_state[LEFT].dL0, leg_state[LEFT].dphi0, leg_state[LEFT].phi[1], leg_state[LEFT].phi[2]);
        module::leg_pos(leg_state[RIGHT].phi[0], leg_state[RIGHT].phi[3], L1, L2, L5, leg_state[RIGHT].dphi[0], leg_state[RIGHT].dphi[3],
                        leg_state[RIGHT].L0, leg_state[RIGHT].phi0, leg_state[RIGHT].dL0, leg_state[RIGHT].dphi0, leg_state[RIGHT].phi[1], leg_state[RIGHT].phi[2]);
        leg_state[LEFT].CoM_x = -(- 0.2208f * leg_state[LEFT].L0 * leg_state[LEFT].L0 + 0.0553f * leg_state[LEFT].L0 + 0.0301f);
        leg_state[LEFT].CoM_y = 0.2716f * leg_state[LEFT].L0 - 0.001f;
        leg_state[RIGHT].CoM_x = -(- 0.2208f * leg_state[RIGHT].L0 * leg_state[RIGHT].L0 + 0.0553f * leg_state[RIGHT].L0 + 0.0301f);
        leg_state[RIGHT].CoM_y = 0.2716f * leg_state[RIGHT].L0 - 0.001f;
        constexpr float dt = 0.001f;

        float raw_ddphi0_L = (leg_state[LEFT].dphi0 - leg_state[LEFT].dphi0_last) / dt;
        float raw_ddphi0_R = (leg_state[RIGHT].dphi0 - leg_state[RIGHT].dphi0_last) / dt;
        leg_state[LEFT].dphi0_last = leg_state[LEFT].dphi0;
        leg_state[RIGHT].dphi0_last = leg_state[RIGHT].dphi0;

        leg_state[LEFT].ddphi0 = filter_ddphi0_L(raw_ddphi0_L);
        leg_state[RIGHT].ddphi0 = filter_ddphi0_R(raw_ddphi0_R);

        leg_state[LEFT].I_l = 0.1158f * leg_state[LEFT].L0 + 0.0143f;
        leg_state[RIGHT].I_l = 0.1158f * leg_state[RIGHT].L0 + 0.0143f;
        module::inverse_contact_force(leg_state[LEFT].phi0, leg_state[LEFT].phi[0], leg_state[LEFT].phi[1], leg_state[LEFT].phi[2], leg_state[LEFT].phi[3], L1, leg_state[LEFT].L0,
                                      motordata_bl0->real_torque, motordata_bl1->real_torque, leg_state[LEFT].Fn, leg_state[LEFT].Tp, LEFT);
        module::inverse_contact_force(leg_state[RIGHT].phi0, leg_state[RIGHT].phi[0], leg_state[RIGHT].phi[1], leg_state[RIGHT].phi[2], leg_state[RIGHT].phi[3], L1, leg_state[RIGHT].L0,
                                      motordata_br0->real_torque, motordata_br1->real_torque, leg_state[RIGHT].Fn, leg_state[RIGHT].Tp, RIGHT);
        float Tp_comp_L = leg_state[LEFT].Tp - leg_state[LEFT].I_l * leg_state[LEFT].ddphi0;
        float Tp_comp_R = leg_state[RIGHT].Tp - leg_state[RIGHT].I_l * leg_state[RIGHT].ddphi0;
        float angle_to_vertical_L = leg_state[LEFT].phi0 - PI / 2 - wbc_state.thetab;
        leg_state[LEFT].P = (leg_state[LEFT].Fn + leg_state[LEFT].spring_force) * arm_cos_f32(angle_to_vertical_L) + Tp_comp_L * arm_sin_f32(angle_to_vertical_L);  
        if (leg_state[LEFT].P < 0.0f) leg_state[LEFT].P = 0.0f;
        float angle_to_vertical_R = leg_state[RIGHT].phi0 - PI / 2 - wbc_state.thetab;
        leg_state[RIGHT].P = (leg_state[RIGHT].Fn + leg_state[RIGHT].spring_force) * arm_cos_f32(angle_to_vertical_R) + Tp_comp_R * arm_sin_f32(angle_to_vertical_R);
        if (leg_state[RIGHT].P < 0.0f) leg_state[RIGHT].P = 0.0f;
        wbc_state.thetall = -leg_state[LEFT].phi0 + PI / 2 + wbc_state.thetab;
        if (wbc_state.thetall > PI)
            wbc_state.thetall -= 2 * PI;
        else if (wbc_state.thetall < -PI)
            wbc_state.thetall += 2 * PI;
        wbc_state.thetalr = -leg_state[RIGHT].phi0 + PI / 2 + wbc_state.thetab;
        if (wbc_state.thetalr > PI)
            wbc_state.thetalr -= 2 * PI;
        else if (wbc_state.thetalr < -PI)
            wbc_state.thetalr += 2 * PI;
        wbc_state.thetall1 = -leg_state[LEFT].dphi0 + wbc_state.thetab1;
        wbc_state.thetalr1 = -leg_state[RIGHT].dphi0 + wbc_state.thetab1;
    }

    // float measure, v_predict, v_final, k_est, dt_est;
    float Chassis_Balance::SpeedEstimation()
    {
        float v_swing_l = wbc_state.thetall1 * leg_state[LEFT].L0 * arm_cos_f32(wbc_state.thetall);
        float v_ext_l   = leg_state[LEFT].dL0 * arm_sin_f32(wbc_state.thetall);
        float vl = leg_state[LEFT].wheel_spd - v_swing_l - v_ext_l; 

        float v_swing_r = wbc_state.thetalr1 * leg_state[RIGHT].L0 * arm_cos_f32(wbc_state.thetalr);
        float v_ext_r   = leg_state[RIGHT].dL0 * arm_sin_f32(wbc_state.thetalr);
        float vr = leg_state[RIGHT].wheel_spd - v_swing_r - v_ext_r;

        return module::spd_estimation(vl, vr, wbc_state.phi1, wbc_state.a_y,
                                      leg_state[LEFT].P, leg_state[RIGHT].P,
                                      leg_state[LEFT].P < 20.0f, leg_state[RIGHT].P < 20.0f,
                                      leg_state[LEFT].dphi0, leg_state[RIGHT].dphi0);
    }
    void Chassis_Balance_Ctrl::Chassis_Controller()
    {
        Chassis_Controller_Update();

        const robot_state_s &robot_state = status_handle_.robot_state;
        const wbc_state_s &wbc_state = balance_.wbc_state;
        const leg_state_s *leg_state = balance_.leg_state;

        const float M_t = (M + 2 * (M_w + M_l));

        if (robot_state.mode[PRESENT] == app::mode_e::DISABLED)
        {
            output[LEFT].Tw = 0;
            output[RIGHT].Tw = 0;
            output[LEFT].Tl = 0;
            output[RIGHT].Tl = 0;
            output[LEFT].final_Tw = 0;
            output[RIGHT].final_Tw = 0;
            output[LEFT].final_Tl0 = 0;
            output[LEFT].final_Tl1 = 0;
            output[RIGHT].final_Tl0 = 0;
            output[RIGHT].final_Tl1 = 0;
            pid_LL->PIDClearI();
            pid_LR->PIDClearI();
            pid_leg_rotateL->PIDClearI();
            pid_leg_rotateR->PIDClearI();
            pid_wheel_current_l->PIDClearI();
            pid_wheel_current_r->PIDClearI();
            reset_leso();
            return;
        }

        bool is_roll_pid_enable = false;
        float F_ff = -M_t * G / 2.0f;

        switch (robot_state.mode[PRESENT])
        {
        case app::mode_e::NORMAL:
        case app::mode_e::SPIN:
            is_roll_pid_enable = true;
            if_jump_controller_enable = false;
            break;
        case app::mode_e::SLOW_START:
            is_roll_pid_enable = false;
            if_jump_controller_enable = false;
            break;
        case app::mode_e::JUMP:
        {
            static app::jump_phase_e jump_phase_last = app::jump_phase_e::COMPLETE;
            const bool phase_changed = (status_handle_.jump_phase_cnt != jump_phase_last);
            if (phase_changed)
            {
                jump_phase_last = status_handle_.jump_phase_cnt;
            }

            switch(status_handle_.jump_phase_cnt)
            {
                case app::jump_phase_e::PRESS:
                    is_roll_pid_enable = true;
                    if_jump_controller_enable = false;
                    break;
                case app::jump_phase_e::TAKE_OFF:
                    {
                        if_jump_controller_enable = true;

                        static float takeoff_acc_err_last = 0.0f;

                        const float dt = 0.001f;
                        const float az_ref = (status_handle_.jump_level_latched == 2) ? 20.0f : 6.2f;
                        const float kp_acc = 5.60f;
                        const float kd_acc = 0.4f;

                        float acc_err = az_ref - wbc_state.a_z;
                        if (phase_changed)
                        {
                            takeoff_acc_err_last = acc_err;
                            filter_takeoff_acc_err_d = module::FirstOrderFilter(0.4700f, 0.5300f);
                        }

                        float acc_err_d_raw = (acc_err - takeoff_acc_err_last) / dt;
                        float acc_err_d = filter_takeoff_acc_err_d(acc_err_d_raw);
                        takeoff_acc_err_last = acc_err;

                        float add_force_total = M_t * (kp_acc * acc_err + kd_acc * acc_err_d);
                        add_force_total = module::clip(add_force_total, 0.0f, M_t * G);
                        float add_force_leg = 0.5f * add_force_total;

                        output[LEFT].F_want = F_ff - add_force_leg;
                        output[RIGHT].F_want = F_ff - add_force_leg;
                        break;
                    }
                case app::jump_phase_e::FLYING:
                    {
                        if_jump_controller_enable = true;

                        output[LEFT].F_want = 0.0f;
                        output[RIGHT].F_want = 0.0f;
                        const float kp_pitch = 0.35f;
                        const float kd_pitch = 0.10f;
                        float pitch_pd = module::clip((kp_pitch * wbc_state.thetab + kd_pitch * wbc_state.thetab1), -0.30f, 0.30f);
                        float theta_ref_L = wbc_state.rotate_angle[LEFT] + pitch_pd;
                        float theta_ref_R = wbc_state.rotate_angle[RIGHT] + pitch_pd;

                        output[LEFT].Tl = -pid_leg_rotateL->PID_handle(theta_ref_L);
                        output[RIGHT].Tl = -pid_leg_rotateR->PID_handle(theta_ref_R);

                        break;
                    }
                case app::jump_phase_e::LANDING:
                    is_roll_pid_enable = true;
                    if_jump_controller_enable = false;
                    break;
                default:
                    if_jump_controller_enable = false;
                    break;
            }
            break;
        }
        case app::mode_e::FLY:
        {
            if_jump_controller_enable = true;
            is_roll_pid_enable = false;
            output[LEFT].F_want = 0.0f;
            output[RIGHT].F_want = 0.0f;
            break;
        }
        case app::mode_e::TOUCH_DOWN:
            is_roll_pid_enable = true;
            if_jump_controller_enable = false;
            break;
        case app::mode_e::SELF_HEAL:
            is_roll_pid_enable = false;
            if_jump_controller_enable = true;
            break;
        default:
            break;
        }

        bool roll_mpc_ok = false;
        float roll_differential_force = 0.0f;
        float roll_length_fallback = 0.0f;
        if ((is_roll_pid_enable) &&
            robot_state.if_off_gnd[LEFT] == 0 && robot_state.if_off_gnd[RIGHT] == 0)
        {
            float roll_err = filter_roll(wbc_state.roll);
            float roll_d = filter_roll1(wbc_state.roll1);

            roll_mpc_ok = balance_roll_mpc.Solve(
                roll_err, roll_d, ROLL_FORCE_ARM, ROLL_INERTIA,
                ROLL_DF_MAX, roll_differential_force);

            float speed_factor = 0.0f;
            if (robot_state.mode[PRESENT] == app::mode_e::SPIN)
            {
                float abs_yaw_rate = fabsf(wbc_state.phi1);
                speed_factor = module::clip((abs_yaw_rate - 3.0f) / 5.0f, 0.0f, 1.0f);
            }

            float Ki_roll = 0.0005f - speed_factor * (0.0005f - 0.0003f);
            Ki_roll += module::clip((fabsf(roll_err) - 0.05f) * 0.01f, 0.0f, 0.002f); 
            
            const float stab_limit = 0.15f;
            stab_roll += Ki_roll * (0.0f - roll_err);
            stab_roll = module::abs_clip(stab_roll, stab_limit);

            float kd_base = 0.02f;
            float kd_var = kd_base + 0.002f * expf(-fabsf(roll_err) * 20.0f);
            float kd_actual = kd_var - speed_factor * (kd_var - 0.00003f);
            
            float d_term = (fabsf(roll_d) > 0.01f) ? (kd_actual * (0.0f - roll_d)) : 0.0f;

            roll_length_fallback = module::abs_clip(stab_roll + d_term, stab_limit);
            F_roll_add = roll_mpc_ok ? roll_differential_force : roll_length_fallback;
        }
        else
        {
            stab_roll = 0.0f;
            F_roll_add = 0.0f;
            balance_roll_mpc.Reset();
        }


        if(!if_jump_controller_enable)
        {
            const float roll_offset = roll_mpc_ok ? 0.0f : roll_length_fallback;
            output[LEFT].F_want = F_ff - pid_LL->PID_handle(module::clip(wbc_state.LL_want[LEFT] - roll_offset, 0.18, 0.40));
            output[RIGHT].F_want = F_ff - pid_LR->PID_handle(module::clip(wbc_state.LL_want[RIGHT] + roll_offset, 0.18, 0.40));
        }
        else
        {
            const float roll_offset = roll_mpc_ok ? 0.0f : roll_length_fallback;
            output[LEFT].F_want = -pid_LL->PID_handle(wbc_state.LL_want[LEFT] - roll_offset);
            output[RIGHT].F_want = -pid_LR->PID_handle(wbc_state.LL_want[RIGHT] + roll_offset);
        }
        if (roll_mpc_ok)
        {
            output[LEFT].F_want += roll_differential_force;
            output[RIGHT].F_want -= roll_differential_force;
        }

        float Tl_comp_L = 0.0f;
        float Tl_comp_R = 0.0f;

        if (robot_state.mode[PRESENT] == app::mode_e::JUMP &&
            status_handle_.jump_phase_cnt == app::jump_phase_e::TAKE_OFF)
        {
            float F_total_L = output[LEFT].F_want + leg_state[LEFT].spring_force;
            float F_total_R = output[RIGHT].F_want + leg_state[RIGHT].spring_force;

            const float k_pitch_comp = 0.85f;

            Tl_comp_L = F_total_L * leg_state[LEFT].CoM_x * k_pitch_comp;
            Tl_comp_R = F_total_R * leg_state[RIGHT].CoM_x * k_pitch_comp;
        }

        output[LEFT].Tl += Tl_comp_L;
        output[RIGHT].Tl += Tl_comp_R;

        float spring_comp_L = leg_state[LEFT].spring_force;
        float spring_comp_R = leg_state[RIGHT].spring_force;

        output_last[LEFT] = output[LEFT];
        output_last[RIGHT] = output[RIGHT];

        static float compliance_weight[2] = {1.0f, 1.0f};
        constexpr float COMPLIANCE_AIR = 0.3f;
        constexpr float COMPLIANCE_IMPACT = 0.50f;
        constexpr float COMPLIANCE_GND = 1.0f;
        constexpr float WEIGHT_FILTER = 0.1f;

        for (int leg = LEFT; leg <= RIGHT; leg++)
        {
            float target_weight = COMPLIANCE_GND;
            if (robot_state.mode[PRESENT] == app::mode_e::FLY)
            {
                if (robot_state.if_off_gnd[leg] == 1) // 腾空
                {
                    target_weight = COMPLIANCE_AIR;
                }
                else if (robot_state.if_off_gnd[leg] == 2) // 冲击
                {
                    target_weight = COMPLIANCE_IMPACT;
                }
            }

            compliance_weight[leg] += (target_weight - compliance_weight[leg]) * WEIGHT_FILTER;

            bool is_jump_flying = (robot_state.mode[PRESENT] == app::mode_e::JUMP &&
                                   status_handle_.jump_phase_cnt == app::jump_phase_e::FLYING);

            if (leg == LEFT) {
                if (!is_jump_flying)
                    output[LEFT].F_want *= compliance_weight[LEFT];
                spring_comp_L *= compliance_weight[LEFT];
            } else {
                if (!is_jump_flying)
                    output[RIGHT].F_want *= compliance_weight[RIGHT];
                spring_comp_R *= compliance_weight[RIGHT];
            }
        }

        module::leg_VMC(leg_state[LEFT].phi0, leg_state[LEFT].phi[0], leg_state[LEFT].phi[1],
                        leg_state[LEFT].phi[2], leg_state[LEFT].phi[3], L1, leg_state[LEFT].L0,
                        output[LEFT].F_want + spring_comp_L, output[LEFT].Tl, output[LEFT].final_Tl0, output[LEFT].final_Tl1);
        module::leg_VMC(leg_state[RIGHT].phi0, leg_state[RIGHT].phi[0], leg_state[RIGHT].phi[1],
                        leg_state[RIGHT].phi[2], leg_state[RIGHT].phi[3], L1, leg_state[RIGHT].L0,
                        output[RIGHT].F_want + spring_comp_R, output[RIGHT].Tl, output[RIGHT].final_Tl0, output[RIGHT].final_Tl1);
    }

    void Chassis_Balance_Ctrl::Power_RLS_Update()
    {
        const float V_bus = 24.0f; 
        const float P_static_half = app::P_STATIC_CONST / 2.0f;

        const float omega_L = balance_.motordata_wl->speed;
        const float omega_R = -balance_.motordata_wr->speed;
        const float tau_L   = balance_.motordata_wl->real_torque;
        const float tau_R   = -balance_.motordata_wr->real_torque;

        Matrixf<2, 1> x_L;
        x_L[0][0] = fabsf(omega_L);
        x_L[1][0] = tau_L * tau_L;

        float I_L = fabsf(balance_.motordata_wl->real_current);
        float P_target_L = V_bus * I_L - tau_L * omega_L - P_static_half;

        rls_L->update(x_L, P_target_L);
        params.k1_L = rls_L->getParamsVector()[0][0];
        params.k2_L = rls_L->getParamsVector()[1][0];

        Matrixf<2, 1> x_R;
        x_R[0][0] = fabsf(omega_R);
        x_R[1][0] = tau_R * tau_R;

        float I_R = fabsf(balance_.motordata_wr->real_current);
        float P_target_R = V_bus * I_R - tau_R * omega_R - P_static_half;

        rls_R->update(x_R, P_target_R);
        params.k1_R = rls_R->getParamsVector()[0][0];
        params.k2_R = rls_R->getParamsVector()[1][0];
    }

    void Chassis_Balance_Ctrl::Chassis_Controller_Update()
    {

        const leg_state_s *leg_state = balance_.leg_state;
        const robot_state_s &robot_state = status_handle_.robot_state;
        const wbc_state_s &wbc_state = balance_.wbc_state;
        static float yaw_rate_cmd_limited = 0.0f;

        if (robot_state.mode[PRESENT] != robot_state.mode[PAST] ||
            robot_state.state_flag[PRESENT] != robot_state.state_flag[PAST])
        {
            balance_nlmpc.Reset();
            balance_roll_mpc.Reset();
            output[LEFT].final_Tw = 0.0f;
            output[RIGHT].final_Tw = 0.0f;
            output[LEFT].final_Tl0 = 0.0f;
            output[LEFT].final_Tl1 = 0.0f;
            output[RIGHT].final_Tl0 = 0.0f;
            output[RIGHT].final_Tl1 = 0.0f;
            pid_wheel_current_l->PIDClearI();
            pid_wheel_current_r->PIDClearI();
            module::reset_kf_stall();
            balance_.clear_s();
        }

        module::lqr_K(leg_state[LEFT].L0, leg_state[RIGHT].L0, module::K_out, lqr_matrix.K_Matrix);

        status_handle_.if_off_gnd_write_in(LEFT, balance_.Gnd_Off_Detect(LEFT));
        status_handle_.if_off_gnd_write_in(RIGHT, balance_.Gnd_Off_Detect(RIGHT));

        fn[LEFT] = leg_state[LEFT].Fn;
        fn[RIGHT] = leg_state[RIGHT].Fn;

        float target_v = robot_state.v[1];

        constexpr float PARK_CMD_THRESH    = 0.05f;  // m/s — 驻车指令死区
        constexpr float PARK_SPD_THRESH    = 0.20f;  // m/s — 驻车速度阈值
        constexpr float PARK_POS_GAIN      = 5.0f;   // 驻车位移误差增益
        constexpr float PARK_MAX_POS_ERR   = 0.30f;  // m — 驻车限幅
        constexpr float PARK_ERR_DEADBAND  = 0.005f; // m — 驻车死区

        constexpr float LAUNCH_RATIO       = 0.50f;  // 启动助力：实际速度 / 原始目标 < 此值时触发
        constexpr float LAUNCH_MIN_RAW     = 0.10f;  // m/s — 启动助力最低原始目标，防止静止误触发
        constexpr float LAUNCH_POS_GAIN    = 0.35f;  // s — 位移 = 增益 × 速度差 (m per m/s deficit)
        constexpr float LAUNCH_MAX_OFFSET  = 0.50f;  // m — 启动助力最大位移偏移

        constexpr float POS_FILTER_ALPHA = 0.314f; // 60 Hz 一阶低通 @ 1kHz: alpha = 1-exp(-2π·60·0.001)
        float raw_pos = 0.0f;

        if (robot_state.mode[PRESENT] == app::mode_e::NORMAL)
        {
            float raw_target = robot_state.v_raw_target;
            float abs_raw = fabsf(raw_target);

            if (abs_raw >= PARK_CMD_THRESH)
            {
                park_brake_state_ = park_brake_e::OFF;

                float abs_spd = fabsf(wbc_state.s1);
                if (abs_spd < LAUNCH_RATIO * abs_raw && abs_raw > LAUNCH_MIN_RAW)
                {
                    float deficit = abs_raw - abs_spd;
                    float offset = LAUNCH_POS_GAIN * deficit;
                    if (offset > LAUNCH_MAX_OFFSET)
                        offset = LAUNCH_MAX_OFFSET;
                    raw_pos = (raw_target > 0.0f ? 1.0f : -1.0f) * offset;
                }
            }
            else
            {
                switch (park_brake_state_)
                {
                case park_brake_e::OFF:
                    if (fabsf(target_v) < PARK_CMD_THRESH)
                        park_brake_state_ = park_brake_e::WAITING;
                    break;

                case park_brake_e::WAITING:
                    if (fabsf(wbc_state.s1) < PARK_SPD_THRESH)
                    {
                        park_brake_state_ = park_brake_e::ENGAGED;
                        park_s_locked_ = wbc_state.s;
                    }
                    break;

                case park_brake_e::ENGAGED:
                {
                    float pos_err = park_s_locked_ - wbc_state.s;
                    if (fabsf(pos_err) < PARK_ERR_DEADBAND)
                        pos_err = 0.0f;
                    pos_err = module::abs_clip(pos_err, PARK_MAX_POS_ERR);
                    raw_pos = PARK_POS_GAIN * pos_err;
                    break;
                }
                }
            }
        }
        else
        {
            park_brake_state_ = park_brake_e::OFF;
        }

        pos_filter_state_ += POS_FILTER_ALPHA * (raw_pos - pos_filter_state_);
        lqr_matrix.Status_vector[0] = pos_filter_state_;

        if (park_brake_state_ != park_brake_e::ENGAGED)
        {
            s_want += target_v * 0.001f;
        }

        v_err = module::abs_clip((target_v - wbc_state.s1), 2.5f);
        if (fabs(v_err) < 0.005f)
            v_err = 0;

        if (robot_state.mode[PRESENT] == app::mode_e::SPIN ||
            robot_state.mode[PRESENT] == app::mode_e::FLY ||
            (robot_state.mode[PRESENT] == app::mode_e::JUMP &&
             status_handle_.jump_phase_cnt == app::jump_phase_e::TAKE_OFF))
        {
            lqr_matrix.Status_vector[1] = 0.0f;
        }
        else
        {
            lqr_matrix.Status_vector[1] = v_err;
        }

        // switch (robot_state.v_level[1])

        // {
        // case 0:
        //     pitch_preset = 0.083f;
        //     break;
        // case 1:
        //     pitch_preset = 0.085f;
        //     break;
        // case 2:
        //     pitch_preset = 0.09f;
        //     break;
        // case -1:
        //     pitch_preset = 0.08f;
        //     break;
        // case -2:
        //     pitch_preset = 0.08f;
        // }

        if (robot_state.mode[PRESENT] <= app::mode_e::UP_STAIR && robot_state.if_follow)
        {
            float raw_yaw_error = robot_state.follow_angle - wbc_state.phi;
            while (raw_yaw_error > PI) raw_yaw_error -= 2.0f * PI;
            while (raw_yaw_error < -PI) raw_yaw_error += 2.0f * PI;

            float abs_yaw_error = fabs(raw_yaw_error);
            float yaw_error_input = raw_yaw_error;

            bool is_stuck = (fabs(wbc_state.s1) < 0.2f) && 
                            (fabs(output_last[LEFT].Tw) + fabs(output_last[RIGHT].Tw) > T_MAX_WHEEL * 1.2f);

            if (abs_yaw_error > 0.8f && is_stuck)
            {
                float penalty_factor = expf(-1.8f * (abs_yaw_error - 0.8f));
                yaw_error_input *= penalty_factor;
            }

            static float smoothed_yaw_error = 0.0f;
            const float max_yaw_step = 0.5f;
            const float yaw_err_out_limit = 0.85f;
            const float yaw_err_slope = 0.6f;
            
            if (is_stuck && abs_yaw_error <= 0.8f) 
            {
                smoothed_yaw_error *= 0.95f; 
            } 
            else 
            {
                smoothed_yaw_error += module::abs_clip(yaw_error_input - smoothed_yaw_error, max_yaw_step);
            }

            lqr_matrix.Status_vector[2] = yaw_err_out_limit * tanhf(smoothed_yaw_error / yaw_err_slope);
        }
        else
        {
            lqr_matrix.Status_vector[2] = 0.0f;
        }

        float yaw_rate_cmd_raw = 0.0f;
        if (robot_state.mode[PRESENT] == app::mode_e::SPIN)
        {
            yaw_rate_cmd_raw = (robot_state.v[2] - wbc_state.phi1);
        }
        else
        {
            if (robot_state.if_follow)
            {
                yaw_rate_cmd_raw = -wbc_state.phi1;
            }
            else
            {
                yaw_rate_cmd_raw =
                    (robot_state.v[2] - wbc_state.phi1) *
                    smooth_saturation(fabs(robot_state.v[1] - wbc_state.s1), 0.05f, 4.0f);
            }
        }

        static float landing_recovery_factor = 1.0f;
        const bool airborne_or_landing =
            (robot_state.mode[PRESENT] == app::mode_e::FLY) ||
            (robot_state.if_off_gnd[LEFT] != 0) ||
            (robot_state.if_off_gnd[RIGHT] != 0);

        if (airborne_or_landing)
        {
            landing_recovery_factor = 0.15f; 
        }
        else
        {
            landing_recovery_factor += 0.005f; 
            if (landing_recovery_factor > 1.0f) 
            {
                landing_recovery_factor = 1.0f;
            }
        }

        float yaw_limit = 0.5f + (5.0f - 0.5f) * landing_recovery_factor;
        float yaw_slew  = 0.08f + (0.4f - 0.08f) * landing_recovery_factor; 

        float yaw_target = module::clip(yaw_rate_cmd_raw, -yaw_limit, yaw_limit);
        yaw_rate_cmd_limited += module::abs_clip(yaw_target - yaw_rate_cmd_limited, yaw_slew);
        
        if (fabsf(yaw_rate_cmd_limited) < 0.001f)
            yaw_rate_cmd_limited = 0.0f;

        if (robot_state.mode[PRESENT] == app::mode_e::SPIN)
            lqr_matrix.Status_vector[3] = yaw_rate_cmd_raw;
        else
            lqr_matrix.Status_vector[3] = yaw_rate_cmd_limited;

        if (robot_state.mode[PRESENT] == app::mode_e::FLY) lqr_matrix.Status_vector[3] = 0.0f;

        // adapt_leg_angle_preset = module::abs_clip(atanf(robot_state.acc[1] / G), 0.25f);
        adapt_leg_angle_preset = 0.0f;
        const bool in_jump_latch_window =
            (robot_state.mode[PRESENT] == app::mode_e::JUMP) &&
            (status_handle_.jump_phase_cnt == app::jump_phase_e::PRESS ||
             status_handle_.jump_phase_cnt == app::jump_phase_e::TAKE_OFF);
        
        const bool in_fly_latch_window = robot_state.mode[PRESENT] == app::mode_e::FLY;

        static float adapt_leg_angle_preset_latched = 0.0f;
        static bool adapt_leg_angle_preset_latched_valid = false;

        if (in_jump_latch_window && !adapt_leg_angle_preset_latched_valid)
        {
            adapt_leg_angle_preset_latched = 0.0f;
            adapt_leg_angle_preset_latched_valid = true;
        }
        else if (in_fly_latch_window)
        {
            adapt_leg_angle_preset_latched = adapt_leg_angle_preset;
            adapt_leg_angle_preset_latched_valid = true;
        }
        else if (!in_jump_latch_window)
        {
            adapt_leg_angle_preset_latched_valid = false;
        }

        // if (robot_state.acc[1] < -0.1f && adapt_leg_angle_preset > 0.0f)
        // {
        //     float penalty = expf(-5.0f * adapt_leg_angle_preset * fabsf(robot_state.acc[1]));
        //     adapt_leg_angle_preset *= penalty;
        // }

        const bool is_jump_press_phase =
            (robot_state.mode[PRESENT] == app::mode_e::JUMP) &&
            (status_handle_.jump_phase_cnt == app::jump_phase_e::PRESS);
        float pitch_preset_used = pitch_preset;
        if (!is_jump_press_phase)
        {
            const float low_pitch_offset = 0.01f;
            const float mid_pitch_offset = 0.0f;
            const float high_pitch_offset = -0.02f;

            float low_ll_ref = 0.17f;
            float mid_ll_ref = 0.25f;
            float high_ll_ref = 0.36f;
            if (robot_state.mode[PRESENT] == app::mode_e::SPIN)
            {
                low_ll_ref = 0.16f;
                mid_ll_ref = 0.22f;
                high_ll_ref = 0.32f;
            }

            float ll_avg_want = 0.5f * (wbc_state.LL_want[LEFT] + wbc_state.LL_want[RIGHT]);
            float pitch_offset = 0.0f;

            if (ll_avg_want <= mid_ll_ref)
            {
                float low_to_mid_ratio = module::clip((ll_avg_want - low_ll_ref) / (mid_ll_ref - low_ll_ref), 0.0f, 1.0f);
                pitch_offset = low_pitch_offset + (mid_pitch_offset - low_pitch_offset) * low_to_mid_ratio;
            }
            else
            {
                float mid_to_high_ratio = module::clip((ll_avg_want - mid_ll_ref) / (high_ll_ref - mid_ll_ref), 0.0f, 1.0f);
                pitch_offset = mid_pitch_offset + (high_pitch_offset - mid_pitch_offset) * mid_to_high_ratio;
            }

            pitch_preset_used += pitch_offset;
        }
        // Temporarily disable the leg swing angle bias while validating NLMPC.
        pitch_preset_used = 0.0f;
        const float adapt_leg_angle_preset_used = 
            ((in_jump_latch_window || in_fly_latch_window) && adapt_leg_angle_preset_latched_valid)
                ? adapt_leg_angle_preset_latched
                : adapt_leg_angle_preset;
        
        if (wbc_state.LL_want[LEFT] > 0.3 && robot_state.mode[PRESENT] != app::mode_e::FLY)
        {
            lqr_matrix.Status_vector[4] = 0 - wbc_state.thetall + pitch_preset_used;
        }
        else
        {
            lqr_matrix.Status_vector[4] = 0 - wbc_state.thetall + adapt_leg_angle_preset_used + pitch_preset_used;
        }

        lqr_matrix.Status_vector[5] = 0 - wbc_state.thetall1;

        if (wbc_state.LL_want[RIGHT] > 0.3 && robot_state.mode[PRESENT] != app::mode_e::FLY)
        {
            lqr_matrix.Status_vector[6] = 0 - wbc_state.thetalr + pitch_preset_used;
        }
        else
        {
            lqr_matrix.Status_vector[6] = 0 - wbc_state.thetalr + adapt_leg_angle_preset_used + pitch_preset_used;
        }
        lqr_matrix.Status_vector[7] = 0 - wbc_state.thetalr1;
        const bool is_in_jump_window = (robot_state.mode[PRESENT] == app::mode_e::JUMP) &&
        (status_handle_.jump_phase_cnt == app::jump_phase_e::PRESS ||
        status_handle_.jump_phase_cnt == app::jump_phase_e::TAKE_OFF);
        const float target_thetab = is_in_jump_window ? 0.1f:0.0f;
        lqr_matrix.Status_vector[8] = module::abs_clip((target_thetab - wbc_state.thetab), 0.52f);
        lqr_matrix.Status_vector[9] = (0.0f - wbc_state.thetab1);

        lqr_matrix.Status_vector_real[0] = wbc_state.s;
        lqr_matrix.Status_vector_real[1] = wbc_state.s1;
        lqr_matrix.Status_vector_real[2] = wbc_state.phi;
        lqr_matrix.Status_vector_real[3] = wbc_state.phi1;
        lqr_matrix.Status_vector_real[4] = wbc_state.thetall;
        lqr_matrix.Status_vector_real[5] = wbc_state.thetall1;
        lqr_matrix.Status_vector_real[6] = wbc_state.thetalr;
        lqr_matrix.Status_vector_real[7] = wbc_state.thetalr1;
        lqr_matrix.Status_vector_real[8] = wbc_state.thetab;
        lqr_matrix.Status_vector_real[9] = wbc_state.thetab1;// assemble real status vector for dobc

        // lqr_matrix.Status_vector[2] = 0;
        // lqr_matrix.Status_vector[3] = 0;
        // 控制器切换
        switch (robot_state.state_flag[PRESENT])
        {
        case app::process_e::DISABLED:
            output[LEFT].Tw = 0;
            output[RIGHT].Tw = 0;
            output[LEFT].Tl = 0;
            output[RIGHT].Tl = 0;
            output[LEFT].final_Tl0 = 0;
            output[LEFT].final_Tl1 = 0;
            output[RIGHT].final_Tl0 = 0;
            output[RIGHT].final_Tl1 = 0;
            reset_leso();
            module::reset_kf();
            break;

        case app::process_e::LQR_ON:
        {
            if (robot_state.state_flag[PAST] != app::process_e::LQR_ON)
            {
                reset_leso();
                module::reset_kf();
                pid_LL->PIDClearI();
                pid_LR->PIDClearI();
                pid_leg_rotateL->PIDClearI();
                pid_leg_rotateR->PIDClearI();
            }

            float nlmpc_output[4]{};
            const uint32_t nlmpc_start = DWT->CYCCNT;
            bool nlmpc_ok = balance_nlmpc.Solve(
                leg_state[LEFT].L0, leg_state[RIGHT].L0, lqr_matrix.Status_vector,
                wbc_state.thetall, wbc_state.thetalr, nlmpc_output);
            const uint32_t nlmpc_cycles = DWT->CYCCNT - nlmpc_start;
            balance_nlmpc.SetCycles(nlmpc_cycles);
            nlmpc_ok = nlmpc_ok && nlmpc_cycles <= NLMPC_CYCLE_LIMIT;

            if (nlmpc_ok)
            {
                output[LEFT].Tw = nlmpc_output[0];
                output[RIGHT].Tw = nlmpc_output[1];
                output[LEFT].Tl = nlmpc_output[2];
                output[RIGHT].Tl = nlmpc_output[3];
            }
            else
            {
                balance_nlmpc.MarkFallback();
                module::calculate_status_vector_fly(
                    lqr_matrix.K_Matrix, lqr_matrix.Status_vector,
                    output[LEFT].Tw, output[RIGHT].Tw, output[LEFT].Tl, output[RIGHT].Tl,
                    robot_state.if_off_gnd[LEFT], robot_state.if_off_gnd[RIGHT], 0.1f);
            }

            reset_leso();

            if (fabs(wbc_state.s1) > 0.5f && pwr_ctrl == true )
            {
                float u_arr[2] = {output[LEFT].Tw, output[RIGHT].Tw};
                float w_arr[2] = {balance_.motordata_wl->speed,-balance_.motordata_wr->speed};
                float k1_arr[2] = {params.k1_L, params.k1_R};
                float k2_arr[2] = {params.k2_L, params.k2_R};
                float u_star[2];

                Power_RLS_Update();
                module::QP(u_arr, w_arr, k1_arr, k2_arr, app::P_MAX_CHASSIS, u_star);

                output[LEFT].final_Tw = u_star[0];
                output[RIGHT].final_Tw = u_star[1];
            }
            else
            {
                output[LEFT].final_Tw = module::abs_clip(output[LEFT].Tw, T_MAX_WHEEL);
                output[RIGHT].final_Tw = module::abs_clip(output[RIGHT].Tw, T_MAX_WHEEL);
            }

            if (robot_state.if_off_gnd[LEFT] != 0)
            {
                output[LEFT].final_Tw = 0.0f;
            }
            
            if (robot_state.if_off_gnd[RIGHT] != 0)
            {
                output[RIGHT].final_Tw = 0.0f;
            }
           
            output[LEFT].Tl = module::abs_clip(output[LEFT].Tl, T_MAX_LEG);
            output[RIGHT].Tl = module::abs_clip(output[RIGHT].Tl, T_MAX_LEG);
            break;
        }

        case app::process_e::PID_ONLY:
            if (robot_state.state_flag[PAST] == app::process_e::LQR_ON)
            {
                reset_leso();
                module::reset_kf();
                pid_wheel_current_l->PIDClearI();
                pid_wheel_current_r->PIDClearI();
            }

            if (robot_state.mode[PRESENT] == app::mode_e::SLOW_START || robot_state.mode[PRESENT] == app::mode_e::SELF_HEAL)
            {
                output[LEFT].Tl = -pid_leg_rotateL->PID_handle(wbc_state.rotate_angle[LEFT]);
                output[RIGHT].Tl = -pid_leg_rotateR->PID_handle(wbc_state.rotate_angle[RIGHT]);
                output[LEFT].Tw = 0;
                output[RIGHT].Tw = 0;
            }
            else if (robot_state.mode[PRESENT] == app::mode_e::FLY || (robot_state.mode[PRESENT] == app::mode_e::JUMP && status_handle_.jump_phase_cnt >= app::jump_phase_e::TAKE_OFF))
            {
                output[LEFT].Tw = 0;
                output[RIGHT].Tw = 0;
            }
            else if (robot_state.mode[PRESENT] == app::mode_e::UP_STAIR)
            {
                output[LEFT].Tl = -pid_leg_rotateL->PID_handle(wbc_state.rotate_angle[LEFT]);
                output[RIGHT].Tl = -pid_leg_rotateR->PID_handle(wbc_state.rotate_angle[RIGHT]);
                output[LEFT].Tw = 0;
                output[RIGHT].Tw = 0;
            }
            if (robot_state.mode[PRESENT] != app::mode_e::UP_STAIR)
            {
                output[LEFT].final_Tw = 0.0f;
                output[RIGHT].final_Tw = 0.0f;
            }
            break;

        case app::process_e::HEALING:
            output[LEFT].Tw = 0;
            output[RIGHT].Tw = 0;
            output[LEFT].Tl = -pid_leg_rotateL->PID_handle(wbc_state.rotate_angle[LEFT]);
            output[RIGHT].Tl = -pid_leg_rotateR->PID_handle(wbc_state.rotate_angle[RIGHT]);
            output[LEFT].final_Tw = 0.0f;
            output[RIGHT].final_Tw = 0.0f;
            pid_wheel_current_l->PIDClearI();
            pid_wheel_current_r->PIDClearI();
            reset_leso();
            module::reset_kf();
            break;

        default:
            output[LEFT].Tw = 0;
            output[RIGHT].Tw = 0;
            output[LEFT].Tl = 0;
            output[RIGHT].Tl = 0;
            output[LEFT].final_Tw = 0.0f;
            output[RIGHT].final_Tw = 0.0f;
            pid_wheel_current_l->PIDClearI();
            pid_wheel_current_r->PIDClearI();
            reset_leso();
            module::reset_kf();
            break;
        }
    }
    float Chassis_Balance::Multi_Turn_Detection(float &round_count, float phi, float &phi_last, bool &first_time)
    {
        if (first_time)
        {
            phi_last = phi;
            first_time = 0;
        }

        if (phi < phi_last && std::fabs(phi - phi_last) > PI)
            round_count++;
        else if (phi > phi_last && std::fabs(phi - phi_last) > PI)
        {
            round_count--;
        }

        phi_last = phi;

        float multi_turn_angle = round_count * 2 * PI + phi;
        return multi_turn_angle;
    }

    int Chassis_Balance::Gnd_Off_Detect(int leg)
    {
        return 0;// 
        float current_P = ((leg == LEFT) ? filter_Fn_L(leg_state[LEFT].P) : filter_Fn_R(leg_state[RIGHT].P)) - M_l * G;
        
        static int gnd_state[2] = {0, 0}; // 0: GROUNDED, 1: AIRBORNE, 2: IMPACT
        static uint32_t impact_timer[2] = {0, 0};
        static float P_last[2] = {0.0f, 0.0f};
        static bool dP_init[2] = {false, false};

        static float dP_buf[2][4] = {{0.0f}};
        static int dP_idx[2] = {0};
        static float LL_want_last[2] = {0.0f, 0.0f};

        bool is_changing_len = fabsf(wbc_state.LL_want[leg] - LL_want_last[leg]) > 1.0f;
        LL_want_last[leg] = wbc_state.LL_want[leg];

        if (!dP_init[leg])
        {
            P_last[leg] = current_P;
            gnd_state[leg] = 0;
            for (int i = 0; i < 4; i++) dP_buf[leg][i] = 0.0f;
            dP_init[leg] = true;
        }

        constexpr float dt = 0.001f;
        float raw_dP = (current_P - P_last[leg]) / dt;
        P_last[leg] = current_P;

        dP_buf[leg][dP_idx[leg]] = raw_dP;
        dP_idx[leg] = (dP_idx[leg] + 1) % 4;

        float filtered_dP = 0.0f;
        for (int i = 0; i < 4; i++) {
            filtered_dP += dP_buf[leg][i];
        }
        filtered_dP *= 0.25f;

        constexpr float P_AIR_THRES = 80.0f;
        constexpr float P_GROUND_RECOVER = 60.0f;
        constexpr float DP_IMPACT_THRES = 2000.0f;
        constexpr uint32_t T_IMPACT_DURATION = 10;
        constexpr float L_EXTENDED_THRES = 0.20f;

        if (is_changing_len)
        {
            gnd_state[leg] = 0;
            return gnd_state[leg];
        }

        switch (gnd_state[leg])
        {
            case 0: 
                if (current_P < P_AIR_THRES && leg_state[leg].L0 > L_EXTENDED_THRES)
                {
                    gnd_state[leg] = 1;
                }
                break;

            case 1: 
                // if (filtered_dP > DP_IMPACT_THRES)
                // {
                //     gnd_state[leg] = 2;
                //     impact_timer[leg] = 0;
                // }
                // else if (current_P > P_GROUND_RECOVER)
                // {
                //      gnd_state[leg] = 0;
                // }
                if (current_P > P_GROUND_RECOVER)
                {
                     gnd_state[leg] = 0;
                }
                break;

            case 2:
                impact_timer[leg]++;
                if (impact_timer[leg] > T_IMPACT_DURATION)
                {
                    if (current_P > P_AIR_THRES)
                    {
                        gnd_state[leg] = 0;
                    }
                    else
                    {
                        gnd_state[leg] = 1;
                    }
                }
                break;
        }

        return gnd_state[leg];
    }

    float Chassis_Balance_Ctrl::friction_compensation(float velocity, int leg)
    {
        const stribeck_s *p = &friction_data[leg];
        float direction = tanhf(velocity * 100.0f);
        float stribeck_effect = (p->Fs - p->Fc) * expf(-fabsf(velocity / p->vs));
        float i_comp = p->sigma * velocity + direction * (p->Fc + stribeck_effect);

        float fade_in = fabsf(velocity) / 0.05f;
        if (fade_in < 1.0f) {
             i_comp *= fade_in; 
        }

        return i_comp;
    }

    void Chassis_Balance_Ctrl::Motor_Send()
    {

        const robot_state_s &robot_state = status_handle_.robot_state;
        const leg_state_s *leg_state = balance_.leg_state;
        if (robot_state.mode[PRESENT] != app::mode_e::DISABLED)
        {
            balance_.motordata_bl0->send_data = module::abs_clip(output[LEFT].final_Tl0, 40);
            balance_.motordata_bl1->send_data = module::abs_clip(output[LEFT].final_Tl1, 40);
            balance_.motordata_br0->send_data = module::abs_clip(-output[RIGHT].final_Tl0, 40);
            balance_.motordata_br1->send_data = module::abs_clip(-output[RIGHT].final_Tl1, 40);
            output[LEFT].final_Tw = module::abs_clip(output[LEFT].final_Tw, 6.0f);
            output[RIGHT].final_Tw = module::abs_clip(output[RIGHT].final_Tw, 6.0f);

            float current_L = torque_to_current(output[LEFT].final_Tw);
            float current_R = torque_to_current(-output[RIGHT].final_Tw);
            balance_.motordata_wl->send_data = pid_wheel_current_l->PID_handle(module::abs_clip(current_L, 16000));
            balance_.motordata_wr->send_data = pid_wheel_current_r->PID_handle(module::abs_clip(current_R, 16000));
        }
        else
        {
            balance_.motordata_bl0->send_data = 0;
            balance_.motordata_bl1->send_data = 0;
            balance_.motordata_br0->send_data = 0;
            balance_.motordata_br1->send_data = 0;
            balance_.motordata_wl->send_data = 0;
            balance_.motordata_wr->send_data = 0;
        }
    }

    float Chassis_Balance_Status_Handle::rotate_handle(float round_count, float single_angle, float thetab)
    {
        if (robot_state.mode[PRESENT] == app::mode_e::SLOW_START)
        {
            static float slow_target = 1.39f;
            if (robot_state.mode[PAST] != app::mode_e::SLOW_START)
            {
                slow_target = (thetab < 0.0f) ? 1.23f : 1.39f;
            }

            float target_angle = round_count * 2 * PI + slow_target;
            float current_angle = round_count * 2 * PI + single_angle;
            float diff = target_angle - current_angle;

            if (single_angle > 2.40f)
            {
                target_angle += 2 * PI;
            }
            else
            {
                while (diff > PI)
                {
                    target_angle -= 2 * PI;
                    diff -= 2 * PI;
                }
                while (diff < -PI)
                {
                    target_angle += 2 * PI;
                    diff += 2 * PI;
                }
            }
            return target_angle;
        }
        else if (robot_state.mode[PRESENT] == app::mode_e::SELF_HEAL)
        {

            if (fabs(thetab) > 0.3f && thetab > 0.0f)
            {
                return ((round_count + 1) * 2 * PI + single_angle);
            }
            else
            {
                return ((round_count + 1) * 2 * PI + single_angle);
            }
        }
        return 0.0f;
    }

    void Chassis_Balance_Status_Handle::FSM(module::RC_ctrl_t *rc_ctrl) // 无头骑士
    {
        subMessage("MST_CHASSIS_CTRL", data_to_chassis);

        const wbc_state_s &wbc_state = balance_.wbc_state;
        const leg_state_s *leg_state = balance_.leg_state;
        static float fly_init_swing_single = PI / 2.0f;

#ifdef NO_HEAD

        auto setSpeed = [](int rocker_value) -> int
        {
            if (rocker_value > 100 && rocker_value <= 400)
                return 1;
            else if (rocker_value > 400)
                return 2;
            else if (rocker_value < -100 && rocker_value >= -400)
                return -1;
            else if (rocker_value < -400)
                return -2;
            else
                return 0;
        };

        robot_state.v_level[1] = setSpeed(rc_ctrl->rocker_l1);

        int speed[3];
        speed[0] = setSpeed(rc_ctrl->rocker_l1);
        speed[1] = setSpeed(rc_ctrl->rocker_l_);
        robot_state.dial_level = setSpeed(rc_ctrl->dial);

        robot_state.v[1] = speed[0];
        robot_state.v[2] = -speed[1];
        robot_state.v_raw_target = static_cast<float>(speed[0]);
        // robot_state.follow_angle += robot_state.v[2] *0.001f;


        robot_state.mode[PAST] = robot_state.mode[PRESENT];
        robot_state.state_flag[PAST] = robot_state.state_flag[PRESENT];


        jump_cmd = jump_state_key_check(rc_ctrl->switch_right == 1);

        if (rc_ctrl->switch_left == 3 && fsm_state.init_flag == 0)
            robot_state.mode[PRESENT] = app::mode_e::SLOW_START;
        else if (rc_ctrl->switch_left == 3 && fsm_state.init_flag == 1)
            robot_state.mode[PRESENT] = app::mode_e::NORMAL;
        else if (rc_ctrl->switch_left == 2)
        {
            robot_state.mode[PRESENT] = app::mode_e::DISABLED;
            fsm_state.init_flag = 0;
        }
        else if (rc_ctrl->switch_left == 1)
            robot_state.mode[PRESENT] = app::mode_e::SELF_HEAL;
        else
            robot_state.mode[PRESENT] = app::mode_e::DISABLED;
#else
        module::TD_Config_s td_input_config = {r_input, h_input, h0_input};
        float v_want = 0.0f;
        v_want = module::abs_clip(data_to_chassis.speed[0] * 2.0f, 4.2f);
        bool is_flipped = data_to_chassis.if_turn ^ robot_state.is_inversed;
        float direction_sign = is_flipped ? -1.0f : 1.0f;
        robot_state.v_raw_target = v_want * direction_sign;
        robot_state.v[1] = td_input[0].update(v_want * direction_sign, td_input_config);
        robot_state.acc[1] = td_input[0].get_derivative() * 0.1f;

        auto set_speed = [](int dial_value) -> int
        {
            if (dial_value == 1)
                return 8;
            else if (dial_value == 2)
                return 14;
            else if (dial_value == -1)
                return -8;
            else if (dial_value == -2)
                return -14;
            else
                return 0;
        };

        robot_state.dial_level = set_speed(data_to_chassis.speed[2]);

        static int save_key_last = 0;
        bool save_key_edge = (data_to_chassis.save_key != save_key_last);
        save_key_last = data_to_chassis.save_key;

        static bool is_healing_active = false;
        static uint32_t fatal_error_timer = 0; // 自救自动触发计时器

        robot_state.mode[PAST] = robot_state.mode[PRESENT];
        robot_state.state_flag[PAST] = robot_state.state_flag[PRESENT];

        // jump_cmd = jump_state_key_check(data_to_chassis.jump_key);
        static int jump_key_last = 0;
        bool jump_key_edge = (data_to_chassis.jump_key != 0) && (jump_key_last == 0);
        jump_key_last = data_to_chassis.jump_key;
        jump_cmd = jump_key_edge ? 2 : 0;

        robot_state.mode[PAST] = robot_state.mode[PRESENT];
        robot_state.state_flag[PAST] = robot_state.state_flag[PRESENT];

        if (data_to_chassis.if_enable == 0)
        {
            robot_state.mode[PRESENT] = app::mode_e::DISABLED;
            is_fatal_error = false;
            is_healing_active = false;
            fatal_error_timer = 0;
            fsm_state.init_flag = 0;
        }
        else if (is_fatal_error && !is_healing_active)
        {
            fatal_error_timer++;
            if (fatal_error_timer >= 2000 || save_key_edge || data_to_chassis.save_key)
            {
                is_healing_active = true;
                robot_state.mode[PRESENT] = app::mode_e::SELF_HEAL;
                fsm_state.init_flag = 0;
                fatal_error_timer = 0;
            }
        }
        else if (is_healing_active)
        {
            robot_state.mode[PRESENT] = app::mode_e::SELF_HEAL;
            fatal_error_timer = 0;
        }
        else if (data_to_chassis.if_enable == 1  && fsm_state.init_flag == 0)
        {
            robot_state.mode[PRESENT] = app::mode_e::SLOW_START;
        }
        else if (data_to_chassis.if_enable == 1 && fsm_state.init_flag == 1)
        {
            if (robot_state.mode[PAST] == app::mode_e::UP_STAIR ||
                robot_state.mode[PAST] == app::mode_e::JUMP ||
                robot_state.mode[PAST] == app::mode_e::FLY ||
                robot_state.mode[PAST] == app::mode_e::TOUCH_DOWN)
            {
                robot_state.mode[PRESENT] = robot_state.mode[PAST]; 
            }
            else if (data_to_chassis.if_free)
                robot_state.mode[PRESENT] = app::mode_e::SPIN;
            else
                robot_state.mode[PRESENT] = app::mode_e::NORMAL;
        }
        else
            robot_state.mode[PRESENT] = app::mode_e::DISABLED;

        static int leg_key_last = 0;
        bool leg_key_edge = (data_to_chassis.leg_key != leg_key_last);
        leg_key_last = data_to_chassis.leg_key;

        if (leg_key_edge)
        {
            robot_state.LL_STATE = static_cast<app::LL_Discribe_e>((static_cast<int>(robot_state.LL_STATE) + 1) % 3);
            leg_change_lock_cnt = 500; // 遥控器控制腿伸缩时 0.5s 不认为离地
        }

#endif

        static int fly_enter_cnt = 0;
        static int fly_time_cnt = 0;
        static int up_stair_cnt = 0;
        static int touch_down_stable_cnt = 0;
        constexpr int FLY_ENTER_HYS = 3;
        constexpr int NORMAL_INIT_LOCK_TICKS = 3000;

        static int impact_lock_cnt = 0;
        constexpr float IMPACT_AX_THRES = 6.3f; // 撞击加速度阈值 (m/s^2)，需实测微调
        constexpr int IMPACT_LOCK_TIME = 100;

        if (fabs(wbc_state.a_x) > IMPACT_AX_THRES)
        {
            impact_lock_cnt = IMPACT_LOCK_TIME;
        }
        else if (impact_lock_cnt > 0)
        {
            impact_lock_cnt--;
        }

        if (normal_init_lock_cnt > 0)
        {
            normal_init_lock_cnt--;
        }

        const bool both_leg_off_ground =
            (robot_state.if_off_gnd[LEFT] == 1) &&
            (robot_state.if_off_gnd[RIGHT] == 1);

        // FLY 退出检测：双腿腿长均小于 0.25m 认为可靠触地
        constexpr float FLY_EXIT_LL_THRES = 0.25f;
        constexpr int   FLY_EXIT_LL_HYS   = 10;
        const bool both_legs_short = (leg_state[LEFT].L0 < FLY_EXIT_LL_THRES) &&
                                     (leg_state[RIGHT].L0 < FLY_EXIT_LL_THRES);

        static int fly_exit_short_leg_cnt = 0;
        if (both_legs_short)
            fly_exit_short_leg_cnt++;
        else
            fly_exit_short_leg_cnt = 0;

        const bool reliable_touch_down = (fly_exit_short_leg_cnt > FLY_EXIT_LL_HYS);

        const bool can_enter_fly =
            (robot_state.mode[PAST] == app::mode_e::NORMAL) &&
            (jump_cmd == 0) &&
            (impact_lock_cnt == 0) &&
            (normal_init_lock_cnt == 0);

        if (can_enter_fly && both_leg_off_ground)
        {
            fly_enter_cnt++;
        }
        else
        {
            fly_enter_cnt = 0;
        }

        if (fly_enter_cnt > FLY_ENTER_HYS)
        {
            // 非 jump 飞行：锁定起飞瞬间两腿平均摆角，供空中保持
            fly_init_swing_single = 0.5f * (leg_state[LEFT].phi0 + leg_state[RIGHT].phi0);
            while (fly_init_swing_single > PI) fly_init_swing_single -= 2.0f * PI;
            while (fly_init_swing_single < -PI) fly_init_swing_single += 2.0f * PI;

            robot_state.mode[PRESENT] = app::mode_e::FLY;
            fly_enter_cnt = 0;

            balance_.wbc_state.s = 0.0f;
            balance_.wbc_state.s1 = 0.0f;
            module::reset_kf();
        }

        if (robot_state.mode[PRESENT] == app::mode_e::FLY)
            fly_time_cnt++;
        else
            fly_time_cnt = 0;

        constexpr int fly_touchdown_min_ticks = 100;
        constexpr int FLY_TIMEOUT_TICKS = 1000;
        if ((robot_state.mode[PAST] == app::mode_e::FLY || robot_state.mode[PRESENT] == app::mode_e::FLY) &&
            (reliable_touch_down || fly_time_cnt > FLY_TIMEOUT_TICKS) &&
            fly_time_cnt > fly_touchdown_min_ticks)
        {
            robot_state.mode[PRESENT] = app::mode_e::NORMAL;
            robot_state.LL_STATE = app::LL_Discribe_e::LOW;
            touch_down_stable_cnt = 0;
        }

        bool is_pose_stable_for_jump = (fabs(wbc_state.thetab) < 0.20f) &&
                                       (fabs(wbc_state.roll) < 0.20f) &&
                                       (fabs(wbc_state.s1) < 2.4f);

        if (jump_cmd != 0 && robot_state.mode[PAST] == app::mode_e::NORMAL && is_pose_stable_for_jump)
        {
            jump_level_latched = (jump_cmd == 2) ? 2 : 1;
            robot_state.mode[PRESENT] = app::mode_e::JUMP;
        }
        const float FATAL_ROLL_THRESHOLD = 0.90f;
        const float FATAL_PITCH_THRESHOLD = 0.82f;
        const float DISASTER_ROLL_THRESHOLD = 1.2f;
        const float DISASTER_PITCH_THRESHOLD = 1.2f;

        // 灾难翻转即时检测 — 全模式有效（SELF_HEAL/DISABLED 除外），不走累积计数
        if ((fabs(wbc_state.roll) > DISASTER_ROLL_THRESHOLD || fabs(wbc_state.thetab) > DISASTER_PITCH_THRESHOLD) &&
            robot_state.mode[PRESENT] != app::mode_e::SELF_HEAL &&
            robot_state.mode[PRESENT] != app::mode_e::DISABLED)
        {
            robot_state.mode[PRESENT] = app::mode_e::DISABLED;
            fsm_state.init_flag = 0;
            is_fatal_error = true;
        }

        // 绝望的机长 v7.0
        static int fatal_err_cnt = 0;

        bool is_fatal_roll = fabs(wbc_state.roll) > FATAL_ROLL_THRESHOLD;
        bool is_fatal_pitch = fabs(wbc_state.thetab) > FATAL_PITCH_THRESHOLD;
        bool is_checking_mode = (robot_state.mode[PRESENT] != app::mode_e::DISABLED) &&
                                (robot_state.mode[PRESENT] != app::mode_e::SELF_HEAL);

        if (is_checking_mode && (is_fatal_roll || is_fatal_pitch))
        {
            fatal_err_cnt++;
        }
        else
        {
            fatal_err_cnt = (fatal_err_cnt > 2) ? (fatal_err_cnt - 2) : 0;
        }

        if (fatal_err_cnt > 50 && is_checking_mode)
        {
            robot_state.mode[PRESENT] = app::mode_e::DISABLED;
            fsm_state.init_flag = 0;
            fatal_err_cnt = 0;
            is_fatal_error = true;
        }
        if (is_fatal_error && robot_state.mode[PRESENT] != app::mode_e::SELF_HEAL)
        {
            robot_state.mode[PRESENT] = app::mode_e::DISABLED;
            fsm_state.init_flag = 0;
        }
        // dial/if_free 控制 SPIN 模式
#ifdef NO_HEAD
        bool spin_cmd = (robot_state.dial_level != 0);
#else
        bool spin_cmd = (robot_state.dial_level != 0) || data_to_chassis.if_free;
        int dial_level_spin = (robot_state.dial_level != 0) ? robot_state.dial_level : (data_to_chassis.if_free ? 1 : 0);
        robot_state.dial_level = dial_level_spin;
#endif
        if (spin_cmd && robot_state.mode[PRESENT] == app::mode_e::NORMAL)
            robot_state.mode[PRESENT] = app::mode_e::SPIN;

        if (robot_state.mode[PRESENT] == app::mode_e::NORMAL)
        {
            if (robot_state.LL_STATE == app::LL_Discribe_e::HIGH) 
            {
                if (leg_state[LEFT].phi0 < 1.23f && leg_state[RIGHT].phi0 < 1.23f)
                {
                    up_stair_cnt += 3; 
                }
                else if (leg_state[LEFT].phi0 < 1.32f && leg_state[RIGHT].phi0 < 1.32f)
                {
                    up_stair_cnt -= 1;
                }
                else
                {
                    up_stair_cnt = 0; 
                }
            }
            else 
            {
                up_stair_cnt = 0;
            }

            if (up_stair_cnt > 120) up_stair_cnt = 120;

            if (up_stair_cnt > 80) 
            {
                robot_state.mode[PRESENT] = app::mode_e::UP_STAIR;
                up_stair_cnt = 0;
            }
        }

        const float CRASH_PITCH_THRESHOLD =
            (robot_state.mode[PRESENT] == app::mode_e::UP_STAIR) ? 0.65f : 0.45f;
        const float FATAL_SPLIT_THRESHOLD = 0.69f;
        // 绝望的机长 v6.0
        static int crash_cnt = 0;

        bool is_pitch_crashed = fabs(wbc_state.thetab) > CRASH_PITCH_THRESHOLD;
        
        bool is_touching_ground = (robot_state.if_off_gnd[LEFT] != 1) || (robot_state.if_off_gnd[RIGHT] != 1);
        
        bool is_in_active_mode = (robot_state.mode[PRESENT] == app::mode_e::NORMAL) || 
                                 (robot_state.mode[PRESENT] == app::mode_e::SPIN) ||
                                 (robot_state.mode[PRESENT] == app::mode_e::UP_STAIR);

        float phi0_diff = leg_state[LEFT].phi0_total - leg_state[RIGHT].phi0_total;
        while (phi0_diff > PI) phi0_diff -= 2.0f * PI;
        while (phi0_diff < -PI) phi0_diff += 2.0f * PI;
        bool is_fatal_split = fabs(phi0_diff) > FATAL_SPLIT_THRESHOLD;
        
        if (((is_pitch_crashed && is_touching_ground) || is_fatal_split) && is_in_active_mode && !is_fatal_error)
        {
            crash_cnt++;
        }
        else
        {
            crash_cnt = 0;
        }

        if (crash_cnt > 100)
        {
            robot_state.mode[PRESENT] = app::mode_e::SLOW_START;
            fsm_state.init_flag = 0;
            crash_cnt = 0;
        }

        // 基于 PAST→PRESENT 转换检测退出 SPIN，避免 FSM 已提前改 mode 导致清理逻辑跳过
        if (robot_state.mode[PAST] == app::mode_e::SPIN && robot_state.mode[PRESENT] != app::mode_e::SPIN)
        {
            robot_state.follow_angle = wbc_state.phi;
            td_input[1].reset();
            balance_.wbc_state.s = 0.0f;
            balance_.wbc_state.s1 = 0.0f;
            module::reset_kf();
        }

        if (robot_state.mode[PRESENT] == app::mode_e::SPIN)
        {
            module::TD_Config_s td_spin_config = {0.9f * r_input, h_input, h0_input};
            robot_state.v[2] = td_input[1].update(robot_state.dial_level, td_spin_config);
            robot_state.state_flag[PRESENT] = app::process_e::LQR_ON;

            static app::LL_Discribe_e last_ll_state = app::LL_Discribe_e::LOW;

            const bool ll_state_changed = (robot_state.LL_STATE != last_ll_state);

            if (ll_state_changed)
            {
                float base_L0 = 0.22f;
                switch (robot_state.LL_STATE)
                {
                case app::LL_Discribe_e::LOW:
                    base_L0 = 0.16f;
                    break;
                
                case app::LL_Discribe_e::MID:
                    base_L0 = 0.22f;
                    break;
                
                case app::LL_Discribe_e::HIGH:
                    base_L0 = 0.32f;
                    break;
                
                default:
                    base_L0 = 0.20f;
                    break;
                }
                robot_state.LL_FSM_Want[LEFT] = base_L0;
                robot_state.LL_FSM_Want[RIGHT] = base_L0;
            }

            last_ll_state = robot_state.LL_STATE;
        }


        if (robot_state.mode[PRESENT] != robot_state.mode[PAST])
        {
            fsm_state.fail_wait_cnt = 0;
            robot_state.state_flag[PAST] = robot_state.state_flag[PRESENT];
            robot_state.state_flag[PRESENT] = app::process_e::DISABLED;

            // 从 DISABLED 切换到 SLOW_START 或 NORMAL 时，把位置积分清零，避免 LQR 按历史位置回零
            if (robot_state.mode[PAST] == app::mode_e::DISABLED &&
                (robot_state.mode[PRESENT] == app::mode_e::SLOW_START ||
                 robot_state.mode[PRESENT] == app::mode_e::NORMAL))
            {
                balance_.wbc_state.s = 0.0f;
            }
        }

        if (robot_state.mode[PRESENT] == app::mode_e::DISABLED)
        {
            robot_state.state_flag[PRESENT] = app::process_e::DISABLED;
            robot_state.v[1] = 0;
            robot_state.v[2] = 0;
            robot_state.follow_angle = wbc_state.phi;
            ramp_leg_len[LEFT].reset(leg_state[LEFT].L0);
            ramp_leg_len[RIGHT].reset(leg_state[RIGHT].L0);
            ramp_leg_rotate[LEFT].reset(leg_state[LEFT].phi0);
            ramp_leg_rotate[RIGHT].reset(leg_state[RIGHT].phi0);
            fsm_state.init_flag = 0;
            is_saved = false;
            robot_state.is_inversed = false;
            robot_state.rotate_speed = 0.001f;
            robot_state.L0_speed = 0.00015f;
            fsm_state.fail_wait_cnt = 0;

        }

        // 只有完全离开站立/空中相关模式时才清除 init_flag，避免 FLY/TOUCH_DOWN 被下一拍拉回慢启动
        if (robot_state.mode[PRESENT] != app::mode_e::NORMAL &&
            robot_state.mode[PRESENT] != app::mode_e::JUMP &&
            robot_state.mode[PRESENT] != app::mode_e::SPIN &&
            robot_state.mode[PRESENT] != app::mode_e::FLY &&
            robot_state.mode[PRESENT] != app::mode_e::TOUCH_DOWN&&
            robot_state.mode[PRESENT] != app::mode_e::UP_STAIR)
        {
            fsm_state.init_flag = 0;
        }

        // slow start — 两阶段：先收腿再转腿，避免边收边转导致不稳定
        if (robot_state.mode[PRESENT] == app::mode_e::SLOW_START)
        {
            static float slow_start_pose_target = 1.39f;
            static int slow_start_phase = 0; // 0=收腿, 1=转腿

            if (robot_state.mode[PAST] != app::mode_e::SLOW_START)
            {
                robot_state.state_flag[PRESENT] = app::process_e::DISABLED;
                ramp_leg_rotate[LEFT].reset(leg_state[LEFT].phi0_total);
                ramp_leg_rotate[RIGHT].reset(leg_state[RIGHT].phi0_total);

                slow_start_pose_target = (wbc_state.thetab < 0.0f) ? 1.23f : 1.39f;
                slow_start_phase = 0;

                // 先保持当前腿位，不收腿时就开始转
                robot_state.rotate_FSM_Want[LEFT] = leg_state[LEFT].phi0_total;
                robot_state.rotate_FSM_Want[RIGHT] = leg_state[RIGHT].phi0_total;
            }

            robot_state.state_flag[PRESENT] = app::process_e::PID_ONLY;
            robot_state.LL_FSM_Want[LEFT] = 0.18f;
            robot_state.LL_FSM_Want[RIGHT] = 0.18f;

            bool legs_retracted = (leg_state[RIGHT].L0 < 0.24 && leg_state[LEFT].L0 < 0.24);

            // 收腿到位后切换到转腿阶段
            if (slow_start_phase == 0 && legs_retracted)
            {
                slow_start_phase = 1;
                ramp_leg_rotate[LEFT].reset(leg_state[LEFT].phi0_total);
                ramp_leg_rotate[RIGHT].reset(leg_state[RIGHT].phi0_total);
                robot_state.rotate_FSM_Want[LEFT] = rotate_handle(leg_state[LEFT].round_count, leg_state[LEFT].phi0, wbc_state.thetab);
                robot_state.rotate_FSM_Want[RIGHT] = rotate_handle(leg_state[RIGHT].round_count, leg_state[RIGHT].phi0, wbc_state.thetab);
            }

            static int cnt = 0;

            bool pose_stable = (leg_state[LEFT].phi0 < slow_start_pose_target + 0.2f  && leg_state[LEFT].phi0 > slow_start_pose_target - 0.3f &&
                                leg_state[RIGHT].phi0 < slow_start_pose_target + 0.2f  && leg_state[RIGHT].phi0 > slow_start_pose_target - 0.3f &&
                                legs_retracted);

            // 只在转腿阶段计数 pose_stable
            if (pose_stable && slow_start_phase == 1)
            {
                cnt++;
            }
            else if (!pose_stable)
            {
                cnt = 0;
            }

            fsm_state.fail_wait_cnt++;

            bool normal_init = (cnt > 200);
            bool timeout_init = (fsm_state.fail_wait_cnt > 500 &&
                                 fabs(wbc_state.thetab) < PI / 3 &&
                                 pose_stable);

            if (normal_init || timeout_init)
            {
                cnt = 0;
                fsm_state.init_flag = 1;
                fsm_state.fail_wait_cnt = 0;
                normal_init_lock_cnt = NORMAL_INIT_LOCK_TICKS;
                robot_state.mode[PRESENT] = app::mode_e::NORMAL;
                robot_state.LL_STATE = app::LL_Discribe_e::LOW;

                {
                    float yaw_off = module::rad_format(data_to_chassis.follow_angle);
                    robot_state.is_inversed = (fabsf(yaw_off) > PI / 2.0f);
                }

                robot_state.follow_angle = wbc_state.phi;
                // 清除慢启动期间积累的速度/位移漂移，防止 LQR 首拍按漂移值输出轮力矩
                balance_.wbc_state.s = 0.0f;
                balance_.wbc_state.s1 = 0.0f;
                module::reset_kf();
            }
        }

        // stand
        if (robot_state.mode[PRESENT] == app::mode_e::NORMAL || robot_state.mode[PRESENT] == app::mode_e::SPIN)

        {
            robot_state.state_flag[PRESENT] = app::process_e::LQR_ON;

            static app::LL_Discribe_e last_ll_state = app::LL_Discribe_e::LOW;
            const bool enter_stand_mode =
                (robot_state.mode[PAST] != app::mode_e::NORMAL && robot_state.mode[PAST] != app::mode_e::SPIN);
            const bool ll_state_changed = (robot_state.LL_STATE != last_ll_state);

            if (enter_stand_mode || ll_state_changed)
            {
                float base_L0 = 0.22f;
                switch (robot_state.LL_STATE)
                {
                case app::LL_Discribe_e::LOW:
                    base_L0 = 0.17f;
                    break;
                
                case app::LL_Discribe_e::MID:
                    base_L0 = 0.25f;
                    break;
                
                case app::LL_Discribe_e::HIGH:
                    base_L0 = 0.36f;
                    break;
                
                default:
                    base_L0 = 0.22f;
                    break;
                }
                robot_state.LL_FSM_Want[LEFT] = base_L0;
                robot_state.LL_FSM_Want[RIGHT] = base_L0;
            }

            last_ll_state = robot_state.LL_STATE;
        }

        // recovery
        if (robot_state.mode[PRESENT] == app::mode_e::SELF_HEAL)
        {
            static float heal_dir = 1.0f;
            static float phi0_total_last[2] = {0.0f, 0.0f};
            static int heal_stall_cnt = 0;
            static int heal_flip_cooldown = 0;

            fsm_state.init_flag = 0;

            if (robot_state.mode[PAST] != app::mode_e::SELF_HEAL)
            {
                fsm_state.fail_wait_cnt = 0;
                robot_state.state_flag[PRESENT] = app::process_e::PID_ONLY;
                robot_state.LL_FSM_Want[LEFT] = 0.34f;
                robot_state.LL_FSM_Want[RIGHT] = 0.34f;
                robot_state.rotate_FSM_Want[LEFT] = leg_state[LEFT].phi0_total;
                robot_state.rotate_FSM_Want[RIGHT] = leg_state[RIGHT].phi0_total;
                phi0_total_last[LEFT] = leg_state[LEFT].phi0_total;
                phi0_total_last[RIGHT] = leg_state[RIGHT].phi0_total;
                ramp_leg_rotate[LEFT].reset(leg_state[LEFT].phi0_total);
                ramp_leg_rotate[RIGHT].reset(leg_state[RIGHT].phi0_total);
                heal_stall_cnt = 0;
                heal_flip_cooldown = 0;

                heal_dir = (wbc_state.thetab > 0 || wbc_state.thetab < -2.8f) ? -1.0f : 1.0f;
            }

            if (save_key_edge && robot_state.state_flag[PRESENT] == app::process_e::HEALING)
            {
                heal_dir *= -1.0f;
            }

            if (leg_state[LEFT].L0 > 0.32f && leg_state[RIGHT].L0 > 0.32f)
            {
                robot_state.state_flag[PRESENT] = app::process_e::HEALING;
            }

            if (robot_state.state_flag[PRESENT] == app::process_e::HEALING)
            {
                if (robot_state.state_flag[PAST] != app::process_e::HEALING)
                {
                    ramp_leg_rotate[LEFT].reset(leg_state[LEFT].phi0_total);
                    ramp_leg_rotate[RIGHT].reset(leg_state[RIGHT].phi0_total);
                    robot_state.rotate_FSM_Want[LEFT] = leg_state[LEFT].phi0_total;
                    robot_state.rotate_FSM_Want[RIGHT] = leg_state[RIGHT].phi0_total;
                }

                float phi0_err = leg_state[LEFT].phi0_total - leg_state[RIGHT].phi0_total;
                while (phi0_err > PI) phi0_err -= 2.0f * PI;
                while (phi0_err < -PI) phi0_err += 2.0f * PI;

                if (fabs(phi0_err) > 0.1f)
                {
                    if (fabs(leg_state[LEFT].phi0 - PI / 2) < fabs(leg_state[RIGHT].phi0 - PI / 2))
                    {
                        robot_state.rotate_FSM_Want[RIGHT] = leg_state[RIGHT].round_count * 2 * PI + leg_state[LEFT].phi0;
                    }
                    else
                    {
                        robot_state.rotate_FSM_Want[LEFT] = leg_state[LEFT].round_count * 2 * PI + leg_state[RIGHT].phi0;
                    }
                }
                else 
                {
                    if (fabs(wbc_state.thetab) >= 0.8f)
                    {

                        robot_state.rotate_FSM_Want[LEFT] = leg_state[LEFT].phi0_total + heal_dir * 2 * PI;
                        robot_state.rotate_FSM_Want[RIGHT] = leg_state[RIGHT].phi0_total + heal_dir * 2 * PI;
                        robot_state.LL_FSM_Want[LEFT] = 0.34f;
                        robot_state.LL_FSM_Want[RIGHT] = 0.34f;

                        float dphi_total_L = fabsf(leg_state[LEFT].phi0_total - phi0_total_last[LEFT]);
                        float dphi_total_R = fabsf(leg_state[RIGHT].phi0_total - phi0_total_last[RIGHT]);
                        phi0_total_last[LEFT] = leg_state[LEFT].phi0_total;
                        phi0_total_last[RIGHT] = leg_state[RIGHT].phi0_total;

                        constexpr float HEAL_STALL_CMD_ERR_THRES = 1.6f;
                        constexpr float HEAL_STALL_MOTION_THRES = 0.003f;
                        constexpr int HEAL_STALL_FLIP_CNT_THRES = 800;
                        constexpr int HEAL_FLIP_COOLDOWN_TICKS = 1500;
                        constexpr float HEAL_FLIP_ALLOW_THETAB = 1.05f;

                        float cmd_err_L = fabsf(robot_state.rotate_FSM_Want[LEFT] - leg_state[LEFT].phi0_total);
                        float cmd_err_R = fabsf(robot_state.rotate_FSM_Want[RIGHT] - leg_state[RIGHT].phi0_total);
                        // bool high_output_cmd = (cmd_err_L > HEAL_STALL_CMD_ERR_THRES) && (cmd_err_R > HEAL_STALL_CMD_ERR_THRES);
                        bool high_output_cmd = true;
                        bool little_motion = (dphi_total_L < HEAL_STALL_MOTION_THRES) || (dphi_total_R < HEAL_STALL_MOTION_THRES);

                        if (heal_flip_cooldown > 0)
                            heal_flip_cooldown--;

                        if (high_output_cmd && little_motion)
                            heal_stall_cnt++;
                        else if (heal_stall_cnt > 0)
                            heal_stall_cnt -= 2;

                        if (heal_stall_cnt < 0)
                            heal_stall_cnt = 0;

                        if (heal_stall_cnt > HEAL_STALL_FLIP_CNT_THRES &&
                            heal_flip_cooldown == 0 &&
                            fabsf(wbc_state.thetab) > HEAL_FLIP_ALLOW_THETAB)
                        {
                            heal_dir *= -1.0f;
                            heal_stall_cnt = 0;
                            heal_flip_cooldown = HEAL_FLIP_COOLDOWN_TICKS;
                            robot_state.rotate_FSM_Want[LEFT] = leg_state[LEFT].phi0_total + heal_dir * 2 * PI;
                            robot_state.rotate_FSM_Want[RIGHT] = leg_state[RIGHT].phi0_total + heal_dir * 2 * PI;
                        }
                    }
                    else 
                    {
                        heal_stall_cnt = 0;
                        robot_state.rotate_FSM_Want[LEFT] = leg_state[LEFT].round_count * 2 * PI + PI / 4;
                        robot_state.rotate_FSM_Want[RIGHT] = leg_state[RIGHT].round_count * 2 * PI + PI / 4;

                        robot_state.LL_FSM_Want[LEFT] = 0.18f;
                        robot_state.LL_FSM_Want[RIGHT] = 0.18f;

                        if (fabs(leg_state[LEFT].phi0 - PI / 4) < 0.2f && 
                            fabs(leg_state[RIGHT].phi0 - PI / 4) < 0.2f &&
                            leg_state[LEFT].L0 < 0.22f && leg_state[RIGHT].L0 < 0.22f)
                        {
                            is_fatal_error = false;
                            is_healing_active = false;
                            normal_init_lock_cnt = NORMAL_INIT_LOCK_TICKS;
                            module::reset_kf();
                        }
                    }
                }
            }
        }

        
        if (robot_state.mode[PRESENT] == app::mode_e::FLY)
        {
            robot_state.state_flag[PRESENT] = app::process_e::LQR_ON;
            robot_state.LL_STATE = app::LL_Discribe_e::LOW;
            robot_state.LL_FSM_Want[LEFT] = 0.24f;
            robot_state.LL_FSM_Want[RIGHT] = 0.24f;
        }

        if (robot_state.mode[PRESENT] == app::mode_e::TOUCH_DOWN)
        {
            robot_state.LL_STATE = app::LL_Discribe_e::LOW;
            robot_state.state_flag[PRESENT] = app::process_e::LQR_ON;

            if (robot_state.mode[PAST] != app::mode_e::TOUCH_DOWN)
            {
                robot_state.LL_FSM_Want[LEFT] = 0.17f;
                robot_state.LL_FSM_Want[RIGHT] = 0.17f;
                ramp_leg_len[LEFT].reset(leg_state[LEFT].L0);
                ramp_leg_len[RIGHT].reset(leg_state[RIGHT].L0);
            }

            const bool body_pose_stable =
                (fabsf(wbc_state.thetab) < 0.25f) &&
                (fabsf(wbc_state.roll) < 0.25f);

            if (body_pose_stable)
                touch_down_stable_cnt++;
            else
                touch_down_stable_cnt = 0;

            if (touch_down_stable_cnt >= 100)
            {
                robot_state.mode[PRESENT] = app::mode_e::NORMAL;
                touch_down_stable_cnt = 0;
                normal_init_lock_cnt = NORMAL_INIT_LOCK_TICKS;
            }
        }
        else
        {
            touch_down_stable_cnt = 0;
        }

        if (robot_state.mode[PRESENT] == app::mode_e::UP_STAIR)
        {
            if (robot_state.mode[PAST] != app::mode_e::UP_STAIR)
            {
                robot_state.up_stair_phase = 1; 
                up_stair_timer = 0;
                robot_state.state_flag[PRESENT] = app::process_e::PID_ONLY;
                ramp_leg_rotate[LEFT].reset(leg_state[LEFT].phi0_total);
                ramp_leg_rotate[RIGHT].reset(leg_state[RIGHT].phi0_total);
            }

            if (robot_state.up_stair_phase == 1)
            {
                robot_state.state_flag[PRESENT] = app::process_e::PID_ONLY;
                robot_state.rotate_FSM_Want[LEFT] = leg_state[LEFT].round_count * 2 * PI - 0.2f;
                robot_state.rotate_FSM_Want[RIGHT] = leg_state[RIGHT].round_count * 2 * PI - 0.2f;
                robot_state.LL_FSM_Want[LEFT] = 0.17f;
                robot_state.LL_FSM_Want[RIGHT] = 0.17f;

                if (fabs(leg_state[LEFT].phi0)< 0.5 && fabs(leg_state[RIGHT].phi0) < 0.5)
                {
                    up_stair_timer++;
                }
                else
                {
                    up_stair_timer = 0;
                }

                if (up_stair_timer > 100)
                {
                    robot_state.up_stair_phase = 2;
                }
            }
            
            if (robot_state.up_stair_phase == 2)
            {
                robot_state.mode[PRESENT] = app::mode_e::SLOW_START;
                fsm_state.init_flag = 0;
                fsm_state.fail_wait_cnt = 0;
                robot_state.LL_STATE = app::LL_Discribe_e::LOW;
                robot_state.state_flag[PRESENT] = app::process_e::DISABLED;
                ramp_leg_rotate[LEFT].reset(leg_state[LEFT].phi0_total);
                ramp_leg_rotate[RIGHT].reset(leg_state[RIGHT].phi0_total);
                robot_state.rotate_FSM_Want[LEFT] = rotate_handle(leg_state[LEFT].round_count, leg_state[LEFT].phi0, wbc_state.thetab);
                robot_state.rotate_FSM_Want[RIGHT] = rotate_handle(leg_state[RIGHT].round_count, leg_state[RIGHT].phi0, wbc_state.thetab);
                balance_.wbc_state.s = 0.0f;
            }
        }

        // jump
        if (robot_state.mode[PRESENT] == app::mode_e::JUMP)
        {
            static int phase_timeout_cnt = 0;

            if (robot_state.mode[PAST] != app::mode_e::JUMP)
            {
                if (jump_level_latched != 1 && jump_level_latched != 2)
                    jump_level_latched = (jump_cmd == 2) ? 2 : 1;

                jump_phase_cnt = app::jump_phase_e::PRESS;
                jump_duration = 0.0f;
                phase_timeout_cnt = 0;
                jump_leg_cnt[LEFT] = 0;
                jump_leg_cnt[RIGHT] = 0;
            }

            switch (jump_phase_cnt)
            {
            case app::jump_phase_e::PRESS:
                robot_state.state_flag[PRESENT] = app::process_e::LQR_ON;
                robot_state.LL_FSM_Want[LEFT] = 0.20f;
                robot_state.LL_FSM_Want[RIGHT] = 0.20f;
                phase_timeout_cnt++;

                ramp_leg_rotate[LEFT].reset(leg_state[LEFT].phi0_total);
                ramp_leg_rotate[RIGHT].reset(leg_state[RIGHT].phi0_total);
                if (leg_state[LEFT].L0 < 0.20f && leg_state[RIGHT].L0 < 0.20f)
                {
                    jump_leg_cnt[LEFT]++;
                    jump_leg_cnt[RIGHT]++;
                }
                else
                {
                    jump_leg_cnt[LEFT] = 0;
                    jump_leg_cnt[RIGHT] = 0;
                }

                if ((jump_leg_cnt[LEFT] > 80 && jump_leg_cnt[RIGHT] > 80) || phase_timeout_cnt > 450)
                {
                    jump_leg_cnt[LEFT] = 0;
                    jump_leg_cnt[RIGHT] = 0;
                    phase_timeout_cnt = 0;
                    fly_init_swing_single = 0.5f * (leg_state[LEFT].phi0 + leg_state[RIGHT].phi0);
                    while (fly_init_swing_single > PI) fly_init_swing_single -= 2.0f * PI;
                    while (fly_init_swing_single < -PI) fly_init_swing_single += 2.0f * PI;
                    jump_phase_cnt = app::jump_phase_e::TAKE_OFF;
                }
                break;
            case app::jump_phase_e::TAKE_OFF:
                {
                    robot_state.state_flag[PRESENT] = app::process_e::LQR_ON;
                    float takeoff_ll = (jump_level_latched == 2) ? TAKE_OFF_LL_HIGH : TAKE_OFF_LL_LOW;
                    takeoff_ll = module::clip(takeoff_ll + 0.02f, 0.20f, 0.40f);
                    robot_state.LL_FSM_Want[LEFT] = takeoff_ll;
                    robot_state.LL_FSM_Want[RIGHT] = takeoff_ll;
                    phase_timeout_cnt++;

                    if ((robot_state.if_off_gnd[LEFT] == 1 && robot_state.if_off_gnd[RIGHT] == 1) && (leg_state[LEFT].L0 > 0.33f && leg_state[RIGHT].L0 > 0.33f))
                    {
                        jump_leg_cnt[LEFT]++;
                        jump_leg_cnt[RIGHT]++;
                    }
                    else
                    {
                        jump_leg_cnt[LEFT]  = (jump_leg_cnt[LEFT]  > 2) ? (jump_leg_cnt[LEFT]  - 2) : 0;
                        jump_leg_cnt[RIGHT] = (jump_leg_cnt[RIGHT] > 2) ? (jump_leg_cnt[RIGHT] - 2) : 0;
                    }

                    constexpr int TAKEOFF_MIN_PUSH_TICKS = 18;
                    constexpr int TAKEOFF_CONFIRM_TICKS = 12;
                    constexpr int TAKEOFF_MAX_PUSH_TICKS = 180;
                    const bool takeoff_ready = (phase_timeout_cnt > TAKEOFF_MIN_PUSH_TICKS) &&
                                              (jump_leg_cnt[LEFT] > TAKEOFF_CONFIRM_TICKS && jump_leg_cnt[RIGHT] > TAKEOFF_CONFIRM_TICKS);

                    if (takeoff_ready || phase_timeout_cnt > TAKEOFF_MAX_PUSH_TICKS)
                    {
                        jump_leg_cnt[LEFT] = 0;
                        jump_leg_cnt[RIGHT] = 0;
                        phase_timeout_cnt = 0;
                        ramp_leg_len[LEFT].reset(leg_state[LEFT].L0);
                        ramp_leg_len[RIGHT].reset(leg_state[RIGHT].L0);
                        jump_phase_cnt = app::jump_phase_e::FLYING;
                    }
                    break;
                }
            case app::jump_phase_e::FLYING:
                {
                robot_state.state_flag[PRESENT] = app::process_e::PID_ONLY;
                robot_state.LL_FSM_Want[LEFT] = 0.23f;
                robot_state.LL_FSM_Want[RIGHT] = 0.23f;
                phase_timeout_cnt++;

                if (robot_state.state_flag[PAST] != app::process_e::PID_ONLY)
                {
                    ramp_leg_rotate[LEFT].reset(leg_state[LEFT].phi0_total);
                    ramp_leg_rotate[RIGHT].reset(leg_state[RIGHT].phi0_total);
                }

                robot_state.rotate_FSM_Want[LEFT] = leg_state[LEFT].round_count * 2 * PI + fly_init_swing_single;
                robot_state.rotate_FSM_Want[RIGHT] = leg_state[RIGHT].round_count * 2 * PI + fly_init_swing_single;

                // 双腿腿长均小于 0.25m 认为触地
                const bool both_legs_short_fly =
                    (leg_state[LEFT].L0 < 0.20f) && (leg_state[RIGHT].L0 < 0.20f);

                static int fly_touch_cnt = 0;
                if (both_legs_short_fly)
                    fly_touch_cnt++;
                else
                    fly_touch_cnt = 0;

                if (fly_touch_cnt > 10 || phase_timeout_cnt > 1200)
                {
                    fly_touch_cnt = 0;
                    phase_timeout_cnt = 0;
                    jump_phase_cnt = app::jump_phase_e::LANDING;
                }
                break;
                }
            case app::jump_phase_e::LANDING:
            {
                robot_state.state_flag[PRESENT] = app::process_e::LQR_ON;
                robot_state.LL_STATE = app::LL_Discribe_e::LOW;
                robot_state.LL_FSM_Want[LEFT] = 0.17f;
                robot_state.LL_FSM_Want[RIGHT] = 0.17f;
                phase_timeout_cnt++;

                // 双腿腿长均小于 0.25m 认为稳定着地
                const bool both_legs_short_land =
                    (leg_state[LEFT].L0 < 0.25f) && (leg_state[RIGHT].L0 < 0.25f);

                static int land_short_cnt = 0;
                if (both_legs_short_land)
                    land_short_cnt++;
                else
                    land_short_cnt = 0;

                if ((land_short_cnt > 50) || phase_timeout_cnt > 700)
                {
                    jump_phase_cnt = app::jump_phase_e::COMPLETE;
                    land_short_cnt = 0;
                    jump_leg_cnt[LEFT] = 0;
                    jump_leg_cnt[RIGHT] = 0;
                    phase_timeout_cnt = 0;
                }
                break;
            }
            case app::jump_phase_e::COMPLETE:
                robot_state.mode[PRESENT] = app::mode_e::NORMAL;
                break;
            default:
                break;
            }
        }
        else
        {
            jump_phase_cnt = app::jump_phase_e::COMPLETE;
            jump_leg_cnt[LEFT] = 0;
            jump_leg_cnt[RIGHT] = 0;
        }

        // follow：有腿 + NORMAL 启用跟随角度控制
        if (robot_state.mode[PRESENT] == app::mode_e::NORMAL)
            robot_state.if_follow = 1;
        else
            robot_state.if_follow = 0;

        // NORMAL：底盘跟随云台 yaw 指向
        if (robot_state.mode[PRESENT] == app::mode_e::NORMAL)
        {
#ifdef NO_HEAD
            // 无头模式：用遥杆积分控制 follow_angle
            const float dt_follow = 0.001f;
            robot_state.follow_angle += robot_state.v[2] * dt_follow;

            if (robot_state.follow_angle > PI)
                robot_state.follow_angle -= 2.0f * PI;
            else if (robot_state.follow_angle < -PI)
                robot_state.follow_angle += 2.0f * PI;
#else
            bool is_flipped = data_to_chassis.if_turn ^ robot_state.is_inversed;
            float head_offset = is_flipped ? PI : 0.0f;
            robot_state.follow_angle = module::rad_format(wbc_state.phi + data_to_chassis.real_angle + head_offset);
#endif
        }

        switch (robot_state.mode[PRESENT])

        {
        case app::mode_e::NORMAL:
                robot_state.L0_speed = 0.0002f;
            break;
        case app::mode_e::SLOW_START:
            robot_state.L0_speed = 0.0005f;
            robot_state.rotate_speed = 0.007f;
            break;
        case app::mode_e::TANK:
            robot_state.L0_speed = 0.00015f;
            robot_state.rotate_speed = 0.001f;
            break;
        case app::mode_e::JUMP:
            robot_state.L0_speed = 0.003f;
            robot_state.rotate_speed = 0.003f;
            break;
        case app ::mode_e::SELF_HEAL:
            robot_state.L0_speed = 0.0004f;
            robot_state.rotate_speed = 0.003f;
            break;
        case app::mode_e::FLY:
            robot_state.L0_speed = 0.001f;
            robot_state.rotate_speed = 0.006f;
            break;
        case app::mode_e::TOUCH_DOWN:
            robot_state.L0_speed = 0.0005f;
            robot_state.rotate_speed = 0.006f;
            break;
        case app::mode_e::UP_STAIR:
            robot_state.L0_speed = 0.0004f;
            robot_state.rotate_speed = 0.0055f;
            break;
        default:
            break;
        }

        balance_.wbc_state.LL_want[LEFT] = ramp_leg_len[LEFT](robot_state.LL_FSM_Want[LEFT], robot_state.L0_speed);
        balance_.wbc_state.LL_want[RIGHT] = ramp_leg_len[RIGHT](robot_state.LL_FSM_Want[RIGHT], robot_state.L0_speed);
        balance_.wbc_state.rotate_angle[LEFT] = ramp_leg_rotate[LEFT](robot_state.rotate_FSM_Want[LEFT], robot_state.rotate_speed);
        balance_.wbc_state.rotate_angle[RIGHT] = ramp_leg_rotate[RIGHT](robot_state.rotate_FSM_Want[RIGHT], robot_state.rotate_speed);

        if (leg_change_lock_cnt > 0) leg_change_lock_cnt--;

        if (chassis_ctrl_ != nullptr)
        {
            static int cnt = 0;
            if (cnt++ > 10)
            {
                cnt = 0;
                chassis_ctrl_->IBC_Send_Pack4(balance_.leg_state[LEFT].L0, balance_.leg_state[RIGHT].L0,
                                         this->is_fatal_error,
                                         balance_.leg_state[LEFT].phi0, balance_.leg_state[RIGHT].phi0,
                                         balance_.wbc_state.phi, balance_.wbc_state.thetab,
                                         static_cast<uint8_t>(robot_state.mode[PRESENT]));
            }
        }
    }

    int Chassis_Balance_Status_Handle::leg_state_key_check(int key)
    {
        static int leg_key_last = 0;
        bool leg_key_edge = (key != leg_key_last);
        leg_key_last = key;
        return leg_key_edge ? 1 : 0;
    }

    int Chassis_Balance_Status_Handle::jump_state_key_check(int key)
    {
        int past_val = robot_state.jump_cnt[PRESENT]; // 记录上一次的值
        if (key)
            robot_state.jump_cnt[PRESENT]++;
        else
            robot_state.jump_cnt[PRESENT] = 0;

        if (past_val > 400 && robot_state.jump_cnt[PRESENT] == 0)
            return 2;
        else if (past_val > 40 && robot_state.jump_cnt[PRESENT] == 0 && !fsm_state.init_flag)
            return 1;
        return 0;
    }

    float smooth_saturation(float x, float width, float smoothness)
    {
        float y;
        if (fabs(x) < width)
        {
            y = 1.0;
        }
        else
        {
            y = exp(-smoothness * pow(fabs(x) - width, 2));
        }
        return y;
    }

    float torque_to_current(float torque)
    {
        if (fabsf(torque) < 0.02f) return 0.0f;

        float coulomb = 0.3703f * tanhf(torque / 0.08f);
        float current = -(45.4049f * (torque / REDUCTION_RATIO_WHEEL) + coulomb) * 819.2f;
        return current;
    }

    float Chassis_Balance::spring_force_estimation(int leg)
    {
        float Fv;
        Fv = -1.8433e+03f * leg_state[leg].L0 * leg_state[leg].L0 + 1.1122e+03f * leg_state[leg].L0 - 49.6586f + 24.0f;
        return Fv;
    }
    void Chassis_Balance_Ctrl::leso(const float *Ad, const float *Bd, float real_state[10], float omega_o)
    {
        const robot_state_s &robot_state = status_handle_.robot_state;

        {
            bool need_init = true;
            for (int i = 0; i < 10; ++i)
            {
                if (x_hat_leso[i] != 0.0f) { need_init = false; break; }
            }
            if (need_init)
            {
                for (int i = 0; i < 10; ++i)
                    x_hat_leso[i] = real_state[i];
            }
        }

        last_u_control[0] = output_last[LEFT].Tw;
        last_u_control[1] = output_last[RIGHT].Tw;
        last_u_control[2] = output_last[LEFT].Tl;
        last_u_control[3] = output_last[RIGHT].Tl;

        if (!pinv_cached || ++pinv_update_counter >= PINV_UPDATE_INTERVAL)
        {
            pinv_update_counter = 0;
            pinv_cached = true;

            float B_continuous[10][4];
            float B_pinv[4][10];
            constexpr float inv_dt = 1.0f / d_t;
            for (int i = 0; i < 10; ++i)
                for (int j = 0; j < 4; ++j) B_continuous[i][j] = Bd[i * 4 + j] * inv_dt;
            module::pinv_B_matrix(B_continuous, B_pinv);

            for (int i = 0; i < 4; ++i)
                for (int j = 0; j < 10; ++j)
                    leso_B_pinv_buf[i * 10 + j] = B_pinv[i][j];
        }

        for (int i = 0; i < 10; ++i)
            leso_ex_buf[i] = real_state[i] - x_hat_leso[i];

        {
            arm_matrix_instance_f32 Bpinv_m, ex_m, s_m;
            arm_mat_init_f32(&Bpinv_m, 4, 10, leso_B_pinv_buf);
            arm_mat_init_f32(&ex_m, 10, 1, leso_ex_buf);
            arm_mat_init_f32(&s_m, 4, 1, leso_s_buf);
            arm_mat_mult_f32(&Bpinv_m, &ex_m, &s_m);
        }

        {
            float k1_sqrt_omega;
            arm_sqrt_f32(omega_o, &k1_sqrt_omega);
            const float lambda1 = 1.5f * k1_sqrt_omega;  // ≈ 8.2 @ ω₀=30
            const float lambda2 = 1.1f * omega_o;         // ≈ 33  @ ω₀=30
            constexpr float delta1 = 0.01f;                // fal(s,0.5) 线性区宽度
            constexpr float delta2 = 0.01f;                // fal(s,0)   线性区宽度

            float L_scaled[10];
            for (int i = 0; i < 10; ++i)
            {
                float base = (i % 2 == 1) ? 0.5f * omega_o : 0.3f * omega_o;
                L_scaled[i] = base * d_t;
            }

            float u_total[4];
            for (int i = 0; i < 4; ++i)
            {
                float s_i = leso_s_buf[i];
                float z1_corr = lambda1 * module::fal(s_i, 0.5f, delta1);
                d_hat_leso[i] += lambda2 * module::fal(s_i, 0.0f, delta2) * d_t;
                u_total[i] = last_u_control[i] + d_hat_leso[i] + z1_corr;
            }

            arm_matrix_instance_f32 A_m, xhat_m, Ax_m;
            arm_mat_init_f32(&A_m, 10, 10, const_cast<float *>(Ad));
            arm_mat_init_f32(&xhat_m, 10, 1, x_hat_leso);
            arm_mat_init_f32(&Ax_m, 10, 1, leso_Axhat_buf);
            arm_mat_mult_f32(&A_m, &xhat_m, &Ax_m);

            arm_matrix_instance_f32 B_m, ut_m, Bu_m;
            arm_mat_init_f32(&B_m, 10, 4, const_cast<float *>(Bd));
            arm_mat_init_f32(&ut_m, 4, 1, u_total);
            arm_mat_init_f32(&Bu_m, 10, 1, leso_Bu_total_buf);
            arm_mat_mult_f32(&B_m, &ut_m, &Bu_m);

            for (int i = 0; i < 10; ++i)
                x_hat_leso[i] = leso_Axhat_buf[i] + leso_Bu_total_buf[i] + L_scaled[i] * leso_ex_buf[i];
        }

        if (robot_state.if_off_gnd[LEFT] != 0)
        {
            d_hat_leso[0] *= 0.85f;
        }
        if (robot_state.if_off_gnd[RIGHT] != 0)
        {
            d_hat_leso[1] *= 0.85f;
        }

        {
            float u_leso[4] = {0};

            for (int i = 0; i < 4; ++i)
            {
                u_leso[i] = -d_hat_leso[i];

                if (fabsf(u_leso[i]) < 0.01f)
                    u_leso[i] = 0.0f;

                float limit = (i < 2) ? leso_limit_w : leso_limit_l;
                u_leso[i] = module::abs_clip(u_leso[i], limit);
            }

            if (robot_state.if_off_gnd[LEFT] != 0)
                u_leso[0] = 0.0f;
            if (robot_state.if_off_gnd[RIGHT] != 0)
                u_leso[1] = 0.0f;

            output[LEFT].Tw += u_leso[0];
            output[RIGHT].Tw += u_leso[1];
            output[LEFT].Tl += u_leso[2];
            output[RIGHT].Tl += u_leso[3];
        }
    }

    void Chassis_Balance_Ctrl::reset_leso()
    {
        for (int i = 0; i < 4; ++i)
        {
            d_hat_leso[i] = 0.0f;
        }
        for (int i = 0; i < 10; ++i)
        {
            x_hat_leso[i] = 0.0f;  // 置零 → 下拍 leso() 首调用自动 init 到真实状态
        }
        for (int i = 0; i < 4; ++i)
        {
            last_u_control[i] = 0.0f;
        }
        pinv_cached = false;
        pinv_update_counter = 0;
    }

    inline void Chassis_Balance_Status_Handle::if_off_gnd_write_in(int leg, int flag)
    {
        bool if_leg_angle_valid = fabs(balance_.leg_state[LEFT].phi0 - (PI / 2 - pitch_preset)) < 0.28 && fabs(balance_.leg_state[RIGHT].phi0 - (PI / 2 - pitch_preset)) < 0.28;
        if (robot_state.mode[PRESENT] == app::mode_e::TOUCH_DOWN || normal_init_lock_cnt > 0 || leg_change_lock_cnt > 0 || !if_leg_angle_valid) robot_state.if_off_gnd[leg] = 0;
        else robot_state.if_off_gnd[leg] = flag;
    }
}
