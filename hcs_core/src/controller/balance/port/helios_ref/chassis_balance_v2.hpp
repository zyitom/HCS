#pragma once

#include "message_center.hpp"
#include "robot_def.hpp"
#include "module_def.hpp"
#include <functional>
#include <bitset>
#include "user_lib.hpp"
#include "IBC.hpp"
#include "fsm.hpp"
#include "pid.hpp"
#include "chassis_ctrl.hpp"
#include "rc.hpp"
#include "balance_def.hpp"
#include "balance_algorithm.hpp"
#include "rls.hpp"

#define USE_IBC_SEND 1
#define USE_IBC_RECEIVE 2
#define NO_IBC 0

#define IBC_TX_DIR_HEADER 0xAA
#define IBC_TX_MOV_HEADER 0xBB

namespace app{

    enum class mode_e
    {
        DISABLED = 0,
        NORMAL = 1,
        SLOW_START = 2,
        TANK = 3,
        UP_STAIR = 4,
        SELF_HEAL = 5,
        SPIN = 6,
        JUMP = 7,
        FLY = 8,
        TOUCH_DOWN = 9,
    }; // 状态

    enum class process_e
    {
        LQR_ON = 0,         // 正常
        DISABLED = 1,       // 无腿模式,初始状态
        PID_ONLY = 2,       // 空中，仅腿长控制
        HEALING = 3,        // 自救
    };// 控制器阶段选择

    enum class LL_Discribe_e
    {
        LOW = 0,  // 155 - 217 mm
        MID = 1,  // 217 - 278 mm
        HIGH = 2, // 278 - 400 mm
    };// 宏观腿长描述

    struct leg_state_s
    {
        float phi[4];        // 电机角度
        float dphi[4];       // 电机角速度
        float phi0, dphi0, dphi0_last, ddphi0;   // 模型腿角度
        float L0, dL0;       // 模型腿长
        float round_count;   // 多圈检测计数
        float phi0_total;    // 真实腿角度，用于pid(腿角度)
        float phi0_last;     // 多圈检测历史位置
        float wheel_spd;     // 轮速
        bool first_time = 1; // 多圈检测标志位
        float Fn = 0;        // 接触力
        float Tp = 0;        // 切向力
        float P = 0;         // 真实接触力
        float spring_force = 0; // 弹簧力估计
        float CoM_x = 0;      // 质心x位置
        float CoM_y = 0;      // 质心y位置
        float I_l = 0;      // 腿部转动惯量
    };

    struct lqr_matrix_s
    {
        float K_Matrix[4][10];
        float Status_vector[10]; // s, s1, phi, phi1, thetall, thetall1, thetalr, thetalr1, thetab, thetab1
        float Status_vector_real[10];
    };

    struct wbc_state_s
    {
        float s, s1, phi, phi1, thetall, thetall1, thetalr, thetalr1, thetab, thetab1; // lqr需要的
        float roll, roll1;                                                             // 协调控制双腿需要的
        float a_x, a_y, a_z;                                                           // 加速度
        float LL_want[2];                                                              // 期望腿长
        float LL_roll_add;                                                             // 倾斜补偿加的腿长                                                             // 腿部前倾预设
        float rotate_angle[2];                                                         // 腿旋转角度（只在初始化和自救用）
        float v_x, v_y, v_z;                                                        // 速度
    };

    struct output_s
    {
        float Tw, Tl;
        float F_want = 0;                        // 腿部期望输出力
        float spring_force[2] = {0, 0};          // 腿部弹簧力估计值
        float pid_roll;
        float final_Tl0, final_Tl1, final_Tw;    // 最终输出
    };

    struct robot_state_s
    {
        int if_off_gnd[2];          // 是否离地:0着地 1:离地 2:冲击
        bool if_slip;                // 是否打滑
        bool if_free;                // 是否自由平衡
        bool if_follow;              // 是否跟随
        bool if_rotate_move[2];      // 是否小陀螺

        mode_e mode[2];              // 0:DISABLE 1：正常 2：缓启动  3:坦克（纯轮子不平衡） 4:上台阶 5：selves 6：准备跳跃 7：跳跃 8：飞行 9：落地 10：自旋
        process_e state_flag[2];     // 控制器选择状态

        float v[3];                  // 速度
        float acc[3];                // 加速度
        int v_level[3];              // 速度档位
        float follow_angle;          // 跟随角度
        float Fn[2];                 // 接触力估计
        int dial_level;
        float s_want;
        float v_raw_target = 0.0f;   // 未经 TD 平滑的原始速度指令 (m/s)，用于启动助力判断

        float LL_FSM_Want[2];        // 状态机期望腿长
        LL_Discribe_e LL_STATE = LL_Discribe_e::LOW;
        float rotate_FSM_Want[2];    // 状态机期望旋转角度
        float rotate_speed;
        float L0_speed;
        float v_lift_FSM_Want; // 期望离地速度

        int jump_cnt[2] = {0, 0};
        int up_stair_phase = 0; // 0: 缩短腿等待, 1: 旋转到 0 度, 3: 完成

        bool is_inversed = false;
    }; // 机器人状态

    struct fsm_state_s{
        bool init_flag;
        bool fail_flag;
        float fail_wait_cnt;
    };// 状态机状态

    enum class jump_phase_e
    {
        PRESS = 0,
        TAKE_OFF = 1,
        FLYING = 2,
        LANDING = 3,
        COMPLETE = 4,
    };

    class Chassis_Balance_Ctrl;
    class Chassis_Balance_Status_Handle;

    class Chassis_Balance
    {
    public:
        Chassis_Balance(module::moto_data_receive_s *motordata_bl0,
                        module::moto_data_receive_s *motordata_bl1,
                        module::moto_data_receive_s *motordata_br0,
                        module::moto_data_receive_s *motordata_br1,
                        module::moto_data_receive_s *motordata_wl,
                        module::moto_data_receive_s *motordata_wr,
                        module::imu_data_t *imu_data_chassis);

        void Chassis_Data_Update();
        float SpeedEstimation();
        float spring_force_estimation(int leg);
        static float Multi_Turn_Detection(float &round_count, float phi, float &phi_last, bool &first_time);
        int Gnd_Off_Detect(int leg);
        inline void Fn_write_in(int leg,float estimation) {leg_state[leg].Fn = estimation;}
        inline void clear_s() {wbc_state.s = 0;}
       
        module::ramp_function_source_t leg_rotate_ramp[2] = {}; // 腿摆动期望角度的斜坡
        module::ramp_function_source_t leg_len_ramp[2] = {};    // 腿长变化斜坡

        wbc_state_s wbc_state = {};
        leg_state_s leg_state[2] = {};

        module::imu_data_t *imu_data;
        module::moto_data_receive_s *motordata_bl0;
        module::moto_data_receive_s *motordata_bl1;
        module::moto_data_receive_s *motordata_br0;
        module::moto_data_receive_s *motordata_br1;
        module::moto_data_receive_s *motordata_wl;
        module::moto_data_receive_s *motordata_wr;

    private:
        Gimbal_Upload_Data_s data_from_gimbal;

    };// 数据存储类

    class Chassis_Balance_Ctrl
    {
    public:
        lqr_matrix_s lqr_matrix = {};
        output_s output[2] = {};
        output_s output_last[2] = {};

        float L_roll_add = 0.0f;
        float F_roll_add = 0.0f;
        float v_err = 0;
        float fn[2] = {0, 0};
        bool if_jump_controller_enable = false;
        bool pwr_ctrl = false;
        float s_want{};
        module::PID *pid_LL;
        module::PID *pid_LR;
        module::PID *pid_LL_soft;
        module::PID *pid_LR_soft;
        module::PID *pid_roll;
        module::PID *pid_leg_rotateL;
        module::PID *pid_leg_rotateR;
        module::PID *pid_rotate;
        module::PID *pid_wheel_current_l;
        module::PID *pid_wheel_current_r;

        explicit Chassis_Balance_Ctrl(Chassis_Balance &balance , Chassis_Balance_Status_Handle &status_handle);
        void leso(const float *Ad, const float *Bd, float real_state[10], float omega_o);
        void reset_leso();
        void Chassis_Controller();
        void Chassis_Controller_Update();
        void Motor_Send();
        void Slip_Detect();
        float friction_compensation(float velocity, int leg);
        void Power_RLS_Update();
        inline float calc_landing_angle(float v_x) { return module::abs_clip(app::K_v_x * v_x, 0.5f); }

    private:
        Chassis_Balance &balance_;
        Chassis_Balance_Status_Handle &status_handle_;
        float d_hat_leso[4] = {0.0f};       // ST-ESO 扰动估计 d̂
        float x_hat_leso[10] = {0.0f};     // ST-ESO 状态估计 x̂
        float last_u_control[4] = {0.0f};   // 上一拍控制量
        float delta_u_w[2] = {0.0f}; 
        module::TD td_yaw;
        float adapt_leg_angle_preset = 0.0f;
        bool massive_yaw_error_flag = false; // 大偏航误差标志位

        // --- 驻车刹车 ---
        enum class park_brake_e : uint8_t
        {
            OFF = 0,     // 纯速度伺服（现状）
            WAITING = 1, // 摇杆回中，等待实际速度衰减
            ENGAGED = 2  // 位置锁定，LQR 弹簧拉回
        };
        park_brake_e park_brake_state_ = park_brake_e::OFF;
        float park_s_locked_ = 0.0f; // ENGAGED 时刻的 wbc_state.s
        float pos_filter_state_ = 0.0f; // 位移 60Hz 低通滤波状态

        float leso_B_pinv_buf[40];       // B_pinv 缓存 (4×10, 未缩放)
        float leso_Axhat_buf[10];        // A_d·x̂ 中间结果
        float leso_Bu_total_buf[10];     // B_d·(u+d̂+z₁_corr) 合并结果
        float leso_ex_buf[10];           // e_x = x − x̂ 状态估计误差
        float leso_s_buf[4];             // s = B†·e_x 滑模面
        float stab_roll = 0.0f;

        int pinv_update_counter = 0;
        static constexpr int PINV_UPDATE_INTERVAL = 5;
        bool pinv_cached = false;
        
        RLS<2> *rls_L;
        RLS<2> *rls_R;

        struct RLS_Params_s
        {
            float k1_L = 0.0f;
            float k2_L = 0.0f;
            float k1_R = 0.0f;
            float k2_R = 0.0f;
        } params;

    }; // 运动控制器类

    class Chassis_Balance_Status_Handle
    {
    public:
        fsm_state_s fsm_state = {};
        robot_state_s robot_state = {};
        
        Chassis_Ctrl_Data_s data_to_chassis = {};
        int jump_leg_cnt[2] = {0,0}; // 腿长控制用零食变量
        jump_phase_e jump_phase_cnt;
        int jump_cmd;// 1小跳，2大跳
        int jump_level_latched = 1; // 进入 JUMP 时锁存：1小跳，2大跳
        float jump_duration = 0.0f; // 跳跃持续时间计数
        bool is_saved = false; 
        bool is_fatal_error = false; 

        explicit Chassis_Balance_Status_Handle(Chassis_Balance &balance_, Chassis_Ctrl *chassis_ctrl);
        Chassis_Balance_Status_Handle(const Chassis_Balance_Status_Handle &) = delete;
        Chassis_Balance_Status_Handle &operator=(const Chassis_Balance_Status_Handle &) = delete;

        float rotate_handle(float round_count, float single_angle, float thetab);
        inline int leg_state_key_check(int key);
        inline int jump_state_key_check(int key);
        inline void if_off_gnd_write_in(int leg, int flag);
        inline void LL_FSM_Want_write_in(int leg, float LL) { robot_state.LL_FSM_Want[leg] = LL; }
        void FSM(module::RC_ctrl_t *rc_ctrl);
        float ll = 0.20f;

        int up_stair_phase = 0;
        int up_stair_timer = 0;
        int normal_init_lock_cnt = 0;
        int leg_change_lock_cnt = 0; // 手动变高腿长时锁定离地检测
        
        module::TD td_input[2];
    private:
        Chassis_Balance &balance_;
        Chassis_Ctrl *chassis_ctrl_;
        float landig_angle = 0.0f;
    };// 状态机类
    
    float smooth_saturation(float x, float width, float smoothness);
    float torque_to_current(float torque);
}
