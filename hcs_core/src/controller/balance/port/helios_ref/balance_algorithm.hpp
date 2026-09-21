#pragma once

#include <stdint.h>
#include "module_def.hpp"

namespace module
{

    void lqr_K(float LL, float LR, float K_Fit_Coefficients[40][6],
               float K_matrix[4][10]);
    void lqr_A(float LL, float LR, float A_Fit_Coefficients[100][6],
               float A_matrix[10][10]);
    void lqr_B(float LL, float LR, float B_Fit_Coefficients[40][6],
               float B_matrix[10][4]);

    void leg_pos(float phi1, float phi4, float l1, float l2, float l5, float dphi1, float dphi4,
                 float &l0, float &phi0, float &dl0, float &dphi0, float &phi2, float &phi3);
    void calculate_status_vector(float K_matrix[4][10], float Status_vector[10], float &Tlwl, float &Tlwr, float &Tbll, float &Tblr);
    void calculate_status_vector_fly(float K_matrix[4][10], float Status_vector[10], float &Tlwl, float &Tlwr, float &Tbll, float &Tblr, bool left, bool right, float improve0);
    void leg_VMC(float phi0, float phi1, float phi2, float phi3, float phi4, float l1, float l0, float F, float Tp, float &T1, float &T2);
    void inverse_contact_force(float phi0, float phi1, float phi2, float phi3, float phi4,float l1, float l0,float T1, float T2,float &F_estimated, float &Tp_estimated, int side);
    void pinv_B_matrix(float B_matrix[10][4], float pinv_B[4][10]);
    struct TD_Config_s
    {
        float r;  // 快速因子
        float h;  // 积分步长
        float h0; // 滤波因子
    };

    struct KF_s
    {
        float cov = 100.0f;
        float v_est = 0.0f;
        float acc_last = 0.0f;

        static constexpr float Q = 1.3f;
        static constexpr float R_base = 400.0f;
        static constexpr float R_max = 1500.0f;
        static constexpr float R_high_speed_min = 5.0f;
        static constexpr float high_speed_thres = 1.5f;
        static constexpr float cov_min = 0.01f;
        static constexpr float cov_max = 100.0f;
        static constexpr float wheel_half_track = 0.2296f;
        static constexpr float impact_p_delta_thres = 30.0f;

        // 新增：工况优化参数
        static constexpr float impact_cooldown_time = 0.15f;  // 冲击冷却时间 150ms
        static constexpr float off_gnd_debounce_time = 0.05f; // 离地防抖时间 50ms
        static constexpr float air_decay_rate = 1.0f;         // 空中速度衰减率 (指数)
        static constexpr float zupt_decay_rate = 10.5f;       // 零偏校准衰减率 (指数)

        // 堵转检测参数
        static constexpr float stall_wheel_spd_thres = 0.1f;
        static constexpr float stall_wheel_drop_thres = 2.0f;
        static constexpr float stall_recover_time = 0.5f;

        // 残差自适应参数
        static constexpr float resid_ema_alpha = 0.99f;  // 残差方差 EMA 系数, ~100ms 窗口@1kHz
        static constexpr float R_swing_gain = 80.0f;     // 腿摆动 R 前馈增益
        static constexpr float R_resid_min = 1.0f;       // Sage-Husa R 估计下限

        bool wheel_stall[2] = {false, false};
        float last_wheel_spd[2] = {0.0f, 0.0f};
        float stall_recover_cnt[2] = {0.0f, 0.0f};
        float last_leg_p[2] = {0.0f, 0.0f};

        // 新增：内部状态计时器
        float off_gnd_timer[2] = {0.0f, 0.0f};
        float impact_timer = 0.0f;

        // 残差自适应状态 (基于上一步后验残差)
        float resid_sq_sum_l = 0.0f; // 左轮残差方差 EMA
        float resid_sq_sum_r = 0.0f; // 右轮残差方差 EMA
        float cov_post = 100.0f;     // 上一步后验协方差 P+, 用于残差法 R 估计
        bool resid_valid = false;    // 上一步残差是否有效(是否完成过一次更新)

        uint32_t dwt_count_KF = 0;

        void reset()
        {
            cov = 100.0f;
            v_est = 0.0f;
            acc_last = 0.0f;
            wheel_stall[0] = false;
            wheel_stall[1] = false;
            last_wheel_spd[0] = 0.0f;
            last_wheel_spd[1] = 0.0f;
            stall_recover_cnt[0] = 0.0f;
            stall_recover_cnt[1] = 0.0f;
            last_leg_p[0] = 0.0f;
            last_leg_p[1] = 0.0f;

            off_gnd_timer[0] = 0.0f;
            off_gnd_timer[1] = 0.0f;
            impact_timer = 0.0f;
            resid_sq_sum_l = 0.0f;
            resid_sq_sum_r = 0.0f;
            cov_post = 100.0f;
            resid_valid = false;
        }
    };

    static KF_s kf_state;

    struct Impedance_Init_Config_s
    {
        float Kp;
        float Kd;
        float alpha; // 速度滤波
    };

    struct Trajectory_Point_s
    {
        float pos;
        float vel;
        float acc;
    };

    struct Impedance_Controller_Init_Config_s
    {
        float Kp;
        float Kd;
        float alpha; // 速度滤波
    };

    class TD
    {
    public:
        TD() : x1(0.0f), x2(0.0f) {}
        float update(float input, const TD_Config_s &config, float delta_u = 0.0f);
        void reset();
        float get_derivative(); // 获取微分信号

        float x1; // 跟踪信号
        float x2; // 微分信号
    };
    class impedance_controller // 可变参数PD阻抗控制器,用于飞行状态的姿态调整
    {
    public:
        explicit impedance_controller(Impedance_Controller_Init_Config_s &config, float *getpos, float *getvel);
        explicit impedance_controller(Impedance_Controller_Init_Config_s &config, float *getpos); // 内部微分获得vel
        float calculate(float set_pos, float set_vel);
        float calculate(const Trajectory_Point_s &traj_point);
        void reset();
        void param_update(float Kp, float Kd);
    private:
        float Kp = 0.0f;
        float Kd = 0.0f;
        float alpha = 0.96f;

        float set_pos;
        float *get_pos_ptr = nullptr;
        float set_vel;
        float *get_vel_ptr = nullptr;
        float filtered_vel;
        float last_pos = 0.0f;      // 上一帧位置，用于微分计算速度
        bool use_pos_diff = false; // 是否使用位置微分模式
        uint32_t dwt_count = 0;    // DWT计数器，用于计算dt
    };

    class trajectory_generator
    {
    public:
        void plan_trajectory(float start_pos, float end_pos, float v_lift, float duration);
        Trajectory_Point_s get_trajectory_point(float current_time);
        void reset();
    private:
        float a0, a1, a2, a3;
        float duration;
        Trajectory_Point_s current_point;
        bool is_planned = false;
    };

    void reset_kf();
    float spd_estimation(float wheel_spd_l, float wheel_spd_r, float yaw_rate_z, float acc_x,
                         float leg_p_l, float leg_p_r, bool if_off_gnd_l, bool if_off_gnd_r,
                         float leg_dphi0_l = 0.0f, float leg_dphi0_r = 0.0f);
    // 堵转检测接口
    bool get_wheel_stall_left();
    bool get_wheel_stall_right();
    void reset_kf_stall();
    void QP(float u[2], float w[2], float k1[2], float k2[2], float P_limit, float u_star[2]);
}
