#pragma once

#include "ffc.hpp" // [harness] FFC_Init_Config_s
// [harness] removed "FreeRTOS.h"
// [harness] removed "task.h"
// [harness] removed "cmsis_os.h"
// [harness] removed "DJI_motor.hpp"
// [harness] removed "gimbal.hpp"
#include "message_center.hpp"
#include "balance_algorithm.hpp"

inline module::PID_Init_Config_s leg_L_pid = {
    // 腿长pid
    600,                  // MaxOutput
    5,                    // IntegralLimit
    2800,                 // Kp
    0,                    // Ki
    60,                   // Kd
    2,                    // Integral_Max
    0,                    // Integral_Min
    0.001,                // Output_LPF
    0.001,                // D_LPF
    0,                    // gama
    0x3f,                 // Improve
    module::POSITION_PID, // pid_mode

    0, // max_err
    0, // deadband

};

inline module::PID_Init_Config_s leg_L_pid_soft = {
    200,                  // MaxOutput
    5,                    // IntegralLimit
    800,                 // Kp
    0,                    // Ki
    250,                   // Kd
    2,                    // Integral_Max
    0,                    // Integral_Min
    0.001,                // Output_LPF
    0.001,                // D_LPF
    0,                    // gama
    0x3f,                 // Improve
    module::POSITION_PID, // pid_mode

    0, // max_err
    0, // deadband

};

inline module::PID_Init_Config_s turn_pid = {
    // 转向pid
    4,                    // MaxOutput
    5,                    // IntegralLimit
    7,                    // Kp
    0,                    // Ki
    2,                    // Kd
    2,                    // Integral_Max
    0,                    // Integral_Min
    0.001,                // Output_LPF
    0.001,                // D_LPF
    0,                    // gama
    0x3f,                 // Improve
    module::POSITION_PID, // pid_mode

    0, // max_err
    0, // deadband

};

inline module::PID_Init_Config_s leg_roll_pid = {
    // roll角pid
    30000,                // MaxOutput
    8000,                 // IntegralLimit
    3500,                 // Kp
    9000,                 // Ki
    0.1,                  // Kd
    20000,                // Integral_Max
    0,                    // Integral_Min
    0.001,                // Output_LPF
    0.001,                // D_LPF
    0,                    // gama
    0x3f,                 // Improve
    module::POSITION_PID, // pid_mode

    0, // max_err
    0, // deadband

};

inline module::PID_Init_Config_s leg_rotate_pid = {
    // 摆动腿pid
    30,                   // MaxOutput
    20,                   // IntegralLimit
    250,                  // Kp
    0,                    // Ki
    7,                    // Kd
    2,                    // Integral_Max
    0,                    // Integral_Min
    0.001,                // Output_LPF
    0.001,                // D_LPF
    0,                    // gama
    0x3f,                 // Improve
    module::POSITION_PID, // pid_mode

    0, // max_err
    0, // deadband

};

inline module::PID_Init_Config_s wheel_current_pid_l = {
    // 左轮电流环pid
    16384,                   // MaxOutput
    8000,                   // IntegralLimit
    1.3,                  // Kp
    5,                  // Ki
    0,                    // Kd
    5,                    // Integral_Max
    -5,                   // Integral_Min
    0.001,                // Output_LPF
    0.001,                // D_LPF
    0,                    // gama
    0x00,                 // Improve
    module::POSITION_PID, // pid_mode

    0, // max_err
    0, // deadband

};

inline module::PID_Init_Config_s wheel_current_pid_r = {
    // 右轮电流环pid
    16384,                   // MaxOutput
    8000,                   // IntegralLimit
    1.3,                  // Kp
    5,                  // Ki
    0,                    // Kd
    5,                    // Integral_Max
    -5,                   // Integral_Min
    0.001,                // Output_LPF
    0.001,                // D_LPF
    0,                    // gama
    0x00,                 // Improve
    module::POSITION_PID, // pid_mode

    0, // max_err
    0, // deadband

};

inline module::Impedance_Controller_Init_Config_s ip_pitch_config = {
    1.0f, // Kp
    0.01f,   // Kd
    0.06f,   // alpha
};

inline module::Impedance_Controller_Init_Config_s ip_roll_config = {
    1.0f, // Kp
    0.01f,   // Kd
    0.06f,   // alpha
};

inline module::Impedance_Controller_Init_Config_s ip_land_ll_config = {
    1000.0f, // Kp
    30.0f,   // Kd
    0.06f,   // alpha
};

inline module::Impedance_Controller_Init_Config_s ip_take_off_F_config = {
    0.0f, // Kp
    0.0f,   // Kd
    0.06f,   // alpha
};

inline module::Impedance_Controller_Init_Config_s ip_pitch_preset_config = {
    0.1f, // Kp
    0.05f,   // Kd
    0.06f,   // alpha
};

extern int16_t SHOOT_DIR;
extern float REDUCTION_RATIO_LOADER;
extern int16_t NUM_PER_CIRCLE;

extern int16_t BLOCK_MAX_OUT;
extern int16_t BLOCK_MIN_SPEED;
extern int16_t BLOCK_MAX_TIME;

extern float PITCH_UP;
extern float PITCH_DN;

extern int16_t shoot_frq_set;
extern int16_t shoot_set_speed;

extern float REDUCE_RATE;
extern float RECOVER_RATE;
extern float TIME_INTERVAL;

namespace app
{
    struct stribeck_s
    {
        float sigma; // 粘滞系数
        float Fc;    // 库仑摩擦
        float Fs;    // 静摩擦
        float vs;    // Stribeck 速度
    }; // 斯特里克模型参数

    struct TD_Config_s
    {
        float r;  // 快速因子：决定跟踪速度
        float h;  // 积分步长：通常等于控制周期
        float h0; // 滤波因子：决定平滑程度
    };

    inline constexpr float G = 9.8f;
    inline constexpr int PAST = 1;
    inline constexpr int PRESENT = 0;
    inline constexpr float L1 = 0.21f;
    inline constexpr float L2 = 0.25f;
    inline constexpr float L5 = 0;
    inline constexpr float Wheel_R = 0.06f;
    inline constexpr float M = 18.0f; // 机体质量
    inline constexpr float I_b = 0.353848333f; // 机身绕俯仰轴的转动惯量
    inline constexpr float I_z = 0.2657f; // 机身绕Z轴（航向）转动惯量
    inline constexpr float I_w = 4.98E-04f; // 轮子绕轴心的转动惯量
    inline constexpr float M_w = 0.25867f; // 轮质量
    inline constexpr float M_l = 2.6f; // 腿部质量
    inline constexpr float L_b = 0.13f; // 机身质心到髋关节的距离
    inline constexpr float pitch_preset{0.11f};
    inline constexpr float REDUCTION_RATIO_WHEEL = 13.94f;
    inline constexpr float leg_angle_offset = 1.163f; // 腿电机零位偏移角度
    inline constexpr float K_T_TOTAL = 0.02f * REDUCTION_RATIO_WHEEL * 1.2f;
    inline constexpr float d_t = 0.001f;
    // LESO 扰动观测器参数 (Linear Extended State Observer)
    inline constexpr float leso_omega_o = 30.0f;   // LESO 带宽 (rad/s), τ=20ms
    inline constexpr float leso_limit_l = 5.0f;     // 腿部扰动补偿限幅 (Nm)
    inline constexpr float leso_limit_w = 2.0f;     // 轮部扰动补偿限幅 (Nm)
    // TD 参数
    inline constexpr float r = 4310.0f;    // 快速因子：决定跟踪速度
    inline constexpr float h = 0.001;     // 积分步长：通常等于控制周期
    inline constexpr float h0 = 3.0f * h; // 滤波因子：决定平滑程度
    inline constexpr float r_input = 25.0f;
    inline constexpr float h_input = 0.001f;
    inline constexpr float h0_input = 10.0f * h_input;
    inline constexpr float r_break = 3.0f;
    inline constexpr float h_break = 0.001f;
    inline constexpr float h0_break = 3.0f * h_break;
    // 气弹簧解算参数
    inline constexpr float K_spring = 1875.0f; // N/m
    inline constexpr float F_prime = 250.0f;        // 气弹簧初始弹力 N
    inline constexpr float Ls = 0.24f;         // 弹簧原长 m
    inline constexpr float S1 = 0.165f;
    // 减速箱摩擦力补偿参数
    inline constexpr stribeck_s friction_data[2] = {
        {0.5756f, 207.4161f, 184.1962f, 0.0124f}, 
        {0.6527f, 210.0311f, 170.1155f, 0.0115f}
    };
    // 功率限制与 QP 参数
    inline constexpr float T_MAX_WHEEL = 5.0f;    // 轮电机最大转矩 (Nm)
    inline constexpr float T_MAX_LEG = 40.0f;      // 腿部电机最大转矩 (Nm)
    inline constexpr float ROLL_INERTIA = 0.35f;   // 横滚轴等效转动惯量 (kg*m^2)，需实机辨识
    inline constexpr float ROLL_FORCE_ARM = 0.20f; // 单腿法向力到横滚轴的力臂 (m)
    inline constexpr float ROLL_DF_MAX = 80.0f;    // Roll MPC 单腿差动力上限 (N)
    // 溜车刹车增强：通过 pitch_preset 用 PD 控制 pitch 角来利用重力减速
    inline constexpr float COAST_PITCH_MIN_SPD = 0.05f; // 启用溜车 pitch 控制的最小速度
    inline constexpr float P_MAX_CHASSIS = 120.0f; // 底盘总功率限制 (W)
    inline constexpr float P_STATIC_CONST = 10.0f; // 静态功率常量
    // 跳跃参数
    inline constexpr float K_v_x = 0.03f; // 着陆角常量 alpha = K_v_x * v_x
    inline constexpr float LOW_JUMP_HEIGHT = 0.15f; // 低跳高度阈值
    inline constexpr float HIGH_JUMP_HEIGHT = 0.35f; // 高跳高度阈值
    inline constexpr float PRIME_LL = 0.18f; // 起跳时腿长预设值
    inline constexpr float TAKE_OFF_LL_HIGH = 0.35f; // 起大跳时腿长
    inline constexpr float TAKE_OFF_LL_LOW = 0.28f;// 起小跳时腿长
    inline constexpr float LANDING_LL = 0.23f; // 着陆时腿长
    inline constexpr float T_push = 0.60f;
    // 离地检测参数
    inline constexpr float GND_OFF_FORCE_THRES_RATIO = 0.07f; // 受力计算置信度的阈值系数
    inline constexpr float GND_OFF_FORCE_WEIGHT = 0.6f;      // 受力置信度权重
    inline constexpr float GND_OFF_DFORCE_THRES = -250.0f;   // 接触力变化率阈值 dP/dt (N/s)
    inline constexpr float GND_OFF_DFORCE_SCALE = 1200.0f;   // 接触力变化率置信度缩放系数
    inline constexpr float GND_OFF_DFORCE_WEIGHT = 0.4f;     // 接触力变化率置信度权重
    inline constexpr float GND_OFF_DFORCE_DEADBAND = 120.0f; // 接触力变化率死区，抑制高频抖动
    inline constexpr float GND_OFF_DFORCE_GATE_RATIO = 0.25f;// dP 判据门控接触力阈值比例
    inline constexpr float GND_OFF_CONFIDENCE_THRES = 0.55f;  // 综合置信度离地阈值
    inline constexpr float GND_OFF_ABS_FORCE_RATIO = 0.28f;  // 绝对离地受力阈值系数
    inline constexpr float GND_TOUCH_ABS_FORCE_RATIO = 0.5f; // 绝对触地受力阈值系数
}

namespace module
{
            inline float K_out[40][6] = {
        {-4.738464565, -35.48929201, 22.4123119, 40.08935945, -12.84990564, -12.39927838},
        {-4.738464565, 22.4123119, -35.48929201, -12.39927839, -12.84990563, 40.08935945},
        {13.3357285, -3.619325205, -30.47089316, -29.08588693, 45.14292544, 17.08371523},
        {13.3357285, -30.47089317, -3.619325201, 17.08371524, 45.14292544, -29.08588693},
        {-14.56227574, -76.83728071, 61.90829484, 103.952001, -62.02072838, -25.90283072},
        {-14.56227574, 61.90829484, -76.83728072, -25.90283074, -62.02072836, 103.952001},
        {35.91257223, -14.50584963, -91.77784993, -76.75033763, 147.6017822, 44.04513751},
        {35.91257223, -91.77784995, -14.50584962, 44.04513752, 147.6017822, -76.75033764},
        {-10.77793501, 36.74789921, -7.898636818, -37.10370643, 4.087919741, 8.427497263},
        {10.77793501, 7.898636818, -36.74789921, -8.427497263, -4.087919741, 37.10370643},
        {-16.34757544, -21.06282289, -25.84248651, 32.58427433, -9.728067745, 33.06868254},
        {16.34757544, 25.84248651, 21.06282289, -33.06868254, 9.728067745, -32.58427433},
        {-2.460235388, 12.0359731, -6.347638933, -6.804579488, -7.288001393, 9.628151592},
        {2.460235388, 6.347638933, -12.0359731, -9.628151593, 7.288001394, 6.804579488},
        {-3.664646346, -11.65047054, -2.282799712, 14.84289599, -5.905726605, 3.310094169},
        {3.664646346, 2.282799712, 11.65047054, -3.310094169, 5.905726604, -14.84289599},
        {-102.8730487, -324.3761187, 10.56947247, 353.8597196, 5.615435098, -10.12566386},
        {-7.478535625, -11.98482455, 22.58457476, 9.831967628, -31.62095557, 8.173789065},
        {311.7944415, -763.2486862, 1.644711033, 705.5723552, 13.26841994, -4.957810899},
        {-11.01339901, 20.43345274, -37.84247547, 18.05864449, -21.75033795, 43.1662546},
        {-1.571362813, -18.63142747, 4.128647442, 1.656629054, 4.396259863, -2.395021772},
        {-0.184379972, -2.694228475, -5.177189961, 11.03752215, -26.87241523, 13.71172887},
        {5.943825534, 1.135536147, -7.418706477, -8.209721814, 6.290864415, 4.520674179},
        {0.6502476682, 5.632119689, 0.05637656629, -6.721887541, 13.19039988, -10.45956984},
        {-7.478535624, 22.58457476, -11.98482456, 8.173789064, -31.62095557, 9.831967629},
        {-102.8730487, 10.56947247, -324.3761188, -10.12566386, 5.615435103, 353.8597196},
        {-11.01339901, -37.84247547, 20.43345274, 43.1662546, -21.75033795, 18.05864449},
        {311.7944415, 1.644711032, -763.2486862, -4.957810897, 13.26841994, 705.5723552},
        {-0.184379972, -5.17718996, -2.694228476, 13.71172887, -26.87241523, 11.03752215},
        {-1.571362813, 4.128647442, -18.63142748, -2.395021774, 4.396259868, 1.656629053},
        {0.6502476682, 0.05637656527, 5.63211969, -10.45956984, 13.19039987, -6.721887541},
        {5.943825534, -7.418706478, 1.135536148, 4.520674181, 6.290864414, -8.209721815},
        {-72.9084549, 147.8267625, 53.06502592, -106.2195152, -49.76181595, -45.18271251},
        {-72.9084549, 53.06502592, 147.8267625, -45.18271251, -49.76181595, -106.2195152},
        {-131.2260712, -457.4213252, 186.006187, 530.9511112, 12.72518922, -209.7216505},
        {-131.2260712, 186.006187, -457.4213252, -209.7216505, 12.72518922, 530.9511112},
        {-6.062397149, 5.461053664, 12.2863092, 1.162763329, -10.90816867, -9.270738248},
        {-6.062397149, 12.2863092, 5.461053663, -9.270738249, -10.90816867, 1.162763329},
        {-3.588208682, -28.80545191, 3.450793468, 24.21027099, 15.9338755, -8.007485217},
        {-3.588208682, 3.450793467, -28.80545191, -8.007485216, 15.9338755, 24.21027099}};
    inline float A_out[100][6] = {
        {0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0},
        {1, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0},
        {1, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0},
        {4.819386772, -222.8343713, 12.38551122, 206.3881185, 25.98810142, -18.24162121},
        {0, 0, 0, 0, 0, 0},
        {11.4460204, -236.7685658, -12.2872938, 226.1356798, -37.1267979, 20.05495111},
        {0, 0, 0, 0, 0, 0},
        {314.7609814, 328.1270446, 10.72575541, -1043.167254, 18.88849787, -14.45632069},
        {0, 0, 0, 0, 0, 0},
        {12.86496366, -190.7629944, -55.09704449, 164.9478391, 49.99616049, 107.0351695},
        {0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0},
        {1, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0},
        {4.819386772, 12.38551122, -222.8343713, -18.24162121, 25.98810142, 206.3881185},
        {0, 0, 0, 0, 0, 0},
        {-11.4460204, 12.2872938, 236.7685658, -20.05495111, 37.1267979, -226.1356798},
        {0, 0, 0, 0, 0, 0},
        {12.86496366, -55.09704449, -190.7629944, 107.0351695, 49.99616049, 164.9478391},
        {0, 0, 0, 0, 0, 0},
        {314.7609814, 10.72575541, 328.1270446, -14.45632069, 18.88849787, -1043.167254},
        {0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0},
        {1, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0},
        {1, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0}};
    inline float B_out[40][6] = {
        {0, 0, 0, 0, 0, 0},
        {1.539662268, 34.05472217, -0.7718288514, -34.75700564, -3.733745321, 1.711815351},
        {0, 0, 0, 0, 0, 0},
        {-7.02205891, 37.51604694, 0.7136681384, -39.48240348, 4.992414823, -1.898526959},
        {0, 0, 0, 0, 0, 0},
        {-90.24044175, 118.0559265, -0.6796106164, -37.0708655, -2.855771562, 1.357196824},
        {0, 0, 0, 0, 0, 0},
        {-3.206500381, 29.50378752, 4.963616391, -28.38912835, -7.020463388, -9.01818268},
        {0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0},
        {1.539662268, -0.7718288514, 34.05472217, 1.711815351, -3.733745321, -34.75700564},
        {0, 0, 0, 0, 0, 0},
        {7.02205891, -0.7136681384, -37.51604694, 1.898526959, -4.992414823, 39.48240348},
        {0, 0, 0, 0, 0, 0},
        {-3.206500381, 4.963616391, 29.50378752, -9.01818268, -7.020463388, -28.38912835},
        {0, 0, 0, 0, 0, 0},
        {-90.24044175, -0.6796106164, 118.0559265, 1.357196824, -2.855771562, -37.0708655},
        {0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0},
        {-1.534103655, -1.278458947, 0.8430217154, 4.383424676, -0.3225513965, -0.7046743147},
        {0, 0, 0, 0, 0, 0},
        {-1.123323223, -2.663859364, -0.8991553776, 6.184595286, 0.1446368918, 0.7663844977},
        {0, 0, 0, 0, 0, 0},
        {43.25367394, -164.19132, 0.7200680261, 188.0982149, -0.3682012218, -0.5622014704},
        {0, 0, 0, 0, 0, 0},
        {-0.9435202868, -1.443521085, -1.443521085, 4.104989175, -0.5072770468, 4.104989175},
        {0, 0, 0, 0, 0, 0},
        {-2.826069552, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0},
        {-1.534103655, 0.8430217154, -1.278458947, -0.7046743147, -0.3225513965, 4.383424676},
        {0, 0, 0, 0, 0, 0},
        {1.123323223, 0.8991553776, 2.663859364, -0.7663844977, -0.1446368918, -6.184595286},
        {0, 0, 0, 0, 0, 0},
        {-0.9435202868, -1.443521085, -1.443521085, 4.104989175, -0.5072770468, 4.104989175},
        {0, 0, 0, 0, 0, 0},
        {43.25367394, 0.7200680261, -164.19132, -0.5622014704, -0.3682012218, 188.0982149},
        {0, 0, 0, 0, 0, 0},
        {-2.826069552, 0, 0, 0, 0, 0}};
}
/*
            
    q_diag = [5, 300, 300, 10, 40000, 18, 40000, 18, 17000, 24]
    r_diag = [1, 1, 0.25, 0.25]

*/

extern module::PID_Init_Config_s gimbal_yaw_pid[2];
extern module::PID_Init_Config_s gimbal_pitch_pid[2];
extern module::FFC_Init_Config_s gimbal_yaw_ffc;

extern module::PID_Init_Config_s gun1_pid[2];
extern module::PID_Init_Config_s gun2_pid[2];
extern module::PID_Init_Config_s dial_cartridge_pid[2];
// [harness] #endif (BALANCE)
