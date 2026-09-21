#pragma once

#include <cstdint>
#include <cmath>
#include <cstring>
#include <array>
#include <functional>
#include "module_def.hpp"

namespace module
{

    // PID 模式
    enum PIDMode : uint32_t
    {
        POSITION_PID = 0,
        DELTA_PID,
    };

    // PID 改进选项
    enum PID_Improvement_t : uint8_t
    {
        NONE = 0x00,                    // 0000 0000
        Integral_Limit = 0x01,          // 0000 0001 积分限幅
        Differential_Forward = 0x02,    // 0000 0010 微分先行
        Trapezoid_Intergral = 0x04,     // 0000 0100 梯形积分
        ChangingIntegrationRate = 0x08, // 0000 1000 变速积分
        DerivativeFilter = 0x10,        // 0001 0000 微分滤波
        OutputFilter = 0x20,            // 0010 0000 输出滤波
    };

    // PID 初始化配置
    struct PID_Init_Config_s
    {
        float MaxOutput{};
        float IntegralLimit{};
        float Kp{};
        float Ki{};
        float Kd{};

        float Integral_Max{};
        float Integral_Min{};
        float Output_LPF{};
        float D_LPF{};
        float gama{};

        uint8_t Improve = NONE;
        uint32_t pid_mode = POSITION_PID;
        float max_err{};
        float deadband{};
    };

    // PID 实例结构体
    struct PIDInstance
    {
        PID_Init_Config_s config = {}; // PID 配置

        float dt{}; // 每次循环的时间间隔
        uint32_t dwt_count{0};

        float set[3]{}; // 目标值（NOW, LAST, LLAST）
        float get[3]{}; // 测量值（NOW, LAST, LLAST）
        float err[3]{}; // 误差（NOW, LAST, LLAST）

        float pout{};  // P输出
        float Iterm{}; // I中的增量项
        float iout{};  // I输出
        float dout{};  // D输出
        float derr{};  // 误差的微分

        float pos_out{};      // 位置式输出
        float last_pos_out{}; // 上次位置式输出
        float last_dout{};    // 上次D输出
        float last_iterm{};   // 上次Iterm

        float delta_u{};        // 增量值
        float delta_out{};      // 增量式输出 = last_delta_out + delta_u
        float last_delta_out{}; // 上次增量式输出

        // 如果需要用到外部传入的观测值，可以通过此指针访问
        float const *get_data=nullptr;

        void reset()
        {
            dt = 0.0f;
            for (auto &val : set)
                val = 0.0f;
            for (auto &val : get)
                val = 0.0f;
            for (auto &val : err)
                val = 0.0f;
            pout = 0.0f;
            Iterm = 0.0f;
            iout = 0.0f;
            dout = 0.0f;
            derr = 0.0f;
            pos_out = 0.0f;
            last_pos_out = 0.0f;
            last_dout = 0.0f;
            last_iterm = 0.0f;
            delta_u = 0.0f;
            delta_out = 0.0f;
            last_delta_out = 0.0f;
        }
        void reset_i()
        {
            iout = 0.0f;
        }
    };

    // PID 类
    class PID final
    {
    public:
        // 构造函数：单环
        explicit PID(PID_Init_Config_s &config, float *get);

        // 构造函数：双环
        explicit PID(PID_Init_Config_s &configpos, PID_Init_Config_s &configv, float *getpos, float *getv);

        // 设置配置：单跟随量
        void PIDSetConfig(float *get);
        void PIDSetConfig(PID_Init_Config_s &config, float *get);

        // 设置配置：双跟随量
        void PIDSetConfig(PID_Init_Config_s &configpos, PID_Init_Config_s &configv, float *getpos, float *getv);

        // 清空全部 PID
        void PIDClearALL();
        //清空I
        void PIDClearI();
        // 计算 PID 输出
        float PID_handle(float target);

        // 可同时维护两个 PID 环
        PIDInstance pidinstance[2] = {};

    private:
        void CheckPointer(float *get, PIDInstance &pid);

        void trapezoid_intergral(PIDInstance &pid);
        void changing_integration(PIDInstance &pid);
        void f_intergral_limit(PIDInstance &pid);
        void differential_forward(PIDInstance &pid);
        void derivative_filter(PIDInstance &pid);
        void output_filter(PIDInstance &pid);
        void apply_improvements(PIDInstance& pid);

        float pid_calc(PIDInstance &pid, float now_get, float now_set);

    private:
        // 定义掩码和函数指针的数组
        using ImproveFunc = void (PID::*)(PIDInstance &);

        struct ImproveEntry
        {
            uint8_t mask;
            ImproveFunc func;
        };
        std::array<ImproveEntry, 5> improveEntries = {{{Trapezoid_Intergral, &PID::trapezoid_intergral},
                                                       {ChangingIntegrationRate, &PID::changing_integration},
                                                       {Differential_Forward, &PID::differential_forward},
                                                       {DerivativeFilter, &PID::derivative_filter},
                                                       {Integral_Limit, &PID::f_intergral_limit}}};
        uint8_t pid_num = 0;
    };
} // namespace module: