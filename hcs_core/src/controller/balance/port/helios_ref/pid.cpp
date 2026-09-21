#include "pid.hpp"
#include "user_lib.hpp"
#include "bsp_dwt.h"
namespace module
{
    //--------------------------------------------------------------------------------
    // 将外部 get 值的地址存入 pid.get_data
    void PID::CheckPointer(float *get, PIDInstance &pid)
    {
        pid.get_data = get;
    }

    //--------------------------------------------------------------------------------
    // 梯形积分:
    void PID::trapezoid_intergral(PIDInstance &pid)
    {
        // 用历史误差平均值 * Ki * dt 来更新积分项
        pid.Iterm = ((pid.err[NOW] + pid.err[LAST]) * 0.5f) * pid.dt;
    }

    //-----:--------------------------------------------------------------------------
    // 变速积分：根据误差大小动态调整积分
    void PID::changing_integration(PIDInstance &pid)
    {
        float cur_err = std::fabs(pid.err[NOW]);

        if (cur_err > pid.config.Integral_Max)
        {
            // 误差超出了上限，舍弃积分
            pid.Iterm = 0.0f;
        }
        else if (cur_err > pid.config.Integral_Min)
        {
            // 误差介于 [Integral_Min, Integral_Max] 之间
            pid.Iterm *= (pid.config.Integral_Max - cur_err) / (pid.config.Integral_Max - pid.config.Integral_Min);
        }
        // 如果误差小于等于 Integral_Min，保持不变，无需处理
    }

    //--------------------------------------------------------------------------------
    // 积分限幅：超过最大输出时，禁止积分继续累积
    void PID::f_intergral_limit(PIDInstance &pid)
    {
        // float temp_iout = pid.iout + pid.Iterm;
        // float temp_output = pid.pout + pid.iout + pid.dout;

        // if ((std::fabs(temp_output) > pid.config.MaxOutput) || (std::fabs(temp_iout) > pid.config.IntegralLimit))
        // {
        //     pid.Iterm = (temp_iout > 0) ? pid.config.IntegralLimit - pid.iout : -pid.config.IntegralLimit - pid.iout;
        //     pid.iout = (temp_iout > 0) ? pid.config.IntegralLimit : -pid.config.IntegralLimit;
        // }
        pid.iout = abs_clip(pid.iout, pid.config.IntegralLimit);
    }

    //--------------------------------------------------------------------------------
    // 微分先行：前向差分
    void PID::differential_forward(PIDInstance &pid)
    {
        // 即计算量： Kd * (上一时刻测量值 - 当前测量值) / dt
        pid.dout = pid.config.Kd * (pid.get[LAST] - pid.get[NOW]) / pid.dt;
    }

    //--------------------------------------------------------------------------------
    // 微分滤波：对 dout 进行低通滤波
    void PID::derivative_filter(PIDInstance &pid)
    {
        // dout = dout*(dt / (dt + D_LPF)) + last_dout*(D_LPF / (dt + D_LPF))
        float alpha = pid.dt / (pid.dt + pid.config.D_LPF);
        float dalpha = pid.config.D_LPF / (pid.dt + pid.config.D_LPF);
        pid.dout = pid.dout * alpha + pid.last_dout * dalpha;
    }

    //--------------------------------------------------------------------------------
    // 输出滤波：对位置式输出进行低通滤波
    void PID::output_filter(PIDInstance &pid)
    {
        float alpha = pid.dt / (pid.dt + pid.config.Output_LPF);
        float dalpha = pid.config.Output_LPF / (pid.dt + pid.config.Output_LPF);
        pid.pos_out = pid.pos_out * alpha + pid.last_pos_out * dalpha;
    }

    //--------------------------------------------------------------------------------
    // 应用改进项
    void PID::apply_improvements(PIDInstance &pid)
    {
        for (const auto &entry : improveEntries)
        {
            if (pid.config.Improve & entry.mask)
            {
                (this->*(entry.func))(pid);
            }
        }
    }

    //--------------------------------------------------------------------------------
    // PID 构造函数：单环
    PID::PID(PID_Init_Config_s &config, float *get)
    {
        PIDSetConfig(config, get);
    }

    //--------------------------------------------------------------------------------
    // PID 构造函数：双环
    PID::PID(PID_Init_Config_s &configpos, PID_Init_Config_s &configv, float *getpos, float *getv)
    {
        PIDSetConfig(configpos, configv, getpos, getv);
    }

    //--------------------------------------------------------------------------------
    // 缺省配置：仅切换跟随的值
    void PID::PIDSetConfig(float *get)
    {
        CheckPointer(get, pidinstance[0]);
    }

    //--------------------------------------------------------------------------------
    // 单环配置
    void PID::PIDSetConfig(PID_Init_Config_s &config, float *get)
    {
        pid_num = 1;
        // 重置数组第 0 个 PIDInstance
        pidinstance[0].reset();
        // 将配置拷入 PIDInstance
        pidinstance[0].config = config;
        // 获取外部观测值指针
        CheckPointer(get, pidinstance[0]);
    }

    //--------------------------------------------------------------------------------
    // 双环配置
    void PID::PIDSetConfig(PID_Init_Config_s &configpos, PID_Init_Config_s &configv, float *getpos, float *getv)
    {
        pid_num = 2;
        // 第 0 个用于速度环
        pidinstance[0].reset();
        pidinstance[0].config = configv;
        CheckPointer(getv, pidinstance[0]);

        // 第 1 个用于位置环
        pidinstance[1].reset();
        pidinstance[1].config = configpos;
        CheckPointer(getpos, pidinstance[1]);
    }

    //--------------------------------------------------------------------------------
    // 清空全部 PID
    void PID::PIDClearALL()
    {
        for (auto &p : pidinstance)
        {
            p.reset();
        }
    }
    //--------------------------------------------------------------------------------
    // 清空全部 I
    void PID::PIDClearI()
    {
        for (auto &p : pidinstance)
        {
            p.reset_i();
        }
    }

    //--------------------------------------------------------------------------------
    // 计算 PID
    float PID::PID_handle(float target)
    {
        switch (pid_num)
        {
        case 1:
        {
            // 单环
            return pid_calc(pidinstance[0], *pidinstance[0].get_data, target);
        }
        case 2:
        {
            // 双环：先计算位置环 (索引 1)，然后将输出作为下一个环的目标
            float output_pos = pid_calc(pidinstance[1], *pidinstance[1].get_data, target);
            // 再将其传给速度环(索引 0)
            return pid_calc(pidinstance[0], *pidinstance[0].get_data, output_pos);
        }
        default:
            return 0.0f;
        }
    }

    //--------------------------------------------------------------------------------
    // PID 核心计算函数
    float PID::pid_calc(PIDInstance &pid, float now_get, float now_set)
    {
        pid.dt = DWT_GetDeltaT(&pid.dwt_count);
        if (pid.dt > 0.05f)
        {
            return 0.0f;
        }

        pid.get[NOW] = now_get;
        pid.set[NOW] = now_set;
        pid.err[NOW] = now_set - now_get;

        // 误差微分
        pid.derr = pid.err[NOW] - pid.err[LAST];

        // 误差过大，或小于死区
        if ((pid.config.max_err > 0.0f && std::fabs(pid.err[NOW]) > pid.config.max_err) ||
            (pid.config.deadband > 0.0f && std::fabs(pid.err[NOW]) < pid.config.deadband))
        {
            return 0.0f;
        }

        switch (pid.config.pid_mode)
        {
        case POSITION_PID:
        {
            pid.pout = pid.config.Kp * pid.err[NOW];
            pid.Iterm = pid.config.Ki * pid.err[NOW] * pid.dt;
            pid.dout = pid.config.Kd * (pid.err[NOW] - pid.err[LAST]) / pid.dt;

            apply_improvements(pid); // 应用改进项

            pid.iout += pid.Iterm;
            pid.pos_out = pid.pout + pid.iout + pid.dout;

            if (pid.config.Improve & OutputFilter)
            {
                output_filter(pid);
            }

            pid.pos_out = abs_clip(pid.pos_out, pid.config.MaxOutput);
            break;
        }
        case DELTA_PID:
        {
            pid.pout = pid.config.Kp * (pid.err[NOW] - pid.err[LAST]);
            pid.iout = pid.config.Ki * pid.err[NOW];
            pid.dout = pid.config.Kd * (pid.err[NOW] - 2.0f * pid.err[LAST] + pid.err[LLAST]);

            pid.iout = abs_clip(pid.iout, pid.config.IntegralLimit);
            pid.delta_u = pid.pout + pid.iout + pid.dout;
            pid.delta_out = pid.last_delta_out + pid.delta_u;

            pid.delta_out = abs_clip(pid.delta_out, pid.config.MaxOutput);
            pid.last_delta_out = pid.delta_out;
            break;
        }
        }

        // 更新历史值
        pid.last_pos_out = pid.pos_out;
        pid.last_dout = pid.dout;
        pid.last_iterm = pid.Iterm;

        pid.err[LLAST] = pid.err[LAST];
        pid.err[LAST] = pid.err[NOW];
        pid.get[LLAST] = pid.get[LAST];
        pid.get[LAST] = pid.get[NOW];
        pid.set[LLAST] = pid.set[LAST];
        pid.set[LAST] = pid.set[NOW];

        // 数值安全检查，防止 NaN
        auto checkNaN = [](float &val)
        { if (std::isnan(val)) { val = 0.0f; } };
        checkNaN(pid.iout);
        checkNaN(pid.pout);
        checkNaN(pid.dout);

        // 如果是位置式，返回位置式输出，否则返回增量式输出
        return (pid.config.pid_mode == POSITION_PID) ? pid.pos_out : pid.delta_out;
    }

} // namespace module