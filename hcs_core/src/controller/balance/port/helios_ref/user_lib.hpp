/***************************************************************************************
 * @file user_lib.hpp
 * @author Leng Wen`ao 我日死你的🐎
 * @brief 提供常用的数学函数和宏定义
 * * @author Leng Wen`ao 我日死你的🐎
 * 包含以下功能：
 * - Deg2Rad Rad2Deg        角度弧度转化
 * - clip/abs_clip          (绝对值)限幅函数
 * - deadline/abs_deadline  (绝对值)死区
 * - loop_clip              循环限幅函数
 * - theta_format           角度转化到正负180度
 * - rad_format             弧度转化到正负PI
 * - MTurnProc              电机多圈处理
 * - KalmanFilter           卡尔曼滤波器
 * - RampFunction           斜坡函数
 * - invSqrt                快速倒数平方根
 * - FirstOrderFilter       一阶低通滤波
 ***************************************************************************************/

#pragma once
extern "C"
{
#include "arm_math.h"
}
#include <cstdint>
#include <cmath>
#include <cstring>
#include "module_def.hpp"
#include <unordered_map>
#include <array>
#define fp32 float

namespace module
{

    constexpr float PI2 = PI * 2.0f;
    [[nodiscard]] constexpr float deg2rad(float deg)
    {
        return deg * 0.0174532925f;
    }

    [[nodiscard]] constexpr float rad2deg(float rad)
    {
        return rad * 57.295779513f;
    }

    template <typename T, typename U, typename V>
    [[nodiscard]] constexpr auto clip(const T value, const U min, const V max) -> decltype(value)
    {
        return (value > max) ? max : ((value < min) ? min : value);
    }

    template <typename T, typename U>
    [[nodiscard]] constexpr auto abs_clip(const T value, const U max) -> decltype(value)
    {
        return clip(value, -max, max);
    }

    template <typename T, typename U, typename V>
    [[nodiscard]] constexpr auto deadline(const T value, const U min, const V max) -> decltype(value)
    {
        if (value > min && value < max)
        {
            return static_cast<T>(0);
        }
        return value;
    }

    template <typename T, typename U>
    [[nodiscard]] constexpr auto abs_deadline(const T value, const U max) -> decltype(value)
    {
        return deadline(value, -max, max);
    }

    template <typename T, typename U, typename V>
    [[nodiscard]] constexpr auto loop_clip(T value, const U minValue, const V maxValue) -> decltype(value)
    {
        T len = maxValue - minValue;
        if (value > maxValue)
        {
            value = minValue + std::fmod(value - minValue, len);
        }
        else if (value < minValue)
        {
            value = maxValue - std::fmod(minValue - value, len);
        }
        return value;
    }

    [[nodiscard]] constexpr float theta_format(const float Ang)
    {
        return loop_clip(Ang, -180.0f, 180.0f);
    }

    [[nodiscard]] constexpr float rad_format(const float Ang)
    {
        return loop_clip(Ang, -PI, PI);
    }

    class MTurnProc
    {
    public:
        MTurnProc() {};
        void set_total_angle(moto_data_receive_s &data, const float Set_Angle)
        {
            is_user_set = true;
            data.total_angle = Set_Angle;
        }
        void operator()(moto_data_receive_s &data, const float Mid_Angle, const float RR = 1.0f, const float PMAX = PI)
        {
            if (is_user_set)
            {
                return; // 用户设置了角度，不进行计算
            }
            if (is_first)
            {
                last_angle = data.angle;
                is_first = false;
            }
            float angle_diff = data.angle - last_angle;
            data.round_cnt += (angle_diff > PMAX) ? -1 : ((angle_diff < -PMAX) ? 1 : 0);
            data.total_angle = (data.round_cnt * PMAX * 2 + data.angle - Mid_Angle) / RR;
            last_angle = data.angle;

            if (RR == 1.0f)
            {
                data.offset_angle = data.angle - Mid_Angle;
                data.offset_angle = rad_format(data.offset_angle);
            }
            else
            {
                data.offset_angle = data.total_angle;
            }
        }

    private:
        bool is_user_set{};
        bool is_first{true};
        float last_angle{};
    };

    /**
     * @name   kalmanCreate
     * @brief  创建一个卡尔曼滤波器
     * @param  T_Q:系统噪声协方差
     *         T_R:测量噪声协方差
     * @attention
     * Z(k)是系统输入,即测量值   X(k|k)是卡尔曼滤波后的值,即最终输出
     * A=1 B=0 H=1 I=1  W(K)  V(k)是高斯白噪声,叠加在测量值上了,可以不用管
     * 一阶H'即为它本身,否则为转置矩阵
     * R固定，Q越大，代表越信任侧量值，Q无穷代表只用测量值
     * 反之，Q越小代表越信任模型预测值，Q为零则是只用模型预测
     */
    class KalmanFilter
    {
    public:
        KalmanFilter(float T_Q, float T_R)
            : Q(T_Q), R(T_R), A(1), B(0), H(1), X_last(0), P_last(0), X_mid(0) {}

        [[nodiscard]] float operator()(const float dat)
        {
            X_mid = A * X_last;                 // 对应公式(1)    x(k|k-1) = A*X(k-1|k-1)+B*U(k)+W(K)
            P_mid = A * P_last + Q;             // 对应公式(2)    p(k|k-1) = A*p(k-1|k-1)*A'+Q
            kg = P_mid / (P_mid + R);           // 对应公式(4)    kg(k) = p(k|k-1)*H'/(H*p(k|k-1)*H'+R)
            X_now = X_mid + kg * (dat - X_mid); // 对应公式(3)    x(k|k) = X(k|k-1)+kg(k)*(Z(k)-H*X(k|k-1))
            P_now = (1 - kg) * P_mid;           // 对应公式(5)    p(k|k) = (I-kg(k)*H)*P(k|k-1)
            P_last = P_now;                     // 状态更新
            X_last = X_now;
            return X_now; // 输出预测结果x(k|k)
        }

    private:
        float Q, R, A, B, H;
        float X_last, P_last;
        float X_mid, P_mid;
        float X_now, P_now;
        float kg;
    };

    class RampFunction
    {
    public:
        RampFunction() : output(0.0f) {}
        [[nodiscard]] float operator()(const float target_value, const float ramp_rate)
        {
            float delta = target_value - output;
            if (fabs(delta) <= ramp_rate)
            {
                return target_value;
            }
            else
            {
                output += ramp_rate * (delta / fabs(delta));
                return output;
            }
        }
        void reset(float value_now)
        {
            output = value_now;
        }

    private:
        float output;
    };

    class FirstOrderFilter
    {
    public:
        FirstOrderFilter(float rate_past_data, float rate_now_data)
            : input(0.0f), out(0.0f), rate_past(rate_past_data), rate_now(rate_now_data)
        {
        }

        [[nodiscard]] float operator()(const float input)
        {
            this->input = input;
            out = rate_past * out + rate_now * this->input;
            return out;
        }

    private:
        float input;
        float out;
        float rate_past;
        float rate_now;
    };

    [[nodiscard]] inline float invSqrt(float number)
    {
        static_assert(std::numeric_limits<float>::is_iec559, "This code requires IEEE 754 floating point");

        uint32_t i = 0x5f3759df - (*reinterpret_cast<uint32_t *>(&number) >> 1);
        i = 0x5f3759df - (i >> 1);
        float y = *reinterpret_cast<float *>(&i);
        return y * (1.5f - (number * 0.5f * y * y));
    }

    struct ramp_function_source_t
    {
        float output;   // 时间间隔
        bool init_flag; // 初始化标志位
    };

    struct first_order_filter_type_t
    {
        float input;        // 输入数据
        float out;          // 滤波输出的数据
        float num[1];       // 滤波参数
        float frame_period; // 滤波的时间间隔 单位 s
    };

    class RecentFloatBuffer
    {
    private:
        static constexpr size_t MAX_HISTORY = 100;

        // 滑动滤波

        // 老斜波函数计算
        [[deprecated]] float ramp_calc(ramp_function_source_t *ramp, float target_value, float ramp_rate);

        struct TimeYawData
        {
            uint32_t id = 0;  // 时间戳ID
            float yaw = 0.0f; // 对应的yaw值
        };

        std::array<TimeYawData, MAX_HISTORY> history{};
        size_t writeIndex = 0;
        size_t count = 0;

        // 根据ID查找对应的yaw值
        float findById(uint32_t id) const
        {
            for (size_t i = 0; i < count; i++)
            {
                // 需要按时间顺序查找，从最旧到最新
                size_t idx = (writeIndex + MAX_HISTORY - count + i) % MAX_HISTORY;
                if (history[idx].id == id)
                    return history[idx].yaw;
            }
            return 0.0f;
        }

    public:
        // 更新时间戳ID和对应的yaw值
        void update(uint32_t id, float yaw)
        {
            history[writeIndex] = {id, yaw};
            writeIndex = (writeIndex + 1) % MAX_HISTORY;
            if (count < MAX_HISTORY)
                count++;
        }

        // 根据时间戳ID获取对应的yaw值
        bool get(uint32_t id, float &data) const
        {
            if (count == 0 || !hasID(id))
                return false;
            data = findById(id);
            return true;
        }

        // 获取当前存储的数据数量
        size_t size() const
        {
            return count;
        }

        // 检查ID是否存在
        bool hasID(uint32_t id) const
        {
            for (size_t i = 0; i < count; i++)
            {
                size_t idx = (writeIndex + MAX_HISTORY - count + i) % MAX_HISTORY;
                if (history[idx].id == id)
                    return true;
            }
            return false;
        }
    };

    // 滑动滤波

    // 老斜波函数计算
    [[deprecated]] float ramp_calc(ramp_function_source_t *ramp, float target_value, float ramp_rate);

    [[deprecated]] void first_order_filter_init(first_order_filter_type_t *first_order_filter_type, float frame_period, float gama);

    void key_slow(float *rec, float target, float slow_Inc);

    float f_atan2(float a, float b); // 反三角函数arctan

    uint16_t floatToFloat16(float value);
    float float16ToFloat(uint16_t float16Value);

    int16_t floatToSignedFixed16(float value, int fractionalBits);
    float signedFixed16ToFloat(int16_t value, int fractionalBits);

    int float_to_uint(float x_float, float x_min, float x_max, int bits);
    float uint_to_float(int x_int, float x_min, float x_max, int bits);

} // namespace module


// [harness] Helios user_lib 缺失的 fal 函数（ST-ESO 的 fal 幂次函数；LESO 为死代码，
// 仅满足链接）。定义取自 v2 的调用约定：fal(e, alpha, delta)。
namespace module {
[[nodiscard]] inline float fal(float e, float alpha, float delta) {
    const float abs_e = std::fabs(e);
    if (abs_e > delta)
        return abs_e * alpha * ((e > 0.0f) ? 1.0f : -1.0f);
    return e / (std::pow(delta, 1.0f - alpha));
}
} // namespace module
