#include "user_lib.hpp"

namespace module
{

  /**
   * @brief          斜波函数初始化
   * @author         RM
   * @param[in]      斜波函数结构体
   * @param[in]      间隔的时间，单位 s
   * @param[in]      最大值
   * @param[in]      最小值
   * @retval         返回空
   */
  float ramp_calc(ramp_function_source_t *ramp, float target_value, float ramp_rate)
  {
    float delta = 0;
    delta = target_value - ramp->output;
    if (ramp->init_flag == 1)
    {
      ramp->init_flag = 0;
      ramp->output = target_value;
      return ramp->output;
    }
    if (fabs(delta) <= ramp_rate)
      return target_value;
    else
    {
      ramp->output += ramp_rate * (delta / fabs(delta));
      return ramp->output;
    }
  }


  /**
   * @brief 斜坡（深大）
   * @note  键盘
   */
  void key_slow(float *rec, float target, float slow_Inc)
  {
    // if(fabs(*rec) - fabs(target) < 0);
    if (fabs(*rec) - fabs(target) > 0)
      slow_Inc = slow_Inc; // 减速时放大6倍

    if (fabs(*rec - target) < slow_Inc)
      *rec = target;
    else
    {
      if ((*rec) > target)
        (*rec) -= slow_Inc;
      if ((*rec) < target)
        (*rec) += slow_Inc;
    }
  }


  float f_atan2(float a, float b)
  {
    float res;
    if (atan2f(a, b) >= 0.0f)
    {
      res = atan2f(a, b);
    }
    else
    {
      res = atan2f(a, b) + 2 * PI;
    }
    return res;
  }

  // 将32位浮点数转换为16位浮点数
  uint16_t floatToFloat16(float value)
  {
    uint32_t f;
    std::memcpy(&f, &value, sizeof(uint32_t));
    uint16_t sign = (f >> 31) & 0x1;
    int16_t exponent = ((f >> 23) & 0xFF) - 127 + 15;
    uint16_t mantissa = (f >> 13) & 0x3FF;

    if (exponent <= 0)
    {
      // Underflow to zero
      exponent = 0;
      mantissa = 0;
    }
    else if (exponent >= 31)
    {
      // Overflow to infinity
      exponent = 31;
      mantissa = 0;
    }

    return (sign << 15) | (exponent << 10) | mantissa;
  }

  // 将16位浮点数转换回32位浮点数
  float float16ToFloat(uint16_t value)
  {
    uint16_t sign = (value >> 15) & 0x1;
    int16_t exponent = (value >> 10) & 0x1F;
    uint16_t mantissa = value & 0x3FF;

    uint32_t f;
    if (exponent == 0)
    {
      // Subnormal number
      f = (sign << 31) | ((exponent + 127 - 15) << 23) | (mantissa << 13);
    }
    else if (exponent == 31)
    {
      // Infinity or NaN
      f = (sign << 31) | (0xFF << 23) | (mantissa << 13);
    }
    else
    {
      // Normalized number
      f = (sign << 31) | ((exponent + 127 - 15) << 23) | (mantissa << 13);
    }

    float result;
    std::memcpy(&result, &f, sizeof(float));
    return result;
  }

  int16_t floatToSignedFixed16(float value, int fractionalBits)
  {
    return static_cast<int16_t>(round(value * (1 << fractionalBits)));
  }

  float signedFixed16ToFloat(int16_t value, int fractionalBits)
  {
    return static_cast<float>(value) / (1 << fractionalBits);
  }

int float_to_uint(float x_float, float x_min, float x_max, int bits)
{
	/* Converts a float to an unsigned int, given range and number of bits */
	float span = x_max - x_min;
	float offset = x_min;
	return (int) ((x_float-offset)*((float)((1<<bits)-1))/span);
}

/**
************************************************************************
* @brief:      	uint_to_float: 无符号整数转换为浮点数函数
* @param[in]:   x_int: 待转换的无符号整数
* @param[in]:   x_min: 范围最小值
* @param[in]:   x_max: 范围最大值
* @param[in]:   bits:  无符号整数的位数
* @retval:     	浮点数结果
* @details:    	将给定的无符号整数 x_int 在指定范围 [x_min, x_max] 内进行线性映射，映射结果为一个浮点数
************************************************************************
**/
float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
	/* converts unsigned int to float, given range and number of bits */
	float span = x_max - x_min;
	float offset = x_min;
	return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}

} // namespace module