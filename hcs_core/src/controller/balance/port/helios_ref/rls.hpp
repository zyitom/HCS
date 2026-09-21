#pragma once
#include "Matrix.hpp"
#include"math.h"


template <uint32_t dim>
class RLS
{
   public:
    /**
     * @brief  删除默认构造函数
     */
    RLS() = delete;

    /**
     * @brief 构造函数，初始化 RLS 参数
     * @param n 参数数量
     * @param delta 初始化逆协方差矩阵的常数
     * @param lambda 遗忘因子，取值范围 (0,1]
     * @param alpha 低通滤波器系数，取值范围 (0,1]
     */
    constexpr RLS(float delta_, float lambda_,float alpha_)
        : dimension(dim), lambda(lambda_), delta(delta_), lastUpdate(0), updateCnt(0), alpha(alpha_)
    {
        this->reset();
    }

  

    /**
      * @brief 重置 RLS 模块
     * @retval 无
      */
    void reset()
    {
        transMatrix  = Matrixf<dim, dim>::eye() * delta;
        gainVector   = Matrixf<dim, 1>::zeros();
        paramsVector = Matrixf<dim, 1>::zeros();
    }

   /**
     * @brief 处理一次 RLS 更新周期
     * @param sampleVector 新的样本输入，以 n x 1 维向量形式表示
     * @param actualOutput 实际反馈的真实输出
     * @retval paramsVector 参数向量
     */
    const Matrixf<dim, 1> &update(Matrixf<dim, 1> &sampleVector, float actualOutput)
    {   
        gainVector = (transMatrix * sampleVector) / (1.0f + (sampleVector.trans() * transMatrix * sampleVector)[0][0] / lambda) / lambda;  // 获取增益向量
        paramsVector += gainVector * (actualOutput - (sampleVector.trans() * paramsVector)[0][0]);                                // 获取参数向量
        transMatrix = (transMatrix - gainVector * sampleVector.trans() * transMatrix) / lambda;  // 获取转移矩阵
        output= (paramsVector.trans() * sampleVector)[0][0];
        filtered_output = alpha * output + (1 - alpha) * filtered_output;  // 应用低通滤波器
        updateCnt++;
        return paramsVector;
    }

     /**
     * @brief 设置默认回归参数
     * @param updatedParams 更新后的参数
     * @retval 无
     */
    void setParamVector(const Matrixf<dim, 1> &updatedParams)
    {
        paramsVector        = updatedParams;
     
    }
     /**
     * @brief 获取参数向量的 函数
     * @param 无
     * @retval paramsVector 参数向量
     */
    constexpr const Matrixf<dim, 1> &getParamsVector() const { return paramsVector; }

   /**
     * @brief 获取预测输出函数
     * @param 无
     * @retval RLS 模块的估计
     */
    const float &getOutput() const { return output; }

  /**
     * @brief 获取预测输出经滤波函数
     * @param 无
     * @retval RLS 模块的估计滤波
     */
    const float &getfiltered_output() const { return filtered_output; }
     /**
     * @brief 获取转移矩阵函数
     * @param 无
     * @retval transMatrix 矩阵函数
     */
    constexpr const Matrixf<dim, dim> &gettransMatrix() const { return transMatrix; }


   private:
 

    uint32_t dimension;  // RLS 空间的维度
    float lambda;        // 遗忘因子
    float delta;         // 转移矩阵的初始化值
    float lastUpdate;  // 上次更新的时间戳
    int32_t updateCnt;     // 总更新次数
    int is_nan;             //出现过nan，保留
    float alpha;         // 低通滤波器系数
    
    /* RLS 相关矩阵 */
    Matrixf<dim, dim> transMatrix;  // 转移矩阵实例
    Matrixf<dim, 1> gainVector;     // 用于参数更新的增益向量
    Matrixf<dim, 1> paramsVector;   // 参数向量
    float output;  // 估计/滤波输出
    float filtered_output;  // 滤波后的输出
};
