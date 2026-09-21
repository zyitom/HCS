#pragma once
extern "C"{
#include "arm_math.h"
}

template <int _rows, int _cols>
class Matrixf
{
   public:
    /**
     * @brief 无输入数据的构造函数
     * @param
     */
    constexpr Matrixf(void) : rows_(_rows), cols_(_cols) { arm_mat_init_f32(&arm_mat_, _rows, _cols, this->data_); }

    /**
     * @brief 带输入数据的构造函数
     * @param data    存储数据的二维数组缓冲区
     */
    constexpr Matrixf(float data[_rows * _cols]) : Matrixf()
    {
        memcpy(this->data_, data, _rows * _cols * sizeof(float));
        arm_mat_init_f32(&arm_mat_, _rows, _cols, this->data_);
    }

    /**
     * @brief 拷贝构造函数
     * @param mat  被拷贝的矩阵
     */
    constexpr Matrixf(const Matrixf<_rows, _cols> &mat) : Matrixf()
    {
        memcpy(this->data_, mat.data_, _rows * _cols * sizeof(float));
        arm_mat_init_f32(&arm_mat_, _rows, _cols, this->data_);
    }

    /**
     * @brief 析构函数
     */
    ~Matrixf(void) {}

    /**
     * @brief 返回矩阵的行数
     * @return _rows  矩阵的行数
     */
    uint32_t rows(void) const { return _rows; }

    /**
     * @brief 返回矩阵的列数
     * @return _cols  矩阵的列数
     */
    uint32_t cols(void) const { return _cols; }

    /**
     * @brief 返回矩阵的元素
     * @param row  行索引
     */
    float *operator[](const int &row) { return &this->data_[row * _cols]; }
    const float *operator[](const int &row) const { return &this->data_[row * _cols]; }

    /**
     * @brief 矩阵的拷贝赋值运算符
     * @param mat   被拷贝的矩阵
     * @return      *this 矩阵
     */
    Matrixf<_rows, _cols> &operator=(const Matrixf<_rows, _cols> mat)
    {
        memcpy(this->data_, mat.data_, _rows * _cols * sizeof(float));
        return *this;
    }

    /**
     * @brief 两个矩阵的加法运算符
     * @param mat  右侧的矩阵
     * @note  该函数返回自身作为结果
     * @return     两个矩阵的和
     */
    Matrixf<_rows, _cols> &operator+=(const Matrixf<_rows, _cols> mat)
    {
        arm_mat_add_f32(&this->arm_mat_, &mat.arm_mat_, &this->arm_mat_);
        return *this;
    }

    /**
     * @brief 两个矩阵的减法运算符
     * @param mat 左侧的矩阵
     * @note  该函数返回自身作为结果
     * @return    两个矩阵的差
     */
    Matrixf<_rows, _cols> &operator-=(const Matrixf<_rows, _cols> mat)
    {
        arm_mat_sub_f32(&this->arm_mat_, &mat.arm_mat_, &this->arm_mat_);
        return *this;
    }

    /**
     * @brief 矩阵与缩放因子的标量运算符
     * @param val 缩放因子
     * @note  该函数返回自身作为结果
     * @return    缩放后的矩阵
     */
    Matrixf<_rows, _cols> &operator*=(const float &val)
    {
        arm_mat_scale_f32(&this->arm_mat_, val, &this->arm_mat_);
        return *this;
    }

    /**
     * @brief 矩阵与除法因子的标量运算符
     * @param val 除法因子
     * @note  该函数返回自身作为结果
     * @retval    矩阵 / val
     * @return    缩放后的矩阵
     */
    Matrixf<_rows, _cols> &operator/=(const float &val)
    {
        arm_mat_scale_f32(&this->arm_mat_, 1.f / val, &this->arm_mat_);
        return *this;
    }

    /**
     * @brief 加法运算符
     * @note 该函数不返回自身，而是返回一个新的矩阵实例
     * @param mat 右侧的矩阵
     * @return 加法运算后的矩阵
     */
    Matrixf<_rows, _cols> operator+(const Matrixf<_rows, _cols> &mat) const
    {
        Matrixf<_rows, _cols> res;
        arm_mat_add_f32(&this->arm_mat_, &mat.arm_mat_, &res.arm_mat_);
        return res;
    }

    /**
     * @brief 减法运算符
     * @note 该函数不返回自身，而是返回一个新的矩阵实例
     * @param mat 右侧的矩阵
     * @return 减法运算后的矩阵
     */
    Matrixf<_rows, _cols> operator-(const Matrixf<_rows, _cols> &mat) const
    {
        Matrixf<_rows, _cols> res;
        arm_mat_sub_f32(&this->arm_mat_, &mat.arm_mat_, &res.arm_mat_);
        return res;
    }

    /**
     * @brief 矩阵与缩放因子的标量运算符
     * @param val 缩放因子
     * @note      该函数不返回自身
     * @return    缩放后的矩阵
     */
    Matrixf<_rows, _cols> operator*(const float &val) const
    {
        Matrixf<_rows, _cols> res;
        arm_mat_scale_f32(&this->arm_mat_, val, &res.arm_mat_);
        return res;
    }

    /**
     * @brief 矩阵与缩放因子的标量运算符
     * @param val 左侧的缩放因子
     * @note      该函数不返回自身
     * @note      这次缩放因子在左侧
     * @return    缩放后的矩阵
     */
    friend Matrixf<_rows, _cols> operator*(const float &val, const Matrixf<_rows, _cols> &mat)
    {
        arm_status s;
        Matrixf<_rows, _cols> res;
        s = arm_mat_scale_f32(&mat.arm_mat_, val, &res.arm_mat_);
        return res;
    }

    /**
     * @brief 矩阵与除法因子的标量运算符
     * @param val 除法因子
     * @note  该函数返回自身作为结果
     * @retval    矩阵 / val
     * @return    缩放后的矩阵
     */
    Matrixf<_rows, _cols> operator/(const float &val) const
    {
        Matrixf<_rows, _cols> res;
        arm_mat_scale_f32(&this->arm_mat_, 1.f / val, &res.arm_mat_);
        return res;
    }

    /**
     * @brief 矩阵乘法
     * @param mat1 左侧的矩阵
     * @param mat2 右侧的矩阵
     * @return 矩阵乘法的结果
     */
    template <int cols2>
    friend Matrixf<_rows, cols2> operator*(const Matrixf<_rows, _cols> &mat1, const Matrixf<_cols, cols2> &mat2)
    {
        Matrixf<_rows, cols2> res;
        arm_mat_mult_f32(&mat1.arm_mat_, &mat2.arm_mat_, &res.arm_mat_);
        return res;
    }

    /**
     * @brief 比较两个矩阵是否相同
     *
     */
    bool operator==(const Matrixf<_rows, _cols> &mat) const
    {
        for (int i = 0; i < _rows * _cols; i++)
        {
            if (this->data_[i] != mat.data_[i])
                return false;
        }
        return true;
    }

    // 子矩阵
    template <int rows, int cols>
    Matrixf<rows, cols> block(const int &start_row, const int &start_col) const
    {
        Matrixf<rows, cols> res;
        for (int row = start_row; row < start_row + rows; row++)
        {
            memcpy((float *)res[0] + (row - start_row) * cols, (float *)this->data_ + row * _cols + start_col, cols * sizeof(float));
        }
        return res;
    }

    /**
     * @brief 返回矩阵的特定行
     * @param row 行索引
     * @retval 矩阵中的行向量
     */
    Matrixf<1, _cols> row(const int &row) const { return block<1, _cols>(row, 0); }

    /**
     * @brief 返回矩阵的特定列
     * @param col 列索引
     * @retval 矩阵中的列向量
     */
    Matrixf<_rows, 1> col(const int &col) const { return block<_rows, 1>(0, col); }

    /**
     * @brief 获取矩阵的转置
     * @param
     * @retval 转置后的矩阵
     */
    Matrixf<_cols, _rows> trans(void) const
    {
        Matrixf<_cols, _rows> res;
        arm_mat_trans_f32(&arm_mat_, &res.arm_mat_);
        return res;
    }
    // 矩阵的迹

    /**
     * @brief 获取矩阵的迹
     * @param
     * @retval 矩阵的迹
     */
    float trace(void) const
    {
        float res = 0;
        for (int i = 0; i < fmin(_rows, _cols); i++)
        {
            res += (*this)[i][i];
        }
        return res;
    }

    /**
     * @brief 获取矩阵的范数
     * @param
     * @retval 矩阵的范数
     */
    float norm(void) const { return sqrtf((this->trans() * *this)[0][0]); }

    /**
     * @brief 获取矩阵的逆
     * @param
     * @retval 矩阵的逆
     */
    Matrixf<_cols, _rows> inv(void) const
    {
        if (_cols != _rows)
            return Matrixf<_cols, _rows>::zeros();

        Matrixf<_cols, _rows> res;
        arm_status status = arm_mat_inverse_f32(&this->arm_mat_, &res);

        if (status == ARM_MATH_SINGULAR)
            return Matrixf<_cols, _rows>::zeros();

        return res;
    }

    /*==============================================================*/
    // 静态函数
    /**
     * @brief 返回一个 _rows x _cols 的零矩阵
     * @tparam _rows 行数
     * @tparam _cols 列数
     * @retval 零矩阵
     */
    static Matrixf<_rows, _cols> zeros(void)
    {
        float data[_rows * _cols] = {0};
        return Matrixf<_rows, _cols>(data);
    }

    /**
     * @brief 返回一个 _rows x _cols 的全一矩阵
     * @tparam _rows 行数
     * @tparam _cols 列数
     * @retval 全一矩阵
     */
    static Matrixf<_rows, _cols> ones(void)
    {
        float data[_rows * _cols] = {0};
        for (int i = 0; i < _rows * _cols; i++)
        {
            data[i] = 1;
        }
        return Matrixf<_rows, _cols>(data);
    }

    /**
     * @brief 返回一个 _rows x _cols 的单位矩阵
     * @tparam _rows 行数
     * @tparam _cols 列数
     * @retval 单位矩阵
     */
    static Matrixf<_rows, _cols> eye(void)
    {
        float data[_rows * _cols] = {0};
        for (int i = 0; i < fmin(_rows, _cols); i++)
        {
            data[i * _cols + i] = 1;
        }
        return Matrixf<_rows, _cols>(data);
    }

    /**
     * @brief 返回一个 _rows x _cols 的对角矩阵
     * @tparam _rows 行数
     * @tparam _cols 列数
     * @param vec 对角线元素
     * @retval 对角矩阵
     */
    static Matrixf<_rows, _cols> diag(Matrixf<_rows, 1> vec)
    {
        Matrixf<_rows, _cols> res = Matrixf<_rows, _cols>::zeros();
        for (int i = 0; i < fmin(_rows, _cols); i++)
        {
            res[i][i] = vec[i][0];
        }
        return res;
    }

   public:
    arm_matrix_instance_f32 arm_mat_;  // ARM 数学库实例

   protected:
    // 矩阵的大小
    int rows_, cols_;
    // 数据缓冲区
    float data_[_rows * _cols];
};

