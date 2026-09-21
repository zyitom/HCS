// 对拍 harness 专用 stub：arm_math 的最小替代（仅编译 Helios 参照代码所需）。
#pragma once

#include <cmath>
#include <cstdint>

#ifndef PI
#define PI 3.14159265358979f
#endif

#define ARM_MATH_SUCCESS 0
#define ARM_MATH_SINGULAR 2
#define ARM_MATH_ARGUMENT_ERROR 3
typedef int arm_status;

typedef struct {
    uint16_t numRows;
    uint16_t numCols;
    float* pData;
} arm_matrix_instance_f32;

inline void arm_mat_init_f32(
    arm_matrix_instance_f32* S, uint16_t nRows, uint16_t nCols, float* pData) {
    S->numRows = nRows;
    S->numCols = nCols;
    S->pData = pData;
}

inline arm_status arm_mat_mult_f32(
    const arm_matrix_instance_f32* pSrcA, const arm_matrix_instance_f32* pSrcB,
    arm_matrix_instance_f32* pDst) {
    for (uint16_t i = 0; i < pDst->numRows; ++i)
        for (uint16_t j = 0; j < pDst->numCols; ++j) {
            float sum = 0.0f;
            for (uint16_t k = 0; k < pSrcA->numCols; ++k)
                sum += pSrcA->pData[i * pSrcA->numCols + k]
                     * pSrcB->pData[k * pSrcB->numCols + j];
            pDst->pData[i * pDst->numCols + j] = sum;
        }
    return ARM_MATH_SUCCESS;
}

inline arm_status arm_mat_trans_f32(
    const arm_matrix_instance_f32* pSrc, arm_matrix_instance_f32* pDst) {
    for (uint16_t i = 0; i < pSrc->numRows; ++i)
        for (uint16_t j = 0; j < pSrc->numCols; ++j)
            pDst->pData[j * pDst->numCols + i] = pSrc->pData[i * pSrc->numCols + j];
    return ARM_MATH_SUCCESS;
}

inline arm_status arm_mat_inverse_f32(
    arm_matrix_instance_f32* pSrc, arm_matrix_instance_f32* pDst) {
    // 高斯-约当求逆（仅用于链接，LESO 为死代码不会执行）。
    const uint16_t n = pSrc->numRows;
    float aug[8][8] = {};
    for (uint16_t i = 0; i < n; ++i) {
        for (uint16_t j = 0; j < n; ++j) {
            aug[i][j] = pSrc->pData[i * n + j];
            aug[i][n + j] = (i == j) ? 1.0f : 0.0f;
        }
    }
    for (uint16_t col = 0; col < n; ++col) {
        uint16_t pivot = col;
        for (uint16_t row = col + 1; row < n; ++row)
            if (std::fabs(aug[row][col]) > std::fabs(aug[pivot][col]))
                pivot = row;
        if (std::fabs(aug[pivot][col]) < 1e-12f)
            return 1;
        if (pivot != col)
            for (uint16_t j = 0; j < 2 * n; ++j)
                std::swap(aug[pivot][j], aug[col][j]);
        const float inv = 1.0f / aug[col][col];
        for (uint16_t j = 0; j < 2 * n; ++j)
            aug[col][j] *= inv;
        for (uint16_t row = 0; row < n; ++row) {
            if (row == col)
                continue;
            const float factor = aug[row][col];
            for (uint16_t j = 0; j < 2 * n; ++j)
                aug[row][j] -= factor * aug[col][j];
        }
    }
    for (uint16_t i = 0; i < n; ++i)
        for (uint16_t j = 0; j < n; ++j)
            pDst->pData[i * n + j] = aug[i][n + j];
    return ARM_MATH_SUCCESS;
}

inline arm_status arm_mat_add_f32(
    const arm_matrix_instance_f32* pSrcA, const arm_matrix_instance_f32* pSrcB,
    arm_matrix_instance_f32* pDst) {
    for (uint32_t i = 0; i < pDst->numRows * pDst->numCols; ++i)
        pDst->pData[i] = pSrcA->pData[i] + pSrcB->pData[i];
    return ARM_MATH_SUCCESS;
}

inline arm_status arm_mat_sub_f32(
    const arm_matrix_instance_f32* pSrcA, const arm_matrix_instance_f32* pSrcB,
    arm_matrix_instance_f32* pDst) {
    for (uint32_t i = 0; i < pDst->numRows * pDst->numCols; ++i)
        pDst->pData[i] = pSrcA->pData[i] - pSrcB->pData[i];
    return ARM_MATH_SUCCESS;
}

inline arm_status arm_mat_scale_f32(
    const arm_matrix_instance_f32* pSrc, float scale, arm_matrix_instance_f32* pDst) {
    for (uint32_t i = 0; i < pDst->numRows * pDst->numCols; ++i)
        pDst->pData[i] = pSrc->pData[i] * scale;
    return ARM_MATH_SUCCESS;
}

inline float arm_cos_f32(float x) {
    return std::cos(x);
}
inline float arm_sin_f32(float x) {
    return std::sin(x);
}
inline void arm_sqrt_f32(float x, float* out) {
    *out = std::sqrt(x);
}
