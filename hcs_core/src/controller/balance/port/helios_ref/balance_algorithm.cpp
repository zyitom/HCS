#include "balance_algorithm.hpp"
#include "module_def.hpp"
#include "user_lib.hpp"
#include "bsp_dwt.h"
extern "C"
{
#include "arm_math.h"
#include "kalman_filter.h"
}

namespace module
{

    void lqr_K(float LL, float LR, float K_Fit_Coefficients[40][6],
               float K_matrix[4][10])
    {
        float A1 = LL * LR;
        float A2 = LL * LL;
        float A3 = LR * LR;
        // 初始化矩阵
        for (int row = 0; row < 4; row++)
        {
            for (int col = 0; col < 10; col++)
            {
                float *coeffs = K_Fit_Coefficients[col * 4 + row];
                K_matrix[row][col] = coeffs[0] + coeffs[1] * LL + coeffs[2] * LR +
                                     coeffs[3] * A2 + coeffs[4] * A1 + coeffs[5] * A3;
            }
        }
    }
    void lqr_A(float LL, float LR, float A_Fit_Coefficients[100][6],
               float A_matrix[10][10])
    {
        float A1 = LL * LR;
        float A2 = LL * LL;
        float A3 = LR * LR;
        // 初始化矩阵
        for (int row = 0; row < 10; row++)
        {
            for (int col = 0; col < 10; col++)
            {
                int num = col * 10 + row;
                if (num == 41 || num == 43 || num == 45 || num == 47 || num == 49 || num == 61 || num == 63 || num == 65 || num == 67 || num == 69 || num == 89 || num == 10 || num == 32 || num == 54 || num == 76 || num == 98) // 其他行都是0，不用计算节省资源
                {
                    float *coeffs = A_Fit_Coefficients[num];
                    A_matrix[row][col] = coeffs[0] + coeffs[1] * LL + coeffs[2] * LR +
                                         coeffs[3] * A2 + coeffs[4] * A1 + coeffs[5] * A3;
                }
                // else
                // A_matrix[row][col]=0;
            }
        }
    }

    void lqr_B(float LL, float LR, float B_Fit_Coefficients[40][6],
               float B_matrix[10][4])
    {
        float A1 = LL * LR;
        float A2 = LL * LL;
        float A3 = LR * LR;
        // 初始化矩阵
        for (int row = 0; row < 10; row++)
        {
            for (int col = 0; col < 4; col++)
            {
                int num = col * 10 + row;
                if (num % 2 != 0) // 观察到b矩阵只有奇数行有值
                {
                    float *coeffs = B_Fit_Coefficients[col * 10 + row];
                    B_matrix[row][col] = coeffs[0] + coeffs[1] * LL + coeffs[2] * LR +
                                         coeffs[3] * A2 + coeffs[4] * A1 + coeffs[5] * A3;
                }
            }
        }
    }

    void leg_pos(float phi1, float phi4, float l1, float l2, float l5, float dphi1, float dphi4,
                 float &l0, float &phi0, float &dl0, float &dphi0, float &phi2, float &phi3)
    {
        float A0, B0, C0, C0_tmp, xb, xd, yb, yc, yd, dxc, dyc;
        float t1, t2;
        xb = l1 * arm_cos_f32(phi1);
        yb = l1 * arm_sin_f32(phi1);
        xd = l5 + l1 * arm_cos_f32(phi4);
        yd = l1 * arm_sin_f32(phi4);
        C0 = xd - xb;
        A0 = 2.0 * l2 * C0;
        yc = yd - yb;
        B0 = 2.0 * l2 * yc;
        C0_tmp = l2 * l2;
        C0 = ((C0_tmp + C0 * C0) + yc * yc) - C0_tmp;

        float sqrt_val_1 = (A0 * A0 + B0 * B0) - C0 * C0;
        if (sqrt_val_1 < 0.0f)
        {
            sqrt_val_1 = 0.0f;
        }
        arm_sqrt_f32(sqrt_val_1, &t1);

        phi2 =
            2.0 * atan2(B0 + t1, A0 + C0);
        C0 = xb + l2 * arm_cos_f32(phi2);
        yc = yb + l2 * arm_sin_f32(phi2);
        phi3 = atan2(yc - yd, C0 - xd);
        C0 -= l5 / 2.0;

        arm_sqrt_f32(C0 * C0 + yc * yc, &t2);
        l0 = t2;
        phi0 = atan2(yc, C0);

        float vel_den = arm_sin_f32(phi2 - phi3);
        if (fabsf(vel_den) < 1e-6f)
        {
            vel_den = (vel_den >= 0.0f) ? 1e-6f : -1e-6f;
        }

        dxc = l1 * arm_sin_f32(phi1 - phi2) * arm_sin_f32(phi3) * dphi1 / vel_den + l1 * arm_sin_f32(phi3 - phi4) * sin(phi2) * dphi4 / vel_den;
        dyc = -l1 * arm_sin_f32(phi1 - phi2) * arm_cos_f32(phi3) * dphi1 / vel_den - l1 * arm_sin_f32(phi3 - phi4) * cos(phi2) * dphi4 / vel_den;

        dl0 = dxc * cos(phi0) + dyc * sin(phi0);
        dphi0 = -dxc * sin(phi0) + dyc * cos(phi0);
    }

    void calculate_status_vector(float K_matrix[4][10], float Status_vector[10], float &Tlwl, float &Tlwr, float &Tbll, float &Tblr)
    {
        Tlwl = K_matrix[0][0] * Status_vector[0] + K_matrix[0][1] * Status_vector[1] + K_matrix[0][2] * Status_vector[2] + K_matrix[0][3] * Status_vector[3] + K_matrix[0][4] * Status_vector[4] + K_matrix[0][5] * Status_vector[5] + K_matrix[0][6] * Status_vector[6] + K_matrix[0][7] * Status_vector[7] + K_matrix[0][8] * Status_vector[8] + K_matrix[0][9] * Status_vector[9];
        Tlwr = K_matrix[1][0] * Status_vector[0] + K_matrix[1][1] * Status_vector[1] + K_matrix[1][2] * Status_vector[2] + K_matrix[1][3] * Status_vector[3] + K_matrix[1][4] * Status_vector[4] + K_matrix[1][5] * Status_vector[5] + K_matrix[1][6] * Status_vector[6] + K_matrix[1][7] * Status_vector[7] + K_matrix[1][8] * Status_vector[8] + K_matrix[1][9] * Status_vector[9];
        Tbll = K_matrix[2][0] * Status_vector[0] + K_matrix[2][1] * Status_vector[1] + K_matrix[2][2] * Status_vector[2] + K_matrix[2][3] * Status_vector[3] + K_matrix[2][4] * Status_vector[4] + K_matrix[2][5] * Status_vector[5] + K_matrix[2][6] * Status_vector[6] + K_matrix[2][7] * Status_vector[7] + K_matrix[2][8] * Status_vector[8] + K_matrix[2][9] * Status_vector[9];
        Tblr = K_matrix[3][0] * Status_vector[0] + K_matrix[3][1] * Status_vector[1] + K_matrix[3][2] * Status_vector[2] + K_matrix[3][3] * Status_vector[3] + K_matrix[3][4] * Status_vector[4] + K_matrix[3][5] * Status_vector[5] + K_matrix[3][6] * Status_vector[6] + K_matrix[3][7] * Status_vector[7] + K_matrix[3][8] * Status_vector[8] + K_matrix[3][9] * Status_vector[9];
    }
    void calculate_status_vector_fly(float K_matrix[4][10], float Status_vector[10], float &Tlwl, float &Tlwr, float &Tbll, float &Tblr, bool left, bool right, float improve0)
    {
        if (left == 1)
        {
            // Tlwl = K_matrix[0][0] * Status_vector[0] + K_matrix[0][1] * Status_vector[1] + K_matrix[0][2] * Status_vector[2] + K_matrix[0][3] * Status_vector[3] + K_matrix[0][4] * Status_vector[4] + K_matrix[0][5] * Status_vector[5] + K_matrix[0][6] * Status_vector[6] + K_matrix[0][7] * Status_vector[7] + K_matrix[0][8] * Status_vector[8] + K_matrix[0][9] * Status_vector[9];
            // Tbll = K_matrix[2][0] * Status_vector[0] + K_matrix[2][1] * Status_vector[1] + K_matrix[2][2] * Status_vector[2] + K_matrix[2][3] * Status_vector[3] + K_matrix[2][4] * Status_vector[4] + K_matrix[2][5] * Status_vector[5] + K_matrix[2][6] * Status_vector[6] + K_matrix[2][7] * Status_vector[7] + K_matrix[2][8] * Status_vector[8] + K_matrix[2][9] * Status_vector[9];
            Tlwl = 0;
            Tbll = (K_matrix[2][4] * Status_vector[4] + K_matrix[2][5] * Status_vector[5]) * improve0;
        }
        else
        {
            Tlwl = K_matrix[0][0] * Status_vector[0] + K_matrix[0][1] * Status_vector[1] + K_matrix[0][2] * Status_vector[2] + K_matrix[0][3] * Status_vector[3] + K_matrix[0][4] * Status_vector[4] + K_matrix[0][5] * Status_vector[5] + K_matrix[0][6] * Status_vector[6] + K_matrix[0][7] * Status_vector[7] + K_matrix[0][8] * Status_vector[8] + K_matrix[0][9] * Status_vector[9];
            Tbll = K_matrix[2][0] * Status_vector[0] + K_matrix[2][1] * Status_vector[1] + K_matrix[2][2] * Status_vector[2] + K_matrix[2][3] * Status_vector[3] + K_matrix[2][4] * Status_vector[4] + K_matrix[2][5] * Status_vector[5] + K_matrix[2][6] * Status_vector[6] + K_matrix[2][7] * Status_vector[7] + K_matrix[2][8] * Status_vector[8] + K_matrix[2][9] * Status_vector[9];
        }
        if (right == 1)
        {
            // Tlwr = K_matrix[1][0] * Status_vector[0] + K_matrix[1][1] * Status_vector[1] + K_matrix[1][2] * Status_vector[2] + K_matrix[1][3] * Status_vector[3] + K_matrix[1][4] * Status_vector[4] + K_matrix[1][5] * Status_vector[5] + K_matrix[1][6] * Status_vector[6] + K_matrix[1][7] * Status_vector[7] + K_matrix[1][8] * Status_vector[8] + K_matrix[1][9] * Status_vector[9];
            // Tblr = K_matrix[3][0] * Status_vector[0] + K_matrix[3][1] * Status_vector[1] + K_matrix[3][2] * Status_vector[2] + K_matrix[3][3] * Status_vector[3] + K_matrix[3][4] * Status_vector[4] + K_matrix[3][5] * Status_vector[5] + K_matrix[3][6] * Status_vector[6] + K_matrix[3][7] * Status_vector[7] + K_matrix[3][8] * Status_vector[8] + K_matrix[3][9] * Status_vector[9];
            Tlwr = 0;
            Tblr = (K_matrix[3][6] * Status_vector[6] + K_matrix[3][7] * Status_vector[7]) * improve0;
        }
        else
        {
            Tlwr = K_matrix[1][0] * Status_vector[0] + K_matrix[1][1] * Status_vector[1] + K_matrix[1][2] * Status_vector[2] + K_matrix[1][3] * Status_vector[3] + K_matrix[1][4] * Status_vector[4] + K_matrix[1][5] * Status_vector[5] + K_matrix[1][6] * Status_vector[6] + K_matrix[1][7] * Status_vector[7] + K_matrix[1][8] * Status_vector[8] + K_matrix[1][9] * Status_vector[9];
            Tblr = K_matrix[3][0] * Status_vector[0] + K_matrix[3][1] * Status_vector[1] + K_matrix[3][2] * Status_vector[2] + K_matrix[3][3] * Status_vector[3] + K_matrix[3][4] * Status_vector[4] + K_matrix[3][5] * Status_vector[5] + K_matrix[3][6] * Status_vector[6] + K_matrix[3][7] * Status_vector[7] + K_matrix[3][8] * Status_vector[8] + K_matrix[3][9] * Status_vector[9];
        }
    }

    void leg_VMC(float phi0, float phi1, float phi2, float phi3, float phi4,
                 float l1, float l0, float F, float Tp, float &T1, float &T2)
    {
        float T1_tmp;
        float T2_tmp;
        float j1_tmp;
        float j3_tmp;
        j1_tmp = arm_sin_f32(phi3 - phi2);
        j3_tmp = arm_sin_f32(phi3 - phi4);
        T1_tmp = phi0 - phi3;
        T2_tmp = phi0 - phi2;
        float vmc_J[4];
        vmc_J[0] = l1 * arm_sin_f32(T1_tmp) * arm_sin_f32(phi1 - phi2) / j1_tmp;
        vmc_J[1] = l1 * arm_cos_f32(T1_tmp) * arm_sin_f32(phi1 - phi2) / (l0 * j1_tmp);
        vmc_J[2] = l1 * arm_sin_f32(T2_tmp) * j3_tmp / j1_tmp;
        vmc_J[3] = l1 * arm_cos_f32(T2_tmp) * j3_tmp / (l0 * j1_tmp);

        T1 = vmc_J[0] * F + vmc_J[1] * Tp;
        T2 = vmc_J[2] * F + vmc_J[3] * Tp;
    }

    void inverse_contact_force(float phi0, float phi1, float phi2, float phi3, float phi4,
                               float l1, float l0,
                               float T1, float T2,
                               float &F_estimated, float &Tp_estimated, int side)
    {
        float j1_tmp = arm_sin_f32(phi3 - phi2);
        float j3_tmp = arm_sin_f32(phi3 - phi4);
        float T1_tmp = phi0 - phi3;
        float T2_tmp = phi0 - phi2;

        float vmc_J[4];
        vmc_J[0] = l1 * arm_sin_f32(T1_tmp) * arm_sin_f32(phi1 - phi2) / j1_tmp;
        vmc_J[1] = l1 * arm_cos_f32(T1_tmp) * arm_sin_f32(phi1 - phi2) / (l0 * j1_tmp);
        vmc_J[2] = l1 * arm_sin_f32(T2_tmp) * j3_tmp / j1_tmp;
        vmc_J[3] = l1 * arm_cos_f32(T2_tmp) * j3_tmp / (l0 * j1_tmp);

        // 逆解： [T1; T2] = J * [F; Tp]
        float det = vmc_J[0] * vmc_J[3] - vmc_J[1] * vmc_J[2];

        if (fabsf(det) < 1e-6f) det = (det >= 0.0f ? 1e-6f : -1e-6f);// 奇异性防护

        float inv_det = 1.0f / det;
        side = (side == 0) ? -1 : 1;
        F_estimated = (T1 * vmc_J[3] - T2 * vmc_J[1]) * inv_det * side;
        Tp_estimated = (-T1 * vmc_J[2] + T2* vmc_J[0]) * inv_det * side;
        Tp_estimated = module::abs_clip(Tp_estimated, 800.0f);
        if (fabsf(Tp_estimated) < 0.05f) Tp_estimated = 0.0f;
    }

    void pinv_B_matrix(float B_matrix[10][4], float pinv_B[4][10])
    {
        float BT_data[40];
        float BTB_data[16];
        float BTB_inv_data[16];

        arm_matrix_instance_f32 B_mat;
        arm_matrix_instance_f32 BT_mat;
        arm_matrix_instance_f32 BTB_mat;
        arm_matrix_instance_f32 BTB_inv_mat;
        arm_matrix_instance_f32 pinv_B_mat;

        arm_mat_init_f32(&B_mat, 10, 4, (float *)B_matrix);
        arm_mat_init_f32(&BT_mat, 4, 10, BT_data);
        arm_mat_init_f32(&BTB_mat, 4, 4, BTB_data);
        arm_mat_init_f32(&BTB_inv_mat, 4, 4, BTB_inv_data);
        arm_mat_init_f32(&pinv_B_mat, 4, 10, (float *)pinv_B);

        // BT = B^T
        arm_mat_trans_f32(&B_mat, &BT_mat);

        // BTB = BT * B
        arm_mat_mult_f32(&BT_mat, &B_mat, &BTB_mat);

        // BTB_inv = (BTB)^-1
        arm_status status = arm_mat_inverse_f32(&BTB_mat, &BTB_inv_mat);

        if (status == ARM_MATH_SUCCESS)
        {
            // pinv_B = BTB_inv * BT
            arm_mat_mult_f32(&BTB_inv_mat, &BT_mat, &pinv_B_mat);
        }
        else
        {
            // 如果求逆失败，将结果置零
            for (int i = 0; i < 4; i++)
            {
                for (int j = 0; j < 10; j++)
                {
                    pinv_B[i][j] = 0.0f;
                }
            }
        }
    }


    void reset_kf()
    {
        kf_state.reset();
    }

    bool get_wheel_stall_left()
    {
        return kf_state.wheel_stall[0];
    }

    bool get_wheel_stall_right()
    {
        return kf_state.wheel_stall[1];
    }

    void reset_kf_stall()
    {
        kf_state.wheel_stall[0] = false;
        kf_state.wheel_stall[1] = false;
        kf_state.stall_recover_cnt[0] = 0.0f;
        kf_state.stall_recover_cnt[1] = 0.0f;
    }

    float spd_estimation(float wheel_spd_l, float wheel_spd_r, float yaw_rate_z, float acc_x,
                         float leg_p_l, float leg_p_r, bool if_off_gnd_l, bool if_off_gnd_r,
                         float leg_dphi0_l, float leg_dphi0_r)
    {
        float dt = DWT_GetDeltaT(&kf_state.dwt_count_KF);
        if (dt > 0.05f || dt < 0.0f)
        {
            dt = 0.001f;
        }

        float wheel_spd_arr[2] = {wheel_spd_l, wheel_spd_r};
        for (int i = 0; i < 2; ++i)
        {
            float wheel_drop = fabsf(kf_state.last_wheel_spd[i]) - fabsf(wheel_spd_arr[i]);
            if (wheel_drop > KF_s::stall_wheel_drop_thres && fabsf(kf_state.last_wheel_spd[i]) > 0.5f)
            {
                kf_state.wheel_stall[i] = true;
                kf_state.stall_recover_cnt[i] = 0.0f;
            }
            if (kf_state.wheel_stall[i])
            {
                kf_state.stall_recover_cnt[i] += dt;
                if (kf_state.stall_recover_cnt[i] > KF_s::stall_recover_time &&
                    fabsf(wheel_spd_arr[i]) > KF_s::stall_wheel_spd_thres)
                {
                    kf_state.wheel_stall[i] = false;
                    kf_state.stall_recover_cnt[i] = 0.0f;
                }
            }
            kf_state.last_wheel_spd[i] = wheel_spd_arr[i];
        }

        bool true_off_gnd_l = false, true_off_gnd_r = false;
        kf_state.off_gnd_timer[0] = if_off_gnd_l ? (kf_state.off_gnd_timer[0] + dt) : 0.0f;
        kf_state.off_gnd_timer[1] = if_off_gnd_r ? (kf_state.off_gnd_timer[1] + dt) : 0.0f;
        if (kf_state.off_gnd_timer[0] > KF_s::off_gnd_debounce_time)
            true_off_gnd_l = true;
        if (kf_state.off_gnd_timer[1] > KF_s::off_gnd_debounce_time)
            true_off_gnd_r = true;

        float p_delta_l = fabsf(leg_p_l - kf_state.last_leg_p[0]);
        float p_delta_r = fabsf(leg_p_r - kf_state.last_leg_p[1]);
        kf_state.last_leg_p[0] = leg_p_l;
        kf_state.last_leg_p[1] = leg_p_r;

        if (p_delta_l > KF_s::impact_p_delta_thres || p_delta_r > KF_s::impact_p_delta_thres)
        {
            kf_state.impact_timer = KF_s::impact_cooldown_time;
        }

        float current_acc = module::abs_clip(acc_x, 20.0f);
        if (kf_state.impact_timer > 0.0f)
        {
            kf_state.impact_timer -= dt;
            current_acc = module::abs_clip(current_acc, 1.0f);
        }

        float f = (kf_state.acc_last + current_acc) * 0.5f;
        float v_predict = kf_state.v_est + f * dt;
        kf_state.acc_last = current_acc;
        kf_state.cov += KF_s::Q * dt;

        float dynamic_R_base = KF_s::R_base + 10.0f * fabsf(yaw_rate_z);

        const float z_left = wheel_spd_l + yaw_rate_z * KF_s::wheel_half_track;
        const float z_right = wheel_spd_r - yaw_rate_z * KF_s::wheel_half_track;

        float R_resid_l, R_resid_r;
        if (kf_state.resid_valid)
        {
            R_resid_l = kf_state.resid_sq_sum_l + kf_state.cov_post;
            R_resid_r = kf_state.resid_sq_sum_r + kf_state.cov_post;
        }
        else
        {
            R_resid_l = KF_s::R_resid_min;
            R_resid_r = KF_s::R_resid_min;
        }
        if (R_resid_l < KF_s::R_resid_min) R_resid_l = KF_s::R_resid_min;
        if (R_resid_r < KF_s::R_resid_min) R_resid_r = KF_s::R_resid_min;

        float R_swing_l = KF_s::R_swing_gain * fabsf(leg_dphi0_l);
        float R_swing_r = KF_s::R_swing_gain * fabsf(leg_dphi0_r);

        float R_left = dynamic_R_base + R_resid_l + R_swing_l;
        float R_right = dynamic_R_base + R_resid_r + R_swing_r;

        if (true_off_gnd_l)
            R_left = KF_s::R_max;
        if (true_off_gnd_r)
            R_right = KF_s::R_max;
        if (kf_state.wheel_stall[0])
            R_left = KF_s::R_max;
        if (kf_state.wheel_stall[1])
            R_right = KF_s::R_max;
        if (kf_state.impact_timer > 0.0f)
        {
            R_left = KF_s::R_max;
            R_right = KF_s::R_max;
        }

        const float z_avg = 0.5f * (fabsf(z_left) + fabsf(z_right));
        if (z_avg < 0.05f)
        {
            R_left /= 5.0f;
            R_right /= 5.0f;
        }

        if (true_off_gnd_l || true_off_gnd_r)
        {
            v_predict *= expf(-KF_s::air_decay_rate * dt);
        }

        R_left = module::clip(R_left, KF_s::R_high_speed_min, KF_s::R_max);
        R_right = module::clip(R_right, KF_s::R_high_speed_min, KF_s::R_max);

        const float w_left = 1.0f / (R_left + 1e-6f);
        const float w_right = 1.0f / (R_right + 1e-6f);
        const float w_sum = w_left + w_right;

        float wheel_measure = v_predict;
        float R_dynamic = KF_s::R_max;
        if (w_sum > 1e-5f)
        {
            wheel_measure = (z_left * w_left + z_right * w_right) / w_sum;
            R_dynamic = 1.0f / w_sum;
        }

        float K = kf_state.cov / (kf_state.cov + R_dynamic);
        float err = wheel_measure - v_predict;
        kf_state.v_est = v_predict + K * err;
        kf_state.cov = (1.0f - K) * kf_state.cov;

        if (kf_state.cov < KF_s::cov_min)
            kf_state.cov = KF_s::cov_min;
        else if (kf_state.cov > KF_s::cov_max)
            kf_state.cov = KF_s::cov_max;

        kf_state.cov_post = kf_state.cov;
        bool trust_l = !(true_off_gnd_l || kf_state.wheel_stall[0] || kf_state.impact_timer > 0.0f);
        bool trust_r = !(true_off_gnd_r || kf_state.wheel_stall[1] || kf_state.impact_timer > 0.0f);
        if (trust_l)
        {
            float resid_l = z_left - kf_state.v_est;
            kf_state.resid_sq_sum_l = KF_s::resid_ema_alpha * kf_state.resid_sq_sum_l
                                    + (1.0f - KF_s::resid_ema_alpha) * resid_l * resid_l;
        }
        if (trust_r)
        {
            float resid_r = z_right - kf_state.v_est;
            kf_state.resid_sq_sum_r = KF_s::resid_ema_alpha * kf_state.resid_sq_sum_r
                                    + (1.0f - KF_s::resid_ema_alpha) * resid_r * resid_r;
        }
        kf_state.resid_valid = true;

        float wheel_avg = 0.5f * (fabsf(wheel_spd_l) + fabsf(wheel_spd_r));
        if (wheel_avg < 0.02f && fabsf(acc_x) < 0.05f)
        {
            kf_state.v_est *= expf(-KF_s::zupt_decay_rate * dt);
            if (fabsf(kf_state.v_est) < 0.005f)
                kf_state.v_est = 0.0f;
        }
        else if (wheel_avg < 0.02f && fabsf(kf_state.v_est) < 0.1f)
        {
            kf_state.v_est *= expf(-(KF_s::zupt_decay_rate * 0.2f) * dt);
        }

        return kf_state.v_est;
    }

    float TD::update(float input, const TD_Config_s &config, float delta_u)
    {
        x1 += delta_u;

        float d = config.r * config.h0 * config.h0;
        float a0 = config.h0 * x2;
        float y = x1 - input + a0;
        float a1 = sqrtf(d * (d + 8.0f * fabsf(y)));
        float a2 = a0 + ((y > 0) ? 1.0f : -1.0f) * (a1 - d) / 2.0f;

        float a;
        if (fabsf(y) > d * config.h0)
            a = a2;
        else
            a = a0 + y;

        float fst;
        if (fabsf(a) > d)
            fst = -config.r * ((a > 0) ? 1.0f : -1.0f);
        else
            fst = -config.r * a / d;

        x1 += x2 * config.h;
        x2 += fst * config.h;

        return x1;
    }

    void TD::reset()
    {
        x1 = 0.0f;
        x2 = 0.0f;
    }

    float TD::get_derivative()
    {
        return x2;
    }

    void QP(float u[2], float w[2], float k1[2], float k2[2], float P_limit, float u_star[2])
    {
        constexpr float T_MAX_W = 4.0f;
        const float u0L = module::abs_clip(u[0], T_MAX_W);
        const float u0R = module::abs_clip(u[1], T_MAX_W);

        const float c0 = k1[0] * fabsf(w[0]) + k1[1] * fabsf(w[1]);

        auto calc_power = [&](float uL, float uR) -> float {
            return (k2[0] * uL * uL + w[0] * uL) +
                   (k2[1] * uR * uR + w[1] * uR) + c0;
        };

        if (calc_power(u0L, u0R) <= P_limit)
        {
            u_star[0] = u0L;
            u_star[1] = u0R;
            return;
        }

        // 根据拉格朗日乘子 lambda 计算使得拉格朗日函数 L(u, lambda) 最小的约束 u
        auto calc_u = [&](float lambda, float &uL, float &uR) {
            uL = module::abs_clip((2.0f * u0L - lambda * w[0]) / (2.0f * (1.0f + lambda * k2[0])), T_MAX_W);
            uR = module::abs_clip((2.0f * u0R - lambda * w[1]) / (2.0f * (1.0f + lambda * k2[1])), T_MAX_W);
        };

        // lambda -> inf
        float u_minL = module::abs_clip(-w[0] / (2.0f * k2[0] + 1e-6f), T_MAX_W);
        float u_minR = module::abs_clip(-w[1] / (2.0f * k2[1] + 1e-6f), T_MAX_W);
        
        if (calc_power(u_minL, u_minR) > P_limit)
        {
            u_star[0] = u_minL;
            u_star[1] = u_minR;
            return;
        }

        // KKT条件二分查找
        float lambda_low = 0.0f;
        float lambda_high = 100.0f;
        float uL, uR;

        // 探查合法的 upper_bound
        calc_u(lambda_high, uL, uR);
        while (calc_power(uL, uR) > P_limit && lambda_high < 1e6f)
        {
            lambda_low = lambda_high;
            lambda_high *= 2.0f;
            calc_u(lambda_high, uL, uR);
        }

        for (int i = 0; i < 20; ++i)
        {
            float lambda_mid = 0.5f * (lambda_low + lambda_high);
            calc_u(lambda_mid, uL, uR);
            if (calc_power(uL, uR) > P_limit)
            {
                lambda_low = lambda_mid;
            }
            else
            {
                lambda_high = lambda_mid;
            }
        }
        calc_u(lambda_high, u_star[0], u_star[1]);
    }

    impedance_controller::impedance_controller(Impedance_Controller_Init_Config_s &config, float *getpos, float *getvel)
    {
        this->Kp = config.Kp;
        this->Kd = config.Kd;
        this->alpha = config.alpha;
        get_pos_ptr = getpos;
        get_vel_ptr = getvel;
        filtered_vel = 0.0f;
        use_pos_diff = false;
    }

    impedance_controller::impedance_controller(Impedance_Controller_Init_Config_s &config, float *getpos)
    {
        this->Kp = config.Kp;
        this->Kd = config.Kd;
        this->alpha = config.alpha;
        get_pos_ptr = getpos;
        get_vel_ptr = nullptr;  // 不使用外部速度
        filtered_vel = 0.0f;
        last_pos = (getpos != nullptr) ? *getpos : 0.0f; // 初始化上一帧位置
        use_pos_diff = true;  // 启用位置微分模式
    }

    float impedance_controller::calculate(float set_pos, float set_vel)
    {
        float get_pos = (get_pos_ptr != nullptr) ? *get_pos_ptr : 0.0f;
        float get_vel = 0.0f;
        float dt = DWT_GetDeltaT(&dwt_count);

        if (dt > 0.05f) // 防止过大dt导致微分异常
        {
            dt = 0.001f;
        }

        if (use_pos_diff)
        {
            // 位置微分模式：通过位置差分计算速度
            get_vel = (get_pos - last_pos) / dt;
            last_pos = get_pos;
        }
        else
        {
            // 外部速度模式：直接使用外部速度
            get_vel = (get_vel_ptr != nullptr) ? *get_vel_ptr : 0.0f;
        }

        filtered_vel = alpha * filtered_vel + (1.0f - alpha) * get_vel;

        float pout = Kp * (set_pos - get_pos);
        float dout = Kd * (set_vel - filtered_vel);

        return pout + dout;
    }

    float impedance_controller::calculate(const Trajectory_Point_s& traj_point)
    {
        return calculate(traj_point.pos, traj_point.vel);
    }

    void impedance_controller::reset()
    {
        filtered_vel = 0.0f;
        last_pos = (get_pos_ptr != nullptr) ? *get_pos_ptr : 0.0f;
    }

    void impedance_controller::param_update(float Kp, float Kd)
    {
        this->Kp = Kp;
        this->Kd = Kd;
    }

    void trajectory_generator::plan_trajectory(float start_pos, float end_pos, float v_lift, float duration)
    {
        if (duration <= 0.0f)
        {
            a0 = start_pos;
            a1 = 0.0f;
            a2 = 0.0f;
            a3 = 0.0f;
        }
        else
        {
            a0 = start_pos;
            a1 = v_lift; // 起跳初始速度
            a2 = (3.0f * (end_pos - start_pos) - 2.0f * v_lift * duration) / (duration * duration);
            a3 = (2.0f * (start_pos - end_pos) + v_lift * duration) / (duration * duration * duration);
        }
        this->duration = duration;
        is_planned = true;
    }

    Trajectory_Point_s trajectory_generator::get_trajectory_point(float current_time)
    {
        if (!is_planned)
        {
            current_point.pos = 0.20f;
            current_point.vel = 0.0f;
            current_point.acc = 0.0f;
            return current_point;
        }

        if (current_time >= duration)
        {
            current_point.pos = a0 + a1 * duration + a2 * duration * duration + a3 * duration * duration * duration;
            current_point.vel = 0.0f; // 结束时速度为0
            current_point.acc = 0.0f; // 结束时加速度为0
        }
        else
        {
            current_point.pos = a0 + a1 * current_time + a2 * current_time * current_time + a3 * current_time * current_time * current_time;
            current_point.vel = a1 + 2.0f * a2 * current_time + 3.0f * a3 * current_time * current_time;
            current_point.acc = 2.0f * a2 + 6.0f * a3 * current_time;
        }
        return current_point;
    }
    
    void trajectory_generator::reset()
    {
        a0 = 0.0f;
        a1 = 0.0f;
        a2 = 0.0f;
        a3 = 0.0f;
        duration = 0.0f;
        is_planned = false;
    }
}
