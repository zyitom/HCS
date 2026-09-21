#pragma once
#include <cstdint>
#include <bitset>

namespace module
{

    struct  moto_data_receive_s
    {
        float temp;         // 电机温度              ℃
        float real_current; // 实际转矩电流           A
        float real_torque;  // 实际转矩              Nm

        int32_t round_cnt;  // 电机转过的圈数
        float angle;        // 单圈内的角度  rad   -phi到phi
        float total_angle;  // 多圈,相对于中值的距离  rad
        float speed;        // 转速   rad/s
        float offset_angle; // 与初始角度的夹角 rad

        bool is_online;
        uint8_t error;

        float send_data;   // 发送数据指针
    }; // 标准电机数据包

    struct  imu_data_t
    {

        float yaw; // rad
        float pitch;
        float roll;

        float dyaw; // rad/s
        float dpitch;
        float droll;

        float total_imu_yaw;
        uint32_t ts;
        float acc_x;
        float acc_y;
        float acc_z; // m/s^2

        int32_t round_cnt = 0;
    };

    // 历史值下标
    enum Index
    {
        LLAST = 0,
        LAST = 1,
        NOW = 2,
    };

}