#pragma once

#include "stdint.h"
#include "vision_protocol.hpp"
#include "user_lib.hpp"

#define GIMBAL_RC_PITCH_K 0.000003f
#define GIMBAL_RC_YAW_K  0.000005f
#define CHASSIS_RC_MOVE_K 0.77 // 10
#define MOUSE_PITCH_K -0.000012f
#define MOUSE_YAW_K 0.000016f
#define LEFT 0
#define RIGHT 1


enum chassis_mode_e
{
    CHASSIS_ZERO_FORCE = 0, // 电流零输入
    CHASSIS_ON,
    CHASSIS_FOLLOW,              // 上电
    CHASSIS_NO_FOLLOW,
};
// 底盘模式

enum gimbal_mode_e
{
    GIMBAL_ZERO_FORCE = 0, // 电流零输入
    GIMBAL_ON,
    GIMBAL_AUTO
};
// 云台模式

enum shoot_mode_e
{
    SHOOT_ZERO_FORCE = 0, // 电流零输入
    SHOOT_ALLOW,          // 发射机构上电
};
// 发射模式

enum  vision_object_e
{
    VISION_MODE_AIM = 0,
    VISION_MODE_SMALL_ENERGY,
    VISION_MODE_BIG_ENERGY,
    VISION_MODE_COUNT,
};
// 视觉识别物体

// 接下来是message_center数据包定义

struct  FSM_To_Chassisctl_s
{

    float speed[3];              // 期望的xyw速度
    bool if_free;                // 小陀螺或者自由下都会free
    float chassis_turn_count;    // 底盘从上次小陀螺回正后转的角度（如1/4为转了底盘的1/4）
    bool if_reset;               // 是否重置底盘（如飞坡前），底盘接到这个为1后需要回到记忆中的初始位置
    chassis_mode_e chassis_mode; // 底盘是否开启（只是遥控器控制的，不包含裁判系统）
    bool if_fly;                 // 是否飞坡

    bool leg_key;         // 腿长按键
    bool save_key;        // 自救按键
    bool jump_key;        // 跳跃按键
    bool if_turn;         // 是否转向,每回头一次变一下方向
    bool manual_rst_ctrl; // 手动重置控制器
    bool if_chassis_dead; // 底盘是否失能
    bool if_auto_jump;    // 是否自动跳跃
}; // 状态机

struct FSM_To_Gimbalctl_s
{
    float yaw; // pitch yaw
    float pitch;
    bool if_auto_aim; // 是否自瞄
    gimbal_mode_e gimbal_mode;
    bool if_chassis_dead;
};

struct FSM_To_Shootctl_s
{
    shoot_mode_e shoot_mode; // 发射机构是否开启
    uint8_t shoot_frq;       // 射频
    bool if_want_fire;       // 左键有没有按下
    bool if_auto_fire;       // 是否自动射击
    bool if_friction_on;     // 摩擦轮是否开启
    float shoot_speed;       // 期望摩擦轮转速调节量
};

struct FSM_To_Vision_t
{
    vision_object_e vision_object; // 自瞄要打的东西
};

struct FSM_To_UI_t
{
  bool if_refresh;   //如果是0，为创建。如果为1，为刷新
  bool if_auto_fire;  //是否自动开火
  vision_object_e vision_object;//0 自瞄，1 小符，2 大符
};

struct Shoot_Ctrl_Data_s
{
    float shoot_frq;
    float shoot_speed;
    uint8_t is_disable;
    uint16_t heat_max; 
    uint16_t heat_cooling;
    uint16_t heat;
    uint16_t current_hp;
};

struct Gimbal_Ctrl_Data_s
{
    float yaw;
    float pitch;
    int pid_type;
    bool is_disable;
    float hero_yaw;
    float hero_pitch;
    bool if_chassis_dead;
};

struct  Chassis_Ctrl_Data_s
{
    float speed[3];
    float follow_angle; // 底盘跟随角度
    float real_angle;
    float yaw_speed;
    uint8_t if_enable;
    uint8_t if_free; // 是否自由
    bool if_IBC_online;
    bool if_fly; // 是否飞坡

    bool leg_key;         // 腿长按键
    bool save_key;        // 自救按键
    bool jump_key;        // 跳跃按键
    bool if_turn;         // 是否转向,每回头一次变一下方向
    bool if_auto_jump;    // 是否自动跳跃
    bool if_manual_rst_ctrl; // 手动重置控制器

    float leg_length[2];
    bool is_fatal_error;
    float leg_angle[2];
    float yaw_real_telemetry;
    float pitch_rounded;     // chassis body pitch rounded to 0.05 rad, for gimbal limit offset
    uint8_t chassis_state;
};

struct Gimbal_Upload_Data_s
{
    float yaw_motor_single_round_angle;
    float yaw_motor_speed;
};

struct Chassis_Ctrl_Pack1_s
{
    uint8_t header; //                                           00

    uint16_t speed[3]; // 期望的xyw速度float16
    uint8_t if_enable;

    uint8_t tx_buff[8];

}; // uint8[8]

struct Chassis_Ctrl_Pack2_s
{
    uint8_t header; //                                           00

    uint16_t motor_angle; //云台电机偏差角
    uint16_t follow_angle; // 底盘跟随角度float16
    uint16_t yaw_rpm;    // 电机旋转速度float16

    uint8_t if_free; // 是否自由
    uint8_t if_fly;
    
    uint8_t tx_buff[8];

}; // uint8[8]

struct Chassis_Ctrl_Pack3_s
{
    uint8_t header; //                                           00

    uint8_t speed[2]; // 期望的x,w速度float8
    uint8_t advanced_ctrl; // 第一二位指示是否自动跳跃(11 / 00)
    // 第三四位指示是否手动重置控制器(11 / 00) 
    // 第五六位指示手动跳跃是否被按下(11 / 00)
    // 第七八位指示数据是否有效(11 / 00)
    uint8_t if_L;
    uint8_t if_turn;
    uint8_t if_save;
    uint8_t if_enable;

    uint8_t tx_buff[8];

} ; // uint8[8]

struct Chassis_Ctrl_Pack4_s
{
    uint8_t header; // 11

    uint8_t leg_length[2]; // float8
    uint8_t mode_and_error;      // bit[7]=is_fatal_error, bit[3:0]=chassis_mode
    uint8_t leg_angle[2];  // float8
    uint8_t yaw_real;      // float8
    uint8_t pitch_rounded; // float8, pitch rounded to nearest 0.05 rad

    uint8_t tx_buff[8];
}; // uint8[8]

struct Vision_Cmd_t
{
    float yaw;
    float pitch;
    bool is_fire;
    bool is_aim_ok;//识别到目标
};


