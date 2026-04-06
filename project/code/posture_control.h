/*
    姿态与车辆控制模块
        - 车体状态计算
        - 舵机控制
        - 电机控制
        - 周期回调
*/

#ifndef _POSTURE_CONTROL_H_
#define _POSTURE_CONTROL_H_

#include "zf_common_headfile.h"
//#include "balance_control.h"
//#include "steer_control.h"
//#include "small_driver_uart_control.h"
//#define WHEEL_CIRCUMFERENCE_CM 20.1f     // 轮子周长(cm)，直径约 6.4cm
#define WHEEL_DIAMETER 6.4f //cm
#define INAV_SPEED_FILTER_ALPHA 0.2f      // 惯导速度低通滤波系数
#define INAV_STATIONARY_SPEED_RPM 30      // 低于该转速认为接近静止
#define INAV_LOCK_HEADING_SAMPLES 20      // 锁定初始航向时的平均采样数


// 车体运行状态变量
extern uint32 sys_times;
extern uint8 system_time_state[20];

extern uint8 run_state;
extern uint8 balance_enable;  // 平衡使能：0=不自动平衡，1=允许平衡
extern uint8 jump_flag;
extern uint32 jump_time;

// 运动与电机控制变量
extern int16 car_speed;
extern float target_speed;
extern float car_distance;

extern int16 left_motor_duty;
extern int16 right_motor_duty;
extern int16 balance_duty_max;
extern int16 turn_duty_max;

// 外部差速注入（循迹模块写入，正值左转，负值右转，单位与 duty 相同）
extern int16 turn_diff_ext;
// 兼容保留：偏航角（°），当前同步为四元数偏航角
extern float imu_yaw_deg;
// 四元数解算偏航角（°）及其差分角速度（°/s），供循迹模块使用
extern float quat_yaw_deg;
extern float quat_yaw_rate_dps;

// 惯性导航推算位置（m），以 inav_heading_ref 方向为 X 轴正方向
// inav_active = 1 时开始积分，由循迹模块控制
extern float inav_x;
extern float inav_y;
extern uint8 inav_active;
extern float inav_heading_ref;  // 坐标系参考航向（°），X 轴正方向对应此航向

// 车辆控制主流程
void car_state_calculate(void);
void car_steer_control(void);
void car_motor_control(void);
void pit_call_back(void);

#endif
