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
#define WHEEL_DIAMETER 6.4f // 假设轮子直径为6.4cm

// 车体运行状态变量
extern uint32 sys_times;
extern uint8 system_time_state[20];

extern uint8 run_state;
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

// 车辆控制主流程
void car_state_calculate(void);
void car_steer_control(void);
void car_motor_control(void);
void pit_call_back(void);

// 导航航向差速叠加量（由 task_slalom 写入，car_motor_control 读取）
// 正值 = 左转差速，负值 = 右转差速
extern int16 nav_yaw_output;

#endif
