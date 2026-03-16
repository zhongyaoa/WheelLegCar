/*
    姿态控制相关函数
        -IMU数据获取
        -互补滤波
        -PID控制
        -姿态闭环控制
*/

#ifndef _POSTURE_CONTROL_H_
#define _POSTURE_CONTROL_H_
#include "zf_common_headfile.h"

typedef struct{
    int16* gyro_raw_data_l; //角速度原始数据的地址
    int16* acc_raw_data_l;  //加速度原始数据的地址
    float gyro_ration;  //角速度置信度
    float acc_ration;   //加速度置信度
    float dt;          //采样时间间隔
    float temp_value; //临时变量
    float filtered_value; //互补滤波后的值
    int16 mechanical_offset; //机械偏置
    

}cascade_common_value_struct;

typedef struct{
    float kp;           // 比例系数
    float ki;           // 积分系数
    float kd;           // 微分系数
    float i_value;      // 积分值
    float i_value_max;  // 积分限幅
    float out;          // 输出值
    float p_value_last; // 上次比例项(用于微分计算)
}pid_cycle_struct;

typedef struct{
    cascade_common_value_struct cascade_common_value; // 互补滤波结构体
    pid_cycle_struct angular_speed_cycle; // 角速度闭环控制结构体
    pid_cycle_struct angle_cycle;         // 角度闭环控制结构体
    pid_cycle_struct speed_cycle;         // 速度闭环控制结构体
}cascade_value_struct;

extern uint32 system_count;//系统计数器
extern cascade_value_struct cascade_value;
extern bool run_flag;
extern int16 car_speed;
extern float target_speed;

void imu_data_get(void);
void pit_isr_callback(void);
void cascade_init(void);
void first_order_complementary_filter(cascade_common_value_struct* filter,int16 gyro_raw_data,int16 acc_raw_data);
void pid_control_pd(pid_cycle_struct* pid_cycle,float target_value,float current_value);

#endif