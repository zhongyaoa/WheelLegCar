#ifndef _BALANCE_CONTROL_H_
#define _BALANCE_CONTROL_H_

#include "zf_common_headfile.h"

// 陀螺仪数据转换（原始数据转换）
#define GYRO_DATA_X               ( imu660ra_gyro_x)      // 陀螺仪 X 轴原始数据
#define GYRO_DATA_Y               (-imu660ra_gyro_y)      // 陀螺仪 Y 轴原始数据（带方向转换）
#define GYRO_DATA_Z               (-imu660ra_gyro_z)      // 陀螺仪 Z 轴原始数据（带方向转换）
#define GYRO_TRANSITION_FACTOR    (16.384f)               // 陀螺仪数据转换系数 (LSB/(°/s))

// 加速度计数据转换
#define ACC_DATA_X                ( imu660ra_acc_x)       // 加速度计 X 轴原始数据
#define ACC_DATA_Y                (-imu660ra_acc_y)       // 加速度计 Y 轴原始数据（带方向转换）
#define ACC_DATA_Z                (-imu660ra_acc_z)       // 加速度计 Z 轴原始数据（带方向转换）
#define ACC_TRANSITION_FACTOR     (4096.0f)               // 加速度计数据转换系数 (LSB/g)

#define ACC_GRAVITY               (9.80665f)              // 重力加速度参考值 (m/s2)

typedef struct quaternion_data
{
    float rot_mat[3][3];                                  // 旋转矩阵
} quaternion_data;

typedef struct quaternion_process
{
    float qua[4];                                         // 四元数数据 (w, x, y, z 顺序)
    float acc_filtered[3] ;                               // 滤波后的加速度数据 (x, y, z 顺序)
}quaternion_process;

typedef struct quaternion_parameter
{
    float acc_err [3];                                    // 加速度计误差 (x, y, z 顺序)
}quaternion_parameter;

typedef struct quaternion_module
{
    quaternion_data data;                                  // 四元数数据结构体
    quaternion_process pro;                              // 四元数处理结构体
    quaternion_parameter parameter;                          // 四元数参数结构体
}quaternion_module;

typedef struct
{
    float p;
    float i; 
    float d;
    float p_value_last;             // 上一次偏差值
    float i_value;                  // PID 积分值
    float i_value_pro;              // PID 积分值的比例（范围0—1，用于限制积分增长速度）
    float i_value_max;              // PID 积分上限
    float out;                      // PID 控制器输出值
    float out_max;                  // PID 控制器输出上限
    float incremental_data[2];      // 增量式 PID 数据缓存（当前偏差值和上一次偏差值）
}pid_cycle_struct;

typedef struct
{
    float correct_kp;
    float correct_ki;
    float call_cycle;
    float mechanical_zero;
    float yaw;
    float rol;
    float pit;
}cascade_common_value_struct;

//级联控制结构体
typedef struct
{
    quaternion_module           quaternion;
    cascade_common_value_struct posture_value;
    pid_cycle_struct            angular_speed_cycle;
    pid_cycle_struct            angle_cycle;
    pid_cycle_struct            speed_cycle;
} cascade_value_struct;

extern cascade_value_struct roll_balance_cascade;
extern cascade_value_struct roll_balance_cascade_resave;

extern cascade_value_struct pitch_balance_cascade;
extern cascade_value_struct pitch_balance_cascade_resave;

void quaternion_module_calculate (cascade_value_struct *cascade_value);

void pid_control_incremental (pid_cycle_struct *pid_cycle, float target,float real);

void quaternion_module_init(cascade_value_struct *cascade_value);

void balance_cascade_init (void);

#endif