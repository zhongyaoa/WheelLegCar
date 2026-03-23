#ifndef SERVO_H_
#define SERVO_H_
#include "zf_common_headfile.h"
#define L1            (TCPWM_CH13_P00_3)                          // 定义主板上舵机对应引脚
#define L2            (TCPWM_CH12_P01_0)                          // 定义主板上舵机对应引脚
#define R1            (TCPWM_CH11_P01_1)                          // 定义主板上舵机对应引脚
#define R2            (TCPWM_CH21_P08_2)                          // 主板上的有刷电机2引脚飞线
#define SERVO_MOTOR_FREQ            (50 )                                       // 定义主板上舵机频率  请务必注意范围 50-300

#define SERVO_MOTOR_L_MAX           (50 )                                       // 定义主板上舵机活动范围 角度
#define SERVO_MOTOR_R_MAX           (150)                                       // 定义主板上舵机活动范围 角度

#define SERVO_MOTOR_DUTY(x)         ((float)PWM_DUTY_MAX/(1000.0/(float)SERVO_MOTOR_FREQ)*(0.5+(float)(x)/90.0))

#if (SERVO_MOTOR_FREQ<50 || SERVO_MOTOR_FREQ>300)
    #error "SERVO_MOTOR_FREQ ERROE!"
#endif

extern float servo_motor_duty;                                                  // 舵机动作角度
extern float servo_motor_dir;                                                      // 舵机动作状态

void servo_init(void);
void servo_set_angle(pwm_channel_enum channel, float angle);


#endif