#include "servo.h"

float servo_motor_duty = 90.0;                                                  // 舵机动作角度
float servo_motor_dir = 1;                                                      // 舵机动作状态

void servo_init(void)
{
    pwm_init(L1, SERVO_MOTOR_FREQ, 300);
    pwm_init(L2, SERVO_MOTOR_FREQ, 300);
    pwm_init(R1, SERVO_MOTOR_FREQ, 300);
    pwm_init(R2, SERVO_MOTOR_FREQ, 300);
    servo_set_angle(L1, 0);
    servo_set_angle(L2, 0);
    servo_set_angle(R1, 0);
    servo_set_angle(R2, 0);
}

void servo_set_angle(pwm_channel_enum channel, float angle)
{
    if(channel == L1 || channel == R2)
    {
        angle = -angle;
    }
    angle = 90.0f - angle; // 将输入的角度转换为轮腿的角度，向下为正，向上为负
    
    if(angle < 0)
        angle = 0;
    else if(angle > 180)
        angle = 180;

    float duty = SERVO_MOTOR_DUTY(angle);
    pwm_set_duty(channel, (uint16)duty);
}