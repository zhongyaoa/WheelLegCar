/*
    无刷电机驱动串口控制
        -串口通讯初始化
        -设置电机占空比
        -获取电机速度信息

*/
#ifndef SMALL_DRIVER_UART_CONTROL_H_
#define SMALL_DRIVER_UART_CONTROL_H_

#include "zf_common_headfile.h"


#define SMALL_DRIVER_UART                       (UART_4)

#define SMALL_DRIVER_BAUDRATE                   (460800)        //460800

#define SMALL_DRIVER_RX                         (UART4_RX_P14_0)

#define SMALL_DRIVER_TX                         (UART4_TX_P14_1)

typedef struct
{
    uint8 send_data_buffer[7];                  // 发送缓冲数组

    uint8 receive_data_buffer[7];               // 接收缓冲数组

    uint8 receive_data_count;                   // 接收计数

    uint8 sum_check_data;                       // 校验位

    int16 receive_left_speed_data;              // 接收到的左侧电机速度数据

    int16 receive_right_speed_data;             // 接收到的右侧电机速度数据

}small_device_value_struct;

extern small_device_value_struct motor_value;
extern uint8 motor_init_ok;




void uart_control_callback(uint8 receive_data);                     // 无刷驱动 串口接收回调函数

void small_driver_set_duty(int16 left_duty, int16 right_duty);      // 无刷驱动 设置电机占空比

void small_driver_get_speed(void);                                  // 无刷驱动 获取速度信息

void small_driver_uart_init(void);                                  // 无刷驱动 串口通讯初始化

float low_pass_filter(float x_n , float alpha);

/**********Motor-Speed-接口(向前为正)********************/
#define Motor_speed_left_raw                motor_value.receive_right_speed_data
#define Motor_speed_right_raw               -motor_value.receive_left_speed_data

#define Motor_speed_left                (int16)low_pass_filter(Motor_speed_left_raw,0.9)
#define Motor_speed_right               (int16)low_pass_filter(Motor_speed_right_raw,0.9)

/*****************************个人定义函数***********************************/
void small_driver_set_speed(int16 target_speed_left,int16 target_speed_right);
void motor_control_init(void);



#endif


