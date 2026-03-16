/*
    控制器模块，包含按键、LED和无线串口调参功能
    - 按键初始化与状态检测
    - LED 初始化与控制
    - 无线串口调参功能实现
    ⚠️ 有依赖问题，不能将头文件加到 zf_common_headfile.h 中
*/
#ifndef CONTROLER_H_
#define CONTROLER_H_

#include "zf_common_headfile.h"
#include "posture_control.h"

#define OPTIMIZER_UART                       (UART_1)
#define OPTIMIZER_BAUDRATE                   (115200)        //蓝牙-串口模块波特率为115200
#define OPTIMIZER_RX                         (UART1_RX_P04_0)
#define OPTIMIZER_TX                         (UART1_TX_P04_1)


#define BUTTON1                    (P20_0)
#define BUTTON2                    (P20_1)
#define BUTTON3                    (P20_2)
#define BUTTON4                    (P20_3)

#define LED                     (P19_0)

typedef enum {
    bt1 = 0,
    bt2,
    bt3,
    bt4
} Button;

typedef enum {
    on = 0,
    off,
    toggle
} LedCmd;

void button_init(void);
bool button_press(Button bt);
void led_init(void);
void led(LedCmd cmd);
void serial_optimizer_callback(cascade_value_struct* cascade_value_ptr); //接收串口信息，调整参数
void serial_optimizer_init(void); //串口初始化
void air_printf(const char* fmt, ...); //无线串口打印函数
void my_wireless_optimizer(uint8 data);


#endif