#include "controler.h"
#include "posture_control.h"
#include <string.h>
#include <stdarg.h>


void button_init(void){
    gpio_init(BUTTON_UP, GPI, GPIO_HIGH, GPI_PULL_UP);               // 初始化 KEY1 输入 默认高电平 上拉输入
    gpio_init(BUTTON_DOWN, GPI, GPIO_HIGH, GPI_PULL_UP);               // 初始化 KEY2 输入 默认高电平 上拉输入
    gpio_init(BUTTON_LEFT, GPI, GPIO_HIGH, GPI_PULL_UP);               // 初始化 KEY3 输入 默认高电平 上拉输入
    gpio_init(BUTTON_NE, GPI, GPIO_HIGH, GPI_PULL_UP);               // 初始化 KEY4 输入 默认高电平 上拉输入
    gpio_init(BUTTON_SE, GPI, GPIO_HIGH, GPI_PULL_UP);               // 初始化 KEY5 输入 默认高电平 上拉输入
    gpio_init(BUTTON_SW, GPI, GPIO_HIGH, GPI_PULL_UP);               // 初始化 KEY6 输入 默认高电平 上拉输入
}

void led_init(void){
    gpio_init(LED, GPO, GPIO_HIGH, GPO_PUSH_PULL);              // 初始化 LED 输出 默认高电平 推挽输出
}

bool button_press(Button bt){
    switch(bt){
        case UP:
            return !gpio_get_level(BUTTON_UP);                       // 按键按下返回 true
        case DOWN:
            return !gpio_get_level(BUTTON_DOWN);                     // 按键按下返回 true
        case LEFT:
            return !gpio_get_level(BUTTON_LEFT);                     // 按键按下返回 true
        case NE:
            return !gpio_get_level(BUTTON_NE);                       // 按键按下返回 true
        case SE:
            return !gpio_get_level(BUTTON_SE);                       // 按键按下返回 true
        case SW:
            return !gpio_get_level(BUTTON_SW);                       // 按键按下返回 true
        default:
            return false;
    }
}

void led(LedCmd cmd){
    switch(cmd){
        case on:
            gpio_set_level(LED, 0);                             // LED 灯打开
            break;
        case off:
            gpio_set_level(LED, 1);                             // LED 灯关闭
            break;
        case toggle:
            gpio_set_level(LED, !gpio_get_level(LED));          // LED 灯取反
            break;
        default:
            break;
    }
}

void serial_optimizer_init(void){
    uart_init(OPTIMIZER_UART, OPTIMIZER_BAUDRATE, OPTIMIZER_RX, OPTIMIZER_TX);      // 串口初始化
    
    uart_rx_interrupt(OPTIMIZER_UART, 1);                                                    // 使能串口接收中断
}

void serial_optimizer_callback(cascade_value_struct* cascade_value_ptr){ //接收串口信息，调整参数
    // 帧格式（8字节）：[0]=0x55, [1]=0x20, [2]=channel, [3]=reserved, [4..7]=little-endian float
    static uint8 rx_buffer[8] = {0};
    static uint8 rx_index = 0;
    uint8 receive_data;

    if(uart_query_byte(OPTIMIZER_UART, &receive_data)) // 接收串口数据（逐字节）
    {
        // 帧头重同步：收到0x55且当前缓冲不是以0x55开头，则重置
        if(receive_data == 0x55 && rx_buffer[0] != 0x55)
        {
            rx_index = 0;
        }

        rx_buffer[rx_index++] = receive_data; // 保存串口数据

        // 收满8字节后处理
        if(rx_index >= 8)
        {
            // 检查帧头和命令字
            if((rx_buffer[0] == 0x55) && (rx_buffer[1] == 0x20))
            {
                uint8 channel = rx_buffer[2];
                float f;
                memcpy(&f, &rx_buffer[4], sizeof(float)); // 小端浮点

                switch(channel)
                {
                    case 0x01: // 通道1：机械偏置
                        cascade_value_ptr->posture_value.mechanical_zero = f;
                        break;
                    case 0x02: // 通道2：角速度闭环 kp
                        cascade_value_ptr->angular_speed_cycle.p = f;
                        break;
                    case 0x03: // 通道3：角速度闭环 kd
                        cascade_value_ptr->angular_speed_cycle.d = f;
                        break;
                    case 0x04: // 通道4：角度闭环 kp
                        cascade_value_ptr->angle_cycle.p = f;
                        break;
                    case 0x05: // 通道5：角度闭环 kd
                        cascade_value_ptr->angle_cycle.d = f;
                        break;
                    case 0x06: // 通道6：速度闭环 kp
                        cascade_value_ptr->speed_cycle.p = f;
                        break;
                    default:
                        // 其他通道暂不处理
                        break;
                }
            }

            // 重置准备接收下一帧
            rx_index = 0;
            memset(rx_buffer, 0, 8);
        }
    }
}

// 使用 OPTIMIZER_UART 无线串口打印信息
void air_printf(const char* fmt, ...)
{
    static uint8 air_printf_buffer[256] = {0};  // 缓冲区用于存储格式化后的字符串
    int32 str_length;
    va_list arg;

    // 解析可变参数列表
    va_start(arg, fmt);
    // 将格式化字符串写入缓冲区
    str_length = vsnprintf((char *)air_printf_buffer, sizeof(air_printf_buffer) - 1, fmt, arg);
    va_end(arg);

    // 如果格式化成功，通过 UART 发送
    if(str_length > 0)
    {
        //uart_write_buffer(OPTIMIZER_UART, air_printf_buffer, (uint32)str_length);
        wireless_uart_send_buffer(air_printf_buffer, (uint32)str_length); // 通过无线串口发送
    }
}

void my_wireless_optimizer(uint8 data)
{
    if('w' == data)
        {
            target_speed = -50.0f;// 前进
        }
        else if('s' == data)
        {
            target_speed = 50.0f; // 后退
        }
        else if('a' == data)
        {
            run_state = true; // 启动控制
        }
        else if('d' == data)
        {
            run_state = false; // 停止控制
        }
        else if('j' == data)
        {
            roll_balance_cascade.posture_value.mechanical_zero += 1; // 增加机械偏置
        }
        else if('k' == data)
        {
            roll_balance_cascade.posture_value.mechanical_zero -= 1; // 减小机械偏置
        }
        else if('!' == data)
        {
            target_speed = 0.0f; 
        }
}