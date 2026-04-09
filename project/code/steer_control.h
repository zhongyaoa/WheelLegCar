#ifndef _STEER_CONTROL_H_
#define _STEER_CONTROL_H_

#include "zf_common_headfile.h"

#define STEER_1_PWM        (TCPWM_CH12_P01_0)    // 舵机控制引脚
#define STEER_1_FRE        (300)                 // 舵机控制频率
#define STEER_1_DIR        (1)                   // 舵机旋转方向(车体升高方向)
#define STEER_1_CENTER     (4500)                // 舵机中心值(初始保持位置，大腿水平)

#define STEER_2_PWM        (TCPWM_CH21_P08_2)    // 舵机控制引脚
#define STEER_2_FRE        (300)                 // 舵机控制频率
#define STEER_2_DIR        (-1)                  // 舵机旋转方向(车体升高方向)
#define STEER_2_CENTER     (4500)                // 舵机中心值(初始保持位置，大腿水平)

#define STEER_3_PWM        (TCPWM_CH13_P00_3)    // 舵机控制引脚
#define STEER_3_FRE        (300)                 // 舵机控制频率
#define STEER_3_DIR        (-1)                  // 舵机旋转方向(车体升高方向)
#define STEER_3_CENTER     (4500)                // 舵机中心值(初始保持位置，大腿水平)

#define STEER_4_PWM        (TCPWM_CH11_P01_1)    // 舵机控制引脚
#define STEER_4_FRE        (300)                 // 舵机控制频率
#define STEER_4_DIR        (1)                   // 舵机旋转方向(车体升高方向)
#define STEER_4_CENTER     (4500)                // 舵机中心值(初始保持位置，大腿水平)

typedef struct
{
    pwm_channel_enum    pwm_pin;             // PWM 通道引脚
    int16               control_frequency;   // 控制频率（Hz）
    int16               steer_dir;           // 转动方向（1 正方向 / -1 反方向）
    int16               center_num;          // 中心位置值（初始位置）

    int16               steer_state;         // 舵机当前状态（0 禁用 / 1 使能）
    int16               now_location;        // 舵机当前位置（PWM 占空比值）
}steer_control_struct;

// 备注信息：初始化后，舵机位置为初始水平位置，最小值为 2400，最大值为 5400，达到最大值时舵机垂直于地面
// 声明 4 路舵机控制结构体实例，供外部文件调用
extern steer_control_struct steer_1;
extern steer_control_struct steer_2;
extern steer_control_struct steer_3;
extern steer_control_struct steer_4;

// ==============================================
// 函数简介    舵机控制初始化函数
// 返回参数    void
// 使用示例    steer_control_init();
// 备注信息    初始化 4 路舵机的基础参数和 PWM 配置，默认使能所有舵机
// ==============================================
void steer_control_init(void);

//=============================================================================
// 函数简介     舵机占空比直接设置函数
// 返回参数     void
// 使用示例     steer_duty_set(&steer_1, 4500);
// 备注信息     直接设置舵机 PWM 占空比，范围限制 -10000 ~ 10000，仅使能时生效
//=============================================================================
void steer_duty_set(steer_control_struct *control_data, int16 duty);

//=============================================================================
// 函数简介     舵机相对位置控制函数
// 返回参数     void
// 使用示例     steer_control(&steer_1, 100);
// 备注信息     按偏移量控制舵机转动，方向由 steer_dir 决定，位置限制 -10000 ~ 10000，仅使能时生效
//=============================================================================
void steer_control(steer_control_struct *control_data, int16 move_num);

#endif

