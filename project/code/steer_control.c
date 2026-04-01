#include "steer_control.h"

// 定义 4 个舵机控制结构体实例，分别对应 4 路舵机
steer_control_struct steer_1;
steer_control_struct steer_2;
steer_control_struct steer_3;
steer_control_struct steer_4;

//=============================================================================
// 函数简介     舵机控制初始化函数
// 返回参数     void
// 使用示例     steer_control_init();
// 备注信息    初始化 4 路舵机的 PWM 引脚、控制频率、转向方向、中心值等参数，
//              设置初始位置为中心值并初始化 PWM，默认使能所有舵机
//=============================================================================
void steer_control_init(void)
{
    steer_1.pwm_pin            = STEER_1_PWM;        // 舵机 1 PWM 引脚配置
    steer_1.control_frequency = STEER_1_FRE;        // 舵机 1 控制频率配置
    steer_1.steer_dir          = STEER_1_DIR;        // 舵机 1 转动方向配置
    steer_1.center_num         = STEER_1_CENTER;     // 舵机 1 中心位置（初始位置）配置

    steer_2.pwm_pin            = STEER_2_PWM;        // 舵机 2 PWM 引脚配置
    steer_2.control_frequency = STEER_2_FRE;        // 舵机 2 控制频率配置
    steer_2.steer_dir          = STEER_2_DIR;        // 舵机 2 转动方向配置
    steer_2.center_num         = STEER_2_CENTER;     // 舵机 2 中心位置（初始位置）配置

    steer_3.pwm_pin            = STEER_3_PWM;        // 舵机 3 PWM 引脚配置
    steer_3.control_frequency = STEER_3_FRE;        // 舵机 3 控制频率配置
    steer_3.steer_dir          = STEER_3_DIR;        // 舵机 3 转动方向配置
    steer_3.center_num         = STEER_3_CENTER;     // 舵机 3 中心位置（初始位置）配置

    steer_4.pwm_pin            = STEER_4_PWM;        // 舵机 4 PWM 引脚配置
    steer_4.control_frequency = STEER_4_FRE;        // 舵机 4 控制频率配置
    steer_4.steer_dir          = STEER_4_DIR;        // 舵机 4 转动方向配置
    steer_4.center_num         = STEER_4_CENTER;     // 舵机 4 中心位置（初始位置）配置

    steer_1.now_location       = steer_1.center_num; // 舵机 1 当前位置初始化为中心位置
    steer_2.now_location       = steer_2.center_num; // 舵机 2 当前位置初始化为中心位置
    steer_3.now_location       = steer_3.center_num; // 舵机 3 当前位置初始化为中心位置
    steer_4.now_location       = steer_4.center_num; // 舵机 4 当前位置初始化为中心位置

    // 初始化 4 路舵机的 PWM 输出，频率为配置值，初始占空比为中心位置值
    pwm_init(steer_1.pwm_pin, steer_1.control_frequency, steer_1.now_location);
    pwm_init(steer_2.pwm_pin, steer_2.control_frequency, steer_2.now_location);
    pwm_init(steer_3.pwm_pin, steer_3.control_frequency, steer_3.now_location);
    pwm_init(steer_4.pwm_pin, steer_4.control_frequency, steer_4.now_location);

    steer_1.steer_state        = 1;                  // 舵机 1 使能（1 为使能，0 为禁用）
    steer_2.steer_state        = 1;                  // 舵机 2 使能（1 为使能，0 为禁用）
    steer_3.steer_state        = 1;                  // 舵机 3 使能（1 为使能，0 为禁用）
    steer_4.steer_state        = 1;                  // 舵机 4 使能（1 为使能，0 为禁用）
}

//=============================================================================
// 函数简介     舵机占空比直接设置函数
// 返回参数     void
// 使用示例     steer_duty_set(&steer_1, 4500);
// 备注信息     直接设置舵机的 PWM 占空比，占空比范围被限制在 -10000 ~ 10000 之间，仅在舵机使能状态下生效
//=============================================================================
void steer_duty_set(steer_control_struct *control_data, int16 duty)
{
    if(control_data->steer_state)                     // 判断舵机是否处于使能状态
    {
        // 将输入的占空比限制在 -10000 ~ 10000 范围内，并更新当前位置
        control_data->now_location = func_limit_ab(duty, -10000, 10000);

        pwm_set_duty(control_data->pwm_pin, duty);    // 设置 PWM 占空比
    }
}

//=============================================================================
// 函数简介     舵机禁用函数
// 返回参数     void
// 使用示例     steer_disable(&steer_1);
// 备注信息     禁用指定舵机，设置舵机状态为禁用，并将 PWM 占空比设置为 0
//=============================================================================
void steer_disable(steer_control_struct *control_data)
{
    control_data->steer_state = 0;                   // 将舵机状态设置为禁用（0 为禁用）
    pwm_set_duty(control_data->pwm_pin, 0);          // PWM 占空比设置为 0，舵机停止工作
}

//=============================================================================
// 函数简介     舵机相对位置控制函数
// 返回参数     void
// 使用示例     steer_control(&steer_1, 100);
// 备注信息     根据偏移量控制舵机相对当前位置转动，偏移量正负由转向方向参数 steer_dir 决定，最终位置限制在 -10000 ~ 10000
//=============================================================================
void steer_control(steer_control_struct *control_data, int16 move_num)
{
    if(control_data->steer_state)                     // 判断舵机是否处于使能状态
    {
        // 根据转向方向计算新位置：steer_dir 为 1 时正向偏移，为 -1 时反向偏移
        control_data->now_location = control_data->now_location + (control_data->steer_dir == 1 ? move_num : -move_num);

        // 将新位置限制在 -10000 ~ 10000 范围内
        control_data->now_location = func_limit_ab(control_data->now_location, -10000, 10000);

        pwm_set_duty(control_data->pwm_pin, control_data->now_location);  // 设置 PWM 占空比更新舵机位置
    }
}