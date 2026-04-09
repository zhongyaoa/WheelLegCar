#include "posture_control.h"
#include "nav_heading.h"

uint32 sys_times = 0;
uint8 system_time_state[20] = {0};

uint8 run_state = 0;
uint8 jump_flag = 0;
uint32 jump_time = 0;

int16 car_speed = 0;
float target_speed = 0.0f;
float car_distance = 0.0f;


int16 left_motor_duty = 0;
int16 right_motor_duty = 0;
int16 balance_duty_max = 10000;
int16 turn_duty_max = 3000;

//=============================================================================
// 函数简介     计算并更新车辆状态标志
// 返回参数     void
// 使用示例     car_state_calculate();
// 备注信息     根据横滚角、转向电流、实际距离等参数切换车辆悬挂状态和运行状态，同时处理 PID 参数的渐变
//=============================================================================
void car_state_calculate(void)
{
    if(func_abs(roll_balance_cascade.posture_value.rol) > 40.0f || func_abs(roll_balance_cascade.posture_value.pit) > 40.0f)
    // 当横滚角与俯仰角绝对值大于 40 度时，认为小车倾倒
    {
        jump_flag = 0;
        run_state = 0;                              // 停止运行
        roll_balance_cascade.angular_speed_cycle.i_value = 0;  // 重置角速度环 PID 积分值
        pitch_balance_cascade.angle_cycle.i_value = 0;
    }
    else
    {
        if(run_state == 0)
        {
            sys_times = 0;
            run_state = 1;
        }
    }


    if(sys_times < 500)                                      // 系统计时小于 500 时，PID 参数从 20% 渐变到 100%
    {
        // 角度环 P 参数渐变 (0.2 到 1.0 倍原始值)
        roll_balance_cascade.angle_cycle.p = roll_balance_cascade_resave.angle_cycle.p * (0.2 + (float)sys_times / 500.0f * 0.8f);
        // 速度环 P 参数渐变 (0.2 到 1.0 倍原始值)
        roll_balance_cascade.speed_cycle.p = roll_balance_cascade_resave.speed_cycle.p * (0.2 + (float)sys_times / 500.0f * 0.8f);

        roll_balance_cascade.angle_cycle.i_value = 0;         // 重置角度环积分值
    }
    else                                                      // 系统计时大于等于 500 时，使用原始 PID 参数
    {
        roll_balance_cascade.angle_cycle.p = roll_balance_cascade_resave.angle_cycle.p;  // 恢复角度环 P 参数
        roll_balance_cascade.speed_cycle.p = roll_balance_cascade_resave.speed_cycle.p;  // 恢复速度环 P 参数
    }

    if(jump_flag)
    {
        roll_balance_cascade.angle_cycle.p = roll_balance_cascade_resave.angle_cycle.p * 0.5f;
        roll_balance_cascade.speed_cycle.p = roll_balance_cascade_resave.speed_cycle.p * 0.5f;

        roll_balance_cascade.angle_cycle.i_value = 0;         // 重置角速度环 PID 积分值
        pitch_balance_cascade.angle_cycle.i_value = 0;
    }
}

//=============================================================================
// 函数简介     车辆舵机控制
// 返回参数     void
// 使用示例     car_steer_control();
// 备注信息     控制跳跃、左右倾斜、腿部前后倾斜、自动复位等操作
//=============================================================================
void car_steer_control(void)
{
    int16 steer_location_offset[4] = {0};
    int16 steer_target_offset[4] = {0};

    static int16 jump_time_num[4] = {110, 110, 30, 100};  // 跳跃各阶段时间  起跳、收腿、准备缓冲、执行缓冲

    static float steer_balance_angle_count = 0;
    static float steer_output_duty_filter = 0;

    int16 steer_output_duty = 0;
    float steer_balance_angle = 0;

    float pitch_offset = (30.0f - func_limit_ab(func_abs(roll_balance_cascade.posture_value.rol + roll_balance_cascade.posture_value.mechanical_zero), 0.0f, 30.0f)) / 30.0f; // 前倾角度偏移量，范围 0~1，前倾越大偏移越小
    steer_output_duty = func_limit_ab((int16)(roll_balance_cascade.speed_cycle.out / 7.0f), -250, 250) * 6;
    steer_output_duty = (int16)((float)steer_output_duty * pitch_offset);

    steer_output_duty_filter = (steer_output_duty_filter * 19 + (float)steer_output_duty) / 20.0f;
    if(jump_flag == 0)
    {
        if(sys_times < 2000)
        {
            steer_balance_angle = 0;
            pitch_balance_cascade.angle_cycle.i_value = 0;
        }
        else
        {
            steer_balance_angle = func_limit_ab(pitch_balance_cascade.angle_cycle.out, -300, 300) * 6;
        }
        steer_balance_angle_count = steer_balance_angle;
    }

    steer_location_offset[0] = (steer_1.now_location - steer_1.center_num) * steer_1.steer_dir;
    steer_location_offset[1] = (steer_2.now_location - steer_2.center_num) * steer_2.steer_dir;
    steer_location_offset[2] = (steer_3.now_location - steer_3.center_num) * steer_3.steer_dir;
    steer_location_offset[3] = (steer_4.now_location - steer_4.center_num) * steer_4.steer_dir;

    steer_target_offset[0] = (int16)( steer_output_duty_filter - (steer_balance_angle_count > 0 ? 0 : steer_balance_angle_count));
    steer_target_offset[1] = (int16)( steer_output_duty_filter + (steer_balance_angle_count < 0 ? 0 : steer_balance_angle_count));
    steer_target_offset[2] = (int16)(-steer_output_duty_filter - (steer_balance_angle_count > 0 ? 0 : steer_balance_angle_count));
    steer_target_offset[3] = (int16)(-steer_output_duty_filter + (steer_balance_angle_count < 0 ? 0 : steer_balance_angle_count));

    if(run_state == 1)
    {
        if(jump_flag == 0)
        {
            steer_control(&steer_1, func_limit_ab(steer_target_offset[0] - steer_location_offset[0], -10, 10));
            steer_control(&steer_2, func_limit_ab(steer_target_offset[1] - steer_location_offset[1], -10, 10));
            steer_control(&steer_3, func_limit_ab(steer_target_offset[2] - steer_location_offset[2], -10, 10));
            steer_control(&steer_4, func_limit_ab(steer_target_offset[3] - steer_location_offset[3], -10, 10));
        }
        else
        {
            jump_time ++;
            if(jump_time < jump_time_num[0])                  // 起跳
            {
                jump_flag = 1;
                steer_duty_set(&steer_1, steer_1.center_num + 2500);
                steer_duty_set(&steer_2, steer_2.center_num - 2500);
                steer_duty_set(&steer_3, steer_3.center_num - 2500);
                steer_duty_set(&steer_4, steer_4.center_num + 2500);
            }
            else if(jump_time < (jump_time_num[0] + jump_time_num[1]))  // 收脚
            {
                jump_flag = 2;
                steer_duty_set(&steer_1, steer_1.center_num);
                steer_duty_set(&steer_2, steer_2.center_num);
                steer_duty_set(&steer_3, steer_3.center_num);
                steer_duty_set(&steer_4, steer_4.center_num);
            }
            else if(jump_time < (jump_time_num[0] + jump_time_num[1] + jump_time_num[2]))  // 预备缓冲
            {
                jump_flag = 3;
                steer_duty_set(&steer_1, steer_1.center_num + 1400);
                steer_duty_set(&steer_2, steer_2.center_num - 1400);
                steer_duty_set(&steer_3, steer_3.center_num - 1400);
                steer_duty_set(&steer_4, steer_4.center_num + 1400);
            }
            else if(jump_time < (jump_time_num[0] + jump_time_num[1] + jump_time_num[2] + jump_time_num[3]))  // 预备缓冲
            {
                jump_flag = 4;
                steer_duty_set(&steer_1, -14);
                steer_duty_set(&steer_2, -14);
                steer_duty_set(&steer_3, -14);
                steer_duty_set(&steer_4, -14);
            }
            else
            {
                jump_flag = 0;
                jump_time = 0;
            }
        }
    }
    else
    {
        steer_control(&steer_1, func_limit_ab(steer_1.center_num - steer_1.now_location, -1, 1) * steer_1.steer_dir);
        steer_control(&steer_2, func_limit_ab(steer_2.center_num - steer_2.now_location, -1, 1) * steer_2.steer_dir);
        steer_control(&steer_3, func_limit_ab(steer_3.center_num - steer_3.now_location, -1, 1) * steer_3.steer_dir);
        steer_control(&steer_4, func_limit_ab(steer_4.center_num - steer_4.now_location, -1, 1) * steer_4.steer_dir);
    }
}

void pit_call_back(void)
{
    sys_times ++;                                      // 系统计时自增

    for(int i = 0; i < 20; i ++)
    {
        system_time_state[i] = (sys_times % (i + 1) == 0 ? 1 : system_time_state[i]);
    }

    imu660ra_get_gyro();                               // 获取 IMU660RA 陀螺仪数据
    imu660ra_get_acc();                                // 获取 IMU660RA 加速度计数据
    quaternion_module_calculate(&roll_balance_cascade); // 计算四元数，更新姿态数据
    nav_heading_update_1ms();                          // 导航航向角积分（gyro_z）

    if(sys_times % 20 == 0)                            // 每 20 个周期执行一次（约 20 * 中断周期）
    {
        //car_speed = (motor_value.receive_left_speed_data - motor_value.receive_right_speed_data) / 2;  //向前走时left_motor_duty为负, right_motor_duty为正, 计算车辆速度：左右电机速度差的一半
        car_speed = (motor_value.receive_right_speed_data - motor_value.receive_left_speed_data) / 2;  // 计算车辆速度：左右电机速度差的一半
        pid_control(&roll_balance_cascade.speed_cycle, target_speed, (float)car_speed);  // 速度环 PID 控制，目标值为 0.0 f
    }

    if(sys_times % 5 == 0)                             // 每 5 个周期执行一次
    {
        // 角度环 PID 控制，目标值为速度环输出减去机械零点
        pid_control(&roll_balance_cascade.angle_cycle, 0.0f - roll_balance_cascade.posture_value.mechanical_zero, roll_balance_cascade.posture_value.rol);

        // 角度环 PID 控制，目标值为速度环输出减去机械零点 //舵机
        pid_control(&pitch_balance_cascade.angle_cycle, 0.0f - pitch_balance_cascade.posture_value.mechanical_zero, roll_balance_cascade.posture_value.pit);
    }

    // 角速度环 PID 控制，目标值为角度环输出，当前值为 X 轴陀螺仪数据
    pid_control(&roll_balance_cascade.angular_speed_cycle, roll_balance_cascade.angle_cycle.out, imu660ra_gyro_x);

    car_state_calculate();                             // 检测车辆状态
    car_steer_control();                                // 车辆舵机控制
    car_motor_control();                                // 车辆电机控制
}

//=============================================================================
// 函数简介     车辆电机占空比控制
// 返回参数     void
// 使用示例     car_motor_control();
// 备注信息     根据运行标志控制左右电机的占空比，通过限幅函数限制输出范围，最终设置电机驱动
//=============================================================================
void car_motor_control(void)
{
    car_distance += ((float)car_speed / 60.0f * WHEEL_DIAMETER * PI * 0.001f);

    if(run_state)                                      // 当运行状态为 1 时，计算电机占空比
    {
        left_motor_duty  = func_limit_ab((int16)roll_balance_cascade.angular_speed_cycle.out, -balance_duty_max, balance_duty_max);  // 左电机占空比: 取角速度环输出，限幅
        right_motor_duty = func_limit_ab((int16)roll_balance_cascade.angular_speed_cycle.out, -balance_duty_max, balance_duty_max);  // 右电机占空比: 取角速度环输出，限幅

        // yaw 阻尼（防偏转）+ 导航航向闭环叠加
        // nav_yaw_output > 0 = 左转：左轮减速，右轮加速
        left_motor_duty  = func_limit_ab(left_motor_duty  - imu660ra_gyro_z / 3 - nav_yaw_output, -turn_duty_max, turn_duty_max);
        right_motor_duty = func_limit_ab(right_motor_duty + imu660ra_gyro_z / 3 + nav_yaw_output, -turn_duty_max, turn_duty_max);
    }
    else                                               // 当运行状态为 0 时，电机关闭
    {
        left_motor_duty  = 0;                         // 左电机占空比设为 0
        right_motor_duty = 0;                         // 右电机占空比设为 0
    }

    small_driver_set_duty(-left_motor_duty, right_motor_duty);  // 设置驱动的电机占空比（左电机取反:小车往前走时,左电机占空比为负,右电机为正）
}