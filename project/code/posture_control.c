#include "posture_control.h"
#include "math.h"
#include "motion_manager.h"
#include "ins_tracker.h"

uint32 sys_times = 0;
uint8 system_time_state[20] = {0};

uint8 run_state = 0;
uint8 balance_enable = 0;  // 平衡使能标志：0=开机不自动平衡，1=允许自动平衡
uint8 jump_flag = 0;
uint32 jump_time = 0;

int16 car_speed = 0;
float target_speed = 0.0f;
float car_distance = 0.0f;

// 偏航锁定控制变量
static float yaw_angle     = 0.0f;  // 积分得到的相对偏航角 (°)
static float yaw_target    = 0.0f;  // 偏航目标角 (°)
static uint8 yaw_locked    = 0;     // 偏航锁定已初始化标志

// 外部差速注入（循迹模块写入，正值左转，负值右转）
int16 turn_diff_ext = 0;
// 兼容保留：偏航角（°），当前同步为四元数偏航角
float imu_yaw_deg   = 0.0f;
// 供外部只读的四元数偏航角（°）与差分角速度（°/s）
float quat_yaw_deg = 0.0f;
float quat_yaw_rate_dps = 0.0f;

// 惯性导航推算位置（m），积分由 pit_call_back 每 1ms 执行
// 坐标系由循迹模块在记录第一个点时确定
float inav_x           = 0.0f;
float inav_y           = 0.0f;
uint8 inav_active      = 0;      // 1=积分开启，0=暂停
float inav_heading_ref = 0.0f;   // 坐标系参考航向（°），Y 轴正方向对应此航向

int16 left_motor_duty = 0;
int16 right_motor_duty = 0;
int16 balance_duty_max = 10000;
int16 turn_duty_max = 300;

static float normalize_angle(float a)
{
    while(a > 180.0f) a -= 360.0f;
    while(a < -180.0f) a += 360.0f;
    return a;
}

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
        yaw_locked = 0;                             // 重置偏航锁定，下次立起来重新锁定方向
        roll_balance_cascade.angular_speed_cycle.i_value = 0;  // 重置角速度环 PID 积分值
        pitch_balance_cascade.angle_cycle.i_value = 0;
    }
    else
    {
        if(run_state == 0 && balance_enable)
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

    static int16 jump_time_num[4] = {110, 80, 30, 100};  // 跳跃各阶段时间  原：起跳110、收腿110、准备缓冲30、执行缓冲100

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
                steer_duty_set(&steer_1, steer_1.center_num + 2500); //反向
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
                steer_duty_set(&steer_1, steer_1.center_num+14);
                steer_duty_set(&steer_2, steer_1.center_num-14);
                steer_duty_set(&steer_3, steer_1.center_num-14);
                steer_duty_set(&steer_4, steer_1.center_num+14);
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
    static uint8 yaw_init = 0;
    static float last_quat_yaw_deg = 0.0f;
    static float inav_speed_mps_filtered = 0.0f;

    motion_manager_update();

    sys_times ++;                                      // 系统计时自增

    for(int i = 0; i < 20; i ++)
    {
        system_time_state[i] = (sys_times % (i + 1) == 0 ? 1 : system_time_state[i]);
    }

    imu660ra_get_gyro();                               // 获取 IMU660RA 陀螺仪数据
    imu660ra_get_acc();                                // 获取 IMU660RA 加速度计数据
    quaternion_module_calculate(&roll_balance_cascade); // 计算四元数，更新姿态数据

    quat_yaw_deg = roll_balance_cascade.posture_value.yaw;
    if(!yaw_init)
    {
        yaw_init = 1;
        quat_yaw_rate_dps = 0.0f;
    }
    else
    {
        float yaw_delta = normalize_angle(quat_yaw_deg - last_quat_yaw_deg);
        quat_yaw_rate_dps = yaw_delta / 0.001f;
    }
    last_quat_yaw_deg = quat_yaw_deg;
    imu_yaw_deg = quat_yaw_deg;

    // 惯性导航：每 1ms 用编码器速度 + 四元数航向积分推算位置
    // car_speed = (right_spd - left_spd)/2，前进时左轮转速为负、右轮为正，故 car_speed > 0 表示前进
    // 对速度做低通滤波，并在低速时冻结积分，降低静止抖动导致的虚假位移
    if(inav_active)
    {
        float v_mps_raw = (float)car_speed / 60.0f * (WHEEL_DIAMETER * 0.01f);

        inav_speed_mps_filtered += (v_mps_raw - inav_speed_mps_filtered) * INAV_SPEED_FILTER_ALPHA;

        if(func_abs(car_speed) < INAV_STATIONARY_SPEED_RPM)
        {
            inav_speed_mps_filtered = 0.0f;
        }
        else
        {
            // 当前航向相对参考航向的偏差（rad）
            float compensated_yaw_deg = tracker_get_compensated_yaw_deg(quat_yaw_deg, inav_heading_ref, 0);
            float heading_rad = (compensated_yaw_deg - inav_heading_ref) * (3.14159265f / 180.0f);
            // Y 轴 = 参考航向方向（前进），X 轴 = 参考航向右侧 90°
            inav_y += inav_speed_mps_filtered * cosf(heading_rad) * 0.001f;  // dt = 1ms
            inav_x += inav_speed_mps_filtered * sinf(heading_rad) * 0.001f;
        }
    }
    else
    {
        inav_speed_mps_filtered = 0.0f;
    }

    if(sys_times % 20 == 0)                            // 每 20 个周期执行一次（约 20 * 中断周期）
    {
        //car_speed = (motor_value.receive_left_speed_data - motor_value.receive_right_speed_data) / 2;  //
        car_speed = (motor_value.receive_right_speed_data - motor_value.receive_left_speed_data) / 2;  // 向前走时left_motor_duty为负, right_motor_duty为正, 计算车辆速度：左右电机速度差的一半
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
    static uint8 motor_stopped = 0;  // 标记是否已发送过停止指令

    car_distance += ((float)car_speed / 60.0f *  WHEEL_DIAMETER * 0.001f * PI);

    if(run_state)                                      // 当运行状态为 1 时，计算电机占空比
    {
        motor_stopped = 0;                            // 运行中，清除停止标记
        left_motor_duty  = func_limit_ab((int16)roll_balance_cascade.angular_speed_cycle.out, -balance_duty_max, balance_duty_max);  // 左电机占空比: 取角速度环输出，限幅
        right_motor_duty = func_limit_ab((int16)roll_balance_cascade.angular_speed_cycle.out, -balance_duty_max, balance_duty_max);  // 右电机占空比: 取角速度环输出，限幅

        // 叠加外部差速（循迹转向注入）
        int16 td = func_limit_ab(turn_diff_ext, -turn_duty_max, turn_duty_max);
        left_motor_duty  = func_limit_ab(left_motor_duty  + td, -balance_duty_max, balance_duty_max);
        right_motor_duty = func_limit_ab(right_motor_duty - td, -balance_duty_max, balance_duty_max);

        small_driver_set_duty(-left_motor_duty, right_motor_duty);  // 设置驱动的电机占空比（左电机取反）
    }
    else                                               // 当运行状态为 0 时，电机关闭
    {
        left_motor_duty  = 0;                         // 左电机占空比设为 0
        right_motor_duty = 0;                         // 右电机占空比设为 0

        if(!motor_stopped)                            // 仅发送一次停止指令，之后不再发送，让轮子自由滑行
        {
            small_driver_set_duty(0, 0);
            motor_stopped = 1;
        }
    }
}