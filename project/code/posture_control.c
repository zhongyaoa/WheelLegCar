#include "posture_control.h"
#include "math.h"

uint32 sys_times = 0;
uint8 system_time_state[20] = {0};

uint8 run_state = 0;
uint8 balance_enable = 0;  // 平衡使能标志：0=开机不自动平衡，1=允许自动平衡
uint8 jump_flag = 0;
uint32 jump_time = 0;

int16 car_speed = 0;
float target_speed = 0.0f;
float car_distance = 0.0f;

// 外部差速注入（循迹模块写入，正值左转，负值右转）
int16 turn_diff_ext = 0;
// 兼容保留：偏航角（°），当前同步为四元数偏航角
float imu_yaw_deg   = 0.0f;
// 供外部只读的四元数偏航角（°）与差分角速度（°/s）
float quat_yaw_deg = 0.0f;
float quat_yaw_rate_dps = 0.0f;
float spin_accum_deg = 0.0f;

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

#define POSTURE_TURN_KP 8.0f
#define POSTURE_TURN_KD 0.5f
#define POSTURE_SPIN_KP 9.0f
#define POSTURE_SPIN_KD 0.25f
#define POSTURE_SPIN_DONE_ERR_DEG 5.0f
#define POSTURE_SPIN_DONE_RATE_DPS 25.0f
#define POSTURE_SPIN_FEED_DUTY 120
#define POSTURE_SPIN_MOTOR_DUTY 500

typedef enum
{
    CAR_SPIN_STATE_IDLE = 0,
    CAR_SPIN_STATE_RUNNING = 1,
} car_spin_state_enum;

static car_spin_state_enum car_spin_state = CAR_SPIN_STATE_IDLE;
static float car_spin_target_yaw = 0.0f;
static int8  car_spin_dir = 1;

static float normalize_angle(float a)
{
    while(a > 180.0f) a -= 360.0f;
    while(a < -180.0f) a += 360.0f;
    return a;
}

void car_state_calculate(void)
{
    if(func_abs(roll_balance_cascade.posture_value.rol) > 40.0f || func_abs(roll_balance_cascade.posture_value.pit) > 40.0f)
    {
        jump_flag = 0;
        run_state = 0;
        roll_balance_cascade.angular_speed_cycle.i_value = 0;
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

    if(sys_times < 500)
    {
        roll_balance_cascade.angle_cycle.p = roll_balance_cascade_resave.angle_cycle.p * (0.2f + (float)sys_times / 500.0f * 0.8f);
        roll_balance_cascade.speed_cycle.p = roll_balance_cascade_resave.speed_cycle.p * (0.2f + (float)sys_times / 500.0f * 0.8f);
        roll_balance_cascade.angle_cycle.i_value = 0;
    }
    else
    {
        roll_balance_cascade.angle_cycle.p = roll_balance_cascade_resave.angle_cycle.p;
        roll_balance_cascade.speed_cycle.p = roll_balance_cascade_resave.speed_cycle.p;
    }

    if(jump_flag)
    {
        roll_balance_cascade.angle_cycle.p = roll_balance_cascade_resave.angle_cycle.p * 0.5f;
        roll_balance_cascade.speed_cycle.p = roll_balance_cascade_resave.speed_cycle.p * 0.5f;
        roll_balance_cascade.angle_cycle.i_value = 0;
        pitch_balance_cascade.angle_cycle.i_value = 0;
    }
}

void car_steer_control(void)
{
    int16 steer_location_offset[4] = {0};
    int16 steer_target_offset[4] = {0};
    static int16 jump_time_num[4] = {110, 80, 30, 100};
    static float steer_balance_angle_count = 0;
    static float steer_output_duty_filter = 0;
    int16 steer_output_duty = 0;
    float steer_balance_angle = 0;
    float pitch_offset = (30.0f - func_limit_ab(func_abs(roll_balance_cascade.posture_value.rol + roll_balance_cascade.posture_value.mechanical_zero), 0.0f, 30.0f)) / 30.0f;

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
            if(jump_time < jump_time_num[0])
            {
                jump_flag = 1;
                steer_duty_set(&steer_1, steer_1.center_num + 2500);
                steer_duty_set(&steer_2, steer_2.center_num - 2500);
                steer_duty_set(&steer_3, steer_3.center_num - 2500);
                steer_duty_set(&steer_4, steer_4.center_num + 2500);
            }
            else if(jump_time < (jump_time_num[0] + jump_time_num[1]))
            {
                jump_flag = 2;
                steer_duty_set(&steer_1, steer_1.center_num);
                steer_duty_set(&steer_2, steer_2.center_num);
                steer_duty_set(&steer_3, steer_3.center_num);
                steer_duty_set(&steer_4, steer_4.center_num);
            }
            else if(jump_time < (jump_time_num[0] + jump_time_num[1] + jump_time_num[2]))
            {
                jump_flag = 3;
                steer_duty_set(&steer_1, steer_1.center_num + 1400);
                steer_duty_set(&steer_2, steer_2.center_num - 1400);
                steer_duty_set(&steer_3, steer_3.center_num - 1400);
                steer_duty_set(&steer_4, steer_4.center_num + 1400);
            }
            else if(jump_time < (jump_time_num[0] + jump_time_num[1] + jump_time_num[2] + jump_time_num[3]))
            {
                jump_flag = 4;
                steer_duty_set(&steer_1, steer_1.center_num + 14);
                steer_duty_set(&steer_2, steer_1.center_num - 14);
                steer_duty_set(&steer_3, steer_1.center_num - 14);
                steer_duty_set(&steer_4, steer_1.center_num + 14);
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

    sys_times ++;

    for(int i = 0; i < 20; i ++)
    {
        system_time_state[i] = (sys_times % (i + 1) == 0 ? 1 : system_time_state[i]);
    }

    imu660ra_get_gyro();
    imu660ra_get_acc();
    quaternion_module_calculate(&roll_balance_cascade);

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

    if(car_spin_state == CAR_SPIN_STATE_RUNNING)
    {
        spin_accum_deg += func_abs(quat_yaw_rate_dps) * 0.001f;
    }

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
            float heading_rad = (quat_yaw_deg - inav_heading_ref) * (3.14159265f / 180.0f);
            inav_y += inav_speed_mps_filtered * cosf(heading_rad) * 0.001f;
            inav_x += inav_speed_mps_filtered * sinf(heading_rad) * 0.001f;
        }
    }
    else
    {
        inav_speed_mps_filtered = 0.0f;
    }

    if(sys_times % 20 == 0)
    {
        car_speed = (motor_value.receive_right_speed_data - motor_value.receive_left_speed_data) / 2;
        pid_control(&roll_balance_cascade.speed_cycle, target_speed, (float)car_speed);
    }

    if(sys_times % 5 == 0)
    {
        pid_control(&roll_balance_cascade.angle_cycle, 0.0f - roll_balance_cascade.posture_value.mechanical_zero, roll_balance_cascade.posture_value.rol);
        pid_control(&pitch_balance_cascade.angle_cycle, 0.0f - pitch_balance_cascade.posture_value.mechanical_zero, roll_balance_cascade.posture_value.pit);
    }

    pid_control(&roll_balance_cascade.angular_speed_cycle, roll_balance_cascade.angle_cycle.out, imu660ra_gyro_x);

    car_state_calculate();
    car_steer_control();
    car_motor_control();
}

int16 car_turn_control(float heading_err, float yaw_rate_dps)
{
    int16 td;

    if(car_spin_state == CAR_SPIN_STATE_RUNNING)
    {
        return turn_diff_ext;
    }

    td = (int16)(POSTURE_TURN_KP * heading_err - POSTURE_TURN_KD * yaw_rate_dps);
    turn_diff_ext = func_limit_ab(-td, -turn_duty_max, turn_duty_max);
    return turn_diff_ext;
}

void car_turn_reset(void)
{
    if(car_spin_state == CAR_SPIN_STATE_IDLE)
    {
        turn_diff_ext = 0;
    }
}

void car_spin_start(float target_yaw_deg, int8 spin_dir)
{
    (void)target_yaw_deg;
    car_spin_target_yaw = 0.0f;
    car_spin_dir = (spin_dir >= 0) ? 1 : -1;
    car_spin_state = CAR_SPIN_STATE_RUNNING;
    spin_accum_deg = 0.0f;
    turn_diff_ext = func_limit_ab((int16)(-car_spin_dir * POSTURE_SPIN_FEED_DUTY), -turn_duty_max, turn_duty_max);
}

uint8 car_spin_update(float current_yaw_deg, float yaw_rate_dps)
{
    (void)current_yaw_deg;
    (void)yaw_rate_dps;

    if(car_spin_state != CAR_SPIN_STATE_RUNNING)
    {
        return 0;
    }

    turn_diff_ext = func_limit_ab((int16)(-car_spin_dir * POSTURE_SPIN_FEED_DUTY), -turn_duty_max, turn_duty_max);
    return 0;
}

void car_spin_stop(void)
{
    car_spin_state = CAR_SPIN_STATE_IDLE;
    spin_accum_deg = 0.0f;
    turn_diff_ext = 0;
}

void car_motor_control(void)
{
    static uint8 motor_stopped = 0;

    car_distance += ((float)car_speed / 60.0f * WHEEL_DIAMETER * 0.001f * PI);

    if(run_state)
    {
        int16 td;
        motor_stopped = 0;

        left_motor_duty  = func_limit_ab((int16)roll_balance_cascade.angular_speed_cycle.out, -balance_duty_max, balance_duty_max);
        right_motor_duty = func_limit_ab((int16)roll_balance_cascade.angular_speed_cycle.out, -balance_duty_max, balance_duty_max);

        if(car_spin_state == CAR_SPIN_STATE_RUNNING)
        {
            td = (int16)(-car_spin_dir * POSTURE_SPIN_MOTOR_DUTY);
        }
        else
        {
            td = func_limit_ab(turn_diff_ext - (int16)(BALANCE_TURN_DAMPING_KD * quat_yaw_rate_dps), -turn_duty_max, turn_duty_max);
        }

        left_motor_duty  = func_limit_ab(left_motor_duty  + td, -balance_duty_max, balance_duty_max);
        right_motor_duty = func_limit_ab(right_motor_duty - td, -balance_duty_max, balance_duty_max);

        small_driver_set_duty(-left_motor_duty, right_motor_duty);
    }
    else
    {
        left_motor_duty  = 0;
        right_motor_duty = 0;

        if(!motor_stopped)
        {
            small_driver_set_duty(0, 0);
            motor_stopped = 1;
        }
    }
}
