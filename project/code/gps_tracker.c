#include "gps_tracker.h"
#include "posture_control.h"
#include "controler.h"

// ===== 状态 =====
tracker_state_enum tracker_state       = TRACKER_STATE_IDLE;
uint8              tracker_point_count = 0;

// ===== 内部存储 =====
static double recorded_lat[GPS_TRACKER_MAX_POINTS];
static double recorded_lon[GPS_TRACKER_MAX_POINTS];

static uint8 current_target_idx = 1;

// 按键消抖计数
static uint32 btn_up_hold   = 0;
static uint32 btn_left_hold = 0;

// 标定时记录的四元数 yaw（°）和对应真北方位角（°）
// heading_north_offset = 真北方位 - quat_yaw_deg（启动时标定，之后为常数）
static float heading_north_offset = 0.0f;

// 循迹 yaw PD 参数
#define TRACKER_YAW_KP   8.0f   // 偏航角度增益 (duty/°)
#define TRACKER_YAW_KD   0.5f   // 偏航角速度阻尼 (duty/(°/s))

//=============================================================================
// 将角度规范化到 (-180, 180]
//=============================================================================
static float normalize_angle(float a)
{
    while(a >  180.0f) a -= 360.0f;
    while(a < -180.0f) a += 360.0f;
    return a;
}

//=============================================================================
// 初始化
//=============================================================================
void gps_tracker_init(void)
{
    tracker_state         = TRACKER_STATE_IDLE;
    tracker_point_count   = 0;
    current_target_idx    = 1;
    btn_up_hold           = 0;
    btn_left_hold         = 0;
    heading_north_offset  = 0.0f;
    turn_diff_ext         = 0;
}

//=============================================================================
// 主循环轮询按键（每 50ms 调用一次）
//=============================================================================
void gps_tracker_button_poll(void)
{
    // ---- button_up：记录点位 ----
    if(button_press(UP))
    {
        btn_up_hold++;
        if(btn_up_hold == 1)  // 上升沿，只触发一次
        {
            if(tracker_state == TRACKER_STATE_IDLE
               && gnss.state
               && tracker_point_count < GPS_TRACKER_MAX_POINTS)
            {
                recorded_lat[tracker_point_count] = gnss.latitude;
                recorded_lon[tracker_point_count] = gnss.longitude;
                tracker_point_count++;
                led(toggle);
                wireless_printf("[TRACKER] Point %d recorded: lat=%.7f lon=%.7f\r\n",
                       tracker_point_count, gnss.latitude, gnss.longitude);
            }
            else if(!gnss.state)
            {
                wireless_printf("[TRACKER] GPS not fixed.\r\n");
            }
            else if(tracker_point_count >= GPS_TRACKER_MAX_POINTS)
            {
                wireless_printf("[TRACKER] Max points (%d) reached.\r\n", GPS_TRACKER_MAX_POINTS);
            }
        }
    }
    else
    {
        btn_up_hold = 0;
    }

    // ---- button_left：开始循迹 ----
    if(button_press(LEFT))
    {
        btn_left_hold++;
        if(btn_left_hold == 1)
        {
            if(tracker_state == TRACKER_STATE_IDLE && tracker_point_count >= 2)
            {
                // 计算起点→第二点的真北方位角
                double azimuth_p0_p1 = get_two_points_azimuth(
                    recorded_lat[0], recorded_lon[0],
                    recorded_lat[1], recorded_lon[1]);

                // 此刻车头对准第二点，quat_yaw_deg 对应该方位
                // 真北方位 = quat_yaw_deg + heading_north_offset
                heading_north_offset = (float)azimuth_p0_p1 - quat_yaw_deg;

                current_target_idx = 1;
                tracker_state      = TRACKER_STATE_RUNNING;
                target_speed       = GPS_TRACKER_CRUISE_SPEED;

                wireless_printf("[TRACKER] Start. Points=%d az=%.1f yaw=%.1f offset=%.1f\r\n",
                       tracker_point_count, (float)azimuth_p0_p1,
                      quat_yaw_deg, heading_north_offset);
            }
            else if(tracker_point_count < 2)
            {
                wireless_printf("[TRACKER] Need at least 2 points.\r\n");
            }
        }
    }
    else
    {
        btn_left_hold = 0;
    }
}

//=============================================================================
// 循迹更新（每次 GPS 数据刷新后调用，约 1Hz）
// gnss_flag 由调用方清零，此函数只读 gnss 结构体
//=============================================================================
void gps_tracker_update(void)
{
    if(tracker_state != TRACKER_STATE_RUNNING)
    {
        turn_diff_ext = 0;
        return;
    }
    if(!gnss.state) return;  // GPS 无效时维持上次差速输出

    double cur_lat = gnss.latitude;
    double cur_lon = gnss.longitude;

    double target_lat = recorded_lat[current_target_idx];
    double target_lon = recorded_lon[current_target_idx];

    // 到当前目标点距离 (m)
    double dist = get_two_points_distance(cur_lat, cur_lon, target_lat, target_lon);

    // 到达判定
    if(dist < GPS_TRACKER_ARRIVE_DIST)
    {
        wireless_printf("[TRACKER] Arrived pt%d (dist=%.2fm)\r\n", current_target_idx, (float)dist);
        current_target_idx++;

        if(current_target_idx >= tracker_point_count)
        {
            tracker_state = TRACKER_STATE_DONE;
            target_speed  = 0.0f;
            turn_diff_ext = 0;
            led(on);
            wireless_printf("[TRACKER] All points done.\r\n");
            return;
        }

        target_lat = recorded_lat[current_target_idx];
        target_lon = recorded_lon[current_target_idx];
        dist = get_two_points_distance(cur_lat, cur_lon, target_lat, target_lon);
    }

    // 目标真北方位角 (0°=北，顺时针)
    double bearing_true = get_two_points_azimuth(cur_lat, cur_lon, target_lat, target_lon);

    // 当前朝向（真北）= quat_yaw_deg + heading_north_offset
    float current_heading = quat_yaw_deg + heading_north_offset;

    // 偏航误差（目标方位 - 当前朝向），规范化到 ±180°
    float heading_err = normalize_angle((float)bearing_true - current_heading);

    // 使用四元数偏航差分角速度（°/s）用于 D 项阻尼
    float gyro_z_dps = quat_yaw_rate_dps;

    // PD 控制输出差速（负→左转，正→右转，符合 car_motor_control 差速约定）
    // 前进时 left_motor_duty 为负，+td 使其绝对值减小 → 左轮变慢 → 右转
    // 因此 td 取反：need_left_turn(err>0) → td 负
    int16 td = (int16)(TRACKER_YAW_KP * heading_err - TRACKER_YAW_KD * gyro_z_dps);
    turn_diff_ext = func_limit_ab(-td, -turn_duty_max, turn_duty_max);

    wireless_printf("[TRACKER] ->pt%d dist=%.1fm bear=%.1f hdg=%.1f err=%.1f td=%d\r\n",
           current_target_idx, (float)dist, (float)bearing_true,
           current_heading, heading_err, (int)turn_diff_ext);
}
