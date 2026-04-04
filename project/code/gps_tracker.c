#include "gps_tracker.h"
#include "posture_control.h"
#include "controler.h"
#include <math.h>

// ===== 状态 =====
tracker_state_enum tracker_state      = TRACKER_STATE_IDLE;
uint8              tracker_point_count = 0;

// ===== 内部存储 =====
static double recorded_lat[GPS_TRACKER_MAX_POINTS];
static double recorded_lon[GPS_TRACKER_MAX_POINTS];

// 当前目标点索引（0-based，循迹从索引1开始）
static uint8 current_target_idx = 1;

// 按键消抖用计时
static uint32 btn_up_hold   = 0;
static uint32 btn_left_hold = 0;

// 循迹偏航：记录开机（按下button_left）时 IMU yaw_angle 作为0°参考
static float tracker_yaw_ref    = 0.0f; // IMU yaw_angle 对应真北的偏移量
// 车头初始朝向：按 button_left 时车头朝向第二个点，记录此时 yaw_angle 作为方位基准
// get_two_points_azimuth 返回真北顺时针的方位角
// 我们以：imu_heading = yaw_angle - tracker_yaw_offset, 真北方位 = imu_heading + heading_north_offset
static float heading_north_offset = 0.0f; // 真北方位 - IMU yaw_angle (常数，按下开始时标定)

//=============================================================================
// 将方位角规范化到 [-180, 180]
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
    tracker_state       = TRACKER_STATE_IDLE;
    tracker_point_count = 0;
    current_target_idx  = 1;
    btn_up_hold         = 0;
    btn_left_hold       = 0;
}

//=============================================================================
// 主循环轮询按键（建议每 50ms 调用一次）
//=============================================================================
void gps_tracker_button_poll(void)
{
    // ---- button_up：记录点位 ----
    if(button_press(UP))
    {
        btn_up_hold++;
        if(btn_up_hold == 1)  // 上升沿
        {
            if(tracker_state == TRACKER_STATE_IDLE
               && gnss.state
               && tracker_point_count < GPS_TRACKER_MAX_POINTS)
            {
                recorded_lat[tracker_point_count] = gnss.latitude;
                recorded_lon[tracker_point_count] = gnss.longitude;
                tracker_point_count++;
                led(toggle);  // 闪烁一次表示记录成功
                printf("[TRACKER] Point %d recorded: lat=%.7f lon=%.7f\r\n",
                       tracker_point_count,
                       gnss.latitude,
                       gnss.longitude);
            }
            else if(!gnss.state)
            {
                printf("[TRACKER] GPS not fixed, cannot record.\r\n");
            }
            else if(tracker_point_count >= GPS_TRACKER_MAX_POINTS)
            {
                printf("[TRACKER] Max points reached.\r\n");
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
        if(btn_left_hold == 1)  // 上升沿
        {
            if(tracker_state == TRACKER_STATE_IDLE
               && tracker_point_count >= 2)
            {
                // 此时车头朝向第二个点，标定 IMU→真北 偏移
                // 计算起点到第二点的真北方位角
                double azimuth_to_p1 = get_two_points_azimuth(
                    recorded_lat[0], recorded_lon[0],
                    recorded_lat[1], recorded_lon[1]);

                // IMU 当前 yaw_angle 对应此方位
                heading_north_offset = (float)azimuth_to_p1 - yaw_angle;
                tracker_yaw_ref      = yaw_angle;

                current_target_idx = 1;
                tracker_state      = TRACKER_STATE_RUNNING;
                target_speed       = GPS_TRACKER_CRUISE_SPEED;

                printf("[TRACKER] Start tracking. Points=%d, azimuth=%.1f, yaw_offset=%.1f\r\n",
                       tracker_point_count, (float)azimuth_to_p1, heading_north_offset);
            }
            else if(tracker_point_count < 2)
            {
                printf("[TRACKER] Need at least 2 points.\r\n");
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
// 注意：gnss_flag 由调用方清零，此函数内只读 gnss 数据
//=============================================================================
void gps_tracker_update(void)
{
    if(tracker_state != TRACKER_STATE_RUNNING) return;
    if(!gnss.state) return;  // GPS 无效时保持当前控制不变

    double cur_lat = gnss.latitude;
    double cur_lon = gnss.longitude;

    double target_lat = recorded_lat[current_target_idx];
    double target_lon = recorded_lon[current_target_idx];

    // 到当前目标点的距离 (m)
    double dist = get_two_points_distance(cur_lat, cur_lon, target_lat, target_lon);

    // 到达判定
    if(dist < GPS_TRACKER_ARRIVE_DIST)
    {
        printf("[TRACKER] Arrived at point %d (dist=%.2fm)\r\n", current_target_idx, (float)dist);
        current_target_idx++;

        if(current_target_idx >= tracker_point_count)
        {
            // 所有点位循迹完成
            tracker_state = TRACKER_STATE_DONE;
            target_speed  = 0.0f;
            led(on);
            printf("[TRACKER] All points done.\r\n");
            return;
        }

        // 更新目标点
        target_lat = recorded_lat[current_target_idx];
        target_lon = recorded_lon[current_target_idx];
        dist = get_two_points_distance(cur_lat, cur_lon, target_lat, target_lon);
    }

    // 计算到目标点的真北方位角 (0°=北, 顺时针)
    double bearing_true = get_two_points_azimuth(cur_lat, cur_lon, target_lat, target_lon);

    // 换算为 IMU yaw 目标角
    // IMU 真实朝向 = yaw_angle + heading_north_offset（真北方位）
    // 期望真北方位 = bearing_true
    // 期望 yaw_angle = bearing_true - heading_north_offset
    float desired_yaw = (float)bearing_true - heading_north_offset;

    // 当前偏航误差（规范化到 ±180°）
    float heading_err = normalize_angle(desired_yaw - yaw_angle);

    // 设置 yaw_target，posture_control 的 yaw PD 会执行纠偏
    // 直接把目标设为：当前 yaw_angle + 误差
    yaw_set_target(yaw_angle + heading_err);

    printf("[TRACKER] ->pt%d dist=%.1fm bear=%.1f err=%.1f\r\n",
           current_target_idx, (float)dist, (float)bearing_true, heading_err);
}
