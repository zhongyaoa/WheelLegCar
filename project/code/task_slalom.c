#include "task_slalom.h"
#include "zf_device_gnss.h"
#include "balance_control.h"
#include "posture_control.h"

//=============================================================================
// 全局变量
//=============================================================================
slalom_state_enum   slalom_state        = SLALOM_IDLE;
int16               nav_yaw_output      = 0;    // 航向差速叠加量，由 posture_control 读取
uint8               slalom_cone_count   = 0;    // 已录入锥桶数量

static gps_point_struct cone_gps[SLALOM_MAX_CONES];   // 锥桶 GPS 坐标表
static gps_point_struct origin_point;                  // 起点坐标
static gps_point_struct turnaround_point;              // 掉头区坐标（直行终点）

static uint8        current_cone_idx    = 0;    // 当前目标锥桶索引
static float        heading_target      = 0.0f; // 目标航向角（0~360，北为0，顺时针）
static float        heading_err_last    = 0.0f; // 上次航向误差（用于微分）
static float        uturn_start_yaw     = 0.0f; // 掉头开始时的 yaw

// 按键消抖计数
static uint16       key_a_count         = 0;
static uint16       key_b_count         = 0;
static uint16       key_up_count        = 0;

#define KEY_DEBOUNCE_MS     20  // 消抖阈值（调用周期为 20ms 时，1次 = 20ms）

//=============================================================================
// 内部函数：将方位角（GPS，北为0顺时针）转换为航向误差（-180~180）
// target_bearing: 0~360, current_bearing: 0~360
//=============================================================================
static float heading_error(float target_bearing, float current_bearing)
{
    float err = target_bearing - current_bearing;
    if (err > 180.0f)  err -= 360.0f;
    if (err < -180.0f) err += 360.0f;
    return err;
}

//=============================================================================
// 内部函数：计算航向 PID 输出（PD控制，无积分防抖）
//=============================================================================
static int16 heading_pid_calc(float err)
{
    float d_err = err - heading_err_last;
    heading_err_last = err;
    float output = SLALOM_HEADING_KP * err + SLALOM_HEADING_KD * d_err;
    return (int16)func_limit_ab(output, -SLALOM_HEADING_OUT_MAX, SLALOM_HEADING_OUT_MAX);
}

//=============================================================================
// 内部函数：按键扫描（低电平有效，消抖）
// 返回 1 = 本次触发短按
//=============================================================================
static uint8 key_scan(gpio_pin_enum pin, uint16 *count)
{
    if (gpio_get_level(pin) == GPIO_LOW)
    {
        (*count)++;
        if (*count == KEY_DEBOUNCE_MS)
        {
            return 1;   // 确认按下
        }
    }
    else
    {
        *count = 0;
    }
    return 0;
}

//=============================================================================
// 内部函数：获取当前 GPS 有效航向（优先用 GPS direction，信号差时降级到 IMU yaw）
// 返回 0~360 度（北为0，顺时针）
//=============================================================================
static float get_current_bearing(void)
{
    if (gnss.state == 1 && gnss.satellite_used >= 4)
    {
        return gnss.direction;  // GPS 地面航向，已是 0~360
    }
    // 降级：将 IMU yaw（-180~180）转为 0~360
    float yaw = roll_balance_cascade.posture_value.yaw;
    return (yaw < 0.0f) ? (yaw + 360.0f) : yaw;
}

//=============================================================================
// task_slalom_init
//=============================================================================
void task_slalom_init(void)
{
    // 初始化按键 GPIO（上拉输入）
    gpio_init(KEY_A_PIN,  GPI, GPIO_HIGH, GPO_PUSH_PULL);
    gpio_init(KEY_B_PIN,  GPI, GPIO_HIGH, GPO_PUSH_PULL);
    gpio_init(KEY_UP_PIN, GPI, GPIO_HIGH, GPO_PUSH_PULL);

    // 初始化 GPS（UART2，TAU1201 双频模块）
    gnss_init(TAU1201);

    slalom_state      = SLALOM_RECORD_MODE;
    slalom_cone_count = 0;
    current_cone_idx  = 0;
    nav_yaw_output    = 0;

    printf("SlalomTask: Enter RECORD_MODE. Press A to record cone, B to finish.\r\n");
}

//=============================================================================
// task_slalom_update — 约 20ms 调用一次
//=============================================================================
void task_slalom_update(void)
{
    // --- 解析 GPS 数据（每次有新数据就解析）---
    if (gnss_flag)
    {
        gnss_data_parse();
        gnss_flag = 0;
    }

    switch (slalom_state)
    {
        //----------------------------------------------------------------------
        // 录坐标模式：赛前手动走一遍，按 A 记录锥桶，按 B 完成
        //----------------------------------------------------------------------
        case SLALOM_RECORD_MODE:
        {
            // 按键 A：记录当前位置为锥桶坐标
            if (key_scan(KEY_A_PIN, &key_a_count))
            {
                if (slalom_cone_count < SLALOM_MAX_CONES && gnss.state == 1)
                {
                    cone_gps[slalom_cone_count].latitude  = gnss.latitude;
                    cone_gps[slalom_cone_count].longitude = gnss.longitude;
                    slalom_cone_count++;
                    printf("Cone[%d] recorded: lat=%.7f lon=%.7f\r\n",
                           slalom_cone_count, gnss.latitude, gnss.longitude);
                }
                else if (gnss.state != 1)
                {
                    printf("GPS invalid, cannot record cone.\r\n");
                }
                else
                {
                    printf("Max cones reached!\r\n");
                }
            }

            // 按键 B：完成录入，等待启动
            if (key_scan(KEY_B_PIN, &key_b_count))
            {
                slalom_state = SLALOM_IDLE;
                printf("SlalomTask: %d cones recorded. Press UP to start.\r\n", slalom_cone_count);
            }
            break;
        }

        //----------------------------------------------------------------------
        // 等待启动：按 UP 键启动
        //----------------------------------------------------------------------
        case SLALOM_IDLE:
        {
            nav_yaw_output = 0;

            if (key_scan(KEY_UP_PIN, &key_up_count))
            {
                slalom_state = SLALOM_GPS_WAIT;
                printf("SlalomTask: Waiting for GPS fix...\r\n");
            }
            break;
        }

        //----------------------------------------------------------------------
        // 等待 GPS 定位有效
        //----------------------------------------------------------------------
        case SLALOM_GPS_WAIT:
        {
            nav_yaw_output = 0;

            if (gnss.state == 1 && gnss.satellite_used >= 4)
            {
                // 记录起点和掉头区坐标
                origin_point.latitude   = gnss.latitude;
                origin_point.longitude  = gnss.longitude;

                // 计算掉头区坐标（沿当前航向前进 SLALOM_STRAIGHT_DIST 米）
                // 使用简化平面近似（距离小，误差可忽略）
                double bearing_rad = gnss.direction * GNSS_PI / 180.0;
                double meters_per_deg_lat = 111320.0;
                double meters_per_deg_lon = 111320.0 * cos(origin_point.latitude * GNSS_PI / 180.0);

                turnaround_point.latitude  = origin_point.latitude
                                           + (SLALOM_STRAIGHT_DIST * cos(bearing_rad)) / meters_per_deg_lat;
                turnaround_point.longitude = origin_point.longitude
                                           + (SLALOM_STRAIGHT_DIST * sin(bearing_rad)) / meters_per_deg_lon;

                // 以当前行进方向为直行航向目标
                heading_target   = gnss.direction;
                heading_err_last = 0.0f;

                slalom_state = SLALOM_STRAIGHT_GO;
                printf("SlalomTask: GPS OK. Start STRAIGHT_GO. heading=%.1f\r\n", heading_target);
            }
            break;
        }

        //----------------------------------------------------------------------
        // 直行到掉头区
        //----------------------------------------------------------------------
        case SLALOM_STRAIGHT_GO:
        {
            double dist_to_turn = get_two_points_distance(
                gnss.latitude, gnss.longitude,
                turnaround_point.latitude, turnaround_point.longitude);

            // 航向 PD 控制
            float cur_bearing = get_current_bearing();
            float err = heading_error(heading_target, cur_bearing);
            nav_yaw_output = heading_pid_calc(err);

            // 前进速度
            target_speed = SLALOM_FORWARD_SPEED;

            printf("STRAIGHT dist=%.2f err=%.1f yaw_out=%d\r\n",
                   (float)dist_to_turn, err, nav_yaw_output);

            // 判断是否到达掉头区
            if (dist_to_turn <= SLALOM_CONE_ARRIVE_DIST)
            {
                target_speed   = 0;
                nav_yaw_output = 0;
                uturn_start_yaw = roll_balance_cascade.posture_value.yaw;
                slalom_state   = SLALOM_U_TURN;
                printf("SlalomTask: Reached turnaround. Start U_TURN.\r\n");
            }
            break;
        }

        //----------------------------------------------------------------------
        // 掉头：原地旋转 ~180°（差速控制，target_speed=0）
        //----------------------------------------------------------------------
        case SLALOM_U_TURN:
        {
            target_speed = 0;

            // 用 nav_yaw_output 驱动差速旋转（正值 = 左转）
            nav_yaw_output = SLALOM_HEADING_OUT_MAX;  // 固定向左旋转

            float yaw_now     = roll_balance_cascade.posture_value.yaw;
            float yaw_turned  = yaw_now - uturn_start_yaw;

            // 归一化到 -180~180
            if (yaw_turned >  180.0f) yaw_turned -= 360.0f;
            if (yaw_turned < -180.0f) yaw_turned += 360.0f;

            printf("U_TURN turned=%.1f\r\n", yaw_turned);

            if (func_abs(yaw_turned) >= SLALOM_UTURN_ANGLE)
            {
                nav_yaw_output   = 0;
                current_cone_idx = 0;

                // 如果没有录入锥桶，直接回到起点直行
                if (slalom_cone_count == 0)
                {
                    // 以起点为唯一目标
                    slalom_state = SLALOM_STOP;
                }
                else
                {
                    slalom_state = SLALOM_RETURN;
                    // 计算第一个目标点的航向
                    heading_target = (float)get_two_points_azimuth(
                        gnss.latitude, gnss.longitude,
                        cone_gps[current_cone_idx].latitude,
                        cone_gps[current_cone_idx].longitude);
                    heading_err_last = 0.0f;
                    printf("SlalomTask: U_TURN done. Start RETURN to cone[0].\r\n");
                }
            }
            break;
        }

        //----------------------------------------------------------------------
        // 返回 + 绕桩：依次导航到每个锥桶间隙中点
        //----------------------------------------------------------------------
        case SLALOM_RETURN:
        {
            if (current_cone_idx >= slalom_cone_count)
            {
                // 所有锥桶穿完，导航回起点
                double dist_to_origin = get_two_points_distance(
                    gnss.latitude, gnss.longitude,
                    origin_point.latitude, origin_point.longitude);

                float bearing_to_origin = (float)get_two_points_azimuth(
                    gnss.latitude, gnss.longitude,
                    origin_point.latitude, origin_point.longitude);

                float cur_bearing = get_current_bearing();
                float err = heading_error(bearing_to_origin, cur_bearing);
                nav_yaw_output = heading_pid_calc(err);
                target_speed = SLALOM_SLALOM_SPEED;

                printf("RETURN_ORIGIN dist=%.2f err=%.1f\r\n", (float)dist_to_origin, err);

                if (dist_to_origin <= SLALOM_CONE_ARRIVE_DIST * 2.0)
                {
                    slalom_state   = SLALOM_STOP;
                    target_speed   = 0;
                    nav_yaw_output = 0;
                    printf("SlalomTask: Reached origin. STOP.\r\n");
                }
                break;
            }

            // 导航到当前目标锥桶
            double dist_to_cone = get_two_points_distance(
                gnss.latitude, gnss.longitude,
                cone_gps[current_cone_idx].latitude,
                cone_gps[current_cone_idx].longitude);

            // 实时更新目标航向
            heading_target = (float)get_two_points_azimuth(
                gnss.latitude, gnss.longitude,
                cone_gps[current_cone_idx].latitude,
                cone_gps[current_cone_idx].longitude);

            float cur_bearing = get_current_bearing();
            float err = heading_error(heading_target, cur_bearing);
            nav_yaw_output = heading_pid_calc(err);
            target_speed = SLALOM_SLALOM_SPEED;

            printf("SLALOM cone[%d] dist=%.2f err=%.1f yaw=%d\r\n",
                   current_cone_idx, (float)dist_to_cone, err, nav_yaw_output);

            if (dist_to_cone <= SLALOM_CONE_ARRIVE_DIST)
            {
                current_cone_idx++;
                heading_err_last = 0.0f;

                if (current_cone_idx < slalom_cone_count)
                {
                    printf("SlalomTask: Cone[%d] passed. Next cone[%d].\r\n",
                           current_cone_idx - 1, current_cone_idx);
                }
                else
                {
                    printf("SlalomTask: All cones passed. Returning to origin.\r\n");
                }
            }
            break;
        }

        //----------------------------------------------------------------------
        // 完成停止
        //----------------------------------------------------------------------
        case SLALOM_STOP:
        {
            target_speed   = 0;
            nav_yaw_output = 0;
            // 此状态保持，不再切换
            break;
        }

        default:
            break;
    }
}
