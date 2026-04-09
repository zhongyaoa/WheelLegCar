#include "task_slalom.h"
#include "nav_heading.h"
#include "zf_device_gnss.h"
#include "zf_device_wireless_uart.h"
#include "balance_control.h"
#include "posture_control.h"

//=============================================================================
// 全局变量
//=============================================================================
slalom_state_enum   slalom_state        = SLALOM_IDLE;
int16               nav_yaw_output      = 0;
uint8               slalom_cone_count   = 0;
uint8               current_cone_idx    = 0;
float               heading_target      = 0.0f;

static gps_point_struct cone_gps[SLALOM_MAX_CONES];
static gps_point_struct origin_point;

static float        heading_err_last    = 0.0f;
static float        uturn_start_heading = 0.0f;   // 掉头起始航向角

// 按键消抖（20ms 周期 × 20次 = 400ms 去抖）
static uint16       key_a_count         = 0;
static uint16       key_b_count         = 0;
static uint16       key_up_count        = 0;
#define KEY_DEBOUNCE_CNT    20

//=============================================================================
// 内部函数：航向误差归一化到 -180~180
//=============================================================================
static float heading_error(float target, float current)
{
    float err = target - current;
    if (err >  180.0f) err -= 360.0f;
    if (err < -180.0f) err += 360.0f;
    return err;
}

//=============================================================================
// 内部函数：航向 PD 控制器
// 每次切换目标前必须将 heading_err_last 清零，避免微分跳变
//=============================================================================
static int16 heading_pid_calc(float err)
{
    float d_err = err - heading_err_last;
    heading_err_last = err;
    // err > 0 → current is CCW from target (target to the right) → need right turn → nav_yaw_output positive
    // nav_yaw_output > 0 → left_duty -= out, right_duty += out → right wheel faster → right turn ✓
    float output = SLALOM_HEADING_KP * err + SLALOM_HEADING_KD * d_err;
    return (int16)func_limit_ab(output, -SLALOM_HEADING_OUT_MAX, SLALOM_HEADING_OUT_MAX);
}

//=============================================================================
// 内部函数：按键扫描（低电平有效，消抖后触发一次）
//=============================================================================
static uint8 key_scan(gpio_pin_enum pin, uint16 *count)
{
    if (gpio_get_level(pin) == GPIO_LOW)
    {
        (*count)++;
        if (*count == KEY_DEBOUNCE_CNT)
            return 1;
    }
    else
    {
        *count = 0;
    }
    return 0;
}

//=============================================================================
// task_slalom_init
//=============================================================================
void task_slalom_init(void)
{
    gpio_init(KEY_A_PIN,  GPI, GPIO_HIGH, GPO_PUSH_PULL);
    gpio_init(KEY_B_PIN,  GPI, GPIO_HIGH, GPO_PUSH_PULL);
    gpio_init(KEY_UP_PIN, GPI, GPIO_HIGH, GPO_PUSH_PULL);

    gnss_init(TAU1201);

    // 采集陀螺仪零偏（此时车必须静止）
    nav_heading_init();

    slalom_state      = SLALOM_RECORD_MODE;
    slalom_cone_count = 0;
    current_cone_idx  = 0;
    nav_yaw_output    = 0;

    wireless_printf("SlalomTask: RECORD_MODE. A=record cone, B=finish.\r\n");
}

//=============================================================================
// task_slalom_update — 约 20ms 调用一次
//=============================================================================
void task_slalom_update(void)
{
    // 解析 GPS，有新数据时做互补修正
    if (gnss_flag)
    {
        gnss_data_parse();
        gnss_flag = 0;

        // 车速 > 0.3 km/h 时 GPS 方向角有意义，做互补修正
        if (gnss.state == 1 && gnss.speed > 0.3f)
        {
            nav_heading_sync_gps(gnss.direction);
        }
    }

    switch (slalom_state)
    {
        //----------------------------------------------------------------------
        // 录坐标模式
        //----------------------------------------------------------------------
        case SLALOM_RECORD_MODE:
        {
            if (key_scan(KEY_A_PIN, &key_a_count))
            {
                if (slalom_cone_count < SLALOM_MAX_CONES && gnss.state == 1)
                {
                    cone_gps[slalom_cone_count].latitude  = gnss.latitude;
                    cone_gps[slalom_cone_count].longitude = gnss.longitude;
                    slalom_cone_count++;
                    wireless_printf("Cone[%d] lat=%.6f lon=%.6f\r\n",
                           slalom_cone_count, gnss.latitude, gnss.longitude);
                }
                else
                {
                    wireless_printf("GPS invalid or max cones.\r\n");
                }
            }
            if (key_scan(KEY_B_PIN, &key_b_count))
            {
                slalom_state = SLALOM_IDLE;
                wireless_printf("SlalomTask: %d cones recorded. Press UP to start.\r\n", slalom_cone_count);
            }
            break;
        }

        //----------------------------------------------------------------------
        // 等待启动
        //----------------------------------------------------------------------
        case SLALOM_IDLE:
        {
            nav_yaw_output = 0;
            target_speed   = 0;

            if (key_scan(KEY_UP_PIN, &key_up_count))
            {
                slalom_state = SLALOM_GPS_WAIT;
                wireless_printf("SlalomTask: Waiting for GPS fix...\r\n");
            }
            break;
        }

        //----------------------------------------------------------------------
        // 等待 GPS 有效并同步航向角基准
        // 博客方法二：让车先向前行驶，等 GPS 方向角稳定后，
        // 令 nav_heading_angle = gnss.direction 完成同步
        //----------------------------------------------------------------------
        case SLALOM_GPS_WAIT:
        {
            nav_yaw_output = 0;
            target_speed   = 0;

            if (gnss.state == 1 && gnss.satellite_used >= 4 && gnss.speed > 0.5f)
            {
                // GPS 方向角已稳定，同步航向角基准
                nav_heading_angle = gnss.direction;
                heading_err_last  = 0.0f;

                // 记录起点
                origin_point.latitude  = gnss.latitude;
                origin_point.longitude = gnss.longitude;

                // 以当前行进方向为直行目标
                heading_target = nav_heading_angle;

                slalom_state = SLALOM_STRAIGHT_GO;
                wireless_printf("SlalomTask: Synced. heading=%.1f. STRAIGHT_GO.\r\n", heading_target);
            }
            else if (gnss.state == 1 && gnss.satellite_used >= 4)
            {
                // GPS 有效但车还没动，给一点速度让它走起来才能获得方向角
                target_speed = SLALOM_FORWARD_SPEED * 0.5f;
            }
            break;
        }

        //----------------------------------------------------------------------
        // 直行到掉头区（靠行驶距离判断）
        //----------------------------------------------------------------------
        case SLALOM_STRAIGHT_GO:
        {
            double dist_from_start = get_two_points_distance(
                gnss.latitude, gnss.longitude,
                origin_point.latitude, origin_point.longitude);

            // 用高频 nav_heading_angle 做 PD 控制
            float err = heading_error(heading_target, nav_heading_angle);
            nav_yaw_output = heading_pid_calc(err);
            target_speed = SLALOM_FORWARD_SPEED;

            // 每 200ms 打印一次（20ms 周期 × 10）
            static uint8 straight_print_cnt = 0;
            if (++straight_print_cnt >= 10)
            {
                straight_print_cnt = 0;
                wireless_printf("STRAIGHT dist=%.2f tgt=%.1f hdg=%.1f err=%.1f out=%d | run=%d rol=%.1f spd=%d gps=%d\r\n",
                       (float)dist_from_start, heading_target, nav_heading_angle,
                       err, nav_yaw_output, run_state,
                       roll_balance_cascade.posture_value.rol, car_speed, gnss.state);
            }

            if (dist_from_start >= SLALOM_STRAIGHT_DIST)
            {
                target_speed          = 0;
                nav_yaw_output        = 0;
                uturn_start_heading   = nav_heading_angle;
                heading_err_last      = 0.0f;
                slalom_state          = SLALOM_U_TURN;
                wireless_printf("SlalomTask: Reached turnaround. U_TURN from heading=%.1f\r\n",
                       uturn_start_heading);
            }
            break;
        }

        //----------------------------------------------------------------------
        // 掉头：用 nav_heading_angle 跟踪转过 180°
        // 输出线性衰减避免超调
        //----------------------------------------------------------------------
        case SLALOM_U_TURN:
        {
            target_speed = 0;

            float turned = nav_heading_angle - uturn_start_heading;
            // 归一化到 -180~180
            if (turned >  180.0f) turned -= 360.0f;
            if (turned < -180.0f) turned += 360.0f;

            float remaining = SLALOM_UTURN_ANGLE - func_abs(turned);

            if (remaining > 0.0f)
            {
                float raw = remaining * SLALOM_UTURN_KP;
                // 最小 300 保证持续旋转，最大限幅
                nav_yaw_output = (int16)func_limit_ab(raw, 300.0f, (float)SLALOM_HEADING_OUT_MAX);
                // 固定向左转（正值使左轮减速、右轮加速）
            }
            else
            {
                nav_yaw_output = 0;

                // 掉头完成，目标航向反转 180°
                heading_target = uturn_start_heading + 180.0f;
                if (heading_target >= 360.0f) heading_target -= 360.0f;
                heading_err_last = 0.0f;

                current_cone_idx = 0;
                slalom_state     = SLALOM_RETURN;
                wireless_printf("SlalomTask: U_TURN done. turned=%.1f. RETURN. tgt_hdg=%.1f\r\n",
                       turned, heading_target);
            }

            wireless_printf("U_TURN turned=%.1f rem=%.1f out=%d\r\n",
                   turned, remaining, nav_yaw_output);
            break;
        }

        //----------------------------------------------------------------------
        // 返回 + 绕桩
        //----------------------------------------------------------------------
        case SLALOM_RETURN:
        {
            if (current_cone_idx >= slalom_cone_count)
            {
                // 所有锥桶穿完，导航回起点
                heading_target = (float)get_two_points_azimuth(
                    gnss.latitude, gnss.longitude,
                    origin_point.latitude, origin_point.longitude);

                float err = heading_error(heading_target, nav_heading_angle);
                nav_yaw_output = heading_pid_calc(err);
                target_speed = SLALOM_SLALOM_SPEED;

                double dist_to_origin = get_two_points_distance(
                    gnss.latitude, gnss.longitude,
                    origin_point.latitude, origin_point.longitude);

                wireless_printf("TO_ORIGIN dist=%.2f err=%.1f\r\n", (float)dist_to_origin, err);

                if (dist_to_origin <= SLALOM_CONE_ARRIVE_DIST * 2.0)
                {
                    target_speed   = 0;
                    nav_yaw_output = 0;
                    slalom_state   = SLALOM_STOP;
                    wireless_printf("SlalomTask: Done! STOP.\r\n");
                }
                break;
            }

            // 实时更新目标航向和距离
            heading_target = (float)get_two_points_azimuth(
                gnss.latitude, gnss.longitude,
                cone_gps[current_cone_idx].latitude,
                cone_gps[current_cone_idx].longitude);

            double dist_to_cone = get_two_points_distance(
                gnss.latitude, gnss.longitude,
                cone_gps[current_cone_idx].latitude,
                cone_gps[current_cone_idx].longitude);

            float err = heading_error(heading_target, nav_heading_angle);
            nav_yaw_output = heading_pid_calc(err);
            target_speed = SLALOM_SLALOM_SPEED;

            wireless_printf("CONE[%d] dist=%.2f err=%.1f hdg=%.1f out=%d\r\n",
                   current_cone_idx, (float)dist_to_cone, err, nav_heading_angle, nav_yaw_output);

            if (dist_to_cone <= SLALOM_CONE_ARRIVE_DIST)
            {
                heading_err_last = 0.0f;
                current_cone_idx++;
                wireless_printf("SlalomTask: Cone[%d] passed.\r\n", current_cone_idx - 1);
            }
            break;
        }

        //----------------------------------------------------------------------
        // 停止
        //----------------------------------------------------------------------
        case SLALOM_STOP:
        {
            target_speed   = 0;
            nav_yaw_output = 0;
            break;
        }

        default:
            break;
    }
}
