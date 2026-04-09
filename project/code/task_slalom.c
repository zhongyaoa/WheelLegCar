#include "task_slalom.h"
#include "zf_device_gnss.h"
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
static gps_point_struct turnaround_point;

static float        heading_err_last    = 0.0f;
static float        uturn_start_yaw     = 0.0f;

// 按键消抖计数（调用周期 20ms，KEY_DEBOUNCE_MS 个周期 = 400ms 确认）
static uint16       key_a_count         = 0;
static uint16       key_b_count         = 0;
static uint16       key_up_count        = 0;
#define KEY_DEBOUNCE_MS     20

//=============================================================================
// 内部函数：航向误差归一化到 -180~180
//=============================================================================
static float heading_error(float target_bearing, float current_bearing)
{
    float err = target_bearing - current_bearing;
    if (err >  180.0f) err -= 360.0f;
    if (err < -180.0f) err += 360.0f;
    return err;
}

//=============================================================================
// 内部函数：航向 PD 控制器
// 调用前必须先更新 heading_err_last（切换状态时清零）
//=============================================================================
static int16 heading_pid_calc(float err)
{
    float d_err = err - heading_err_last;
    heading_err_last = err;
    float output = SLALOM_HEADING_KP * err + SLALOM_HEADING_KD * d_err;
    return (int16)func_limit_ab(output, -SLALOM_HEADING_OUT_MAX, SLALOM_HEADING_OUT_MAX);
}

//=============================================================================
// 内部函数：按键扫描，低电平有效，消抖后触发一次
//=============================================================================
static uint8 key_scan(gpio_pin_enum pin, uint16 *count)
{
    if (gpio_get_level(pin) == GPIO_LOW)
    {
        (*count)++;
        if (*count == KEY_DEBOUNCE_MS)
            return 1;
    }
    else
    {
        *count = 0;
    }
    return 0;
}

//=============================================================================
// 内部函数：获取当前 GPS 地面航向（0~360，北为0顺时针）
// 车速 > 0.3 km/h 时 GPS direction 才有意义；否则用 IMU yaw 补偿
//=============================================================================
static float get_current_bearing(void)
{
    if (gnss.state == 1 && gnss.speed > 0.3f)
    {
        return gnss.direction;
    }
    // IMU yaw 范围 -180~180，转为 0~360
    float yaw = roll_balance_cascade.posture_value.yaw;
    return (yaw < 0.0f) ? (yaw + 360.0f) : yaw;
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

    slalom_state      = SLALOM_RECORD_MODE;
    slalom_cone_count = 0;
    current_cone_idx  = 0;
    nav_yaw_output    = 0;

    printf("SlalomTask: RECORD_MODE. Press A=record cone, B=finish recording.\r\n");
}

//=============================================================================
// task_slalom_update — 约 20ms 调用一次
//=============================================================================
void task_slalom_update(void)
{
    if (gnss_flag)
    {
        gnss_data_parse();
        gnss_flag = 0;
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
                    printf("Cone[%d] lat=%.6f lon=%.6f\r\n",
                           slalom_cone_count, gnss.latitude, gnss.longitude);
                }
                else
                {
                    printf("GPS invalid or max cones reached.\r\n");
                }
            }
            if (key_scan(KEY_B_PIN, &key_b_count))
            {
                slalom_state = SLALOM_IDLE;
                printf("SlalomTask: %d cones recorded. Press UP to start.\r\n", slalom_cone_count);
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
                printf("SlalomTask: Waiting for GPS fix...\r\n");
            }
            break;
        }

        //----------------------------------------------------------------------
        // 等待 GPS 有效并记录起点
        // 注意：静止时 gnss.direction 无意义，掉头区坐标必须在车开始走之后才能
        // 确定。这里只记录起点，掉头区在 STRAIGHT_GO 里用实时 GPS 判断距离。
        //----------------------------------------------------------------------
        case SLALOM_GPS_WAIT:
        {
            nav_yaw_output = 0;
            target_speed   = 0;

            if (gnss.state == 1 && gnss.satellite_used >= 4)
            {
                origin_point.latitude   = gnss.latitude;
                origin_point.longitude  = gnss.longitude;

                // 掉头区坐标暂时设为无效（0,0），在 STRAIGHT_GO 里靠距离判断
                turnaround_point.latitude  = 0.0;
                turnaround_point.longitude = 0.0;

                // 初始目标航向用 GPS 双天线测向（若有效）；否则用 IMU yaw
                if (gnss.antenna_direction_state == 1)
                    heading_target = gnss.antenna_direction;
                else
                {
                    float yaw = roll_balance_cascade.posture_value.yaw;
                    heading_target = (yaw < 0.0f) ? (yaw + 360.0f) : yaw;
                }

                heading_err_last = 0.0f;
                slalom_state = SLALOM_STRAIGHT_GO;
                printf("SlalomTask: GPS OK. STRAIGHT_GO. heading_target=%.1f\r\n", heading_target);
            }
            break;
        }

        //----------------------------------------------------------------------
        // 直行到掉头区（靠距离判断，不靠预算坐标）
        //----------------------------------------------------------------------
        case SLALOM_STRAIGHT_GO:
        {
            double dist_from_start = get_two_points_distance(
                gnss.latitude, gnss.longitude,
                origin_point.latitude, origin_point.longitude);

            float cur_bearing = get_current_bearing();
            float err = heading_error(heading_target, cur_bearing);
            nav_yaw_output = heading_pid_calc(err);
            target_speed = SLALOM_FORWARD_SPEED;

            printf("STRAIGHT dist=%.2f err=%.1f out=%d\r\n",
                   (float)dist_from_start, err, nav_yaw_output);

            if (dist_from_start >= SLALOM_STRAIGHT_DIST)
            {
                // 记录掉头区位置（此时是真实 GPS 坐标）
                turnaround_point.latitude  = gnss.latitude;
                turnaround_point.longitude = gnss.longitude;

                target_speed    = 0;
                nav_yaw_output  = 0;

                // 记录掉头起始 yaw，目标是转 180°
                uturn_start_yaw  = roll_balance_cascade.posture_value.yaw;
                // 目标 yaw = 当前 yaw ± 180（用 IMU 跟踪）
                // 掉头方向固定向左（正 nav_yaw_output）
                heading_err_last = 0.0f;

                slalom_state = SLALOM_U_TURN;
                printf("SlalomTask: Reached turnaround. Start U_TURN from yaw=%.1f\r\n",
                       uturn_start_yaw);
            }
            break;
        }

        //----------------------------------------------------------------------
        // 掉头：用 IMU yaw 跟踪转过 180°
        // 掉头期间 target_speed 保持 0，balance 自己维持平衡
        // nav_yaw_output 产生差速让车原地转
        // 关键：nav_yaw_output 必须小于 balance_duty_max，否则超出 turn_duty_max 被截断
        //----------------------------------------------------------------------
        case SLALOM_U_TURN:
        {
            target_speed = 0;   // 不前进，只旋转

            float yaw_now    = roll_balance_cascade.posture_value.yaw;
            float yaw_turned = yaw_now - uturn_start_yaw;

            // 归一化到 -180~180
            if (yaw_turned >  180.0f) yaw_turned -= 360.0f;
            if (yaw_turned < -180.0f) yaw_turned += 360.0f;

            // 目标转 180°，剩余误差用于渐减输出（防止超调）
            float remaining = SLALOM_UTURN_ANGLE - func_abs(yaw_turned);
            if (remaining > 0.0f)
            {
                // 输出随剩余角度线性衰减，最小保持 300 确保持续旋转
                float raw_out = remaining * SLALOM_UTURN_KP;
                int16 yaw_out = (int16)func_limit_ab(raw_out, 300, SLALOM_HEADING_OUT_MAX);
                nav_yaw_output = yaw_out;   // 正值 = 左转（nav_yaw_output > 0 → 左轮减速右轮加速）
            }
            else
            {
                // 转够了
                nav_yaw_output  = 0;
                current_cone_idx = 0;

                // 掉头后目标航向 = 原来方向反向（0~360）
                heading_target = heading_target + 180.0f;
                if (heading_target >= 360.0f) heading_target -= 360.0f;

                heading_err_last = 0.0f;
                slalom_state = SLALOM_RETURN;
                printf("SlalomTask: U_TURN done. yaw_turned=%.1f. RETURN start.\r\n", yaw_turned);
            }

            printf("U_TURN turned=%.1f remaining=%.1f out=%d\r\n",
                   yaw_turned, remaining, nav_yaw_output);
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
                double dist_to_origin = get_two_points_distance(
                    gnss.latitude, gnss.longitude,
                    origin_point.latitude, origin_point.longitude);

                heading_target = (float)get_two_points_azimuth(
                    gnss.latitude, gnss.longitude,
                    origin_point.latitude, origin_point.longitude);

                float err = heading_error(heading_target, get_current_bearing());
                nav_yaw_output = heading_pid_calc(err);
                target_speed = SLALOM_SLALOM_SPEED;

                printf("TO_ORIGIN dist=%.2f err=%.1f\r\n", (float)dist_to_origin, err);

                if (dist_to_origin <= SLALOM_CONE_ARRIVE_DIST * 2.0)
                {
                    target_speed   = 0;
                    nav_yaw_output = 0;
                    slalom_state   = SLALOM_STOP;
                    printf("SlalomTask: Done! STOP.\r\n");
                }
                break;
            }

            // 实时更新目标航向和距离
            double dist_to_cone = get_two_points_distance(
                gnss.latitude, gnss.longitude,
                cone_gps[current_cone_idx].latitude,
                cone_gps[current_cone_idx].longitude);

            heading_target = (float)get_two_points_azimuth(
                gnss.latitude, gnss.longitude,
                cone_gps[current_cone_idx].latitude,
                cone_gps[current_cone_idx].longitude);

            float err = heading_error(heading_target, get_current_bearing());
            nav_yaw_output = heading_pid_calc(err);
            target_speed = SLALOM_SLALOM_SPEED;

            printf("CONE[%d] dist=%.2f err=%.1f out=%d\r\n",
                   current_cone_idx, (float)dist_to_cone, err, nav_yaw_output);

            if (dist_to_cone <= SLALOM_CONE_ARRIVE_DIST)
            {
                heading_err_last = 0.0f;    // 切换目标前清零微分项
                current_cone_idx++;
                printf("SlalomTask: Cone[%d] passed.\r\n", current_cone_idx - 1);
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
