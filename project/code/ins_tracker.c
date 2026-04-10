#include "ins_tracker.h"
#include "posture_control.h"
#include "controler.h"
#include "subject1.h"
#include "math.h"

// ===== 状态 =====
tracker_state_enum tracker_state       = TRACKER_STATE_IDLE;
uint8              tracker_point_count = 0;

// ===== 内部存储：各点位在 inav 坐标系中的 (x, y)（单位：m）=====
static float recorded_x[INAV_TRACKER_MAX_POINTS];
static float recorded_y[INAV_TRACKER_MAX_POINTS];

// 初始航向锁定滤波
static uint8 heading_lock_count = 0;
static float heading_lock_sum = 0.0f;

// 初始航向锁定滤波
//static uint8 heading_lock_count = 0;
//static float heading_lock_sum = 0.0f;

// 记录第一个点时锁定的初始航向角（°），后续回到此航向=0误差
static float initial_heading_deg = 0.0f;

static uint8 current_target_idx = 1;

// 按键消抖计数
static uint32 btn_up_hold   = 0;
static uint32 btn_left_hold = 0;

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
// 两点间平面距离 (m)
//=============================================================================
static float point_distance(float x0, float y0, float x1, float y1)
{
    float dx = x1 - x0;
    float dy = y1 - y0;
    return sqrtf(dx * dx + dy * dy);
}

//=============================================================================
// 根据距离与转角计算自适应循迹速度
// 距离越近、转角越大，速度越低
//=============================================================================
static float tracker_calc_target_speed(float dist, float heading_err)
{
    float abs_heading_err = fabsf(heading_err);
    float distance_ratio = func_limit_ab(dist / INAV_TRACKER_SLOWDOWN_DIST, 0.0f, 1.0f);
    float angle_ratio;
    float speed_scale;

    if(abs_heading_err <= INAV_TRACKER_TURN_SLOWDOWN_ANG)
    {
        angle_ratio = 1.0f;
    }
    else if(abs_heading_err >= INAV_TRACKER_TURN_STOP_ANG)
    {
        angle_ratio = 0.0f;
    }
    else
    {
        angle_ratio = 1.0f - (abs_heading_err - INAV_TRACKER_TURN_SLOWDOWN_ANG)
                            / (INAV_TRACKER_TURN_STOP_ANG - INAV_TRACKER_TURN_SLOWDOWN_ANG);
    }

    speed_scale = distance_ratio * angle_ratio;
    return INAV_TRACKER_MIN_SPEED + (INAV_TRACKER_CRUISE_SPEED - INAV_TRACKER_MIN_SPEED) * speed_scale;
}

//=============================================================================
// 对目标速度做斜坡限制，避免速度指令跳变
//=============================================================================
static float tracker_ramp_target_speed(float current_speed, float desired_speed)
{
    float delta = desired_speed - current_speed;

    if(delta > INAV_TRACKER_SPEED_RAMP_STEP)
    {
        delta = INAV_TRACKER_SPEED_RAMP_STEP;
    }
    else if(delta < -INAV_TRACKER_SPEED_RAMP_STEP)
    {
        delta = -INAV_TRACKER_SPEED_RAMP_STEP;
    }

    return current_speed + delta;
}

//=============================================================================
// 从 (x0,y0) 到 (x1,y1) 的方位角（°），以初始航向为 0°，顺时针为正
// 坐标系：X 轴正方向 = initial_heading_deg 方向
//=============================================================================
static float point_bearing(float x0, float y0, float x1, float y1)
{
    float dx = x1 - x0;
    float dy = y1 - y0;
    // atan2f 返回弧度：dy/dx，其中 y 轴正方向对应 inav 坐标系的前方（初始航向）
    // 转换为角度，X 轴为前方时用 atan2(dx, dy)
    float bearing_rad = atan2f(dx, dy);
    float bearing_deg = bearing_rad * (180.0f / 3.14159265f);
    return bearing_deg;
}

//=============================================================================
// 初始化
//=============================================================================
void ins_tracker_init(void)
{
    tracker_state         = TRACKER_STATE_IDLE;
    tracker_point_count   = 0;
    current_target_idx    = 1;
    btn_up_hold           = 0;
    btn_left_hold         = 0;
    initial_heading_deg   = 0.0f;
    heading_lock_count    = 0;
    heading_lock_sum      = 0.0f;
    inav_active           = 0;
    inav_x                = 0.0f;
    inav_y                = 0.0f;
    turn_diff_ext         = 0;
}

//=============================================================================
// 外部注入航点并启动循迹（由 UI 发车时调用）
// px, py: 按行驶顺序排列的航点坐标数组
// count:  航点总数（index 0 = 起点）
// 调用前需确保 inav_x/y 已清零、inav_active 已开启、inav_heading_ref 已设定
//=============================================================================
void ins_tracker_start_with_points(const float *px, const float *py, uint8 count)
{
    uint8 i;
    if(count < 2 || count > INAV_TRACKER_MAX_POINTS) return;

    // 将外部航点拷贝到内部存储
    for(i = 0; i < count; i++)
    {
        recorded_x[i] = px[i];
        recorded_y[i] = py[i];
    }

    tracker_point_count = count;
    current_target_idx  = 1;            // 第一个目标是 index 1（跳过起点自身）
    initial_heading_deg = inav_heading_ref;
    tracker_state       = TRACKER_STATE_RUNNING;
    target_speed        = INAV_TRACKER_MIN_SPEED;

#if S1_ISDEBUG
    wireless_printf("[TRK] Start: pts=%d heading_ref=%.1f\r\n", count, initial_heading_deg);
#endif
}

//=============================================================================
// 主循环轮询按键（每 50ms 调用一次）
//=============================================================================
void ins_tracker_button_poll(void)
{
    // ---- button_up：记录点位 ----
    if(button_press(UP))
    {
        btn_up_hold++;
        if(btn_up_hold == 1)  // 上升沿，只触发一次
        {
            if(tracker_state == TRACKER_STATE_IDLE
               && tracker_point_count < INAV_TRACKER_MAX_POINTS)
            {
                if(tracker_point_count == 0)
                {
                    heading_lock_sum += quat_yaw_deg;
                    heading_lock_count++;

                    if(heading_lock_count < INAV_LOCK_HEADING_SAMPLES)
                    {
                        //wireless_printf("[INAV] Lock heading... %d/%d yaw=%.1f\r\n",
                         //               heading_lock_count, INAV_LOCK_HEADING_SAMPLES, quat_yaw_deg);
                        return;
                    }

                    // 第一个点：平均锁定初始航向，坐标系清零
                    initial_heading_deg    = heading_lock_sum / (float)INAV_LOCK_HEADING_SAMPLES;
                    inav_heading_ref       = initial_heading_deg;
                    inav_x                 = 0.0f;
                    inav_y                 = 0.0f;
                    inav_active            = 1;  // 开始惯性导航积分

                    recorded_x[0]          = 0.0f;
                    recorded_y[0]          = 0.0f;
                    tracker_point_count    = 1;
                    heading_lock_count     = 0;
                    heading_lock_sum       = 0.0f;
                    led(toggle);
                    //wireless_printf("[INAV] Point 0 (origin) locked. heading_ref=%.1f deg\r\n",
                     //               initial_heading_deg);
                }
                else
                {
                    // 后续点：记录当前推算坐标
                    recorded_x[tracker_point_count] = inav_x;
                    recorded_y[tracker_point_count] = inav_y;
                    tracker_point_count++;
                    led(toggle);
                    //wireless_printf("[INAV] Point %d recorded: x=%.2f y=%.2f\r\n",
                     //               tracker_point_count - 1,
                       //             recorded_x[tracker_point_count - 1],
                 //                   recorded_y[tracker_point_count - 1]);
                }
            }
            else if(tracker_point_count >= INAV_TRACKER_MAX_POINTS)
            {
                //wireless_printf("[INAV] Max points (%d) reached.\r\n", INAV_TRACKER_MAX_POINTS);
            }
        }
    }
    else
    {
        btn_up_hold = 0;
        heading_lock_count = 0;
        heading_lock_sum = 0.0f;
    }

    // ---- button_left：开始循迹 ----
    if(button_press(LEFT))
    {
        btn_left_hold++;
        if(btn_left_hold == 1)
        {
            if(tracker_state == TRACKER_STATE_IDLE && tracker_point_count >= 2)
            {
                // 将起点（point 0）追加为最终目标，使小车回到起点
                // 注意：point 0 始终是 (0,0)，即出发坐标
                // 确保目标序列为: 1 → 2 → … → (count-1) → 0
                // 我们不物理追加到数组，而是在 update 中特殊处理末尾

                // 重置推算坐标，从起点出发
                inav_x      = 0.0f;
                inav_y      = 0.0f;
                inav_active = 1;

                current_target_idx = 1;
                tracker_state      = TRACKER_STATE_RUNNING;
                target_speed       = INAV_TRACKER_MIN_SPEED;

                //wireless_printf("[INAV] Start tracking. Points=%d heading_ref=%.1f\r\n",
                  //              tracker_point_count, initial_heading_deg);
            }
            else if(tracker_point_count < 2)
            {
                //wireless_printf("[INAV] Need at least 2 points (origin + 1).\r\n");
            }
        }
    }
    else
    {
        btn_left_hold = 0;
    }
}

//=============================================================================
// 循迹更新（在主循环中建议每 10~20ms 调用一次）
//=============================================================================
void ins_tracker_update(void)
{
    if(tracker_state != TRACKER_STATE_RUNNING)
    {
        car_turn_reset();
        return;
    }

    float cur_x = inav_x;
    float cur_y = inav_y;

    // 确定当前目标坐标
    // current_target_idx 范围：1 … tracker_point_count-1
    // 所有航点（包括回起点和超程点）已由 UI 层注入，tracker 只需顺序访问
    if(current_target_idx >= tracker_point_count)
    {
        // 所有航点均已到达，循迹完成
        tracker_state = TRACKER_STATE_DONE;
        target_speed  = 0.0f;
        car_turn_reset();
        inav_active   = 0;
        led(on);
#if S1_ISDEBUG
        wireless_printf("[TRK] All done. x=%.2f y=%.2f\r\n", cur_x, cur_y);
#endif
        return;
    }

    float target_x = recorded_x[current_target_idx];
    float target_y = recorded_y[current_target_idx];

    float dist = point_distance(cur_x, cur_y, target_x, target_y);

    // 到达判定
    if(dist < INAV_TRACKER_ARRIVE_DIST)
    {
#if S1_ISDEBUG
        wireless_printf("[TRK] Arrived pt%d dist=%.2f x=%.2f y=%.2f\r\n",
                        current_target_idx, dist, cur_x, cur_y);
#endif
        current_target_idx++;

        // 检查是否所有航点均已到达
        if(current_target_idx >= tracker_point_count)
        {
            tracker_state = TRACKER_STATE_DONE;
            target_speed  = 0.0f;
            car_turn_reset();
            inav_active   = 0;
            led(on);
#if S1_ISDEBUG
            wireless_printf("[TRK] All done. x=%.2f y=%.2f\r\n", cur_x, cur_y);
#endif
            return;
        }

        // 更新目标
        target_x = recorded_x[current_target_idx];
        target_y = recorded_y[current_target_idx];
        dist = point_distance(cur_x, cur_y, target_x, target_y);
    }

    // 目标方位角（相对于 initial_heading_deg 为 0° 的坐标系）
    float bearing_local = point_bearing(cur_x, cur_y, target_x, target_y);

    // 当前车头相对初始航向的偏差（°）
    float current_heading_rel = normalize_angle(quat_yaw_deg - initial_heading_deg);

    // 偏航误差
    float heading_err = normalize_angle(bearing_local - current_heading_rel);

    // 按距离与转角自适应调速：离目标越近、转弯越大，速度越低
    float desired_speed = tracker_calc_target_speed(dist, heading_err);
    target_speed = tracker_ramp_target_speed(target_speed, desired_speed);

    // 调用 posture_control 中的转向控制函数输出差速
    car_turn_control(heading_err, quat_yaw_rate_dps);

#if S1_ISDEBUG
    {
        static uint8 dbg_cnt = 0;
        if(++dbg_cnt >= 20)
        {
            dbg_cnt = 0;
            wireless_printf("[TRK] ->pt%d dist=%.2f bear=%.1f hdg_rel=%.1f err=%.1f spd=%.0f td=%d x=%.2f y=%.2f\r\n",
                            current_target_idx, dist, bearing_local, current_heading_rel,
                            heading_err, target_speed, (int)turn_diff_ext, cur_x, cur_y);
        }
    }
#endif
}
