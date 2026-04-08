#include "ins_tracker.h"
#include "posture_control.h"
#include "controler.h"
#include "math.h"

// imu660ra_gyro_z: 原始计数 (int16)，用 imu660ra_gyro_transition() 转为 °/s
// 该宏定义在 zf_device_imu660ra.h（已通过 zf_common_headfile.h 包含）
extern int16 imu660ra_gyro_z;

// ===== 状态 =====
tracker_state_enum tracker_state       = TRACKER_STATE_IDLE;
uint8              tracker_point_count = 0;

// ===== 内部存储：各点位在 inav 坐标系中的 (x, y)（单位：m）=====
static float recorded_x[INAV_TRACKER_MAX_POINTS];
static float recorded_y[INAV_TRACKER_MAX_POINTS];

// 初始航向锁定滤波
static uint8 heading_lock_count = 0;
static float heading_lock_sum = 0.0f;

// 记录第一个点时锁定的初始航向角（°），后续回到此航向=0误差
static float initial_heading_deg = 0.0f;

static uint8 current_target_idx = 1;

// 按键消抖计数
static uint32 btn_up_hold   = 0;
static uint32 btn_left_hold = 0;

// ===== 导航外环 → 控制内环 共享状态（volatile 保证中断间可见性）=====
// nav_update（10ms）写，ctrl_update（5ms）读
// Cortex-M7 对齐 float 单次赋值是原子操作，无需关中断
static volatile float s_heading_err   = 0.0f;  // 偏航误差 (°)，正值=需向左转
static volatile float s_speed_desired = 0.0f;  // 导航层期望速度（无斜坡）

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
// 从 (x0,y0) 到 (x1,y1) 的方位角（°），以初始航向为 0°，顺时针为正
// 坐标系：Y 轴正方向 = initial_heading 方向（前进），X 轴正方向 = 右侧
//=============================================================================
static float point_bearing(float x0, float y0, float x1, float y1)
{
    float dx = x1 - x0;
    float dy = y1 - y0;
    // atan2(dx, dy)：Y轴为前方（0°），顺时针为正
    float bearing_rad = atan2f(dx, dy);
    return bearing_rad * (180.0f / 3.14159265f);
}

//=============================================================================
// 根据距离与转角计算自适应目标速度
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
    s_heading_err         = 0.0f;
    s_speed_desired       = INAV_TRACKER_MIN_SPEED;
}

//=============================================================================
// 外部注入航点并启动循迹（由 UI 发车时调用）
//=============================================================================
void ins_tracker_start_with_points(const float *px, const float *py, uint8 count)
{
    uint8 i;
    if(count < 2 || count > INAV_TRACKER_MAX_POINTS) return;

    for(i = 0; i < count; i++)
    {
        recorded_x[i] = px[i];
        recorded_y[i] = py[i];
    }

    tracker_point_count = count;
    current_target_idx  = 1;
    initial_heading_deg = inav_heading_ref;
    tracker_state       = TRACKER_STATE_RUNNING;
    target_speed        = INAV_TRACKER_MIN_SPEED;
    s_speed_desired     = INAV_TRACKER_MIN_SPEED;
    s_heading_err       = 0.0f;
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
                        return;
                    }

                    // 第一个点：平均锁定初始航向，坐标系清零
                    initial_heading_deg    = heading_lock_sum / (float)INAV_LOCK_HEADING_SAMPLES;
                    inav_heading_ref       = initial_heading_deg;
                    inav_x                 = 0.0f;
                    inav_y                 = 0.0f;
                    inav_active            = 1;

                    recorded_x[0]          = 0.0f;
                    recorded_y[0]          = 0.0f;
                    tracker_point_count    = 1;
                    heading_lock_count     = 0;
                    heading_lock_sum       = 0.0f;
                    led(toggle);
                }
                else
                {
                    recorded_x[tracker_point_count] = inav_x;
                    recorded_y[tracker_point_count] = inav_y;
                    tracker_point_count++;
                    led(toggle);
                }
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
                inav_x      = 0.0f;
                inav_y      = 0.0f;
                inav_active = 1;

                current_target_idx = 1;
                tracker_state      = TRACKER_STATE_RUNNING;
                target_speed       = INAV_TRACKER_MIN_SPEED;
                s_speed_desired    = INAV_TRACKER_MIN_SPEED;
            }
        }
    }
    else
    {
        btn_left_hold = 0;
    }
}

//=============================================================================
// 导航外环（每 10ms 调用，含重浮点运算）
// 负责：距离计算、目标方位角、前瞻航点切换、自适应速度计算
// 结果写入 s_heading_err 和 s_speed_desired（volatile，供控制内环读取）
//=============================================================================
void ins_tracker_nav_update(void)
{
    if(tracker_state != TRACKER_STATE_RUNNING)
    {
        return;
    }

    if(current_target_idx >= tracker_point_count)
    {
        // 所有航点均已到达，循迹完成
        tracker_state  = TRACKER_STATE_DONE;
        target_speed   = 0.0f;
        turn_diff_ext  = 0;
        inav_active    = 0;
        led(on);
        return;
    }

    float cur_x = inav_x;
    float cur_y = inav_y;
    float target_x = recorded_x[current_target_idx];
    float target_y = recorded_y[current_target_idx];

    float dist = point_distance(cur_x, cur_y, target_x, target_y);

    // ── 目标方位角与当前航向误差 ────────────────────────────────────────────
    float bearing_local      = point_bearing(cur_x, cur_y, target_x, target_y);
    float current_heading_rel = normalize_angle(quat_yaw_deg - initial_heading_deg);
    float heading_err         = normalize_angle(bearing_local - current_heading_rel);

    // ── 航点切换判定：仅凭距离判定到达 ─────────────────────────────────────
    // 注意：科目一需要在调头点掉头后沿回程蛇行，回程各航点在刚到调头点时均
    //       相对车身偏侧/偏后，若使用"目标在身后"或"前瞻偏角"等条件极易
    //       在同一帧内级联跳过所有回程航点，导致到调头点即宣告完成。
    //       因此只使用 dist < ARRIVE_DIST 作为唯一切换触发条件，简单可靠。
    if(dist < INAV_TRACKER_ARRIVE_DIST)
    {
        current_target_idx++;

        if(current_target_idx >= tracker_point_count)
        {
            // 全部到达
            tracker_state  = TRACKER_STATE_DONE;
            target_speed   = 0.0f;
            turn_diff_ext  = 0;
            inav_active    = 0;
            led(on);
            return;
        }

        // 切换后立刻重算，避免等下一个 10ms 周期出现转向滞后
        target_x = recorded_x[current_target_idx];
        target_y = recorded_y[current_target_idx];
        dist     = point_distance(cur_x, cur_y, target_x, target_y);
        bearing_local = point_bearing(cur_x, cur_y, target_x, target_y);
        heading_err   = normalize_angle(bearing_local - current_heading_rel);
    }

    // ── 写共享变量（volatile 原子写）────────────────────────────────────────
    s_heading_err   = heading_err;
    s_speed_desired = tracker_calc_target_speed(dist, heading_err);
}

//=============================================================================
// 控制内环（每 5ms 调用，仅含乘加运算）
// 负责：读取导航层计算结果，执行航向 PD 控制，输出 turn_diff_ext 和 target_speed
// KD 项使用 imu660ra_gyro_z（陀螺仪原始信号，比 quat_yaw_rate_dps 差分更干净）
//
// ⚠ 符号约定：
//   heading_err > 0  → 目标在左侧 → 需左转  → turn_diff_ext 应为正
//   imu660ra_gyro_z > 0 → 机体正在左转（逆时针）→ 应产生右转阻尼（正 KD 减小 td）
//   若上电测试时符号方向反，将 TRACKER_YAW_KD 改为负值即可
//=============================================================================
void ins_tracker_ctrl_update(void)
{
    if(tracker_state != TRACKER_STATE_RUNNING)
    {
        turn_diff_ext = 0;
        return;
    }

    // 读导航层共享状态
    float heading_err  = s_heading_err;
    float speed_target = s_speed_desired;

    // 航向 PD 控制
    // imu660ra_gyro_z 为原始计数，imu660ra_gyro_transition() 将其转为 °/s
    float gyro_z_dps = imu660ra_gyro_transition(imu660ra_gyro_z);
    int16 td = (int16)(TRACKER_YAW_KP * heading_err + TRACKER_YAW_KD * gyro_z_dps);
    turn_diff_ext = func_limit_ab(-td, -turn_duty_max, turn_duty_max);

    // 速度斜坡（在此处做，保证 target_speed 变化平滑）
    target_speed = tracker_ramp_target_speed(target_speed, speed_target);
}

//=============================================================================
// 兼容包装：顺序调用导航外环 + 控制内环
// 适用于主循环或旧调用点（50ms 轮询）；推荐改为在 PIT 中分频调用两个子函数
//=============================================================================
void ins_tracker_update(void)
{
    ins_tracker_nav_update();
    ins_tracker_ctrl_update();
}
