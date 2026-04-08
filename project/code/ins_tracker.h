/*
    惯性导航循迹模块 (Inertial Navigation Tracker)
        - button_up  : 记录当前推算点位（最多 INAV_TRACKER_MAX_POINTS 个）
                       第一个点记录时锁定初始航向、坐标清零为起点
        - button_left: 开始循迹（把车放回起点，头朝初始航向发车）
        - 循迹时用 quat_yaw_deg 控制方向，inav_x/inav_y 里程计判断换点
        - 最后一个目标为第 0 号点（起点），到达后停止
*/
#ifndef _INS_TRACKER_H_
#define _INS_TRACKER_H_

#include "zf_common_typedef.h"

#define INAV_TRACKER_MAX_POINTS        40         // 最多记录点位数（含起点，需容纳去程+回程）

// ── 速度参数（可调）────────────────────────────────────────────────────────
#define INAV_TRACKER_CRUISE_SPEED      800.0f     // 直线巡航速度（RPM），原600→800
#define INAV_TRACKER_MIN_SPEED         250.0f     // 最低循迹速度，原200→250
#define INAV_TRACKER_SPEED_RAMP_STEP   80.0f      // 每次循迹更新允许的目标速度最大变化量，原60→80

// ── 距离参数 ────────────────────────────────────────────────────────────────
#define INAV_TRACKER_ARRIVE_DIST       0.15f      // 到达判定距离 (m)，原0.1→0.15
#define INAV_TRACKER_SLOWDOWN_DIST     0.7f       // 距离小于该值时开始按距离降速 (m)，原1.0→0.7

// ── 角度降速参数（按转角减速，用于大角度转弯保护）──────────────────────────
#define INAV_TRACKER_TURN_SLOWDOWN_ANG 25.0f      // 转角大于该值时开始按角度降速 (deg)
#define INAV_TRACKER_TURN_STOP_ANG     70.0f      // 转角接近该值时降到最低速度 (deg)

// ── 前瞻切换参数 ────────────────────────────────────────────────────────────
// 同时满足距离近 + 偏角大，则提前切换下一航点，避免到达后急转
#define INAV_TRACKER_LOOKAHEAD_DIST    0.40f      // 前瞻切换距离阈值 (m)
#define INAV_TRACKER_LOOKAHEAD_ANG     40.0f      // 前瞻切换角度阈值 (deg)

// ── Yaw PD 控制参数 ──────────────────────────────────────────────────────────
// KD 作用于 imu660ra_gyro_z（陀螺仪原始信号，°/s），信号比差分 yaw_rate 更干净
#define TRACKER_YAW_KP                 8.0f       // 偏航角度增益 (duty/°)
#define TRACKER_YAW_KD                 -1.5f       // 偏航角速度阻尼 (duty/(°/s))，原0.5→1.5

typedef enum
{
    TRACKER_STATE_IDLE      = 0,  // 空闲/记录点位阶段
    TRACKER_STATE_RUNNING   = 1,  // 循迹中
    TRACKER_STATE_DONE      = 2,  // 循迹完毕
} tracker_state_enum;

extern tracker_state_enum tracker_state;
extern uint8 tracker_point_count;

// 在主循环中轮询按键（建议每 10~20ms 调用一次）
void ins_tracker_button_poll(void);

// ── 循迹更新（拆分为两层，均在 pit_call_back 中调用）──────────────────────
// 导航外环：含 sqrtf/atan2f 等重运算，在 pit_call_back 中每 10ms 调用一次
//   完成：距离计算、目标方位角、前瞻航点切换、自适应速度
void ins_tracker_nav_update(void);

// 控制内环：仅乘加运算，在 pit_call_back 中每 5ms 调用一次
//   完成：航向 PD 输出 → turn_diff_ext，速度斜坡 → target_speed
void ins_tracker_ctrl_update(void);

// 兼容包装：顺序调用 nav_update + ctrl_update（主循环或旧调用点使用）
void ins_tracker_update(void);

// 初始化循迹模块
void ins_tracker_init(void);

// 外部注入航点并启动循迹（由 UI 在发车时调用）
// points: 航点坐标数组（按行驶顺序排列）
// count:  航点个数（含起点，至少 2 个）
// 调用后 tracker_state 自动变为 TRACKER_STATE_RUNNING
void ins_tracker_start_with_points(const float *px, const float *py, uint8 count);

#endif
