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
#define INAV_TRACKER_ARRIVE_DIST       0.1f       // 到达判定距离 (m)
#define INAV_TRACKER_CRUISE_SPEED      (600.0f) // 直线巡航速度（负值=前进，与 target_speed 约定一致）
#define INAV_TRACKER_MIN_SPEED         (200.0f)  // 最低循迹速度，避免转向时完全没速度
#define INAV_TRACKER_SLOWDOWN_DIST     1.0f       // 距离小于该值时开始按距离降速 (m)
#define INAV_TRACKER_TURN_SLOWDOWN_ANG 25.0f      // 转角大于该值时开始按角度降速 (deg)
#define INAV_TRACKER_TURN_STOP_ANG     70.0f      // 转角接近该值时降到最低速度 (deg)
#define INAV_TRACKER_SPEED_RAMP_STEP   60.0f      // 每次循迹更新允许的目标速度最大变化量

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

// 在主循环中周期调用以更新循迹逻辑（建议每 10~20ms 调用一次）
void ins_tracker_update(void);

// 初始化循迹模块
void ins_tracker_init(void);

// 外部注入航点并启动循迹（由 UI 在发车时调用）
// points: 航点坐标数组（按行驶顺序排列）
// count:  航点个数（含起点，至少 2 个）
// 调用后 tracker_state 自动变为 TRACKER_STATE_RUNNING
void ins_tracker_start_with_points(const float *px, const float *py, uint8 count);

#endif
