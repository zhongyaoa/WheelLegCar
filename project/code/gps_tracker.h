/*
    惯性导航循迹模块 (Inertial Navigation Tracker)
        - button_up  : 记录当前推算点位（最多 INAV_TRACKER_MAX_POINTS 个）
                       第一个点记录时锁定初始航向、坐标清零为起点
        - button_left: 开始循迹（把车放回起点，头朝初始航向发车）
        - 循迹时用 quat_yaw_deg 控制方向，inav_x/inav_y 里程计判断换点
        - 最后一个目标为第 0 号点（起点），到达后停止
*/
#ifndef _GPS_TRACKER_H_
#define _GPS_TRACKER_H_

#include "zf_common_typedef.h"

#define INAV_TRACKER_MAX_POINTS     10      // 最多记录点位数（含起点）
#define INAV_TRACKER_ARRIVE_DIST    0.1f    // 到达判定距离 (m)
#define INAV_TRACKER_CRUISE_SPEED   (-120.0f) // 循迹前进速度（负值=前进，与 target_speed 约定一致）

typedef enum
{
    TRACKER_STATE_IDLE      = 0,  // 空闲/记录点位阶段
    TRACKER_STATE_RUNNING   = 1,  // 循迹中
    TRACKER_STATE_DONE      = 2,  // 循迹完毕
} tracker_state_enum;

extern tracker_state_enum tracker_state;
extern uint8 tracker_point_count;

// 在主循环中轮询按键（建议每 50ms 调用一次）
void gps_tracker_button_poll(void);

// 在主循环中周期调用以更新循迹逻辑（每 50ms 与按键轮询同频即可）
void gps_tracker_update(void);

// 初始化循迹模块
void gps_tracker_init(void);

#endif
