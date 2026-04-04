/*
    GPS 循迹模块
        - button_up：记录当前 GPS 点位（最多 GPS_TRACKER_MAX_POINTS 个）
        - button_left：开始循迹（车头对准第二个点位放到第一个点位处）
        - 循迹时通过 IMU yaw 积分控制方向，GPS 距离判断换点
*/
#ifndef _GPS_TRACKER_H_
#define _GPS_TRACKER_H_

#include "zf_common_typedef.h"
#include "zf_device_gnss.h"

#define GPS_TRACKER_MAX_POINTS      10      // 最多记录点位数
#define GPS_TRACKER_ARRIVE_DIST     1.5     // 到达判定距离 (m)
#define GPS_TRACKER_CRUISE_SPEED    (-120.0f)// 循迹前进速度（负值=前进，与 target_speed 约定一致）
#define GPS_TRACKER_YAW_KP          6.0f    // 偏航纠偏 P 增益（度→duty映射，实际由posture yaw PD执行）

typedef enum
{
    TRACKER_STATE_IDLE      = 0,  // 空闲/记录点位阶段
    TRACKER_STATE_RUNNING   = 1,  // 循迹中
    TRACKER_STATE_DONE      = 2,  // 循迹完毕
} tracker_state_enum;

extern tracker_state_enum tracker_state;
extern uint8 tracker_point_count;

// 在主循环中每次 GPS 数据更新后调用
void gps_tracker_update(void);

// 在主循环中轮询按键（建议每 50ms 调用一次）
void gps_tracker_button_poll(void);

// 初始化循迹模块（在 gnss_init 之后调用）
void gps_tracker_init(void);

#endif
