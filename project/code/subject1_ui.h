/*
    科目一 UI 模块 —— 绕桩前进
    ─────────────────────────────────────────────────────────
    状态机流程：
        HOME  →（UP 进入采集）→  COLLECT  →（LEFT 完成采集）→  PREVIEW
        PREVIEW →（LEFT 预备）→  STANDBY  →（LEFT 发车）→  RUNNING  →（到达终点）→  DONE
        任意状态 →（SW 长按 1s）→  HOME

    采集阶段按键：
        UP     : 记录当前点（需先通过 DOWN 选好类型）
        DOWN   : 切换点位类型（起点 → 调头点 → 桩位点 → 循环）
        LEFT   : 完成采集，进入路线预览（COLLECT）/ 发车（PREVIEW）
        SE     : 删除上一个记录的点
        SW     : 返回首页（任意界面均可，需长按）

    注意：起点和调头点各只能有一个；桩位点最多 (INAV_TRACKER_MAX_POINTS-2) 个。
    ─────────────────────────────────────────────────────────
*/

#ifndef _SUBJECT1_UI_H_
#define _SUBJECT1_UI_H_

#include "zf_common_typedef.h"

// ─── 点位类型 ────────────────────────────────────────────
typedef enum
{
    POINT_TYPE_START   = 0,   // 起点  S
    POINT_TYPE_TURN    = 1,   // 调头点 T
    POINT_TYPE_CONE    = 2,   // 桩位点 P
} point_type_enum;

// ─── 记录的单个点位 ──────────────────────────────────────
typedef struct
{
    float          x;         // 惯性导航 x 坐标（m）
    float          y;         // 惯性导航 y 坐标（m）
    point_type_enum type;     // 点位类型
} waypoint_t;

// ─── UI 主状态机 ─────────────────────────────────────────
typedef enum
{
    UI_STATE_HOME      = 0,   // 首页
    UI_STATE_COLLECT   = 1,   // 采集点位
    UI_STATE_PREVIEW   = 2,   // 路线预览
    UI_STATE_STANDBY   = 3,   // 预备（维持平衡，等待发车）
    UI_STATE_RUNNING   = 4,   // 行进中
    UI_STATE_DONE      = 5,   // 完成
} ui_state_enum;

// ─── 最大点位数 ──────────────────────────────────────────
#define S1_MAX_WAYPOINTS  20  // 含起点（index 0 固定为 0,0）

// ─── 外部可见状态 ─────────────────────────────────────────
extern ui_state_enum  s1_ui_state;
extern waypoint_t     s1_waypoints[S1_MAX_WAYPOINTS];
extern uint8          s1_waypoint_count;   // 已记录点数（含起点）

// ─── 接口 ─────────────────────────────────────────────────
void subject1_ui_init(void);         // 初始化（在 main 中调用一次）
void subject1_ui_poll(void);         // 主循环轮询（建议每 50ms 调用）

#endif /* _SUBJECT1_UI_H_ */
