/*
    比赛 UI 模块 —— 多科目管理（行进绕桩 / 定点排雷 / 颠簸路段）
    ─────────────────────────────────────────────────────────
    比赛流程：
        1. 发车前：依次进入各科目采点（人推车采集路径数据）
        2. 比赛阶段：从首页选择科目，发车，完成后可重跑或换科目

    首页按键：
        UP/DOWN : 切换选中科目
        LEFT    : 进入选中科目（有数据→路线预览，无数据→采集）
        SE(1s)  : 清空当前科目数据，重新采集

    采集阶段按键：
        UP     : 记录当前点（需先通过 DOWN 选好类型）
        DOWN   : 切换点位类型（起点 → 调头点 → 桩位点 → 循环）
        LEFT   : 完成采集，进入路线预览
        SE     : 删除上一个记录的点
        SW(1s) : 返回首页（数据保留，不清空）

    完成科目后：
        UP     : 重跑（保留采点数据，直接进预备→发车）
        SW     : 返回首页（数据保留）

    注意：起点和调头点各只能有一个；桩位点最多 (INAV_TRACKER_MAX_POINTS-2) 个。
    科目2/3 当前为 Coming Soon 占位。
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
extern uint8          selected_subject;    // 首页当前高亮科目 (0=S1, 1=S2, 2=S3)

// ─── 接口 ─────────────────────────────────────────────────
void subject1_ui_init(void);         // 初始化（在 main 中调用一次）
void subject1_ui_poll(void);         // 主循环轮询（建议每 50ms 调用）

#endif /* _SUBJECT1_UI_H_ */
