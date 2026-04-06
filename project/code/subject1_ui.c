/*
    科目一 UI 模块实现 —— 绕桩前进
    ─────────────────────────────────────────────────────────
    功能概述：
      1. 首页选择科目一
      2. 采集三种点位（起点 / 调头点 / 桩位点）
      3. 预览路线图（含绕桩路线绘制）
      4. 发车行进、完成
    ─────────────────────────────────────────────────────────
    屏幕: IPS200  240(W) x 320(H)  竖屏
    按键: UP / DOWN / LEFT / NE / SE / SW
    ─────────────────────────────────────────────────────────
*/

#include "subject1_ui.h"
#include "controler.h"
#include "posture_control.h"
#include "zf_common_headfile.h"
#include <math.h>
#include <string.h>

// ===================== 屏幕颜色定义 =====================
#define COLOR_BG           RGB565_BLACK
#define COLOR_TITLE        RGB565_WHITE
#define COLOR_TEXT          RGB565_WHITE
#define COLOR_HIGHLIGHT    RGB565_YELLOW
#define COLOR_START_PT     RGB565_GREEN     // 起点颜色
#define COLOR_TURN_PT      RGB565_RED       // 调头点颜色
#define COLOR_CONE_PT      RGB565_ORANGE    // 桩位点颜色（橙色近似）
#define COLOR_ROUTE_FWD    RGB565_66CCFF    // 去程路线
#define COLOR_ROUTE_RET    RGB565_PURPLE    // 回程路线
#define COLOR_CAR          RGB565_GREEN     // 行进中小车标记
#define COLOR_SELECTED     RGB565_YELLOW    // 选中项高亮
#define COLOR_GRID         0x2104           // 暗灰网格

// 橙色和紫色的 RGB565 近似值
#ifndef RGB565_ORANGE
#define RGB565_ORANGE      0xFD20
#endif
#ifndef RGB565_PURPLE
#define RGB565_PURPLE      0xA01F
#endif

// ===================== IPS200 屏幕接口类型 =====================
#define IPS200_TYPE        (IPS200_TYPE_SPI)

// ===================== 全局状态 =====================
ui_state_enum  s1_ui_state        = UI_STATE_HOME;
waypoint_t     s1_waypoints[S1_MAX_WAYPOINTS];
uint8          s1_waypoint_count  = 0;

// ===================== 内部状态 =====================
static point_type_enum s1_cur_point_type = POINT_TYPE_START;   // 当前选择的点类型
static uint8  s1_has_start  = 0;   // 是否已记录起点
static uint8  s1_has_turn   = 0;   // 是否已记录调头点
static uint8  s1_screen_dirty = 1; // 是否需要重绘屏幕

// 按键消抖计数（每次 poll 约 50ms）
static uint32 btn_up_cnt   = 0;
static uint32 btn_down_cnt = 0;
static uint32 btn_left_cnt = 0;
static uint32 btn_ne_cnt   = 0;
static uint32 btn_se_cnt   = 0;
static uint32 btn_sw_cnt   = 0;

// 行进中刷新降频
static uint32 run_display_cnt = 0;

// ===================== 按键边沿检测宏 =====================
// 返回 1 表示本次为上升沿（第一次按下）
static uint8 btn_rising(Button bt, uint32 *cnt)
{
    if(button_press(bt))
    {
        (*cnt)++;
        if(*cnt == 1) return 1;   // 上升沿
        return 0;
    }
    else
    {
        *cnt = 0;
        return 0;
    }
}

// 长按检测（hold_ticks * 50ms = 所需时长）
static uint8 btn_long_press(Button bt, uint32 *cnt, uint32 hold_ticks)
{
    if(button_press(bt))
    {
        (*cnt)++;
        if(*cnt == hold_ticks) return 1;
        return 0;
    }
    else
    {
        *cnt = 0;
        return 0;
    }
}

// ===================== 点类型名称 =====================
static const char* point_type_name(point_type_enum t)
{
    switch(t)
    {
        case POINT_TYPE_START: return "START";
        case POINT_TYPE_TURN:  return "TURN ";
        case POINT_TYPE_CONE:  return "CONE ";
        default:               return "?????";
    }
}

static char point_type_char(point_type_enum t)
{
    switch(t)
    {
        case POINT_TYPE_START: return 'S';
        case POINT_TYPE_TURN:  return 'T';
        case POINT_TYPE_CONE:  return 'P';
        default:               return '?';
    }
}

static uint16 point_type_color(point_type_enum t)
{
    switch(t)
    {
        case POINT_TYPE_START: return COLOR_START_PT;
        case POINT_TYPE_TURN:  return COLOR_TURN_PT;
        case POINT_TYPE_CONE:  return COLOR_CONE_PT;
        default:               return COLOR_TEXT;
    }
}

// ===================== 绘制辅助 =====================

// 画一个实心小方块（模拟粗点）
static void draw_dot(uint16 cx, uint16 cy, uint16 radius, uint16 color)
{
    uint16 x, y;
    for(y = cy - radius; y <= cy + radius; y++)
    {
        for(x = cx - radius; x <= cx + radius; x++)
        {
            if(x < 240 && y < 320)
            {
                ips200_draw_point(x, y, color);
            }
        }
    }
}

// 画一个空心圆圈（Bresenham 简化版）
static void draw_circle(uint16 cx, uint16 cy, uint16 r, uint16 color)
{
    int16 x = 0, y = (int16)r;
    int16 d = 1 - (int16)r;

    while(x <= y)
    {
        ips200_draw_point((uint16)(cx + x), (uint16)(cy + y), color);
        ips200_draw_point((uint16)(cx - x), (uint16)(cy + y), color);
        ips200_draw_point((uint16)(cx + x), (uint16)(cy - y), color);
        ips200_draw_point((uint16)(cx - x), (uint16)(cy - y), color);
        ips200_draw_point((uint16)(cx + y), (uint16)(cy + x), color);
        ips200_draw_point((uint16)(cx - y), (uint16)(cy + x), color);
        ips200_draw_point((uint16)(cx + y), (uint16)(cy - x), color);
        ips200_draw_point((uint16)(cx - y), (uint16)(cy - x), color);

        if(d < 0)
        {
            d += 2 * x + 3;
        }
        else
        {
            d += 2 * (x - y) + 5;
            y--;
        }
        x++;
    }
}

// =========================================================================
//  HOME 页面：显示科目选择菜单
// =========================================================================
static void draw_home(void)
{
    ips200_full(COLOR_BG);

    // 标题
    ips200_set_color(COLOR_TITLE, COLOR_BG);
    ips200_show_string(40, 30,  "WHEEL-LEG  CAR");
    ips200_show_string(40, 55,  "--------------");

    // 菜单条目
    ips200_set_color(COLOR_SELECTED, COLOR_BG);
    ips200_show_string(30, 100, "> Subject 1");
    ips200_set_color(COLOR_TEXT, COLOR_BG);
    ips200_show_string(40, 118, "Slalom Course");

    ips200_set_color(0x8410, COLOR_BG);  // 灰色，尚未开放
    ips200_show_string(30, 160, "  Subject 2");
    ips200_show_string(40, 178, "(Coming Soon)");

    ips200_show_string(30, 210, "  Subject 3");
    ips200_show_string(40, 228, "(Coming Soon)");

    // 底部提示
    ips200_set_color(COLOR_HIGHLIGHT, COLOR_BG);
    ips200_show_string(20, 296, "[UP] Enter Subject 1");
}

// =========================================================================
//  COLLECT 页面：显示采集信息
// =========================================================================
static void draw_collect(void)
{
    uint8 i;

    ips200_full(COLOR_BG);

    // 标题
    ips200_set_color(COLOR_HIGHLIGHT, COLOR_BG);
    ips200_show_string(20, 4, "== Collect Points ==");

    // 当前选择的点位类型
    ips200_set_color(point_type_color(s1_cur_point_type), COLOR_BG);
    ips200_show_string(8, 28, "Type:");
    ips200_show_string(56, 28, point_type_name(s1_cur_point_type));

    // 状态标志
    ips200_set_color(COLOR_TEXT, COLOR_BG);
    ips200_show_string(140, 28, "S:");
    ips200_show_string(164, 28, s1_has_start ? "Y" : "N");
    ips200_show_string(180, 28, "T:");
    ips200_show_string(204, 28, s1_has_turn  ? "Y" : "N");

    // 当前惯导坐标
    ips200_set_color(COLOR_TEXT, COLOR_BG);
    ips200_show_string(8,  48, "X:");
    ips200_show_float(28,  48, inav_x, 3, 2);
    ips200_show_string(120, 48, "Y:");
    ips200_show_float(140, 48, inav_y, 3, 2);

    // 已记录点位列表（最多显示 10 行）
    ips200_set_color(COLOR_TITLE, COLOR_BG);
    ips200_show_string(8, 72, "# Type   X      Y");
    ips200_draw_line(8, 88, 232, 88, 0x4208);

    for(i = 0; i < s1_waypoint_count && i < 10; i++)
    {
        uint16 row_y = 92 + i * 16;
        ips200_set_color(point_type_color(s1_waypoints[i].type), COLOR_BG);

        // 序号
        ips200_show_uint(8,  row_y, i, 2);
        // 类型字符
        ips200_show_char(36, row_y, point_type_char(s1_waypoints[i].type));
        // 坐标
        ips200_show_float(60,  row_y, s1_waypoints[i].x, 3, 2);
        ips200_show_float(150, row_y, s1_waypoints[i].y, 3, 2);
    }

    // 总数
    ips200_set_color(COLOR_TEXT, COLOR_BG);
    ips200_show_string(8,  260, "Total:");
    ips200_show_uint(60, 260, s1_waypoint_count, 2);

    // 底部操作提示
    ips200_set_color(COLOR_HIGHLIGHT, COLOR_BG);
    ips200_show_string(0,  280, "UP:Add DOWN:Type");
    ips200_show_string(0,  296, "NE:Done SE:Del SW:Back");
}

// =========================================================================
//  PREVIEW 页面：绘制完整路线图
//  路线逻辑：起点 → 桩1 → 桩2 → … → 桩N → 调头点
//           调头点 → 桩N → … → 桩2 → 桩1 → 起点
//  绕桩方式：去程从左侧绕，回程从右侧绕（偏移绘制模拟绕桩弧线）
// =========================================================================

// 路线绘图区域
#define MAP_X0     10      // 地图区左上角 X
#define MAP_Y0     30      // 地图区左上角 Y
#define MAP_W      220     // 地图区宽度
#define MAP_H      250     // 地图区高度

static void draw_preview(void)
{
    uint8 i;
    int16 j;
    float min_x, max_x, min_y, max_y;
    float range_x, range_y, scale;
    float offset_x, offset_y;

    ips200_full(COLOR_BG);

    // 标题
    ips200_set_color(COLOR_HIGHLIGHT, COLOR_BG);
    ips200_show_string(20, 4, "== Route Preview ==");

    if(s1_waypoint_count < 2)
    {
        ips200_set_color(COLOR_TEXT, COLOR_BG);
        ips200_show_string(40, 150, "Not enough pts!");
        return;
    }

    // ─── 计算坐标范围（自动缩放适配屏幕） ───
    min_x = max_x = s1_waypoints[0].x;
    min_y = max_y = s1_waypoints[0].y;
    for(i = 1; i < s1_waypoint_count; i++)
    {
        if(s1_waypoints[i].x < min_x) min_x = s1_waypoints[i].x;
        if(s1_waypoints[i].x > max_x) max_x = s1_waypoints[i].x;
        if(s1_waypoints[i].y < min_y) min_y = s1_waypoints[i].y;
        if(s1_waypoints[i].y > max_y) max_y = s1_waypoints[i].y;
    }

    // 扩展边界留 10% 边距
    range_x = max_x - min_x;
    range_y = max_y - min_y;
    if(range_x < 0.1f) range_x = 1.0f;
    if(range_y < 0.1f) range_y = 1.0f;
    min_x -= range_x * 0.1f;
    max_x += range_x * 0.1f;
    min_y -= range_y * 0.1f;
    max_y += range_y * 0.1f;
    range_x = max_x - min_x;
    range_y = max_y - min_y;

    // 等比缩放
    {
        float scale_x = (float)MAP_W / range_x;
        float scale_y = (float)MAP_H / range_y;
        scale = (scale_x < scale_y) ? scale_x : scale_y;
    }

    // 居中偏移
    offset_x = MAP_X0 + (MAP_W - range_x * scale) / 2.0f;
    offset_y = MAP_Y0 + (MAP_H - range_y * scale) / 2.0f;

    // 坐标转换宏内联辅助变量
    #define SCR_X(wx) ((uint16)(offset_x + ((wx) - min_x) * scale))
    #define SCR_Y(wy) ((uint16)(offset_y + MAP_H - ((wy) - min_y) * scale))  // Y 轴翻转

    // ─── 构建航迹序列 ───
    // 去程顺序：起点(S) → 桩1(P) → 桩2(P) → … → 桩N(P) → 调头点(T)
    // 回程顺序：调头点(T) → 桩N(P) → … → 桩2(P) → 桩1(P) → 起点(S)
    // 先找出各类点索引
    int16 start_idx = -1;
    int16 turn_idx  = -1;
    int16 cone_indices[S1_MAX_WAYPOINTS];
    uint8 cone_count = 0;

    for(i = 0; i < s1_waypoint_count; i++)
    {
        switch(s1_waypoints[i].type)
        {
            case POINT_TYPE_START: start_idx = i; break;
            case POINT_TYPE_TURN:  turn_idx  = i; break;
            case POINT_TYPE_CONE:  cone_indices[cone_count++] = i; break;
        }
    }

    // ─── 绘制去程路线 ───
    if(start_idx >= 0)
    {
        uint16 prev_sx = SCR_X(s1_waypoints[start_idx].x);
        uint16 prev_sy = SCR_Y(s1_waypoints[start_idx].y);

        // 起点 → 桩1 → 桩2 → … → 桩N
        for(i = 0; i < cone_count; i++)
        {
            uint16 sx = SCR_X(s1_waypoints[cone_indices[i]].x);
            uint16 sy = SCR_Y(s1_waypoints[cone_indices[i]].y);
            ips200_draw_line(prev_sx, prev_sy, sx, sy, COLOR_ROUTE_FWD);
            prev_sx = sx;
            prev_sy = sy;
        }
        // 桩N → 调头点
        if(turn_idx >= 0)
        {
            uint16 tx = SCR_X(s1_waypoints[turn_idx].x);
            uint16 ty = SCR_Y(s1_waypoints[turn_idx].y);
            ips200_draw_line(prev_sx, prev_sy, tx, ty, COLOR_ROUTE_FWD);
        }
    }

    // ─── 绘制回程路线（偏移表示不同路径） ───
    if(turn_idx >= 0 && start_idx >= 0)
    {
        uint16 prev_sx = SCR_X(s1_waypoints[turn_idx].x);
        uint16 prev_sy = SCR_Y(s1_waypoints[turn_idx].y);

        // 调头点 → 桩N → 桩(N-1) → … → 桩1
        for(j = (int16)cone_count - 1; j >= 0; j--)
        {
            // 回程路线偏移 3 像素，模拟不同路径
            uint16 sx = SCR_X(s1_waypoints[cone_indices[j]].x) + 3;
            uint16 sy = SCR_Y(s1_waypoints[cone_indices[j]].y) + 3;
            ips200_draw_line(prev_sx, prev_sy, sx, sy, COLOR_ROUTE_RET);
            prev_sx = sx;
            prev_sy = sy;
        }
        // 桩1 → 起点
        {
            uint16 sx = SCR_X(s1_waypoints[start_idx].x) + 3;
            uint16 sy = SCR_Y(s1_waypoints[start_idx].y) + 3;
            ips200_draw_line(prev_sx, prev_sy, sx, sy, COLOR_ROUTE_RET);
        }
    }

    // ─── 绘制各点位标记（覆盖在路线之上） ───
    for(i = 0; i < s1_waypoint_count; i++)
    {
        uint16 sx = SCR_X(s1_waypoints[i].x);
        uint16 sy = SCR_Y(s1_waypoints[i].y);
        uint16 color = point_type_color(s1_waypoints[i].type);

        switch(s1_waypoints[i].type)
        {
            case POINT_TYPE_START:
                draw_dot(sx, sy, 4, color);              // 大实心点表示起点
                ips200_set_color(color, COLOR_BG);
                ips200_show_char(sx + 6, sy - 4, 'S');
                break;
            case POINT_TYPE_TURN:
                draw_dot(sx, sy, 4, color);              // 大实心点表示调头点
                ips200_set_color(color, COLOR_BG);
                ips200_show_char(sx + 6, sy - 4, 'T');
                break;
            case POINT_TYPE_CONE:
                draw_circle(sx, sy, 5, color);           // 空心圆表示桩
                draw_dot(sx, sy, 1, color);              // 中心小点
                break;
        }
    }

    // ─── 图例 ───
    ips200_set_color(COLOR_ROUTE_FWD, COLOR_BG);
    ips200_show_string(8,   284, "-- Go");
    ips200_set_color(COLOR_ROUTE_RET, COLOR_BG);
    ips200_show_string(80,  284, "-- Ret");

    // ─── 底部提示 ───
    ips200_set_color(COLOR_HIGHLIGHT, COLOR_BG);
    ips200_show_string(8, 302, "[LEFT]Go! [SW]Back");

    #undef SCR_X
    #undef SCR_Y
}

// =========================================================================
//  RUNNING 页面：行进状态实时显示
// =========================================================================
static void draw_running(void)
{
    ips200_full(COLOR_BG);

    ips200_set_color(COLOR_HIGHLIGHT, COLOR_BG);
    ips200_show_string(30, 10, "== RUNNING ==");

    ips200_set_color(COLOR_TEXT, COLOR_BG);
    ips200_show_string(8,  40,  "Speed:");
    ips200_show_float(70,  40,  target_speed, 5, 1);

    ips200_show_string(8,  60,  "X:");
    ips200_show_float(28,  60,  inav_x, 4, 2);
    ips200_show_string(120, 60, "Y:");
    ips200_show_float(140, 60, inav_y, 4, 2);

    ips200_show_string(8,  80,  "Yaw:");
    ips200_show_float(48,  80,  quat_yaw_deg, 4, 1);

    ips200_show_string(8,  100, "L_Duty:");
    ips200_show_int(68, 100, left_motor_duty, 5);
    ips200_show_string(8,  116, "R_Duty:");
    ips200_show_int(68, 116, right_motor_duty, 5);

    ips200_show_string(8,  140, "Dist:");
    ips200_show_float(56,  140, car_distance, 5, 2);

    // 底部提示
    ips200_set_color(COLOR_HIGHLIGHT, COLOR_BG);
    ips200_show_string(8, 302, "[SW] Emergency Stop");
}

// =========================================================================
//  DONE 页面
// =========================================================================
static void draw_done(void)
{
    ips200_full(COLOR_BG);

    ips200_set_color(COLOR_START_PT, COLOR_BG);
    ips200_show_string(50, 100, "== FINISHED ==");
    ips200_show_string(30, 140, "Course Complete!");

    ips200_set_color(COLOR_TEXT, COLOR_BG);
    ips200_show_string(8,  180, "Dist:");
    ips200_show_float(56,  180, car_distance, 5, 2);

    ips200_set_color(COLOR_HIGHLIGHT, COLOR_BG);
    ips200_show_string(20, 296, "[SW] Back to Home");
}

// =========================================================================
//  状态切换辅助
// =========================================================================
static void switch_state(ui_state_enum new_state)
{
    s1_ui_state    = new_state;
    s1_screen_dirty = 1;
}

static void reset_collect_data(void)
{
    s1_waypoint_count = 0;
    s1_has_start      = 0;
    s1_has_turn       = 0;
    s1_cur_point_type = POINT_TYPE_START;
    memset(s1_waypoints, 0, sizeof(s1_waypoints));

    // 惯导清零
    inav_x      = 0.0f;
    inav_y      = 0.0f;
    inav_active = 0;
}

// =========================================================================
//  各状态按键处理
// =========================================================================

// ─── HOME ─────────────────────────────────────────────────
static void poll_home(void)
{
    // UP 进入科目一采集
    if(btn_rising(UP, &btn_up_cnt))
    {
        reset_collect_data();
        switch_state(UI_STATE_COLLECT);
    }
}

// ─── COLLECT ──────────────────────────────────────────────
static void poll_collect(void)
{
    // DOWN：切换点位类型
    if(btn_rising(DOWN, &btn_down_cnt))
    {
        s1_cur_point_type = (point_type_enum)((s1_cur_point_type + 1) % 3);
        s1_screen_dirty = 1;
    }

    // UP：记录当前点
    if(btn_rising(UP, &btn_up_cnt))
    {
        if(s1_waypoint_count >= S1_MAX_WAYPOINTS)
        {
            // 已满，不操作
        }
        else if(s1_cur_point_type == POINT_TYPE_START && s1_has_start)
        {
            // 起点已记录，不重复
        }
        else if(s1_cur_point_type == POINT_TYPE_TURN && s1_has_turn)
        {
            // 调头点已记录，不重复
        }
        else
        {
            // 第一个点时锁定惯导起点
            if(s1_waypoint_count == 0)
            {
                inav_heading_ref = quat_yaw_deg;
                inav_x           = 0.0f;
                inav_y           = 0.0f;
                inav_active      = 1;
            }

            waypoint_t *wp = &s1_waypoints[s1_waypoint_count];
            wp->x    = inav_x;
            wp->y    = inav_y;
            wp->type = s1_cur_point_type;

            if(s1_cur_point_type == POINT_TYPE_START) s1_has_start = 1;
            if(s1_cur_point_type == POINT_TYPE_TURN)  s1_has_turn  = 1;

            s1_waypoint_count++;
            led(toggle);
            s1_screen_dirty = 1;
        }
    }

    // SE：删除上一个点
    if(btn_rising(SE, &btn_se_cnt))
    {
        if(s1_waypoint_count > 0)
        {
            s1_waypoint_count--;
            waypoint_t *removed = &s1_waypoints[s1_waypoint_count];
            if(removed->type == POINT_TYPE_START) s1_has_start = 0;
            if(removed->type == POINT_TYPE_TURN)  s1_has_turn  = 0;
            s1_screen_dirty = 1;
        }
    }

    // NE：完成采集 → 预览（需至少有 起点+调头点）
    if(btn_rising(NE, &btn_ne_cnt))
    {
        if(s1_has_start && s1_has_turn && s1_waypoint_count >= 2)
        {
            inav_active = 0;   // 暂停惯导积分
            switch_state(UI_STATE_PREVIEW);
        }
        // 否则不切换（条件不满足）
    }

    // SW 长按返回首页（20 ticks = 1s）
    if(btn_long_press(SW, &btn_sw_cnt, 20))
    {
        reset_collect_data();
        switch_state(UI_STATE_HOME);
    }
}

// ─── PREVIEW ──────────────────────────────────────────────
static void poll_preview(void)
{
    // LEFT：发车
    if(btn_rising(LEFT, &btn_left_cnt))
    {
        // 重置惯导为起点
        inav_x      = 0.0f;
        inav_y      = 0.0f;
        inav_active = 1;
        inav_heading_ref = quat_yaw_deg;

        // 启动车辆
        run_state    = 1;
        target_speed = 0.0f;

        switch_state(UI_STATE_RUNNING);
    }

    // SW 长按返回首页
    if(btn_long_press(SW, &btn_sw_cnt, 20))
    {
        reset_collect_data();
        switch_state(UI_STATE_HOME);
    }
}

// ─── RUNNING ──────────────────────────────────────────────
static void poll_running(void)
{
    // 行进中只做紧急停车和定期刷新
    // SW 长按紧急停车
    if(btn_long_press(SW, &btn_sw_cnt, 10))  // 0.5s 紧急停
    {
        run_state    = 0;
        target_speed = 0.0f;
        turn_diff_ext = 0;
        inav_active  = 0;
        switch_state(UI_STATE_DONE);
    }

    // 定期刷新显示（每 500ms = 10 次 poll）
    run_display_cnt++;
    if(run_display_cnt >= 10)
    {
        run_display_cnt = 0;
        s1_screen_dirty = 1;
    }
}

// ─── DONE ─────────────────────────────────────────────────
static void poll_done(void)
{
    // SW 返回首页
    if(btn_rising(SW, &btn_sw_cnt))
    {
        reset_collect_data();
        switch_state(UI_STATE_HOME);
    }
}

// =========================================================================
//  公开接口
// =========================================================================

void subject1_ui_init(void)
{
    // 初始化屏幕
    ips200_set_dir(IPS200_PORTAIT);
    ips200_set_color(COLOR_TEXT, COLOR_BG);
    ips200_init(IPS200_TYPE);
    ips200_set_font(IPS200_8X16_FONT);

    // 重置状态
    s1_ui_state     = UI_STATE_HOME;
    s1_screen_dirty = 1;
    reset_collect_data();

    // 画首页
    draw_home();
}

void subject1_ui_poll(void)
{
    // ─── 处理按键 ────────────────
    switch(s1_ui_state)
    {
        case UI_STATE_HOME:    poll_home();    break;
        case UI_STATE_COLLECT: poll_collect(); break;
        case UI_STATE_PREVIEW: poll_preview(); break;
        case UI_STATE_RUNNING: poll_running(); break;
        case UI_STATE_DONE:    poll_done();    break;
    }

    // ─── 按需重绘屏幕 ───────────
    if(s1_screen_dirty)
    {
        s1_screen_dirty = 0;

        switch(s1_ui_state)
        {
            case UI_STATE_HOME:    draw_home();    break;
            case UI_STATE_COLLECT: draw_collect(); break;
            case UI_STATE_PREVIEW: draw_preview(); break;
            case UI_STATE_RUNNING: draw_running(); break;
            case UI_STATE_DONE:    draw_done();    break;
        }
    }
}
