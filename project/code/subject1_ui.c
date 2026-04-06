/*
    科目一 UI 模块实现 —— 绕桩前进
    ─────────────────────────────────────────────────────────
    轨迹规则：
      去程: 起点 S  ─── 直线 ───  调头点 T
      回程: 调头点 T ─── 蛇形绕桩 ─── 起点 S
            桩必须左右交替（第1桩在左则第2桩在右，反之亦然）
      因此恰好存在两条合法路线：
        路线A: 回程首桩在车左侧
        路线B: 回程首桩在车右侧
    ─────────────────────────────────────────────────────────
    屏幕: IPS200  240(W) x 320(H)  竖屏
    按键: UP / DOWN / LEFT / SE / SW  (NE 已损坏，不使用)
    ─────────────────────────────────────────────────────────
*/

#include "subject1_ui.h"
#include "controler.h"
#include "posture_control.h"
#include "ins_tracker.h"
#include "zf_common_headfile.h"
#include <math.h>
#include <string.h>

// ===================== 屏幕颜色定义 =====================
#define COLOR_BG           RGB565_BLACK
#define COLOR_TITLE        RGB565_WHITE
#define COLOR_TEXT          RGB565_WHITE
#define COLOR_HIGHLIGHT    RGB565_YELLOW
#define COLOR_START_PT     RGB565_GREEN
#define COLOR_TURN_PT      RGB565_RED
#define COLOR_CONE_PT      RGB565_ORANGE
#define COLOR_ROUTE_FWD    RGB565_66CCFF     // 去程（直线）
#define COLOR_ROUTE_A      RGB565_GREEN      // 路线A
#define COLOR_ROUTE_B      RGB565_PURPLE     // 路线B
#define COLOR_ROUTE_DIM    0x4208            // 未选中路线（暗灰）
#define COLOR_CAR          RGB565_GREEN
#define COLOR_SELECTED     RGB565_YELLOW

#ifndef RGB565_ORANGE
#define RGB565_ORANGE      0xFD20
#endif
#ifndef RGB565_PURPLE
#define RGB565_PURPLE      0xA01F
#endif

// ===================== IPS200 屏幕接口类型 =====================
#define IPS200_TYPE        (IPS200_TYPE_SPI)

// ===================== 绕桩偏移距离 (m) =====================
#define CONE_BYPASS_DIST   0.35f   // 绕桩时偏离桩位的横向距离

// ===================== 全局状态 =====================
ui_state_enum  s1_ui_state        = UI_STATE_HOME;
waypoint_t     s1_waypoints[S1_MAX_WAYPOINTS];
uint8          s1_waypoint_count  = 0;

// ===================== 内部状态 =====================
static point_type_enum s1_cur_point_type = POINT_TYPE_START;
static uint8  s1_has_start  = 0;
static uint8  s1_has_turn   = 0;
static uint8  s1_screen_dirty = 1;

// 路线选择: 0 = 路线A（首桩左绕）, 1 = 路线B（首桩右绕）
static uint8  s1_route_sel  = 0;

// 按键消抖计数
static uint32 btn_up_cnt   = 0;
static uint32 btn_down_cnt = 0;
static uint32 btn_left_cnt = 0;
static uint32 btn_se_cnt   = 0;
static uint32 btn_sw_cnt   = 0;

// 行进中刷新降频
static uint32 run_display_cnt = 0;

// ===================== 按键边沿检测 =====================
static uint8 btn_rising(Button bt, uint32 *cnt)
{
    if(button_press(bt))
    {
        (*cnt)++;
        if(*cnt == 1) return 1;
        return 0;
    }
    else
    {
        *cnt = 0;
        return 0;
    }
}

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

// ===================== 点类型辅助 =====================
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
static void draw_dot(uint16 cx, uint16 cy, uint16 radius, uint16 color)
{
    uint16 x, y;
    for(y = cy - radius; y <= cy + radius; y++)
        for(x = cx - radius; x <= cx + radius; x++)
            if(x < 240 && y < 320)
                ips200_draw_point(x, y, color);
}

static void draw_circle(uint16 cx, uint16 cy, uint16 r, uint16 color)
{
    int16 x = 0, y = (int16)r;
    int16 d = 1 - (int16)r;
    while(x <= y)
    {
        ips200_draw_point((uint16)(cx+x),(uint16)(cy+y),color);
        ips200_draw_point((uint16)(cx-x),(uint16)(cy+y),color);
        ips200_draw_point((uint16)(cx+x),(uint16)(cy-y),color);
        ips200_draw_point((uint16)(cx-x),(uint16)(cy-y),color);
        ips200_draw_point((uint16)(cx+y),(uint16)(cy+x),color);
        ips200_draw_point((uint16)(cx-y),(uint16)(cy+x),color);
        ips200_draw_point((uint16)(cx+y),(uint16)(cy-x),color);
        ips200_draw_point((uint16)(cx-y),(uint16)(cy-x),color);
        if(d < 0) d += 2*x + 3;
        else { d += 2*(x-y) + 5; y--; }
        x++;
    }
}

// =========================================================================
//  绕桩航点生成
//  输入: 桩位坐标数组 cone_x/y[0..cone_count-1]（回程顺序，即离调头点最近的在前）
//        from_x/y = 出发点（调头点），to_x/y = 终点（起点）
//        first_side: +1 = 首桩从左边绕, -1 = 首桩从右边绕
//  输出: out_x/y[] 填充绕桩航点，返回航点数
//
//  算法: 对每个桩，计算从上一个航点到下一个航点的行进方向，
//        取其法线方向偏移桩位坐标，交替左右。
// =========================================================================
static uint8 generate_bypass_route(
    const float *cone_x, const float *cone_y, uint8 cone_count,
    float from_x, float from_y,
    float to_x,   float to_y,
    int8  first_side,
    float *out_x,  float *out_y)
{
    uint8 out_count = 0;
    uint8 i;
    int8  side = first_side;  // +1=左偏, -1=右偏

    // [0] = 出发点（调头点）
    out_x[out_count] = from_x;
    out_y[out_count] = from_y;
    out_count++;

    for(i = 0; i < cone_count; i++)
    {
        // 确定行进方向: 从前一个点到后一个点
        float prev_x, prev_y, next_x, next_y;
        prev_x = (i == 0) ? from_x : cone_x[i - 1];
        prev_y = (i == 0) ? from_y : cone_y[i - 1];
        next_x = (i == cone_count - 1) ? to_x : cone_x[i + 1];
        next_y = (i == cone_count - 1) ? to_y : cone_y[i + 1];

        float dir_x = next_x - prev_x;
        float dir_y = next_y - prev_y;
        float len = sqrtf(dir_x * dir_x + dir_y * dir_y);
        if(len < 0.001f) len = 0.001f;
        // 单位方向
        dir_x /= len;
        dir_y /= len;

        // 法线: 左手法线 (-dir_y, dir_x) 为行进方向左侧
        float nx = -dir_y;
        float ny =  dir_x;

        // 偏移: side>0 往左偏, side<0 往右偏
        out_x[out_count] = cone_x[i] + side * CONE_BYPASS_DIST * nx;
        out_y[out_count] = cone_y[i] + side * CONE_BYPASS_DIST * ny;
        out_count++;

        side = -side;  // 交替
    }

    // 最后一个点: 终点（起点）
    out_x[out_count] = to_x;
    out_y[out_count] = to_y;
    out_count++;

    return out_count;
}

// =========================================================================
//  HOME 页面
// =========================================================================
static void draw_home(void)
{
    ips200_full(COLOR_BG);
    ips200_set_color(COLOR_TITLE, COLOR_BG);
    ips200_show_string(40, 30, "WHEEL-LEG  CAR");
    ips200_show_string(40, 55, "--------------");

    ips200_set_color(COLOR_SELECTED, COLOR_BG);
    ips200_show_string(30, 100, "> Subject 1");
    ips200_set_color(COLOR_TEXT, COLOR_BG);
    ips200_show_string(40, 118, "Slalom Course");

    ips200_set_color(0x8410, COLOR_BG);
    ips200_show_string(30, 160, "  Subject 2");
    ips200_show_string(40, 178, "(Coming Soon)");
    ips200_show_string(30, 210, "  Subject 3");
    ips200_show_string(40, 228, "(Coming Soon)");

    ips200_set_color(COLOR_HIGHLIGHT, COLOR_BG);
    ips200_show_string(20, 296, "[UP] Enter Subject 1");
}

// =========================================================================
//  COLLECT 页面
// =========================================================================
static void draw_collect(void)
{
    uint8 i;
    ips200_full(COLOR_BG);

    ips200_set_color(COLOR_HIGHLIGHT, COLOR_BG);
    ips200_show_string(20, 4, "== Collect Points ==");

    ips200_set_color(point_type_color(s1_cur_point_type), COLOR_BG);
    ips200_show_string(8, 28, "Type:");
    ips200_show_string(56, 28, point_type_name(s1_cur_point_type));

    ips200_set_color(COLOR_TEXT, COLOR_BG);
    ips200_show_string(140, 28, "S:");
    ips200_show_string(164, 28, s1_has_start ? "Y" : "N");
    ips200_show_string(180, 28, "T:");
    ips200_show_string(204, 28, s1_has_turn  ? "Y" : "N");

    ips200_set_color(COLOR_TEXT, COLOR_BG);
    ips200_show_string(8,  48, "X:");
    ips200_show_float(28,  48, inav_x, 3, 2);
    ips200_show_string(120, 48, "Y:");
    ips200_show_float(140, 48, inav_y, 3, 2);

    ips200_set_color(COLOR_TITLE, COLOR_BG);
    ips200_show_string(8, 72, "# Type   X      Y");
    ips200_draw_line(8, 88, 232, 88, 0x4208);

    for(i = 0; i < s1_waypoint_count && i < 10; i++)
    {
        uint16 row_y = 92 + i * 16;
        ips200_set_color(point_type_color(s1_waypoints[i].type), COLOR_BG);
        ips200_show_uint(8,  row_y, i, 2);
        ips200_show_char(36, row_y, point_type_char(s1_waypoints[i].type));
        ips200_show_float(60,  row_y, s1_waypoints[i].x, 3, 2);
        ips200_show_float(150, row_y, s1_waypoints[i].y, 3, 2);
    }

    ips200_set_color(COLOR_TEXT, COLOR_BG);
    ips200_show_string(8, 260, "Total:");
    ips200_show_uint(60, 260, s1_waypoint_count, 2);

    ips200_set_color(COLOR_HIGHLIGHT, COLOR_BG);
    ips200_show_string(0, 280, "UP:Add DOWN:Type");
    ips200_show_string(0, 296, "LEFT:Done SE:Del SW:Bk");
}

// =========================================================================
//  PREVIEW 页面 —— 同时画出两条路线，用户选择
// =========================================================================
#define MAP_X0     10
#define MAP_Y0     24
#define MAP_W      220
#define MAP_H      230

// 内部临时存储: 提取出各类点索引供 preview/launch 共用
static int16  s1_start_idx;
static int16  s1_turn_idx;
static int16  s1_cone_indices[S1_MAX_WAYPOINTS];
static uint8  s1_cone_count;

// 回程桩序列（从调头点往回走的顺序 = 采集顺序倒序）
static float  s1_ret_cone_x[S1_MAX_WAYPOINTS];
static float  s1_ret_cone_y[S1_MAX_WAYPOINTS];

// 两条路线的绕桩航点
static float  s1_routeA_x[S1_MAX_WAYPOINTS * 2 + 4];
static float  s1_routeA_y[S1_MAX_WAYPOINTS * 2 + 4];
static uint8  s1_routeA_count;
static float  s1_routeB_x[S1_MAX_WAYPOINTS * 2 + 4];
static float  s1_routeB_y[S1_MAX_WAYPOINTS * 2 + 4];
static uint8  s1_routeB_count;

static void prepare_routes(void)
{
    uint8 i;
    int16 j;

    // 提取各类点
    s1_start_idx = -1;
    s1_turn_idx  = -1;
    s1_cone_count = 0;
    for(i = 0; i < s1_waypoint_count; i++)
    {
        switch(s1_waypoints[i].type)
        {
            case POINT_TYPE_START: s1_start_idx = i; break;
            case POINT_TYPE_TURN:  s1_turn_idx  = i; break;
            case POINT_TYPE_CONE:  s1_cone_indices[s1_cone_count++] = i; break;
        }
    }

    if(s1_start_idx < 0 || s1_turn_idx < 0) return;

    float start_x = s1_waypoints[s1_start_idx].x;
    float start_y = s1_waypoints[s1_start_idx].y;
    float turn_x  = s1_waypoints[s1_turn_idx].x;
    float turn_y  = s1_waypoints[s1_turn_idx].y;

    // 回程桩序列: 采集顺序即回程经过顺序（用户按回程先后打点）
    for(i = 0; i < s1_cone_count; i++)
    {
        s1_ret_cone_x[i] = s1_waypoints[s1_cone_indices[i]].x;
        s1_ret_cone_y[i] = s1_waypoints[s1_cone_indices[i]].y;
    }

    // 路线A: 首桩左绕 (+1)
    s1_routeA_count = generate_bypass_route(
        s1_ret_cone_x, s1_ret_cone_y, s1_cone_count,
        turn_x, turn_y, start_x, start_y,
        +1, s1_routeA_x, s1_routeA_y);

    // 路线B: 首桩右绕 (-1)
    s1_routeB_count = generate_bypass_route(
        s1_ret_cone_x, s1_ret_cone_y, s1_cone_count,
        turn_x, turn_y, start_x, start_y,
        -1, s1_routeB_x, s1_routeB_y);
}

static void draw_preview(void)
{
    uint8 i;
    float min_x, max_x, min_y, max_y;
    float range_x, range_y, scale;
    float offset_x, offset_y;

    ips200_full(COLOR_BG);

    ips200_set_color(COLOR_HIGHLIGHT, COLOR_BG);
    ips200_show_string(20, 2, "== Select Route ==");

    if(s1_start_idx < 0 || s1_turn_idx < 0 || s1_waypoint_count < 2)
    {
        ips200_set_color(COLOR_TEXT, COLOR_BG);
        ips200_show_string(40, 150, "Not enough pts!");
        return;
    }

    // ─── 计算坐标范围（包含所有原始点 + 两条路线的偏移点）───
    min_x = max_x = s1_waypoints[0].x;
    min_y = max_y = s1_waypoints[0].y;
    for(i = 0; i < s1_waypoint_count; i++)
    {
        if(s1_waypoints[i].x < min_x) min_x = s1_waypoints[i].x;
        if(s1_waypoints[i].x > max_x) max_x = s1_waypoints[i].x;
        if(s1_waypoints[i].y < min_y) min_y = s1_waypoints[i].y;
        if(s1_waypoints[i].y > max_y) max_y = s1_waypoints[i].y;
    }
    for(i = 0; i < s1_routeA_count; i++)
    {
        if(s1_routeA_x[i] < min_x) min_x = s1_routeA_x[i];
        if(s1_routeA_x[i] > max_x) max_x = s1_routeA_x[i];
        if(s1_routeA_y[i] < min_y) min_y = s1_routeA_y[i];
        if(s1_routeA_y[i] > max_y) max_y = s1_routeA_y[i];
    }
    for(i = 0; i < s1_routeB_count; i++)
    {
        if(s1_routeB_x[i] < min_x) min_x = s1_routeB_x[i];
        if(s1_routeB_x[i] > max_x) max_x = s1_routeB_x[i];
        if(s1_routeB_y[i] < min_y) min_y = s1_routeB_y[i];
        if(s1_routeB_y[i] > max_y) max_y = s1_routeB_y[i];
    }

    range_x = max_x - min_x;
    range_y = max_y - min_y;
    if(range_x < 0.1f) range_x = 1.0f;
    if(range_y < 0.1f) range_y = 1.0f;
    min_x -= range_x * 0.15f;
    max_x += range_x * 0.15f;
    min_y -= range_y * 0.15f;
    max_y += range_y * 0.15f;
    range_x = max_x - min_x;
    range_y = max_y - min_y;

    {
        float scale_x = (float)MAP_W / range_x;
        float scale_y = (float)MAP_H / range_y;
        scale = (scale_x < scale_y) ? scale_x : scale_y;
    }
    offset_x = MAP_X0 + (MAP_W - range_x * scale) / 2.0f;
    offset_y = MAP_Y0 + (MAP_H - range_y * scale) / 2.0f;

    #define SCR_X(wx) ((uint16)(offset_x + ((wx) - min_x) * scale))
    #define SCR_Y(wy) ((uint16)(offset_y + MAP_H - ((wy) - min_y) * scale))

    // ─── 去程直线 S → T（始终画） ───
    {
        uint16 sx = SCR_X(s1_waypoints[s1_start_idx].x);
        uint16 sy = SCR_Y(s1_waypoints[s1_start_idx].y);
        uint16 tx = SCR_X(s1_waypoints[s1_turn_idx].x);
        uint16 ty = SCR_Y(s1_waypoints[s1_turn_idx].y);
        ips200_draw_line(sx, sy, tx, ty, COLOR_ROUTE_FWD);
    }

    // ─── 画路线A回程（未选中=暗灰, 选中=绿色亮） ───
    {
        uint16 colorA = (s1_route_sel == 0) ? COLOR_ROUTE_A : COLOR_ROUTE_DIM;
        for(i = 0; i + 1 < s1_routeA_count; i++)
        {
            ips200_draw_line(
                SCR_X(s1_routeA_x[i]),   SCR_Y(s1_routeA_y[i]),
                SCR_X(s1_routeA_x[i+1]), SCR_Y(s1_routeA_y[i+1]),
                colorA);
        }
    }

    // ─── 画路线B回程（未选中=暗灰, 选中=紫色亮） ───
    {
        uint16 colorB = (s1_route_sel == 1) ? COLOR_ROUTE_B : COLOR_ROUTE_DIM;
        for(i = 0; i + 1 < s1_routeB_count; i++)
        {
            ips200_draw_line(
                SCR_X(s1_routeB_x[i]),   SCR_Y(s1_routeB_y[i]),
                SCR_X(s1_routeB_x[i+1]), SCR_Y(s1_routeB_y[i+1]),
                colorB);
        }
    }

    // ─── 绘制各点位标记 ───
    for(i = 0; i < s1_waypoint_count; i++)
    {
        uint16 sx = SCR_X(s1_waypoints[i].x);
        uint16 sy = SCR_Y(s1_waypoints[i].y);
        uint16 color = point_type_color(s1_waypoints[i].type);

        switch(s1_waypoints[i].type)
        {
            case POINT_TYPE_START:
                draw_dot(sx, sy, 4, color);
                ips200_set_color(color, COLOR_BG);
                ips200_show_char(sx + 6, sy - 4, 'S');
                break;
            case POINT_TYPE_TURN:
                draw_dot(sx, sy, 4, color);
                ips200_set_color(color, COLOR_BG);
                ips200_show_char(sx + 6, sy - 4, 'T');
                break;
            case POINT_TYPE_CONE:
                draw_circle(sx, sy, 5, color);
                draw_dot(sx, sy, 1, color);
                break;
        }
    }

    // ─── 图例 + 选中标记 ───
    ips200_set_color(COLOR_ROUTE_FWD, COLOR_BG);
    ips200_show_string(8, 260, "-- S->T");

    ips200_set_color((s1_route_sel == 0) ? COLOR_ROUTE_A : COLOR_ROUTE_DIM, COLOR_BG);
    ips200_show_string(8, 278, (s1_route_sel == 0) ? ">A 1st Left" : " A 1st Left");

    ips200_set_color((s1_route_sel == 1) ? COLOR_ROUTE_B : COLOR_ROUTE_DIM, COLOR_BG);
    ips200_show_string(8, 294, (s1_route_sel == 1) ? ">B 1st Right" : " B 1st Right");

    // ─── 底部按键提示 ───
    ips200_set_color(COLOR_HIGHLIGHT, COLOR_BG);
    ips200_show_string(130, 278, "UP/DN:Switch");
    ips200_show_string(130, 294, "LEFT:Go SW:Bk");

    #undef SCR_X
    #undef SCR_Y
}

// =========================================================================
//  RUNNING 页面
// =========================================================================
static void draw_running(void)
{
    ips200_full(COLOR_BG);
    ips200_set_color(COLOR_HIGHLIGHT, COLOR_BG);
    ips200_show_string(30, 10, "== RUNNING ==");

    ips200_set_color(COLOR_TEXT, COLOR_BG);
    ips200_show_string(8,  40, "Speed:");
    ips200_show_float(70,  40, target_speed, 5, 1);
    ips200_show_string(8,  60, "X:");
    ips200_show_float(28,  60, inav_x, 4, 2);
    ips200_show_string(120, 60, "Y:");
    ips200_show_float(140, 60, inav_y, 4, 2);
    ips200_show_string(8,  80, "Yaw:");
    ips200_show_float(48,  80, quat_yaw_deg, 4, 1);
    ips200_show_string(8, 100, "L_Duty:");
    ips200_show_int(68, 100, left_motor_duty, 5);
    ips200_show_string(8, 116, "R_Duty:");
    ips200_show_int(68, 116, right_motor_duty, 5);
    ips200_show_string(8, 140, "Dist:");
    ips200_show_float(56, 140, car_distance, 5, 2);

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
    ips200_show_string(8, 180, "Dist:");
    ips200_show_float(56, 180, car_distance, 5, 2);
    ips200_set_color(COLOR_HIGHLIGHT, COLOR_BG);
    ips200_show_string(20, 296, "[SW] Back to Home");
}

// =========================================================================
//  状态切换辅助
// =========================================================================
static void switch_state(ui_state_enum new_state)
{
    s1_ui_state     = new_state;
    s1_screen_dirty = 1;
}

static void reset_collect_data(void)
{
    s1_waypoint_count = 0;
    s1_has_start      = 0;
    s1_has_turn       = 0;
    s1_cur_point_type = POINT_TYPE_START;
    s1_route_sel      = 0;
    memset(s1_waypoints, 0, sizeof(s1_waypoints));
    inav_x      = 0.0f;
    inav_y      = 0.0f;
    inav_active = 0;
}

// =========================================================================
//  各状态按键处理
// =========================================================================

static void poll_home(void)
{
    if(btn_rising(UP, &btn_up_cnt))
    {
        reset_collect_data();
        switch_state(UI_STATE_COLLECT);
    }
}

static void poll_collect(void)
{
    if(btn_rising(DOWN, &btn_down_cnt))
    {
        s1_cur_point_type = (point_type_enum)((s1_cur_point_type + 1) % 3);
        s1_screen_dirty = 1;
    }

    if(btn_rising(UP, &btn_up_cnt))
    {
        if(s1_waypoint_count >= S1_MAX_WAYPOINTS) {}
        else if(s1_cur_point_type == POINT_TYPE_START && s1_has_start) {}
        else if(s1_cur_point_type == POINT_TYPE_TURN  && s1_has_turn)  {}
        else
        {
            if(s1_waypoint_count == 0)
            {
                inav_heading_ref = quat_yaw_deg;
                inav_x = 0.0f;
                inav_y = 0.0f;
                inav_active = 1;
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

    if(btn_rising(LEFT, &btn_left_cnt))
    {
        if(s1_has_start && s1_has_turn && s1_waypoint_count >= 2)
        {
            inav_active = 0;
            s1_route_sel = 0;
            prepare_routes();
            switch_state(UI_STATE_PREVIEW);
        }
    }

    if(btn_long_press(SW, &btn_sw_cnt, 20))
    {
        reset_collect_data();
        switch_state(UI_STATE_HOME);
    }
}

static void poll_preview(void)
{
    // UP / DOWN: 切换路线
    if(btn_rising(UP, &btn_up_cnt))
    {
        s1_route_sel = (s1_route_sel == 0) ? 1 : 0;
        s1_screen_dirty = 1;
    }
    if(btn_rising(DOWN, &btn_down_cnt))
    {
        s1_route_sel = (s1_route_sel == 0) ? 1 : 0;
        s1_screen_dirty = 1;
    }

    // LEFT: 发车（使用选中的路线）
    if(btn_rising(LEFT, &btn_left_cnt))
    {
        // 构建完整航点序列: 去程(直线) + 回程(绕桩)
        float  full_x[S1_MAX_WAYPOINTS * 2 + 8];
        float  full_y[S1_MAX_WAYPOINTS * 2 + 8];
        uint8  full_count = 0;

        float start_x = s1_waypoints[s1_start_idx].x;
        float start_y = s1_waypoints[s1_start_idx].y;
        float turn_x  = s1_waypoints[s1_turn_idx].x;
        float turn_y  = s1_waypoints[s1_turn_idx].y;

        // 去程: 起点 → 调头点（直线，只需两点）
        full_x[full_count] = start_x;
        full_y[full_count] = start_y;
        full_count++;
        full_x[full_count] = turn_x;
        full_y[full_count] = turn_y;
        full_count++;

        // 回程: 使用选中路线的绕桩航点（跳过第一个点=调头点，因为已在去程末尾）
        const float *sel_x = (s1_route_sel == 0) ? s1_routeA_x : s1_routeB_x;
        const float *sel_y = (s1_route_sel == 0) ? s1_routeA_y : s1_routeB_y;
        uint8 sel_count     = (s1_route_sel == 0) ? s1_routeA_count : s1_routeB_count;
        uint8 k;
        for(k = 1; k < sel_count; k++)  // 从1开始跳过调头点
        {
            full_x[full_count] = sel_x[k];
            full_y[full_count] = sel_y[k];
            full_count++;
        }

        // 重置惯导
        inav_x           = 0.0f;
        inav_y           = 0.0f;
        inav_active      = 1;
        inav_heading_ref = quat_yaw_deg;

        // 注入 ins_tracker 并启动
        run_state = 1;
        ins_tracker_start_with_points(full_x, full_y, full_count);

        switch_state(UI_STATE_RUNNING);
    }

    if(btn_long_press(SW, &btn_sw_cnt, 20))
    {
        reset_collect_data();
        switch_state(UI_STATE_HOME);
    }
}

static void poll_running(void)
{
    if(tracker_state == TRACKER_STATE_DONE)
    {
        run_state     = 0;
        target_speed  = 0.0f;
        turn_diff_ext = 0;
        inav_active   = 0;
        switch_state(UI_STATE_DONE);
        return;
    }

    if(btn_long_press(SW, &btn_sw_cnt, 10))
    {
        run_state     = 0;
        target_speed  = 0.0f;
        turn_diff_ext = 0;
        inav_active   = 0;
        tracker_state = TRACKER_STATE_DONE;
        switch_state(UI_STATE_DONE);
    }

    run_display_cnt++;
    if(run_display_cnt >= 10)
    {
        run_display_cnt = 0;
        s1_screen_dirty = 1;
    }
}

static void poll_done(void)
{
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
    ips200_set_dir(IPS200_PORTAIT);
    ips200_set_color(COLOR_TEXT, COLOR_BG);
    ips200_init(IPS200_TYPE);
    ips200_set_font(IPS200_8X16_FONT);

    s1_ui_state     = UI_STATE_HOME;
    s1_screen_dirty = 1;
    reset_collect_data();
    draw_home();
}

void subject1_ui_poll(void)
{
    switch(s1_ui_state)
    {
        case UI_STATE_HOME:    poll_home();    break;
        case UI_STATE_COLLECT: poll_collect(); break;
        case UI_STATE_PREVIEW: poll_preview(); break;
        case UI_STATE_RUNNING: poll_running(); break;
        case UI_STATE_DONE:    poll_done();    break;
    }

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
