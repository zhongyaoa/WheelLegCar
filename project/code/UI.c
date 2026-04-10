/*
    科目一 UI 绘制模块 —— 只负责界面显示
*/

#include "UI.h"
#include "posture_control.h"
#include "zf_common_headfile.h"

#define COLOR_BG           RGB565_BLACK
#define COLOR_TITLE        RGB565_WHITE
#define COLOR_TEXT         RGB565_WHITE
#define COLOR_HIGHLIGHT    RGB565_YELLOW
#define COLOR_START_PT     RGB565_GREEN
#define COLOR_TURN_PT      RGB565_RED
#define COLOR_CONE_PT      RGB565_ORANGE
#define COLOR_ROUTE_FWD    RGB565_66CCFF
#define COLOR_ROUTE_A      RGB565_GREEN
#define COLOR_ROUTE_B      RGB565_PURPLE
#define COLOR_ROUTE_DIM    0x4208
#define COLOR_SELECTED     RGB565_YELLOW

#ifndef RGB565_ORANGE
#define RGB565_ORANGE      0xFD20
#endif
#ifndef RGB565_PURPLE
#define RGB565_PURPLE      0xA01F
#endif

#define IPS200_TYPE        (IPS200_TYPE_SPI)
#define MAP_X0             10
#define MAP_Y0             24
#define MAP_W              220
#define MAP_H              230

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
        if(d < 0) d += 2 * x + 3;
        else { d += 2 * (x - y) + 5; y--; }
        x++;
    }
}

static void draw_home(void)
{
    static const char* subj_names[3] = {"1 Slalom Course", "2 Mine Clearance", "3 Rough Terrain"};
    const char* subj_state_tag;
    uint8 i;

    ips200_full(COLOR_BG);
    ips200_set_color(COLOR_TITLE, COLOR_BG);
    ips200_show_string(40, 8, "WHEEL-LEG  CAR");
    ips200_show_string(40, 26, "--------------");

    for(i = 0; i < 3; i++)
    {
        uint16 row_y = 56 + i * 60;
        uint8  is_sel = (i == selected_subject);

        if(i == 0)
        {
            subj_state_tag = (s1_task_data.waypoint_count > 0) ? "[READY]" : "[NO DATA]";
        }
        else
        {
            subj_state_tag = "[SOON]";
        }

        ips200_set_color(is_sel ? COLOR_SELECTED : COLOR_TEXT, COLOR_BG);
        ips200_show_string(10, row_y, is_sel ? ">" : " ");
        ips200_show_string(22, row_y, subj_names[i]);
        ips200_set_color(is_sel ? COLOR_HIGHLIGHT : 0x8410, COLOR_BG);
        ips200_show_string(22, row_y + 18, subj_state_tag);
    }

    ips200_draw_line(8, 238, 232, 238, 0x4208);
    ips200_set_color(COLOR_HIGHLIGHT, COLOR_BG);
    ips200_show_string(8, 246, "UP/DN:Select  LEFT:Enter");
    ips200_show_string(8, 264, "SE(1s):Clear & Re-Collect");
}

static void draw_collect(void)
{
    uint8 i;

    ips200_full(COLOR_BG);
    ips200_set_color(COLOR_HIGHLIGHT, COLOR_BG);
    ips200_show_string(20, 4, "== Collect Points ==");

    ips200_set_color(point_type_color(s1_task_data.current_point_type), COLOR_BG);
    ips200_show_string(8, 28, "Type:");
    ips200_show_string(56, 28, point_type_name(s1_task_data.current_point_type));

    ips200_set_color(COLOR_TEXT, COLOR_BG);
    ips200_show_string(140, 28, "S:");
    ips200_show_string(164, 28, s1_task_data.has_start ? "Y" : "N");
    ips200_show_string(180, 28, "T:");
    ips200_show_string(204, 28, s1_task_data.has_turn ? "Y" : "N");

    ips200_show_string(8, 48, "X:");
    ips200_show_float(28, 48, inav_x, 3, 2);
    ips200_show_string(120, 48, "Y:");
    ips200_show_float(140, 48, inav_y, 3, 2);

    ips200_set_color(COLOR_TITLE, COLOR_BG);
    ips200_show_string(8, 72, "# Type   X      Y");
    ips200_draw_line(8, 88, 232, 88, 0x4208);

    for(i = 0; (i < s1_task_data.waypoint_count) && (i < 10); i++)
    {
        uint16 row_y = 92 + i * 16;
        ips200_set_color(point_type_color(s1_task_data.points[i].type), COLOR_BG);
        ips200_show_uint(8, row_y, i, 2);
        ips200_show_char(36, row_y, point_type_char(s1_task_data.points[i].type));
        ips200_show_float(60, row_y, s1_task_data.points[i].x, 3, 2);
        ips200_show_float(150, row_y, s1_task_data.points[i].y, 3, 2);
    }

    ips200_set_color(COLOR_TEXT, COLOR_BG);
    ips200_show_string(8, 260, "Total:");
    ips200_show_uint(60, 260, s1_task_data.waypoint_count, 2);

    ips200_set_color(COLOR_HIGHLIGHT, COLOR_BG);
    ips200_show_string(0, 280, "UP:Add DOWN:Type");
    ips200_show_string(0, 296, "LEFT:Done SE:Del SW:Bk");
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

    if((s1_preview_data.start_idx < 0) || (s1_preview_data.turn_idx < 0) || (s1_task_data.waypoint_count < 2))
    {
        ips200_set_color(COLOR_TEXT, COLOR_BG);
        ips200_show_string(40, 150, "Not enough pts!");
        return;
    }

    min_x = max_x = s1_task_data.points[0].x;
    min_y = max_y = s1_task_data.points[0].y;

    for(i = 0; i < s1_task_data.waypoint_count; i++)
    {
        if(s1_task_data.points[i].x < min_x) min_x = s1_task_data.points[i].x;
        if(s1_task_data.points[i].x > max_x) max_x = s1_task_data.points[i].x;
        if(s1_task_data.points[i].y < min_y) min_y = s1_task_data.points[i].y;
        if(s1_task_data.points[i].y > max_y) max_y = s1_task_data.points[i].y;
    }
    for(i = 0; i < s1_preview_data.routeA_count; i++)
    {
        if(s1_preview_data.routeA_x[i] < min_x) min_x = s1_preview_data.routeA_x[i];
        if(s1_preview_data.routeA_x[i] > max_x) max_x = s1_preview_data.routeA_x[i];
        if(s1_preview_data.routeA_y[i] < min_y) min_y = s1_preview_data.routeA_y[i];
        if(s1_preview_data.routeA_y[i] > max_y) max_y = s1_preview_data.routeA_y[i];
    }
    for(i = 0; i < s1_preview_data.routeB_count; i++)
    {
        if(s1_preview_data.routeB_x[i] < min_x) min_x = s1_preview_data.routeB_x[i];
        if(s1_preview_data.routeB_x[i] > max_x) max_x = s1_preview_data.routeB_x[i];
        if(s1_preview_data.routeB_y[i] < min_y) min_y = s1_preview_data.routeB_y[i];
        if(s1_preview_data.routeB_y[i] > max_y) max_y = s1_preview_data.routeB_y[i];
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

    {
        uint16 sx = SCR_X(s1_task_data.points[s1_preview_data.start_idx].x);
        uint16 sy = SCR_Y(s1_task_data.points[s1_preview_data.start_idx].y);
        uint16 tx = SCR_X(s1_task_data.points[s1_preview_data.turn_idx].x);
        uint16 ty = SCR_Y(s1_task_data.points[s1_preview_data.turn_idx].y);
        ips200_draw_line(sx, sy, tx, ty, COLOR_ROUTE_FWD);
    }

    {
        uint16 colorA = (s1_task_data.route_sel == 0) ? COLOR_ROUTE_A : COLOR_ROUTE_DIM;
        for(i = 0; i + 1 < s1_preview_data.routeA_count; i++)
        {
            ips200_draw_line(
                SCR_X(s1_preview_data.routeA_x[i]), SCR_Y(s1_preview_data.routeA_y[i]),
                SCR_X(s1_preview_data.routeA_x[i + 1]), SCR_Y(s1_preview_data.routeA_y[i + 1]),
                colorA);
        }
    }

    {
        uint16 colorB = (s1_task_data.route_sel == 1) ? COLOR_ROUTE_B : COLOR_ROUTE_DIM;
        for(i = 0; i + 1 < s1_preview_data.routeB_count; i++)
        {
            ips200_draw_line(
                SCR_X(s1_preview_data.routeB_x[i]), SCR_Y(s1_preview_data.routeB_y[i]),
                SCR_X(s1_preview_data.routeB_x[i + 1]), SCR_Y(s1_preview_data.routeB_y[i + 1]),
                colorB);
        }
    }

    for(i = 0; i < s1_task_data.waypoint_count; i++)
    {
        uint16 sx = SCR_X(s1_task_data.points[i].x);
        uint16 sy = SCR_Y(s1_task_data.points[i].y);
        uint16 color = point_type_color(s1_task_data.points[i].type);

        switch(s1_task_data.points[i].type)
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

    ips200_set_color(COLOR_ROUTE_FWD, COLOR_BG);
    ips200_show_string(8, 260, "-- S->T");
    ips200_set_color((s1_task_data.route_sel == 0) ? COLOR_ROUTE_A : COLOR_ROUTE_DIM, COLOR_BG);
    ips200_show_string(8, 278, (s1_task_data.route_sel == 0) ? ">A 1st Left" : " A 1st Left");
    ips200_set_color((s1_task_data.route_sel == 1) ? COLOR_ROUTE_B : COLOR_ROUTE_DIM, COLOR_BG);
    ips200_show_string(8, 294, (s1_task_data.route_sel == 1) ? ">B 1st Right" : " B 1st Right");
    ips200_set_color(COLOR_HIGHLIGHT, COLOR_BG);
    ips200_show_string(130, 278, "UP/DN:Switch");
    ips200_show_string(130, 294, "L:Rdy SW:Bk");

    #undef SCR_X
    #undef SCR_Y
}

static void draw_standby(void)
{
    ips200_full(COLOR_BG);
    ips200_set_color(COLOR_HIGHLIGHT, COLOR_BG);
    ips200_show_string(30, 10, "== STANDBY ==");

    ips200_set_color(COLOR_TEXT, COLOR_BG);
    ips200_show_string(20, 60, "Balancing...");
    ips200_show_string(20, 90, "Place car on ground");
    ips200_show_string(20, 110, "then press [LEFT]");
    ips200_show_string(8, 150, "Roll:");
    ips200_show_float(56, 150, roll_balance_cascade.posture_value.rol, 4, 1);
    ips200_show_string(8, 170, "Pitch:");
    ips200_show_float(56, 170, roll_balance_cascade.posture_value.pit, 4, 1);
    ips200_show_string(8, 190, "State:");
    ips200_show_string(56, 190, run_state ? "BAL" : "WAIT");

    ips200_set_color(COLOR_HIGHLIGHT, COLOR_BG);
    ips200_show_string(8, 280, "[LEFT] Launch!");
    ips200_show_string(8, 296, "[SW] Cancel");
}

static void draw_running(void)
{
    ips200_full(COLOR_BG);
    ips200_set_color(COLOR_HIGHLIGHT, COLOR_BG);
    ips200_show_string(30, 10, "== RUNNING ==");
    ips200_set_color(COLOR_TEXT, COLOR_BG);
    ips200_show_string(8, 40, "Speed:");
    ips200_show_float(70, 40, target_speed, 5, 1);
    ips200_show_string(8, 60, "X:");
    ips200_show_float(28, 60, inav_x, 4, 2);
    ips200_show_string(120, 60, "Y:");
    ips200_show_float(140, 60, inav_y, 4, 2);
    ips200_show_string(8, 80, "Yaw:");
    ips200_show_float(48, 80, quat_yaw_deg, 4, 1);
    ips200_show_string(8, 100, "L_Duty:");
    ips200_show_int(68, 100, left_motor_duty, 5);
    ips200_show_string(8, 116, "R_Duty:");
    ips200_show_int(68, 116, right_motor_duty, 5);
    ips200_show_string(8, 140, "Dist:");
    ips200_show_float(56, 140, car_distance, 5, 2);
    ips200_set_color(COLOR_HIGHLIGHT, COLOR_BG);
    ips200_show_string(8, 302, "[SW] Emergency Stop");
}

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
    ips200_show_string(20, 260, "[UP]  Re-run (keep data)");
    ips200_show_string(20, 280, "[SW]  Back to Home");
}

void subject1_ui_init(void)
{
    ips200_set_dir(IPS200_PORTAIT);
    ips200_set_color(COLOR_TEXT, COLOR_BG);
    ips200_init(IPS200_TYPE);
    ips200_set_font(IPS200_8X16_FONT);
}

void subject1_ui_redraw(void)
{
    switch(s1_ui_state)
    {
        case UI_STATE_HOME:    draw_home();    break;
        case UI_STATE_COLLECT: draw_collect(); break;
        case UI_STATE_PREVIEW: draw_preview(); break;
        case UI_STATE_STANDBY: draw_standby(); break;
        case UI_STATE_RUNNING: draw_running(); break;
        case UI_STATE_DONE:    draw_done();    break;
    }
}
