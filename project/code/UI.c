/*
    公共比赛 UI 绘制模块 —— 当前承载各科目共用界面显示
*/

#include "UI.h"
#include "subject2.h"
#include "posture_control.h"
#include "zf_common_headfile.h"

#define COLOR_BG           RGB565_BLACK
#define COLOR_TITLE        RGB565_WHITE
#define COLOR_TEXT         RGB565_WHITE
#define COLOR_HIGHLIGHT    RGB565_YELLOW
#define COLOR_START_PT     RGB565_GREEN
#define COLOR_TURN_PT      RGB565_RED
#define COLOR_CONE_PT      RGB565_ORANGE
#define COLOR_MINE_PT      RGB565_ORANGE
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
    int16 x_start = (int16)cx - (int16)radius;
    int16 x_end   = (int16)cx + (int16)radius;
    int16 y_start = (int16)cy - (int16)radius;
    int16 y_end   = (int16)cy + (int16)radius;
    int16 x, y;

    if(x_start < 0) x_start = 0;
    if(y_start < 0) y_start = 0;
    if(x_end >= 240) x_end = 239;
    if(y_end >= 320) y_end = 319;

    for(y = y_start; y <= y_end; y++)
    {
        for(x = x_start; x <= x_end; x++)
        {
            ips200_draw_point((uint16)x, (uint16)y, color);
        }
    }
}

static void draw_circle(uint16 cx, uint16 cy, uint16 r, uint16 color)
{
    int16 x = 0, y = (int16)r;
    int16 d = 1 - (int16)r;
    while(x <= y)
    {
        if((cx + x) < 240 && (cy + y) < 320) ips200_draw_point((uint16)(cx + x), (uint16)(cy + y), color);
        if(cx >= (uint16)x && (cy + y) < 320) ips200_draw_point((uint16)(cx - x), (uint16)(cy + y), color);
        if((cx + x) < 240 && cy >= (uint16)y) ips200_draw_point((uint16)(cx + x), (uint16)(cy - y), color);
        if(cx >= (uint16)x && cy >= (uint16)y) ips200_draw_point((uint16)(cx - x), (uint16)(cy - y), color);
        if((cx + y) < 240 && (cy + x) < 320) ips200_draw_point((uint16)(cx + y), (uint16)(cy + x), color);
        if(cx >= (uint16)y && (cy + x) < 320) ips200_draw_point((uint16)(cx - y), (uint16)(cy + x), color);
        if((cx + y) < 240 && cy >= (uint16)x) ips200_draw_point((uint16)(cx + y), (uint16)(cy - x), color);
        if(cx >= (uint16)y && cy >= (uint16)x) ips200_draw_point((uint16)(cx - y), (uint16)(cy - x), color);
        if(d < 0) d += 2 * x + 3;
        else { d += 2 * (x - y) + 5; y--; }
        x++;
    }
}

static void draw_subject_home(void)
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
            subj_state_tag = (g_subject1_ui_ctx.task_data->waypoint_count > 0) ? "[READY]" : "[NO DATA]";
        }
        else if(i == 1)
        {
            subj_state_tag = (g_subject2_ui_ctx.task_data->point_count > 1) ? "[READY]" : "[NO DATA]";
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

static void draw_subject1_collect(void)
{
    uint8 i;
    const subject1_task_data_t *task_data = g_subject1_ui_ctx.task_data;

    ips200_full(COLOR_BG);
    ips200_set_color(COLOR_HIGHLIGHT, COLOR_BG);
    ips200_show_string(20, 4, "== Collect Points ==");

    ips200_set_color(point_type_color(task_data->current_point_type), COLOR_BG);
    ips200_show_string(8, 28, "Type:");
    ips200_show_string(56, 28, point_type_name(task_data->current_point_type));

    ips200_set_color(COLOR_TEXT, COLOR_BG);
    ips200_show_string(140, 28, "S:");
    ips200_show_string(164, 28, task_data->has_start ? "Y" : "N");
    ips200_show_string(180, 28, "T:");
    ips200_show_string(204, 28, task_data->has_turn ? "Y" : "N");

    ips200_show_string(8, 48, "X:");
    ips200_show_float(28, 48, inav_x, 3, 2);
    ips200_show_string(120, 48, "Y:");
    ips200_show_float(140, 48, inav_y, 3, 2);

    ips200_set_color(COLOR_TITLE, COLOR_BG);
    ips200_show_string(8, 72, "# Type   X      Y");
    ips200_draw_line(8, 88, 232, 88, 0x4208);

    for(i = 0; (i < task_data->waypoint_count) && (i < 10); i++)
    {
        uint16 row_y = 92 + i * 16;
        ips200_set_color(point_type_color(task_data->points[i].type), COLOR_BG);
        ips200_show_uint(8, row_y, i, 2);
        ips200_show_char(36, row_y, point_type_char(task_data->points[i].type));
        ips200_show_float(60, row_y, task_data->points[i].x, 3, 2);
        ips200_show_float(150, row_y, task_data->points[i].y, 3, 2);
    }

    ips200_set_color(COLOR_TEXT, COLOR_BG);
    ips200_show_string(8, 260, "Total:");
    ips200_show_uint(60, 260, task_data->waypoint_count, 2);

    ips200_set_color(COLOR_HIGHLIGHT, COLOR_BG);
    ips200_show_string(0, 280, "UP:Add DOWN:Type");
    ips200_show_string(0, 296, "LEFT:Done SE:Del SW:Bk");
}
static void draw_subject1_preview(void)
{
    uint8 i;
    float min_x, max_x, min_y, max_y;
    float range_x, range_y, scale;
    float offset_x, offset_y;
    const subject1_task_data_t *task_data = g_subject1_ui_ctx.task_data;
    const subject1_preview_data_t *preview_data = g_subject1_ui_ctx.preview_data;

    ips200_full(COLOR_BG);
    ips200_set_color(COLOR_HIGHLIGHT, COLOR_BG);
    ips200_show_string(20, 2, "== Select Route ==");

    if((preview_data->start_idx < 0) || (preview_data->turn_idx < 0) || (task_data->waypoint_count < 2))
    {
        ips200_set_color(COLOR_TEXT, COLOR_BG);
        ips200_show_string(40, 150, "Not enough pts!");
        return;
    }

    min_x = max_x = task_data->points[0].x;
    min_y = max_y = task_data->points[0].y;

    for(i = 0; i < task_data->waypoint_count; i++)
    {
        if(task_data->points[i].x < min_x) min_x = task_data->points[i].x;
        if(task_data->points[i].x > max_x) max_x = task_data->points[i].x;
        if(task_data->points[i].y < min_y) min_y = task_data->points[i].y;
        if(task_data->points[i].y > max_y) max_y = task_data->points[i].y;
    }
    for(i = 0; i < preview_data->routeA_count; i++)
    {
        if(preview_data->routeA_x[i] < min_x) min_x = preview_data->routeA_x[i];
        if(preview_data->routeA_x[i] > max_x) max_x = preview_data->routeA_x[i];
        if(preview_data->routeA_y[i] < min_y) min_y = preview_data->routeA_y[i];
        if(preview_data->routeA_y[i] > max_y) max_y = preview_data->routeA_y[i];
    }
    for(i = 0; i < preview_data->routeB_count; i++)
    {
        if(preview_data->routeB_x[i] < min_x) min_x = preview_data->routeB_x[i];
        if(preview_data->routeB_x[i] > max_x) max_x = preview_data->routeB_x[i];
        if(preview_data->routeB_y[i] < min_y) min_y = preview_data->routeB_y[i];
        if(preview_data->routeB_y[i] > max_y) max_y = preview_data->routeB_y[i];
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
        uint16 sx = SCR_X(task_data->points[preview_data->start_idx].x);
        uint16 sy = SCR_Y(task_data->points[preview_data->start_idx].y);
        uint16 tx = SCR_X(task_data->points[preview_data->turn_idx].x);
        uint16 ty = SCR_Y(task_data->points[preview_data->turn_idx].y);
        ips200_draw_line(sx, sy, tx, ty, COLOR_ROUTE_FWD);
    }

    {
        uint16 colorA = (task_data->route_sel == 0) ? COLOR_ROUTE_A : COLOR_ROUTE_DIM;
        for(i = 0; i + 1 < preview_data->routeA_count; i++)
        {
            ips200_draw_line(
                SCR_X(preview_data->routeA_x[i]), SCR_Y(preview_data->routeA_y[i]),
                SCR_X(preview_data->routeA_x[i + 1]), SCR_Y(preview_data->routeA_y[i + 1]),
                colorA);
        }
    }

    {
        uint16 colorB = (task_data->route_sel == 1) ? COLOR_ROUTE_B : COLOR_ROUTE_DIM;
        for(i = 0; i + 1 < preview_data->routeB_count; i++)
        {
            ips200_draw_line(
                SCR_X(preview_data->routeB_x[i]), SCR_Y(preview_data->routeB_y[i]),
                SCR_X(preview_data->routeB_x[i + 1]), SCR_Y(preview_data->routeB_y[i + 1]),
                colorB);
        }
    }

    for(i = 0; i < task_data->waypoint_count; i++)
    {
        uint16 sx = SCR_X(task_data->points[i].x);
        uint16 sy = SCR_Y(task_data->points[i].y);
        uint16 color = point_type_color(task_data->points[i].type);

        switch(task_data->points[i].type)
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
    ips200_set_color((task_data->route_sel == 0) ? COLOR_ROUTE_A : COLOR_ROUTE_DIM, COLOR_BG);
    ips200_show_string(8, 278, (task_data->route_sel == 0) ? ">A 1st Left" : " A 1st Left");
    ips200_set_color((task_data->route_sel == 1) ? COLOR_ROUTE_B : COLOR_ROUTE_DIM, COLOR_BG);
    ips200_show_string(8, 294, (task_data->route_sel == 1) ? ">B 1st Right" : " B 1st Right");
    ips200_set_color(COLOR_HIGHLIGHT, COLOR_BG);
    ips200_show_string(130, 278, "UP/DN:Switch");
    ips200_show_string(130, 294, "L:Rdy SW:Bk");

    #undef SCR_X
    #undef SCR_Y
}

static void draw_subject1_standby(void)
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

static void draw_subject1_running(void)
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

static void draw_subject1_done(void)
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

static void draw_subject1(void)
{
    switch(*g_subject1_ui_ctx.ui_state)
    {
        case UI_STATE_HOME:    draw_subject_home();     break;
        case UI_STATE_COLLECT: draw_subject1_collect(); break;
        case UI_STATE_PREVIEW: draw_subject1_preview(); break;
        case UI_STATE_STANDBY: draw_subject1_standby(); break;
        case UI_STATE_RUNNING: draw_subject1_running(); break;
        case UI_STATE_DONE:    draw_subject1_done();    break;
    }
}

static void draw_subject2(void)
{
    uint8 i;
    float min_x, max_x, min_y, max_y;
    float range_x, range_y, scale;
    float offset_x, offset_y;
    const subject2_task_data_t *task_data = g_subject2_ui_ctx.task_data;

    ips200_full(COLOR_BG);

    switch(*g_subject2_ui_ctx.ui_state)
    {
        case UI_STATE_COLLECT:
            ips200_set_color(COLOR_HIGHLIGHT, COLOR_BG);
            ips200_show_string(24, 4, "== Mine Collect ==");
            ips200_set_color(COLOR_TEXT, COLOR_BG);
            ips200_show_string(8, 28, "Type:");
            if(task_data->current_point_type == S2_POINT_START) ips200_show_string(56, 28, "START");
            else if(task_data->current_point_type == S2_POINT_MINE) ips200_show_string(56, 28, "MINE");
            else ips200_show_string(56, 28, "TURNBACK");
            ips200_show_string(136, 28, "Mine:");
            ips200_show_uint(184, 28, task_data->mine_count, 2);
            ips200_show_string(8, 48, "S:");
            ips200_show_string(24, 48, task_data->has_start ? "Y" : "N");
            ips200_show_string(56, 48, "T:");
            ips200_show_string(72, 48, task_data->has_turnback ? "Y" : "N");
            ips200_show_string(104, 48, "X:");
            ips200_show_float(124, 48, inav_x, 3, 2);
            ips200_show_string(8, 66, "Y:");
            ips200_show_float(28, 66, inav_y, 3, 2);
            ips200_set_color(COLOR_TITLE, COLOR_BG);
            ips200_show_string(8, 72, "# Type   X      Y");
            ips200_draw_line(8, 88, 232, 88, 0x4208);
            for(i = 0; (i < task_data->point_count) && (i < 10); i++)
            {
                uint16 row_y = 92 + i * 16;
                uint16 color = (task_data->points[i].type == S2_POINT_START) ? COLOR_START_PT :
                               (task_data->points[i].type == S2_POINT_MINE) ? COLOR_MINE_PT : COLOR_TURN_PT;
                const char *type_name = (task_data->points[i].type == S2_POINT_START) ? "START" :
                                        (task_data->points[i].type == S2_POINT_MINE) ? "MINE " : "TURN ";
                ips200_set_color(color, COLOR_BG);
                ips200_show_uint(8, row_y, i, 2);
                ips200_show_string(28, row_y, type_name);
                ips200_show_float(84, row_y, task_data->points[i].x, 3, 2);
                ips200_show_float(154, row_y, task_data->points[i].y, 3, 2);
            }
            ips200_set_color(COLOR_HIGHLIGHT, COLOR_BG);
            ips200_show_string(0, 280, "UP:Add DN:Type");
            ips200_show_string(0, 296, "L:Done SE:Del SW:Bk");
            break;

        case UI_STATE_PREVIEW:
            ips200_set_color(COLOR_HIGHLIGHT, COLOR_BG);
            ips200_show_string(22, 2, "== Mine Route View ==");

            if(task_data->point_count < 2)
            {
                ips200_set_color(COLOR_TEXT, COLOR_BG);
                ips200_show_string(40, 150, "Not enough pts!");
                break;
            }

            min_x = max_x = task_data->points[0].x;
            min_y = max_y = task_data->points[0].y;
            for(i = 0; i < task_data->point_count; i++)
            {
                if(task_data->points[i].x < min_x) min_x = task_data->points[i].x;
                if(task_data->points[i].x > max_x) max_x = task_data->points[i].x;
                if(task_data->points[i].y < min_y) min_y = task_data->points[i].y;
                if(task_data->points[i].y > max_y) max_y = task_data->points[i].y;
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

            #define S2_SCR_X(wx) ((uint16)(offset_x + ((wx) - min_x) * scale))
            #define S2_SCR_Y(wy) ((uint16)(offset_y + MAP_H - ((wy) - min_y) * scale))

            for(i = 0; i + 1 < task_data->point_count; i++)
            {
                uint16 color = (task_data->points[i + 1].type == S2_POINT_TURNBACK) ? COLOR_TURN_PT : COLOR_ROUTE_FWD;
                ips200_draw_line(
                    S2_SCR_X(task_data->points[i].x), S2_SCR_Y(task_data->points[i].y),
                    S2_SCR_X(task_data->points[i + 1].x), S2_SCR_Y(task_data->points[i + 1].y),
                    color);
            }

            if(task_data->has_turnback)
            {
                uint16 sx = S2_SCR_X(task_data->points[task_data->point_count - 1].x);
                uint16 sy = S2_SCR_Y(task_data->points[task_data->point_count - 1].y);
                uint16 ox = S2_SCR_X(task_data->points[0].x);
                uint16 oy = S2_SCR_Y(task_data->points[0].y);
                ips200_draw_line(sx, sy, ox, oy, COLOR_ROUTE_DIM);
            }

            for(i = 0; i < task_data->point_count; i++)
            {
                uint16 sx = S2_SCR_X(task_data->points[i].x);
                uint16 sy = S2_SCR_Y(task_data->points[i].y);
                if(task_data->points[i].type == S2_POINT_START)
                {
                    draw_dot(sx, sy, 4, COLOR_START_PT);
                    ips200_set_color(COLOR_START_PT, COLOR_BG);
                    ips200_show_char(sx + 6, sy - 4, 'S');
                }
                else if(task_data->points[i].type == S2_POINT_MINE)
                {
                    draw_circle(sx, sy, 5, COLOR_MINE_PT);
                    draw_dot(sx, sy, 1, COLOR_MINE_PT);
                    ips200_set_color(COLOR_MINE_PT, COLOR_BG);
                    ips200_show_uint(sx + 6, sy - 4, i, 2);
                }
                else
                {
                    draw_dot(sx, sy, 4, COLOR_TURN_PT);
                    ips200_set_color(COLOR_TURN_PT, COLOR_BG);
                    ips200_show_char(sx + 6, sy - 4, 'T');
                }
            }

            ips200_set_color(COLOR_ROUTE_FWD, COLOR_BG);
            ips200_show_string(8, 260, "Blue: go mines");
            ips200_set_color(COLOR_ROUTE_DIM, COLOR_BG);
            ips200_show_string(8, 278, "Gray: return home");
            ips200_set_color(COLOR_HIGHLIGHT, COLOR_BG);
            ips200_show_string(8, 296, "LEFT:Rdy  SW:Back");

            #undef S2_SCR_X
            #undef S2_SCR_Y
            break;

        case UI_STATE_STANDBY:
            ips200_set_color(COLOR_HIGHLIGHT, COLOR_BG);
            ips200_show_string(28, 10, "== S2 STANDBY ==");
            ips200_set_color(COLOR_TEXT, COLOR_BG);
            ips200_show_string(16, 60, "Put car at start point");
            ips200_show_string(16, 80, "heading same as collect");
            ips200_show_string(16, 110, "Mine count:");
            ips200_show_uint(104, 110, task_data->mine_count, 2);
            ips200_show_string(16, 140, "Turnback ready:");
            ips200_show_string(128, 140, task_data->has_turnback ? "YES" : "NO");
            ips200_set_color(COLOR_HIGHLIGHT, COLOR_BG);
            ips200_show_string(8, 280, "[LEFT] Launch!");
            ips200_show_string(8, 296, "[SW] Cancel");
            break;

        case UI_STATE_RUNNING:
            ips200_set_color(COLOR_HIGHLIGHT, COLOR_BG);
            ips200_show_string(28, 10, "== S2 RUNNING ==");
            ips200_set_color(COLOR_TEXT, COLOR_BG);
            ips200_show_string(8, 40, "Phase:");
            if(*g_subject2_ui_ctx.phase == S2_PHASE_TO_MINE) ips200_show_string(56, 40, "TO MINE");
            else if(*g_subject2_ui_ctx.phase == S2_PHASE_SPINNING) ips200_show_string(56, 40, "SPINNING");
            else if(*g_subject2_ui_ctx.phase == S2_PHASE_TO_TURNBACK) ips200_show_string(56, 40, "TO TURN");
            else if(*g_subject2_ui_ctx.phase == S2_PHASE_RETURN) ips200_show_string(56, 40, "RETURN");
            else ips200_show_string(56, 40, "DONE");
            ips200_show_string(8, 62, "TgtIdx:");
            ips200_show_uint(64, 62, *g_subject2_ui_ctx.current_target_idx, 2);
            ips200_show_string(120, 62, "Mine:");
            ips200_show_uint(168, 62, *g_subject2_ui_ctx.current_mine_idx, 2);
            ips200_show_string(8, 84, "SpinDir:");
            ips200_show_string(72, 84, (*g_subject2_ui_ctx.spin_dir > 0) ? "CW" : "CCW");
            ips200_show_string(120, 84, "Lap:");
            ips200_show_uint(160, 84, *g_subject2_ui_ctx.spin_lap_count, 2);
            ips200_show_string(8, 106, "X:");
            ips200_show_float(28, 106, inav_x, 4, 2);
            ips200_show_string(120, 106, "Y:");
            ips200_show_float(140, 106, inav_y, 4, 2);
            ips200_show_string(8, 128, "Yaw:");
            ips200_show_float(48, 128, quat_yaw_deg, 4, 1);
            ips200_show_string(8, 150, "Target:");
            ips200_show_float(64, 150, *g_subject2_ui_ctx.spin_target_yaw, 4, 1);
            ips200_show_string(8, 172, "Speed:");
            ips200_show_float(56, 172, target_speed, 5, 1);
            ips200_set_color(COLOR_HIGHLIGHT, COLOR_BG);
            ips200_show_string(8, 302, "[SW] Emergency Stop");
            break;

        case UI_STATE_DONE:
            ips200_set_color(COLOR_START_PT, COLOR_BG);
            ips200_show_string(42, 100, "== S2 FINISHED ==");
            ips200_set_color(COLOR_TEXT, COLOR_BG);
            ips200_show_string(28, 136, "Mine clearance done");
            ips200_show_string(28, 156, "Returned to start");
            ips200_show_string(28, 186, "Mines:");
            ips200_show_uint(84, 186, task_data->mine_count, 2);
            ips200_set_color(COLOR_HIGHLIGHT, COLOR_BG);
            ips200_show_string(20, 260, "[UP]  Re-run");
            ips200_show_string(20, 280, "[SW]  Back to Home");
            break;

        default:
            ips200_set_color(COLOR_HIGHLIGHT, COLOR_BG);
            ips200_show_string(22, 16, "== SUBJECT 2 ==");
            ips200_set_color(COLOR_TEXT, COLOR_BG);
            ips200_show_string(20, 120, "LEFT to collect route");
            break;
    }
}

static void draw_subject3(void)
{
    ips200_full(COLOR_BG);
    ips200_set_color(COLOR_HIGHLIGHT, COLOR_BG);
    ips200_show_string(16, 16, "== SUBJECT 3 ==");
    ips200_set_color(COLOR_TEXT, COLOR_BG);
    ips200_show_string(20, 110, "Rough terrain UI");
    ips200_show_string(20, 132, "to be added later.");
    ips200_set_color(COLOR_HIGHLIGHT, COLOR_BG);
    ips200_show_string(48, 280, "SW: Back Home");
}

void competition_ui_init(void)
{
    ips200_set_dir(IPS200_PORTAIT);
    ips200_set_color(COLOR_TEXT, COLOR_BG);
    ips200_init(IPS200_TYPE);
    ips200_set_font(IPS200_8X16_FONT);
}

void competition_ui_redraw(void)
{
    if((s1_ui_state == UI_STATE_HOME) && (s2_ui_state == UI_STATE_HOME))
    {
        draw_subject_home();
    }
    else if(selected_subject == 0)
    {
        draw_subject1();
    }
    else if(selected_subject == 1)
    {
        draw_subject2();
    }
    else
    {
        draw_subject3();
    }
}
