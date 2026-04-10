/*
    科目一任务流程模块 —— 绕桩前进
*/

#include "subject1.h"
#include "UI.h"
#include "controler.h"
#include "posture_control.h"
#include "ins_tracker.h"
#include "zf_common_headfile.h"
#include <math.h>
#include <string.h>

#define CONE_BYPASS_DIST   0.35f
#define OVERMEASURE        0.1f

// Flash 存储布局 (Section 0, Page 0)
// [0]        : 魔数 0x53314F4B，用于判断数据是否有效
// [1]        : collect_heading_ref (float)
// [2]        : waypoint_count (uint8)
// [3]        : has_start (uint8)
// [4]        : has_turn (uint8)
// [5+i*3+0]  : points[i].x (float)
// [5+i*3+1]  : points[i].y (float)
// [5+i*3+2]  : points[i].type (uint8)
#define S1_FLASH_SECTION    (0)
#define S1_FLASH_PAGE       (0)
#define S1_FLASH_MAGIC      (0x53314F4Bu)
#define S1_FLASH_HDR_LEN    (5)

ui_state_enum           s1_ui_state      = UI_STATE_HOME;
subject1_task_data_t    s1_task_data     = {{0}, 0, 0, 0, POINT_TYPE_START, 0, 0.0f};
subject1_preview_data_t s1_preview_data;
uint8                   selected_subject = 0;

const subject1_ui_context_t g_subject1_ui_ctx =
{
    &s1_ui_state,
    &s1_task_data,
    &s1_preview_data,
};

static uint32 btn_up_cnt   = 0;
static uint32 btn_down_cnt = 0;
static uint32 btn_left_cnt = 0;
static uint32 btn_se_cnt   = 0;
static uint32 btn_sw_cnt   = 0;

static uint32 run_display_cnt = 0;
static uint8  s1_screen_dirty = 1;

static uint8 btn_rising(Button bt, uint32 *cnt)
{
    if(button_press(bt))
    {
        (*cnt)++;
        if(*cnt == 1) return 1;
        return 0;
    }

    *cnt = 0;
    return 0;
}

static uint8 btn_long_press(Button bt, uint32 *cnt, uint32 hold_ticks)
{
    if(button_press(bt))
    {
        (*cnt)++;
        if(*cnt == hold_ticks) return 1;
        return 0;
    }

    *cnt = 0;
    return 0;
}

static void switch_state(ui_state_enum new_state)
{
    s1_ui_state = new_state;
    s1_screen_dirty = 1;
}

static void subject1_save_to_flash(void)
{
    uint8 i;
    uint32 total = S1_FLASH_HDR_LEN + (uint32)s1_task_data.waypoint_count * 3;

    flash_buffer_clear();
    flash_union_buffer[0].uint32_type           = S1_FLASH_MAGIC;
    flash_union_buffer[1].float_type            = s1_task_data.collect_heading_ref;
    flash_union_buffer[2].uint8_type            = s1_task_data.waypoint_count;
    flash_union_buffer[3].uint8_type            = s1_task_data.has_start;
    flash_union_buffer[4].uint8_type            = s1_task_data.has_turn;

    for(i = 0; i < s1_task_data.waypoint_count; i++)
    {
        flash_union_buffer[S1_FLASH_HDR_LEN + i * 3 + 0].float_type  = s1_task_data.points[i].x;
        flash_union_buffer[S1_FLASH_HDR_LEN + i * 3 + 1].float_type  = s1_task_data.points[i].y;
        flash_union_buffer[S1_FLASH_HDR_LEN + i * 3 + 2].uint8_type  = (uint8)s1_task_data.points[i].type;
    }

    if(flash_check(S1_FLASH_SECTION, S1_FLASH_PAGE))
        flash_erase_page(S1_FLASH_SECTION, S1_FLASH_PAGE);
    flash_write_page_from_buffer(S1_FLASH_SECTION, S1_FLASH_PAGE, total);
}

static uint8 subject1_load_from_flash(void)
{
    uint8 i;
    uint8 wpc;
    uint32 total;

    flash_read_page_to_buffer(S1_FLASH_SECTION, S1_FLASH_PAGE, S1_FLASH_HDR_LEN);

    if(flash_union_buffer[0].uint32_type != S1_FLASH_MAGIC)
        return 0;

    wpc = flash_union_buffer[2].uint8_type;
    if(wpc > S1_MAX_WAYPOINTS)
        return 0;

    total = S1_FLASH_HDR_LEN + (uint32)wpc * 3;
    flash_read_page_to_buffer(S1_FLASH_SECTION, S1_FLASH_PAGE, total);

    memset(&s1_task_data, 0, sizeof(s1_task_data));
    s1_task_data.collect_heading_ref  = flash_union_buffer[1].float_type;
    s1_task_data.waypoint_count       = wpc;
    s1_task_data.has_start            = flash_union_buffer[3].uint8_type;
    s1_task_data.has_turn             = flash_union_buffer[4].uint8_type;
    s1_task_data.current_point_type   = POINT_TYPE_START;

    for(i = 0; i < wpc; i++)
    {
        s1_task_data.points[i].x    = flash_union_buffer[S1_FLASH_HDR_LEN + i * 3 + 0].float_type;
        s1_task_data.points[i].y    = flash_union_buffer[S1_FLASH_HDR_LEN + i * 3 + 1].float_type;
        s1_task_data.points[i].type = (point_type_enum)flash_union_buffer[S1_FLASH_HDR_LEN + i * 3 + 2].uint8_type;
    }

    return 1;
}

static void reset_collect_data(void)
{
    memset(&s1_task_data, 0, sizeof(s1_task_data));
    memset(&s1_preview_data, 0, sizeof(s1_preview_data));
    s1_task_data.current_point_type = POINT_TYPE_START;
    inav_x      = 0.0f;
    inav_y      = 0.0f;
    inav_active = 0;
}

static void clear_selected_subject_data(void)
{
    if(selected_subject == 0)
    {
        reset_collect_data();
        if(flash_check(S1_FLASH_SECTION, S1_FLASH_PAGE))
            flash_erase_page(S1_FLASH_SECTION, S1_FLASH_PAGE);
    }
}

static uint8 subject1_can_enter_preview(void)
{
    return (s1_task_data.has_start && s1_task_data.has_turn && s1_task_data.waypoint_count >= 2);
}

static uint8 subject1_add_current_point(void)
{
    waypoint_t *wp;

    if(s1_task_data.waypoint_count >= S1_MAX_WAYPOINTS) return 0;
    if(s1_task_data.current_point_type == POINT_TYPE_START && s1_task_data.has_start) return 0;
    if(s1_task_data.current_point_type == POINT_TYPE_TURN  && s1_task_data.has_turn) return 0;

    if(s1_task_data.waypoint_count == 0)
    {
        inav_heading_ref = quat_yaw_deg;
        s1_task_data.collect_heading_ref = quat_yaw_deg;
        inav_x = 0.0f;
        inav_y = 0.0f;
        inav_active = 1;
    }

    wp = &s1_task_data.points[s1_task_data.waypoint_count];
    wp->x    = inav_x;
    wp->y    = inav_y;
    wp->type = s1_task_data.current_point_type;

    if(s1_task_data.current_point_type == POINT_TYPE_START) s1_task_data.has_start = 1;
    if(s1_task_data.current_point_type == POINT_TYPE_TURN)  s1_task_data.has_turn  = 1;

    s1_task_data.waypoint_count++;
    led(toggle);
    subject1_save_to_flash();
    return 1;
}

static uint8 subject1_delete_last_point(void)
{
    waypoint_t *removed;

    if(s1_task_data.waypoint_count == 0) return 0;

    s1_task_data.waypoint_count--;
    removed = &s1_task_data.points[s1_task_data.waypoint_count];
    if(removed->type == POINT_TYPE_START) s1_task_data.has_start = 0;
    if(removed->type == POINT_TYPE_TURN)  s1_task_data.has_turn  = 0;
    memset(removed, 0, sizeof(*removed));
    subject1_save_to_flash();
    return 1;
}

static uint8 generate_bypass_route(
    const float *cone_x, const float *cone_y, uint8 cone_count,
    float from_x, float from_y,
    float to_x,   float to_y,
    int8  first_side,
    float *out_x, float *out_y)
{
    uint8 out_count = 0;
    uint8 i;
    int8  side = first_side;

    out_x[out_count] = from_x;
    out_y[out_count] = from_y;
    out_count++;

    for(i = 0; i < cone_count; i++)
    {
        float prev_x = (i == 0) ? from_x : cone_x[i - 1];
        float prev_y = (i == 0) ? from_y : cone_y[i - 1];
        float next_x = (i == cone_count - 1) ? to_x : cone_x[i + 1];
        float next_y = (i == cone_count - 1) ? to_y : cone_y[i + 1];
        float dir_x = next_x - prev_x;
        float dir_y = next_y - prev_y;
        float len = sqrtf(dir_x * dir_x + dir_y * dir_y);
        float nx, ny;

        if(len < 0.001f) len = 0.001f;
        dir_x /= len;
        dir_y /= len;
        nx = -dir_y;
        ny =  dir_x;

        out_x[out_count] = cone_x[i] + side * CONE_BYPASS_DIST * nx;
        out_y[out_count] = cone_y[i] + side * CONE_BYPASS_DIST * ny;
        out_count++;
        side = -side;
    }

    out_x[out_count] = to_x;
    out_y[out_count] = to_y;
    out_count++;

    return out_count;
}

static void subject1_prepare_routes(void)
{
    uint8 i;
    float start_x, start_y, turn_x, turn_y;

    s1_preview_data.start_idx = -1;
    s1_preview_data.turn_idx  = -1;
    s1_preview_data.cone_count = 0;

    for(i = 0; i < s1_task_data.waypoint_count; i++)
    {
        switch(s1_task_data.points[i].type)
        {
            case POINT_TYPE_START: s1_preview_data.start_idx = i; break;
            case POINT_TYPE_TURN:  s1_preview_data.turn_idx  = i; break;
            case POINT_TYPE_CONE:  s1_preview_data.cone_indices[s1_preview_data.cone_count++] = i; break;
        }
    }

    if(s1_preview_data.start_idx < 0 || s1_preview_data.turn_idx < 0) return;

    start_x = s1_task_data.points[s1_preview_data.start_idx].x;
    start_y = s1_task_data.points[s1_preview_data.start_idx].y;
    turn_x  = s1_task_data.points[s1_preview_data.turn_idx].x;
    turn_y  = s1_task_data.points[s1_preview_data.turn_idx].y;

    for(i = 0; i < s1_preview_data.cone_count; i++)
    {
        s1_preview_data.ret_cone_x[i] = s1_task_data.points[s1_preview_data.cone_indices[i]].x;
        s1_preview_data.ret_cone_y[i] = s1_task_data.points[s1_preview_data.cone_indices[i]].y;
    }

    s1_preview_data.routeA_count = generate_bypass_route(
        s1_preview_data.ret_cone_x, s1_preview_data.ret_cone_y, s1_preview_data.cone_count,
        turn_x, turn_y, start_x, start_y,
        +1, s1_preview_data.routeA_x, s1_preview_data.routeA_y);

    s1_preview_data.routeB_count = generate_bypass_route(
        s1_preview_data.ret_cone_x, s1_preview_data.ret_cone_y, s1_preview_data.cone_count,
        turn_x, turn_y, start_x, start_y,
        -1, s1_preview_data.routeB_x, s1_preview_data.routeB_y);
}

static void subject1_launch(void)
{
    float  full_x[S1_MAX_WAYPOINTS * 2 + 8];
    float  full_y[S1_MAX_WAYPOINTS * 2 + 8];
    uint8  full_count = 0;
    uint8  k;
    float start_x = s1_task_data.points[s1_preview_data.start_idx].x;
    float start_y = s1_task_data.points[s1_preview_data.start_idx].y;
    float turn_x  = s1_task_data.points[s1_preview_data.turn_idx].x;
    float turn_y  = s1_task_data.points[s1_preview_data.turn_idx].y;
    const float *sel_x = (s1_task_data.route_sel == 0) ? s1_preview_data.routeA_x : s1_preview_data.routeB_x;
    const float *sel_y = (s1_task_data.route_sel == 0) ? s1_preview_data.routeA_y : s1_preview_data.routeB_y;
    uint8 sel_count = (s1_task_data.route_sel == 0) ? s1_preview_data.routeA_count : s1_preview_data.routeB_count;

    full_x[full_count] = start_x;
    full_y[full_count] = start_y;
    full_count++;
    full_x[full_count] = turn_x;
    full_y[full_count] = turn_y;
    full_count++;

    for(k = 1; k < sel_count; k++)
    {
        full_x[full_count] = sel_x[k];
        full_y[full_count] = sel_y[k];
        full_count++;
    }

    if(full_count >= 2)
    {
        float prev_x = full_x[full_count - 2];
        float prev_y = full_y[full_count - 2];
        float last_x = full_x[full_count - 1];
        float last_y = full_y[full_count - 1];
        float dx = last_x - prev_x;
        float dy = last_y - prev_y;
        float len = sqrtf(dx * dx + dy * dy);
        if(len > 0.001f)
        {
            full_x[full_count] = last_x + (dx / len) * OVERMEASURE;
            full_y[full_count] = last_y + (dy / len) * OVERMEASURE;
            full_count++;
        }
    }

    inav_x           = 0.0f;
    inav_y           = 0.0f;
    inav_active      = 1;
    inav_heading_ref = s1_task_data.collect_heading_ref;

    run_state = 1;
    ins_tracker_start_with_points(full_x, full_y, full_count);
    switch_state(UI_STATE_RUNNING);
}

void subject1_clear_and_recollect(void)
{
    reset_collect_data();
    if(flash_check(S1_FLASH_SECTION, S1_FLASH_PAGE))
        flash_erase_page(S1_FLASH_SECTION, S1_FLASH_PAGE);
    s1_ui_state     = UI_STATE_COLLECT;
    s1_screen_dirty = 1;
}

void subject1_init(void)
{
    s1_ui_state = UI_STATE_HOME;
    s1_screen_dirty = 1;

    if(!subject1_load_from_flash())
    {
        reset_collect_data();
    }
}

void subject1_poll(void)
{
    if(btn_rising(UP, &btn_up_cnt))
    {
        if(s1_ui_state == UI_STATE_HOME)
        {
            selected_subject = (selected_subject + 2) % 3;
            s1_screen_dirty = 1;
        }
        else if(s1_ui_state == UI_STATE_PREVIEW)
        {
            s1_task_data.route_sel = (s1_task_data.route_sel == 0) ? 1 : 0;
            s1_screen_dirty = 1;
        }
        else if(s1_ui_state == UI_STATE_DONE)
        {
            switch_state(UI_STATE_STANDBY);
        }
        else if(s1_ui_state == UI_STATE_COLLECT && subject1_add_current_point())
        {
            s1_screen_dirty = 1;
        }
    }

    if(btn_rising(DOWN, &btn_down_cnt))
    {
        if(s1_ui_state == UI_STATE_HOME)
        {
            selected_subject = (selected_subject + 1) % 3;
            s1_screen_dirty = 1;
        }
        else if(s1_ui_state == UI_STATE_COLLECT)
        {
            s1_task_data.current_point_type = (point_type_enum)((s1_task_data.current_point_type + 1) % 3);
            s1_screen_dirty = 1;
        }
        else if(s1_ui_state == UI_STATE_PREVIEW)
        {
            s1_task_data.route_sel = (s1_task_data.route_sel == 0) ? 1 : 0;
            s1_screen_dirty = 1;
        }
    }

    if(btn_rising(LEFT, &btn_left_cnt))
    {
        if(s1_ui_state == UI_STATE_HOME)
        {
            if(selected_subject == 0)
            {
                if(subject1_can_enter_preview())
                {
                    inav_active = 0;
                    subject1_prepare_routes();
                    switch_state(UI_STATE_PREVIEW);
                }
                else
                {
                    switch_state(UI_STATE_COLLECT);
                }
            }
        }
        else if(s1_ui_state == UI_STATE_COLLECT)
        {
            if(subject1_can_enter_preview())
            {
                inav_active = 0;
                s1_task_data.route_sel = 0;
                subject1_prepare_routes();
                switch_state(UI_STATE_PREVIEW);
            }
        }
        else if(s1_ui_state == UI_STATE_PREVIEW)
        {
            balance_enable = 1;
            switch_state(UI_STATE_STANDBY);
        }
        else if(s1_ui_state == UI_STATE_STANDBY)
        {
            subject1_launch();
        }
    }

    if(btn_rising(SE, &btn_se_cnt))
    {
        if(s1_ui_state == UI_STATE_COLLECT && subject1_delete_last_point())
        {
            s1_screen_dirty = 1;
        }
    }

    if(btn_rising(SW, &btn_sw_cnt))
    {
        if(s1_ui_state == UI_STATE_DONE)
        {
            run_state      = 0;
            balance_enable = 0;
            switch_state(UI_STATE_HOME);
        }
    }

    if(btn_long_press(SW, &btn_sw_cnt, 20))
    {
        if(s1_ui_state == UI_STATE_COLLECT)
        {
            inav_active = 0;
            switch_state(UI_STATE_HOME);
        }
        else if(s1_ui_state == UI_STATE_PREVIEW)
        {
            switch_state(UI_STATE_HOME);
        }
        else if(s1_ui_state == UI_STATE_STANDBY)
        {
            run_state      = 0;
            balance_enable = 0;
            switch_state(UI_STATE_PREVIEW);
        }
    }

    if(s1_ui_state == UI_STATE_RUNNING)
    {
        if(tracker_state == TRACKER_STATE_DONE)
        {
            target_speed  = 0.0f;
            turn_diff_ext = 0;
            inav_active   = 0;
            switch_state(UI_STATE_DONE);
        }
        else if(btn_long_press(SW, &btn_sw_cnt, 10))
        {
            run_state      = 0;
            balance_enable = 0;
            target_speed   = 0.0f;
            turn_diff_ext  = 0;
            inav_active    = 0;
            tracker_state  = TRACKER_STATE_DONE;
            switch_state(UI_STATE_DONE);
        }
    }

    if((s1_ui_state == UI_STATE_STANDBY) || (s1_ui_state == UI_STATE_RUNNING))
    {
        run_display_cnt++;
        if(run_display_cnt >= 10)
        {
            run_display_cnt = 0;
            s1_screen_dirty = 1;
        }
    }

    if(s1_screen_dirty)
    {
        s1_screen_dirty = 0;
        competition_ui_redraw();
    }
}
