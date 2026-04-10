/*
    科目二任务流程模块 —— 定点排雷
*/

#include "subject2.h"
#include "UI.h"
#include "controler.h"
#include "posture_control.h"
#include "zf_common_headfile.h"
#include <math.h>
#include <string.h>

ui_state_enum        s2_ui_state   = UI_STATE_HOME;
subject2_task_data_t s2_task_data  = {{0}, 0, 0, 0, 0, S2_POINT_START, 0.0f};
subject2_phase_enum  s2_phase      = S2_PHASE_IDLE;

static uint8  s2_current_target_idx = 0;
static uint8  s2_current_mine_idx   = 0;
static int8   s2_spin_dir           = 1;
static uint8  s2_spin_lap_count     = 0;
static float  s2_spin_target_yaw    = 0.0f;
static float  s2_spin_accum_deg     = 0.0f;
static float  s2_yaw_bias_deg       = 0.0f;

const subject2_ui_context_t g_subject2_ui_ctx =
{
    &s2_ui_state,
    &s2_task_data,
    &s2_phase,
    &s2_current_target_idx,
    &s2_current_mine_idx,
    &s2_spin_dir,
    &s2_spin_lap_count,
    &s2_spin_target_yaw,
    &s2_yaw_bias_deg,
};

static uint32 btn_up_cnt   = 0;
static uint32 btn_down_cnt = 0;
static uint32 btn_left_cnt = 0;
static uint32 btn_se_cnt   = 0;
static uint32 btn_sw_cnt   = 0;
static uint32 run_display_cnt = 0;
static uint8  s2_screen_dirty = 1;

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

void subject_home_poll(void)
{
    if(btn_rising(UP, &btn_up_cnt))
    {
        selected_subject = (selected_subject + 2) % 3;
        competition_ui_redraw();
    }

    if(btn_rising(DOWN, &btn_down_cnt))
    {
        selected_subject = (selected_subject + 1) % 3;
        competition_ui_redraw();
    }

    if(btn_rising(LEFT, &btn_left_cnt))
    {
        if(selected_subject == 0)
        {
            if(s1_task_data.has_start && s1_task_data.has_turn && s1_task_data.waypoint_count >= 2)
            {
                inav_active = 0;
                s1_ui_state = UI_STATE_PREVIEW;
            }
            else
            {
                s1_ui_state = UI_STATE_COLLECT;
            }
        }
        else if(selected_subject == 1)
        {
            if(s2_task_data.has_start && s2_task_data.has_turnback && s2_task_data.mine_count > 0)
            {
                inav_active = 0;
                s2_ui_state = UI_STATE_PREVIEW;
            }
            else
            {
                s2_ui_state = UI_STATE_COLLECT;
            }
        }
        competition_ui_redraw();
    }
}

static float normalize_angle(float a)
{
    while(a > 180.0f) a -= 360.0f;
    while(a < -180.0f) a += 360.0f;
    return a;
}

static float point_distance(float x0, float y0, float x1, float y1)
{
    float dx = x1 - x0;
    float dy = y1 - y0;
    return sqrtf(dx * dx + dy * dy);
}

static float point_bearing(float x0, float y0, float x1, float y1)
{
    float dx = x1 - x0;
    float dy = y1 - y0;
    return atan2f(dx, dy) * (180.0f / 3.14159265f);
}

static float ramp_speed(float current_speed, float desired_speed)
{
    float delta = desired_speed - current_speed;

    if(delta > S2_SPEED_RAMP_STEP) delta = S2_SPEED_RAMP_STEP;
    else if(delta < -S2_SPEED_RAMP_STEP) delta = -S2_SPEED_RAMP_STEP;

    return current_speed + delta;
}

static float subject2_normalize_angle(float a)
{
    while(a > 180.0f) a -= 360.0f;
    while(a < -180.0f) a += 360.0f;
    return a;
}

static float subject2_current_yaw_deg(void)
{
    return subject2_normalize_angle(quat_yaw_deg + s2_yaw_bias_deg);
}

static void switch_state(ui_state_enum new_state)
{
    s2_ui_state = new_state;
    s2_screen_dirty = 1;
}

static void subject2_reset_data(void)
{
    memset(&s2_task_data, 0, sizeof(s2_task_data));
    s2_task_data.current_point_type = S2_POINT_START;
    s2_phase = S2_PHASE_IDLE;
    s2_current_target_idx = 0;
    s2_current_mine_idx = 0;
    s2_spin_dir = 1;
    s2_spin_lap_count = 0;
    s2_spin_target_yaw = 0.0f;
    s2_spin_accum_deg = 0.0f;
    s2_yaw_bias_deg = 0.0f;
    inav_x = 0.0f;
    inav_y = 0.0f;
    inav_active = 0;
    target_speed = 0.0f;
    car_turn_reset();
    car_spin_stop();
}

static uint8 subject2_can_enter_preview(void)
{
    return (s2_task_data.has_start && s2_task_data.has_turnback && s2_task_data.mine_count > 0);
}

static uint8 subject2_add_current_point(void)
{
    subject2_waypoint_t *wp;

    if(s2_task_data.point_count >= S2_MAX_ROUTE_POINTS) return 0;
    if(s2_task_data.current_point_type == S2_POINT_START && s2_task_data.has_start) return 0;
    if(s2_task_data.current_point_type == S2_POINT_TURNBACK && s2_task_data.has_turnback) return 0;

    if(s2_task_data.point_count == 0)
    {
        inav_heading_ref = quat_yaw_deg;
        s2_task_data.collect_heading_ref = quat_yaw_deg;
        inav_x = 0.0f;
        inav_y = 0.0f;
        inav_active = 1;
    }

    wp = &s2_task_data.points[s2_task_data.point_count];
    wp->x = inav_x;
    wp->y = inav_y;
    wp->type = s2_task_data.current_point_type;

    if(wp->type == S2_POINT_START) s2_task_data.has_start = 1;
    else if(wp->type == S2_POINT_MINE) s2_task_data.mine_count++;
    else if(wp->type == S2_POINT_TURNBACK) s2_task_data.has_turnback = 1;

    s2_task_data.point_count++;
    led(toggle);
    return 1;
}

static uint8 subject2_delete_last_point(void)
{
    subject2_waypoint_t *removed;

    if(s2_task_data.point_count == 0) return 0;

    s2_task_data.point_count--;
    removed = &s2_task_data.points[s2_task_data.point_count];

    if(removed->type == S2_POINT_START) s2_task_data.has_start = 0;
    else if(removed->type == S2_POINT_MINE && s2_task_data.mine_count > 0) s2_task_data.mine_count--;
    else if(removed->type == S2_POINT_TURNBACK) s2_task_data.has_turnback = 0;

    memset(removed, 0, sizeof(*removed));
    return 1;
}

static void subject2_cycle_point_type(void)
{
    s2_task_data.current_point_type = (subject2_point_type_enum)((s2_task_data.current_point_type + 1) % 3);
}

static void subject2_start_spin(uint8 mine_idx)
{
    s2_current_mine_idx = mine_idx;
    s2_spin_lap_count = 0;
    s2_spin_accum_deg = 0.0f;
    s2_spin_dir = (mine_idx & 0x01) ? -1 : 1;
    s2_spin_target_yaw = 720.0f;
    target_speed = 0.0f;
    car_spin_start(0.0f, s2_spin_dir);
    s2_phase = S2_PHASE_SPINNING;
}

static void subject2_launch(void)
{
    inav_x = 0.0f;
    inav_y = 0.0f;
    inav_active = 1;
    inav_heading_ref = s2_task_data.collect_heading_ref;
    s2_current_target_idx = 1;
    s2_current_mine_idx = 0;
    s2_phase = S2_PHASE_TO_MINE;
    target_speed = 0.0f;
    run_state = 1;
    switch_state(UI_STATE_RUNNING);
}

void subject2_init(void)
{
    s2_ui_state = UI_STATE_HOME;
    s2_screen_dirty = 1;
    subject2_reset_data();
}

void subject2_poll(void)
{
    if(btn_rising(UP, &btn_up_cnt))
    {
        if(s2_ui_state == UI_STATE_COLLECT && subject2_add_current_point())
        {
            s2_screen_dirty = 1;
        }
        else if(s2_ui_state == UI_STATE_DONE)
        {
            switch_state(UI_STATE_STANDBY);
        }
    }

    if(btn_rising(DOWN, &btn_down_cnt))
    {
        if(s2_ui_state == UI_STATE_COLLECT)
        {
            subject2_cycle_point_type();
            s2_screen_dirty = 1;
        }
    }

    if(btn_rising(LEFT, &btn_left_cnt))
    {
        if(s2_ui_state == UI_STATE_COLLECT)
        {
            if(subject2_can_enter_preview())
            {
                inav_active = 0;
                switch_state(UI_STATE_PREVIEW);
            }
        }
        else if(s2_ui_state == UI_STATE_PREVIEW)
        {
            balance_enable = 1;
            switch_state(UI_STATE_STANDBY);
        }
        else if(s2_ui_state == UI_STATE_STANDBY)
        {
            subject2_launch();
        }
    }

    if(btn_rising(SE, &btn_se_cnt))
    {
        if(s2_ui_state == UI_STATE_COLLECT && subject2_delete_last_point())
        {
            s2_screen_dirty = 1;
        }
    }

    if(btn_rising(SW, &btn_sw_cnt))
    {
        if(s2_ui_state == UI_STATE_DONE)
        {
            run_state = 0;
            balance_enable = 0;
            switch_state(UI_STATE_HOME);
        }
    }

    if(btn_long_press(SE, &btn_se_cnt, 20))
    {
        if(s2_ui_state == UI_STATE_HOME && selected_subject == 1)
        {
            subject2_reset_data();
            switch_state(UI_STATE_COLLECT);
        }
    }

    if(btn_long_press(SW, &btn_sw_cnt, 20))
    {
        if(s2_ui_state == UI_STATE_COLLECT)
        {
            inav_active = 0;
            switch_state(UI_STATE_HOME);
        }
        else if(s2_ui_state == UI_STATE_PREVIEW)
        {
            switch_state(UI_STATE_HOME);
        }
        else if(s2_ui_state == UI_STATE_STANDBY)
        {
            run_state = 0;
            balance_enable = 0;
            switch_state(UI_STATE_PREVIEW);
        }
    }

    if(s2_ui_state == UI_STATE_RUNNING)
    {
        if(s2_phase == S2_PHASE_DONE)
        {
            target_speed = 0.0f;
            car_turn_reset();
            inav_active = 0;
            switch_state(UI_STATE_DONE);
        }
        else if(btn_long_press(SW, &btn_sw_cnt, 10))
        {
            run_state = 0;
            balance_enable = 0;
            target_speed = 0.0f;
            inav_active = 0;
            car_turn_reset();
            car_spin_stop();
            s2_phase = S2_PHASE_DONE;
            switch_state(UI_STATE_DONE);
        }
    }

    if((s2_ui_state == UI_STATE_STANDBY) || (s2_ui_state == UI_STATE_RUNNING))
    {
        run_display_cnt++;
        if(run_display_cnt >= 10)
        {
            run_display_cnt = 0;
            s2_screen_dirty = 1;
        }
    }

    if(s2_screen_dirty)
    {
        s2_screen_dirty = 0;
        competition_ui_redraw();
    }
}

void subject2_update(void)
{
    float target_x;
    float target_y;
    float dist;
    float bearing_local;
    float current_heading_rel;
    float heading_err;
    float yaw_used_deg;

    if(s2_ui_state != UI_STATE_RUNNING)
    {
        return;
    }

    if(s2_phase == S2_PHASE_SPINNING)
    {
        target_speed = 0.0f;
        turn_duty_max = 1200;
        s2_spin_accum_deg = spin_accum_deg;
        car_spin_update(quat_yaw_deg, quat_yaw_rate_dps);

        if(s2_spin_accum_deg >= 720.0f)
        {
            float spin_yaw_bias = (s2_spin_dir > 0) ? S2_SPIN_YAW_BIAS_CW_DEG : S2_SPIN_YAW_BIAS_CCW_DEG;
            car_spin_stop();
            s2_yaw_bias_deg = subject2_normalize_angle(s2_yaw_bias_deg + spin_yaw_bias);
            turn_duty_max = 300;
            s2_phase = (s2_current_target_idx < (s2_task_data.point_count - 1)) ? S2_PHASE_TO_MINE : S2_PHASE_TO_TURNBACK;
            s2_current_target_idx++;
            s2_screen_dirty = 1;
        }
        return;
    }

    turn_duty_max = 300;

    if(s2_phase == S2_PHASE_DONE || s2_task_data.point_count < 2)
    {
        return;
    }

    target_x = s2_task_data.points[s2_current_target_idx].x;
    target_y = s2_task_data.points[s2_current_target_idx].y;
    dist = point_distance(inav_x, inav_y, target_x, target_y);

    if(dist < S2_ARRIVE_DIST)
    {
        if(s2_task_data.points[s2_current_target_idx].type == S2_POINT_TURNBACK)
        {
            s2_phase = S2_PHASE_RETURN;
            s2_current_target_idx = 0;
            s2_screen_dirty = 1;
            return;
        }
        else if(s2_phase == S2_PHASE_TO_MINE)
        {
            subject2_start_spin(s2_current_target_idx - 1);
            s2_screen_dirty = 1;
            return;
        }
        else if(s2_phase == S2_PHASE_RETURN)
        {
            s2_phase = S2_PHASE_DONE;
            target_speed = 0.0f;
            car_turn_reset();
            inav_active = 0;
            s2_screen_dirty = 1;
            return;
        }
    }

    bearing_local = point_bearing(inav_x, inav_y, target_x, target_y);
    yaw_used_deg = subject2_current_yaw_deg();
    current_heading_rel = normalize_angle(yaw_used_deg - s2_task_data.collect_heading_ref);
    heading_err = normalize_angle(bearing_local - current_heading_rel);

    target_speed = ramp_speed(target_speed, S2_CRUISE_SPEED);
    car_turn_control(heading_err, quat_yaw_rate_dps);
}
