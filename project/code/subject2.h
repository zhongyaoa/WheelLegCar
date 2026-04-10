#ifndef _SUBJECT2_H_
#define _SUBJECT2_H_

#include "zf_common_typedef.h"
#include "subject1.h"

typedef enum
{
    S2_POINT_START    = 0,
    S2_POINT_MINE     = 1,
    S2_POINT_TURNBACK = 2,
} subject2_point_type_enum;

typedef enum
{
    S2_PHASE_IDLE        = 0,
    S2_PHASE_TO_MINE     = 1,
    S2_PHASE_SPINNING    = 2,
    S2_PHASE_TO_TURNBACK = 3,
    S2_PHASE_RETURN      = 4,
    S2_PHASE_DONE        = 5,
} subject2_phase_enum;

#define S2_MAX_MINES                    12
#define S2_MAX_ROUTE_POINTS             (S2_MAX_MINES + 3)
#define S2_MINE_SPIN_LAPS               1
#define S2_ARRIVE_DIST                  0.10f
#define S2_CRUISE_SPEED                 450.0f
#define S2_SPEED_RAMP_STEP              40.0f
#define S2_SPIN_HOLD_DEG                12.0f
#define S2_SPIN_YAW_BIAS_CW_DEG         0.0f
#define S2_SPIN_YAW_BIAS_CCW_DEG        0.0f

typedef struct
{
    float x;
    float y;
    subject2_point_type_enum type;
} subject2_waypoint_t;

typedef struct
{
    subject2_waypoint_t points[S2_MAX_ROUTE_POINTS];
    uint8 point_count;
    uint8 mine_count;
    uint8 has_start;
    uint8 has_turnback;
    subject2_point_type_enum current_point_type;
    float collect_heading_ref;
} subject2_task_data_t;

typedef struct
{
    ui_state_enum *ui_state;
    subject2_task_data_t *task_data;
    subject2_phase_enum *phase;
    uint8 *current_target_idx;
    uint8 *current_mine_idx;
    int8 *spin_dir;
    uint8 *spin_lap_count;
    float *spin_target_yaw;
    float *yaw_bias_deg;
} subject2_ui_context_t;

void subject_home_poll(void);

extern ui_state_enum          s2_ui_state;
extern subject2_task_data_t   s2_task_data;
extern subject2_phase_enum    s2_phase;
extern const subject2_ui_context_t g_subject2_ui_ctx;

void subject2_init(void);
void subject2_poll(void);
void subject2_update(void);

#endif
