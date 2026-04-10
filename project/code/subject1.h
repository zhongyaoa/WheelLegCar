#ifndef _SUBJECT1_H_
#define _SUBJECT1_H_

#include "zf_common_typedef.h"

typedef enum
{
    POINT_TYPE_START   = 0,
    POINT_TYPE_TURN    = 1,
    POINT_TYPE_CONE    = 2,
} point_type_enum;

typedef struct
{
    float           x;
    float           y;
    point_type_enum type;
} waypoint_t;

typedef enum
{
    UI_STATE_HOME      = 0,
    UI_STATE_COLLECT   = 1,
    UI_STATE_PREVIEW   = 2,
    UI_STATE_STANDBY   = 3,
    UI_STATE_RUNNING   = 4,
    UI_STATE_DONE      = 5,
} ui_state_enum;

#define S1_MAX_WAYPOINTS  20

typedef struct
{
    waypoint_t       points[S1_MAX_WAYPOINTS];
    uint8            waypoint_count;
    uint8            has_start;
    uint8            has_turn;
    point_type_enum  current_point_type;
    uint8            route_sel;
    float            collect_heading_ref;
} subject1_task_data_t;

typedef struct
{
    int16  start_idx;
    int16  turn_idx;
    int16  cone_indices[S1_MAX_WAYPOINTS];
    uint8  cone_count;
    float  ret_cone_x[S1_MAX_WAYPOINTS];
    float  ret_cone_y[S1_MAX_WAYPOINTS];
    float  routeA_x[S1_MAX_WAYPOINTS * 2 + 4];
    float  routeA_y[S1_MAX_WAYPOINTS * 2 + 4];
    uint8  routeA_count;
    float  routeB_x[S1_MAX_WAYPOINTS * 2 + 4];
    float  routeB_y[S1_MAX_WAYPOINTS * 2 + 4];
    uint8  routeB_count;
} subject1_preview_data_t;

typedef struct
{
    ui_state_enum           *ui_state;
    subject1_task_data_t    *task_data;
    subject1_preview_data_t *preview_data;
} subject1_ui_context_t;

extern ui_state_enum           s1_ui_state;
extern subject1_task_data_t    s1_task_data;
extern subject1_preview_data_t s1_preview_data;
extern uint8                   selected_subject;
extern const subject1_ui_context_t g_subject1_ui_ctx;

void subject1_init(void);
void subject1_poll(void);

#endif /* _SUBJECT1_H_ */
