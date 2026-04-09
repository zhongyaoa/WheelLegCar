#ifndef _MOTION_MANAGER_H_
#define _MOTION_MANAGER_H_

#include "zf_common_typedef.h"

typedef enum
{
    MOTION_MODE_IDLE = 0,
    MOTION_MODE_STRAIGHT,
    MOTION_MODE_CURVE,
    MOTION_MODE_SPIN,
} motion_mode_enum;

typedef struct
{
    uint8 balance_enabled;
    motion_mode_enum mode;
    float linear_speed;
    int16 turn_diff;
    uint8 jump_request;
    uint8 jump_busy;
} motion_manager_struct;

extern motion_manager_struct motion_manager;

void motion_manager_init(void);
void motion_manager_update(void);

void motion_manager_set_balance_enable(uint8 enable);
void motion_manager_hold(void);
void motion_manager_stop(void);
void motion_manager_emergency_stop(void);

void motion_manager_set_straight(float speed);
void motion_manager_set_curve(float speed, int16 turn_diff);
void motion_manager_set_spin(int16 turn_diff);

uint8 motion_manager_request_jump(void);
uint8 motion_manager_is_jump_busy(void);

#endif
