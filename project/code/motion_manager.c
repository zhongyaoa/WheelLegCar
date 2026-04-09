#include "motion_manager.h"
#include "posture_control.h"

motion_manager_struct motion_manager;

static int16 limit_turn_diff(int16 turn_diff)
{
    return func_limit_ab(turn_diff, -turn_duty_max, turn_duty_max);
}

void motion_manager_init(void)
{
    motion_manager.balance_enabled = 0;
    motion_manager.mode = MOTION_MODE_IDLE;
    motion_manager.linear_speed = 0.0f;
    motion_manager.turn_diff = 0;
    motion_manager.jump_request = 0;
    motion_manager.jump_busy = 0;

    balance_enable = 0;
    target_speed = 0.0f;
    turn_diff_ext = 0;
}

void motion_manager_set_balance_enable(uint8 enable)
{
    motion_manager.balance_enabled = enable ? 1 : 0;
    balance_enable = motion_manager.balance_enabled;
}

void motion_manager_hold(void)
{
    motion_manager.mode = MOTION_MODE_IDLE;
    motion_manager.linear_speed = 0.0f;
    motion_manager.turn_diff = 0;
}

void motion_manager_stop(void)
{
    motion_manager_hold();
}

void motion_manager_emergency_stop(void)
{
    motion_manager_hold();
    motion_manager_set_balance_enable(0);
    motion_manager.jump_request = 0;
    motion_manager.jump_busy = 0;
}

void motion_manager_set_straight(float speed)
{
    motion_manager.mode = MOTION_MODE_STRAIGHT;
    motion_manager.linear_speed = speed;
    motion_manager.turn_diff = 0;
}

void motion_manager_set_curve(float speed, int16 turn_diff)
{
    motion_manager.mode = MOTION_MODE_CURVE;
    motion_manager.linear_speed = speed;
    motion_manager.turn_diff = limit_turn_diff(turn_diff);
}

void motion_manager_set_spin(int16 turn_diff)
{
    motion_manager.mode = MOTION_MODE_SPIN;
    motion_manager.linear_speed = 0.0f;
    motion_manager.turn_diff = limit_turn_diff(turn_diff);
}

uint8 motion_manager_request_jump(void)
{
    if(motion_manager.jump_busy || motion_manager.jump_request || jump_flag)
    {
        return 0;
    }

    motion_manager.jump_request = 1;
    return 1;
}

uint8 motion_manager_is_jump_busy(void)
{
    return motion_manager.jump_busy;
}

void motion_manager_update(void)
{
    motion_manager.jump_busy = (jump_flag != 0);

    if(motion_manager.jump_request && !motion_manager.jump_busy)
    {
        jump_flag = 1;
        jump_time = 0;
        motion_manager.jump_request = 0;
        motion_manager.jump_busy = 1;
    }

    balance_enable = motion_manager.balance_enabled;

    switch(motion_manager.mode)
    {
        case MOTION_MODE_STRAIGHT:
            target_speed = motion_manager.linear_speed;
            turn_diff_ext = 0;
            break;

        case MOTION_MODE_CURVE:
            target_speed = motion_manager.linear_speed;
            turn_diff_ext = limit_turn_diff(motion_manager.turn_diff);
            break;

        case MOTION_MODE_SPIN:
            target_speed = 0.0f;
            turn_diff_ext = limit_turn_diff(motion_manager.turn_diff);
            break;

        case MOTION_MODE_IDLE:
        default:
            target_speed = 0.0f;
            turn_diff_ext = 0;
            break;
    }

    if(!motion_manager.balance_enabled)
    {
        target_speed = 0.0f;
        turn_diff_ext = 0;
    }
}
