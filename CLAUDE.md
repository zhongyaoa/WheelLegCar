# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is an embedded C firmware project for a **wheel-leg balance robot** (轮腿小车) targeting the **CYT4BB7** microcontroller (Infineon/Cypress, dual Cortex-M7 cores). The project is developed using **IAR Embedded Workbench 9.40.1** and is built/flashed entirely through the IAR IDE — there is no command-line build system (no Makefile, CMake, etc.).

## Build & Flash

- Open `project/iar/cyt4bb7.eww` in IAR Embedded Workbench to load the workspace (contains both CM7_0 and CM7_1 projects).
- After moving or reopening the project: close all open files → Project → Clean → wait for completion.
- CM7_0 is the primary core running all control logic. CM7_1 is a secondary core (currently mostly empty).
- To clean temporary build files: run `project/iar/删除临时文件IAR.bat`.

## Repository Structure

```
project/
  user/         # Entry points and ISR handlers (add to IAR project, not here)
    main_cm7_0.c      # CM7_0 main: inits balance, motor, IMU, steer, PIT timer
    main_cm7_1.c      # CM7_1 main: secondary core (mostly unused)
    cm7_0_isr.c       # All interrupt handlers for CM7_0
    cm7_1_isr.c       # ISR handlers for CM7_1
  code/         # All custom control logic goes here (flat, no subdirectories)
    balance_control.c/h       # Quaternion attitude estimation, PID structs
    posture_control.c/h       # Control loop callbacks, motor/steer orchestration
    steer_control.c/h         # 4-servo leg control (PWM-based)
    small_driver_uart_control.c/h  # UART protocol to brushless motor driver
libraries/
  sdk/          # SEEKFREE CYT4BB7 open-source SDK (do not modify)
```

## Architecture & Control Flow

The robot runs a **1ms periodic interrupt** (`pit0_ch0_isr` → `pit_call_back`) that executes the full control pipeline:

1. **IMU data acquisition**: `imu660ra_get_gyro()` + `imu660ra_get_acc()` — raw data from IMU660RA
2. **Attitude estimation**: `quaternion_module_calculate(&roll_balance_cascade)` — Mahony complementary filter fusing gyro + accelerometer into roll/pitch/yaw
3. **Speed loop** (every 20ms): `pid_control(&roll_balance_cascade.speed_cycle, target_speed, car_speed)` — outer loop
4. **Angle loop** (every 5ms): `pid_control(&roll_balance_cascade.angle_cycle, ...)` — middle loop, roll; `pitch_balance_cascade.angle_cycle` for pitch (used for leg servo balance)
5. **Angular velocity loop** (every 1ms): `pid_control(&roll_balance_cascade.angular_speed_cycle, ...)` — inner loop, drives motors
6. **State check**: `car_state_calculate()` — detects fall (|rol| or |pit| > 40°), manages PID soft-start on recovery
7. **Servo control**: `car_steer_control()` — 4-servo V-leg control for height/pitch balance and jump sequences
8. **Motor output**: `car_motor_control()` → `small_driver_set_duty()` via UART4 at 460800 baud

### Cascade PID Structure (`cascade_value_struct`)

Each axis (roll, pitch) has a `cascade_value_struct` containing:
- `quaternion` — Mahony filter state (quaternion, filtered acc, error integrals)
- `posture_value` — roll/pitch/yaw angles, Kp/Ki correction gains, call cycle
- `angular_speed_cycle` — inner PID (gyro → motor duty)
- `angle_cycle` — middle PID (angle → angular velocity target)
- `speed_cycle` — outer PID (wheel speed → angle target)

`roll_balance_cascade` controls fore-aft balance (motors). `pitch_balance_cascade` controls the servo leg pitch. Both have `_resave` backup copies for PID soft-start after recovery from a fall.

### IMU Axis Convention (from README)

- `gyro_z` negative = turning left, positive = turning right
- `gyro_x` positive = forward lean, negative = backward lean
- `acc_z` ≈ -4100 at rest
- `ACC_DATA_Y` and `ACC_DATA_Z` are sign-inverted from raw IMU data (see `balance_control.h` macros)

### Motor UART Protocol (`small_driver_uart_control`)

7-byte packet: `[0xA5][func][d1][d2][d3][d4][checksum]`
- Function `0x01`: set duty (left high/low, right high/low)
- Function `0x02`: request speed feedback (driver replies at 10ms intervals)
- Checksum = sum of bytes 0–5
- UART4 pins: RX=P14_0, TX=P14_1

### Servo Layout (`steer_control`)

4 servos in a V-leg configuration (2 per side, front/rear):
- Steer 1 & 4: dir=+1, Steer 2 & 3: dir=-1
- Center value: 4500 (horizontal); max 5400 = vertical; min 2400
- Jump sequence: extend (2500 offset) → retract → buffer compress (1400) → buffer hold (-14 absolute)

## Adding New Code

Place new `.c`/`.h` files flat in `project/code/` (no subdirectories per SDK convention), then add them to the IAR project file manually.

## Key Hardware Pin Assignments

| Peripheral | Pin |
|---|---|
| Motor UART RX | P14_0 (UART4) |
| Motor UART TX | P14_1 (UART4) |
| Servo 1 PWM | P01_0 (TCPWM_CH12) |
| Servo 2 PWM | P08_2 (TCPWM_CH21) |
| Servo 3 PWM | P00_3 (TCPWM_CH13) |
| Servo 4 PWM | P01_1 (TCPWM_CH11) |
| Button UP | P10_4 |
| Button DOWN | P10_3 |
| Button LEFT | P10_2 |
| Button A | P09_0 |
| Button B | P08_3 |
| GPS UART | UART2 |
| Wireless UART | UART1 |
