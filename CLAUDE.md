# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is an embedded C firmware project for a wheel-leg robot (轮腿小车) based on the **CYT4BB7** dual-core Cortex-M7 microcontroller, using the [Seekfree (逐飞科技) open-source library](https://gitee.com/seekfree/CYT4BB7_Library/tree/master). The robot uses brushless motors (UART-controlled) and servo motors (PWM) for wheel-leg locomotion, with IMU-based balance control.

## Build System

The project uses **IAR Embedded Workbench 9.40.1**. The workspace file is `project/iar/cyt4bb7.eww`.

- **Build:** Open `project/iar/cyt4bb7.eww` in IAR and use Project → Make (F7)
- **Clean:** Project → Clean (required after moving the project or opening it for the first time — close all open files first, then run Clean and wait for the progress bar)
- **Delete temp files:** Run `project/iar/删除临时文件IAR.bat` to clean build artifacts

There is no command-line build system; all compilation is done through IAR.

## Architecture

### Dual-Core Structure

The CYT4BB7 has two CM7 cores:
- **CM7_0** (`project/user/main_cm7_0.c`): Main application core. Entry point calls a single `zrun_test_*` function (test-driven development pattern — comment/uncomment tests to switch modes). Interrupts are in `project/user/cm7_0_isr.c`.
- **CM7_1** (`project/user/main_cm7_1.c`): Secondary core, currently unused (empty loop).

### Library Layers (do not modify)

`libraries/zf_common/zf_common_headfile.h` is the single include for all library headers — every source file in the project includes this. Layers:
1. `libraries/sdk/` — Cypress official chip SDK
2. `libraries/zf_common/` — Seekfree common utilities (types, clock, debug, FIFO, math)
3. `libraries/zf_driver/` — Peripheral drivers (GPIO, UART, PWM, ADC, SPI, I2C, PIT timers, encoders)
4. `libraries/zf_device/` — External device drivers (IMU660RA, MT9V03X camera, wireless UART, GNSS, displays)
5. `libraries/zf_components/` — Seekfree Assistant visualization tool

### User Code (`project/code/`)

All new user headers **must** be added to `libraries/zf_common/zf_common_headfile.h` (in the "code里面自己加的" section) to be accessible project-wide — except `controler.h`, which has a dependency conflict and must be included directly.

| File | Role |
|---|---|
| `posture_control.c/h` | Balance control: IMU data acquisition, first-order complementary filter, cascade PD control (speed → angle → angular rate) |
| `servo.c/h` | 4-servo leg control (L1, L2, R1, R2) via PWM; angle convention: 0° = neutral, positive = up |
| `small_driver_uart_control.c/h` | Brushless motor driver over UART4 at 460800 baud; provides `small_driver_set_duty()` and speed feedback |
| `controler.c/h` | Button/LED GPIO, wireless serial parameter tuning (`serial_optimizer_callback`), `air_printf` over wireless UART |
| `test_zrun.c/h` | All test/demo entry functions (e.g., `zrun_test_balance_wireless_uart`) called from `main_cm7_0.c` |

### Control Loop

`pit_isr_callback()` fires every 1ms via PIT channel 10 (`pit0_ch10_isr` in `cm7_0_isr.c`):
- Every 1ms: IMU read + angular rate PD loop + motor output
- Every 5ms: Complementary filter + angle PD loop
- Every 20ms: Motor speed read + speed PD loop

Cascade order: speed setpoint → `speed_cycle` → `angle_cycle` setpoint → `angle_cycle` → `angular_speed_cycle` setpoint → `angular_speed_cycle` → `small_driver_set_duty()`

### UART Assignments

| UART | Use |
|---|---|
| UART_0 | Debug serial (printf) |
| UART_1 | Wireless UART module (BLE6A20) — `air_printf`, `my_wireless_optimizer` |
| UART_2 | GNSS module |
| UART_4 | Brushless motor driver (460800 baud) |

### Wireless Control Commands

Characters received over wireless UART (`my_wireless_optimizer`):
- `w` / `s`: Forward / reverse (target_speed ±50)
- `a` / `d`: Start / stop balance control (`run_flag`)
- `j` / `k`: Increase / decrease `mechanical_offset` by 1
- `!`: Stop (target_speed = 0)

### Serial Parameter Tuning Protocol

`serial_optimizer_callback` accepts 8-byte frames (`0x55 0x20 channel reserved float[4]`) to tune PID parameters at runtime:
- Ch 1: `mechanical_offset`, Ch 2/3: angular rate kp/kd, Ch 4/5: angle kp/kd, Ch 6: speed kp

## Key Constants to Tune

- `cascade_value.cascade_common_value.mechanical_offset` (default 545): Physical balance point of the robot
- IMU zero-bias corrections in `imu_data_get()`: gyro_x −4, gyro_y +6
- PID gains in `cascade_init()`: angular rate kp=0.5, angle kp=10.4, speed kp=2.0
- Servo angle convention in `servo_set_angle()`: L1/R2 channels are sign-inverted relative to L2/R1
