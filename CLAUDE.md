# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is an embedded firmware project for a **wheel-leg balancing robot** (轮腿平衡小车) based on the **CYT4BB7** microcontroller (Infineon/Cypress Traveo T2G). The codebase is built on the SEEKFREE (逐飞科技) open-source CYT4BB library and is developed using **IAR Embedded Workbench 9.40.1**.

**Hardware:**
- MCU: CYT4BB7 (dual-core Arm Cortex-M7)
- IMU: IMU660RA (gyro + accelerometer via SPI)
- Motors: Brushless motors driven via UART (460800 baud, UART4)
- Servos: 4× PWM servos (300 Hz, TCPWM channels) for leg joints
- Camera: 总钻风 (TSL1401 line scan)
- Wireless: BLE/UART module on UART1 (115200 baud) for parameter tuning

## Build System

There is **no Makefile or CMake**. Building is done exclusively through IAR Embedded Workbench:

- Open `project/iar/cyt4bb7.eww` in IAR Embedded Workbench 9.40.1
- **After opening a project or moving it**: Project → Clean, then rebuild
- The workspace contains two cores: CM7_0 (main control) and CM7_1

To delete temporary IAR build files: run `project/iar/删除临时文件IAR.bat`

There are no automated tests, linters, or CI pipelines in this repository.

## Architecture

### Dual-Core Execution

- **CM7_0** (`project/user/main_cm7_0.c`): Main control core. Runs the balance control loop, servo control, motor control, and IMU reading.
- **CM7_1** (`project/user/main_cm7_1.c`): Secondary core (currently minimal use).
- Interrupts for CM7_0 are defined in `project/user/cm7_0_isr.c`.

### Control Flow (CM7_0)

```
main_cm7_0.c
├── balance_cascade_init()    → initializes cascade PID + quaternion (balance_control)
├── small_driver_uart_init()  → UART4 to brushless motor driver
├── imu660ra_init()           → SPI IMU
├── steer_control_init()      → 4× PWM servos
└── pit_ms_init(PIT_CH0, 1)  → 1ms timer → pit0_ch0_isr() → pit_call_back()

pit_call_back() [1ms periodic, posture_control.c]:
├── imu660ra_get_gyro/acc()
├── quaternion_module_calculate()   → updates roll/pitch/yaw (balance_control)
├── Every 5ms:  pid_control(angle_cycle)
├── Every 20ms: pid_control(speed_cycle)
├── Every 1ms:  pid_control(angular_speed_cycle)
├── car_state_calculate()           → fall detection, PID ramp-up
├── car_steer_control()             → leg servo positions + jump sequence
└── car_motor_control()             → motor duty + yaw lock PD
```

### Key Modules (`project/code/`)

| File | Purpose |
|------|---------|
| `balance_control.c/h` | Quaternion attitude estimation (Mahony filter), cascade PID structures (`cascade_value_struct`), PID algorithms |
| `posture_control.c/h` | 1ms timer callback orchestrating all control loops; fall detection; motor/servo coordination |
| `steer_control.c/h` | 4-servo leg joint control via PWM; jump sequence state machine |
| `small_driver_uart_control.c/h` | UART protocol to brushless motor driver; speed feedback parsing; `small_driver_set_duty()` |
| `controler.c/h` | Buttons, LED, and wireless UART parameter tuner (`serial_optimizer_callback`) — **cannot be added to `zf_common_headfile.h`** due to include dependency issues |

### Cascade PID Structure

Roll balance uses a 3-loop cascade:
1. **Speed loop** (`speed_cycle`): every 20ms, target speed → output feeds angle loop target
2. **Angle loop** (`angle_cycle`): every 5ms, roll angle → output feeds angular speed loop target
3. **Angular speed loop** (`angular_speed_cycle`): every 1ms, gyro_x → motor duty output

Pitch is single-loop (angle_cycle only) driving servo leg extension.

Yaw is controlled by a PD controller integrating gyro_z to maintain heading lock.

### Library Layers (`libraries/`)

```
libraries/
├── sdk/          # Infineon CYT4BB official SDK (cy_project.h, etc.)
├── zf_common/    # SEEKFREE common layer (typedef, clock, debug, FIFO, interrupts)
├── zf_driver/    # Peripheral drivers (UART, SPI, PWM/TCPWM, GPIO, ADC, DMA, PIT, encoder)
├── zf_device/    # External device drivers (IMU660RA, GNSS, BLE, cameras, WiFi, OLED)
├── zf_components/# SEEKFREE assistant (Bluetooth parameter tuning protocol)
└── zf_device/zf_device_config.h  # Device enable/disable configuration
```

### Adding User Headers

New user code headers must be added to `libraries/zf_common/zf_common_headfile.h` — this is the single include used across all user files. **Exception:** `controler.h` cannot be added there (circular dependency); include it directly where needed.

### UART Assignments

| UART | Purpose |
|------|---------|
| UART0 | Debug serial (default) |
| UART1 | Wireless BLE module / parameter optimizer (115200) |
| UART2 | GNSS/GPS module |
| UART4 | Brushless motor driver (460800) |

### Motor/Servo Sign Conventions

- Left motor duty is **negated** when calling `small_driver_set_duty(-left, right)` — forward motion uses negative left, positive right.
- Servo PWM range: 2400–5400; center ~4450–4850 depending on servo; direction sign set per servo in `steer_control.h`.
