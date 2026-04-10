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

## 四、比赛细则
### 科目1：行进绕桩
  这个科目考察车模平台行进特性。 要求车模通过GPS、IMU或者其他导航方式， 在规定的场地内完成直行和绕桩科目。
- 要求出发之后先直行到掉头区， 然后在返回的路径中通过锥桶给定的间隙。
- 有 N 个锥桶， 就会有 N-1 个间隙， 需要完成 N-1 除穿越过程。锥桶排列不一定在一条直线上。
- 两个锥桶之间的最小距离为 1.5米 。

### 科目2：定点排雷
  定点排雷科目考察车模定位特性以及定点旋转功能。
- 场地内预先铺设若干的雷区。 通过白色胶带粘贴成边长为 100厘米 的方框。 白色胶带宽度为 10cm 。
- 白色雷区通常先铺设在蓝色背景布板，或者KT板上， 然后再摆放在比赛现场， 也可以直接在红色操场赛道上进行铺设。
- 车模通过雷区的顺序没有规定。 车模在雷区内旋转的方向没有规定， 但至少旋转两周。
- 车模在雷区旋转过程中， 不允许车轮冲出白色边框。
- 比赛计时是从发车区开始，然后再从掉头区返回到发车区结束。

### 科目3：颠簸路段
  颠簸路段科目考察轮腿车模车身姿态控制以及跳跃功能。

#### 比赛过程
  在比赛场地内设置 交错单边桥、颠簸路段、草地、三级台阶坡道等元素。 车模从出发区出发， 依次通过上述路段 元素， 并在掉头区返回。 通过上述颠簸赛道元素的顺序可以自行选择。 其中三级台阶坡道元素需要正反两个方向各通过一次。

#### 颠簸元素
（1）交错单边桥
  这种路障是针对平衡轮腿组别运动特点设计的。 它是由若干个左右交错摆放的三角楔形路障构成。 楔形路障的高度小于5厘米。 楔形路障的颜色为 黑色。 交错单边桥是超差轮腿车模在运动过程中的腿关节运动来保持车模平衡的功能。
  交错单边桥之间的间隔，限定在 10cm ~ 25cm 之间。 车模在通过时，不能够从旁边绕行。 如果没有压到部署的单边桥， 每遗漏一个， 罚时 10秒钟。
  轮腿车模在通过交错单边桥的时候，不允许跳跃通过。

（2）颠簸路段
  颠簸路段是由之前室内赛道的路肩铺设，路肩的高度 2厘米，宽度为 2.5厘米，每条路肩相距10厘米左右；整个颠簸路段的宽度为1米左右，长度大于等于1米，或者将1米的颠簸路段分成两个50cm长的颠簸路段间隔50cm布置。

（3）草地路段
  草坪区域使用人工草坪铺设在路面，增加车模运行的阻力。

（4）三级台阶和坡道
  其中中间台阶表面为蓝色， 便于轮腿车模能够识别到台阶的边缘。
  比赛的时候，车模需要正反两个方向通过三级台阶两次。 第一次是先通过台阶上到坡道， 然后在通过坡道下来， 第二次先通过坡道行驶到台阶顶部， 然后在跳跃下来台阶。