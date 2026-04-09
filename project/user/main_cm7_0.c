#include "zf_common_headfile.h"
#include "task_slalom.h"
#include "ui.h"

// **************************** 代码区域 ****************************


int main(void)
{
    clock_init(SYSTEM_CLOCK_250M);      // 时钟配置及系统初始化<务必保留>
    debug_init();                       // 调试串口信息初始化
    // 此处编写用户代码 例如外设初始化代码等
    balance_cascade_init();             // 串级平衡控制初始化
    small_driver_uart_init();           // 小车控制串口初始化
    imu660ra_init();
    steer_control_init();               // 舵机控制初始化
    ui_init();                          // 屏幕 UI 初始化
    task_slalom_init();                 // 科目一绕桩导航初始化（含 GPS 初始化）
    pit_ms_init(PIT_CH0, 1);
    wireless_uart_init();

    // 此处编写用户代码 例如外设初始化代码等
    uint8 ui_tick = 0;
    while(true)
    {
        task_slalom_update();           // 科目一导航任务（20ms 周期）

        ui_tick++;
        if(ui_tick >= UI_REFRESH_DIVIDER)   // 每 100ms 刷新一次屏幕
        {
            ui_tick = 0;
            ui_update();
        }

        system_delay_ms(20);
    }
}
