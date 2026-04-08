/*
imu660ra数据与车辆运动关系
车辆左转时，gyro_z 为负，右转时为正
车辆前倾时，gyro_x 为正，后倾时为负
车辆右倾时，gyro_y 为正，左倾时为负
车辆往前加速时，acc_y 为正，往回加速时为负
静止时，acc_z 为-4100
*/

#include "zf_common_headfile.h"
#include "controler.h"
#include "ins_tracker.h"
#include "subject1_ui.h"
// 打开新的工程或者工程移动了位置务必执行以下操作
// 第一步 关闭上面所有打开的文件
// 第二步 project->clean  等待下方进度条走完

// **************************** 代码区域 ****************************

int main(void)
{
    clock_init(SYSTEM_CLOCK_250M);      // 时钟配置及系统初始化<务必保留>
    debug_init();                       // 调试串口信息初始化
    wireless_uart_init();
    

    balance_cascade_init();             // 串级平衡控制初始化
    small_driver_uart_init();           // 小车控制串口初始化
    imu660ra_init();
    steer_control_init();               // 舵机控制初始化

    button_init();                      // 按键初始化
    led_init();                         // LED 初始化
    led(off);
    ins_tracker_init();                 // 惯性导航循迹模块初始化

    pit_ms_init(PIT_CH0, 1);           // 1ms 定时器，触发 pit_call_back
    subject1_ui_init();                 // 科目一 UI 初始化（含屏幕初始化）       
    wireless_printf("\r\n UI init ok. [UP]=Enter Subject 1\r\n");

    uint32 btn_poll_tick = 0;
    while(true)
    {
        // 每 50ms 轮询一次 UI 按键与循迹更新
        btn_poll_tick++;
        if(btn_poll_tick >= 5)
        {
            btn_poll_tick = 0;
            // UI 轮询（处理按键 + 屏幕刷新）
            subject1_ui_poll();
            // 注意：ins_tracker_nav_update() / ins_tracker_ctrl_update() 已迁移至
            // pit_call_back()（分别每10ms/5ms调用），此处无需再调用
        }

        system_delay_ms(10);
    }
        
        
}

// **************************** 代码区域 ****************************
