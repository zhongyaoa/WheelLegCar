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
    balance_cascade_init();
    imu660ra_init();
    steer_control_init();   

    pit_ms_init(PIT_CH0, 1);           // 1ms 定时器，触发 pit_call_back

    uint32 btn_poll_tick = 0;
    while(true)
    {

        system_delay_ms(10);
        //printf("%d,%d,%d,%d,%d,%d\n", imu660ra_acc_x, imu660ra_acc_y, imu660ra_acc_z, imu660ra_gyro_x, imu660ra_gyro_y, imu660ra_gyro_z);
        printf("%f\n", roll_balance_cascade.posture_value.yaw);
    }
        
        
}

// **************************** 代码区域 ****************************
