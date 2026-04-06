/*********************************************************************************************************************
* CYT4BB Opensourec Library 即（ CYT4BB 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
*
* 本文件是 CYT4BB 开源库的一部分
*
* CYT4BB 开源库 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
*
* 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更多细节请参见 GPL
*
* 您应该在收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请参阅<https://www.gnu.org/licenses/>
*
* 额外注明：
* 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
* 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
* 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
* 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
*
* 文件名称          main_cm7_0
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          IAR 9.40.1
* 适用平台          CYT4BB
* 店铺链接          https://seekfree.taobao.com/
*
* 修改记录
* 日期              作者                备注
* 2024-1-4       pudding            first version
********************************************************************************************************************/

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
    wireless_printf("\r\n inav_tracker_init ok.\r\n");
    wireless_printf("[UP]=记录点位（第1次锁定起点+航向）  [LEFT]=开始循迹\r\n");

    pit_ms_init(PIT_CH0, 1);           // 1ms 定时器，触发 pit_call_back

    uint32 btn_poll_tick = 0;
    while(true)
    {
        // 每 50ms 轮询一次按键与循迹更新
        btn_poll_tick++;
        if(btn_poll_tick >= 5)
        {
            btn_poll_tick = 0;
            ins_tracker_button_poll();
            ins_tracker_update();
        }

        system_delay_ms(10);
    }
        
        
}

// **************************** 代码区域 ****************************
