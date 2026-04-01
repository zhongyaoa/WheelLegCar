#include "zf_common_headfile.h"

// 打开新的工程或者工程移动了位置务必执行以下操作
// 第一步 关闭上面所有打开的文件
// 第二步 project->clean  等待下方进度条走完

/* 
舵机端口占用无刷电机2的P8_2引脚
*/


/*中断通道占用说明：
pit_chan10      zrun_test_balance

*/


int main(void){
    clock_init(SYSTEM_CLOCK_250M);                     // 时钟初始化
    debug_init();                                      // debug 串口初始化
    //测试函数
    //zrun_test_gps();
    //zrun_test_display();
    //zrun_test_led();
    //zrun_test_servo();
    //zrun_test_buzzer();
    //zrun_test_controler();
    //zrun_test_balance();
    //zrun_test_wireless_uart();
    zrun_test_balance_wireless_uart();
    //zrun_test_airprintf();
    //zrun_test_cam();
    //zrun_test_wifi();
    //zrun_test_motor_read_speed();
    return 0;
}
