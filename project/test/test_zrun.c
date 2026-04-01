#include "test_zrun.h"
#include "controler.h"
#include "posture_control.h"
#include "servo.h"

// **************************** 变量定义区域 ****************************
//****************摄像头参数设置*************/
#define INCLUDE_BOUNDARY_TYPE   3

// 边界的点数量远大于图像高度，便于保存回弯的情况
#define BOUNDARY_NUM            (MT9V03X_H * 3 / 2)

uint8 xy_x1_boundary[BOUNDARY_NUM], xy_x2_boundary[BOUNDARY_NUM], xy_x3_boundary[BOUNDARY_NUM];
uint8 xy_y1_boundary[BOUNDARY_NUM], xy_y2_boundary[BOUNDARY_NUM], xy_y3_boundary[BOUNDARY_NUM];

uint8 x1_boundary[MT9V03X_H], x2_boundary[MT9V03X_H], x3_boundary[MT9V03X_H];
uint8 y1_boundary[MT9V03X_W], y2_boundary[MT9V03X_W], y3_boundary[MT9V03X_W];

// 图像备份数组，在发送前将图像备份再进行发送，这样可以避免图像出现撕裂的问题
uint8 image_copy[MT9V03X_H][MT9V03X_W];
#define LED1                    (P19_0)
// **************************** test_wireless_uart ****************************
#define LED1                    (P19_0)
uint8 data_buffer[32];
uint8 data_len;
// **************************** test_buzzer ****************************
#define BUZZER_PIN              (P19_4)
// **************************** test_display ****************************
#define IPS200_TYPE     (IPS200_TYPE_SPI)                                       // 八位并口两寸屏 这里宏定义填写 IPS200_TYPE_PARALLEL8 SPI 串口两寸屏 这里宏定义填写 IPS200_TYPE_SPI

void zrun_test_gps(void){
    gnss_init(TAU1201);
    wireless_uart_init(); //无线串口初始化 用于输出 gnss 数据
    air_printf("dis:%f\r\n", get_two_points_distance(45.714935, 126.626564, 45.714855, 126.626960));        // 计算两点距离示例
    air_printf("dir:%f\r\n", get_two_points_azimuth(45.714935, 126.626564, 45.714855, 126.626960));         // 计算两点夹角示例
    
    // 此处编写用户代码 例如外设初始化代码等
    while(true)
    {
        // 此处编写需要循环执行的代码

        //gnss数据接收与解析都是通过串口中断调用gnss_uart_callback函数进行实现的
        //数据解析完毕之后gnss_flag标志位会置1
        if(gnss_flag)
        {
            gnss_flag = 0;

            gnss_data_parse();           //开始解析数据

            printf("now time:\r\n");                                            // 输出年月日时分秒
            printf("year-%d, month-%d, day-%d\r\n", gnss.time.year, gnss.time.month, gnss.time.day);           // 输出年月日时分秒
            printf("hour-%d, minute-%d, second-%d\r\n", gnss.time.hour, gnss.time.minute, gnss.time.second);   // 输出年月日时分秒
            printf("gnss_state       = %d\r\n", gnss.state);              //输出当前定位有效模式 1：定位有效  0：定位无效
            air_printf("latitude        = %f\r\n", gnss.latitude);           //输出纬度信息
            air_printf("longitude       = %f\r\n", gnss.longitude);          //输出经度信息
            printf("speed           = %f\r\n", gnss.speed);              //输出速度信息
            printf("direction       = %f\r\n", gnss.direction);          //输出方向信息
            printf("satellite_used  = %d\r\n", gnss.satellite_used);     //输出当前用于定位的卫星数量
            printf("height          = %f\r\n", gnss.height);             //输出当前gnss天线所处高度
        }
        system_delay_ms(1000);//这里延时主要目的是为了降低输出速度，便于在 逐飞助手 中观察数据，实际使用的时候不需要这样去延时

        // 此处编写需要循环执行的代码
    }
}

void zrun_test_display(void){
     uint16_t data[128];
    int16_t data_index = 0;
    for( ; 64 > data_index; data_index ++)
        data[data_index] = data_index;
    for(data_index = 64; 128 > data_index; data_index ++)
        data[data_index] = 128 - data_index;

    ips200_set_dir(IPS200_PORTAIT);
    ips200_set_color(RGB565_RED, RGB565_BLACK);
    ips200_init(IPS200_TYPE);

    // 此处编写用户代码 例如外设初始化代码等
    while(true)
    {
        // 此处编写需要循环执行的代码

        ips200_clear();
        ips200_show_rgb565_image(0, 120, (const uint16 *)gImage_seekfree_logo, 240, 80, 240, 80, 0);    // 显示一个RGB565色彩图片 原图240*80 显示240*80 低位在前
        system_delay_ms(1500);

        ips200_full(RGB565_GRAY);
        ips200_show_string( 0 , 16*7,   "SEEKFREE");                            // 显示字符串
        ips200_show_chinese(80, 16*7,   16, (const uint8 *)chinese_test, 4, RGB565_BLUE);               // 显示汉字

        // 显示的 flaot 数据 最多显示 8bit 位整数 最多显示 6bit 位小数
        ips200_show_float(  0 , 16*8,   -13.141592,     1, 6);                  // 显示 float 数据 1bit 整数 6bit 小数 应当显示 -3.141592 总共会有 9 个字符的显示占位
        ips200_show_float(  80, 16*8,   13.141592,      8, 4);                  // 显示 float 数据 8bit 整数 4bit 小数 应当显示 13.1415 总共会有 14 个字符的显示占位 后面会有 5 个字符的空白占位

        ips200_show_int(    0 , 16*9,   -127,           2);                     // 显示 int8 数据
        ips200_show_uint(   80, 16*9,   255,            4);                     // 显示 uint8 数据

        ips200_show_int(    0 , 16*10,  -32768,         4);                     // 显示 int16 数据
        ips200_show_uint(   80, 16*10,  65535,          6);                     // 显示 uint16 数据

        ips200_show_int(    0 , 16*11,  -2147483648,    8);                     // 显示 int32 数据 8bit 整数 应当显示 -47483648
        ips200_show_uint(   80, 16*11,  4294967295,     8);                     // 显示 uint32 数据 10bit 整数 应当显示 4294967295

        system_delay_ms(1000);

        ips200_full(RGB565_GRAY);
        ips200_show_wave(88, 144, data, 128, 64,  64, 32);                      // 显示一个三角波形 波形宽度 128 波形最大值 64 显示宽度 64 显示最大值 32
        system_delay_ms(1000);
        ips200_full(RGB565_GRAY);
        ips200_show_wave(56, 128, data, 128, 64, 128, 64);                      // 显示一个三角波形 波形宽度 128 波形最大值 64 显示宽度 128 显示最大值 64
        system_delay_ms(1000);

        // 使用画线函数 从顶上两个角画射线
        ips200_clear();
        for(data_index = 0; 240 > data_index; data_index += 10)
        {
            ips200_draw_line(0, 0, data_index, 320 - 1, RGB565_66CCFF);
            system_delay_ms(20);
        }
        ips200_draw_line(0, 0, 240 - 1, 320 - 1, RGB565_66CCFF);
        for(data_index = 310; 0 <= data_index; data_index -= 10)
        {
            ips200_draw_line(0, 0, 240 - 1, data_index, RGB565_66CCFF);
            system_delay_ms(20);
        }

        ips200_draw_line(240 - 1, 0, 239, 320 - 1, RGB565_66CCFF);
        for(data_index = 230; 0 <= data_index; data_index -= 10)
        {
            ips200_draw_line(240 - 1, 0, data_index, 320 - 1, RGB565_66CCFF);
            system_delay_ms(20);
        }
        ips200_draw_line(240 - 1, 0, 0, 320 - 1, RGB565_66CCFF);
        for(data_index = 310; 0 <= data_index; data_index -= 10)
        {
            ips200_draw_line(240 - 1, 0, 0, data_index, RGB565_66CCFF);
            system_delay_ms(20);
        }
        system_delay_ms(1000);

        ips200_full(RGB565_RED);
        system_delay_ms(500);
        ips200_full(RGB565_GREEN);
        system_delay_ms(500);
        ips200_full(RGB565_BLUE);
        system_delay_ms(500);
        ips200_full(RGB565_WHITE);
        system_delay_ms(500);

        // 此处编写需要循环执行的代码
    }
}

void zrun_test_buzzer(void){
    uint16 count = 0;

    gpio_init(BUZZER_PIN, GPO, GPIO_LOW, GPO_PUSH_PULL);                        // 初始化 BUZZER_PIN 输出 默认低电平 推挽输出模式
    
    
    // 此处编写用户代码 例如外设初始化代码等
    while(true)
    {
        // 此处编写需要循环执行的代码


        if(count < 10)
            gpio_toggle_level(BUZZER_PIN);
        else if(count < 20)
            gpio_set_level(BUZZER_PIN, GPIO_LOW);
        else
            count = 0;
        count ++;
        system_delay_ms(100);
      
      
        // 此处编写需要循环执行的代码
    }
}

void zrun_test_wireless_uart(void)
{
    clock_init(SYSTEM_CLOCK_250M); 	// 时钟配置及系统初始化<务必保留>
    debug_init();                          // 调试串口信息初始化
    
    // 此处编写用户代码 例如外设初始化代码等
    uint8 count = 0;

    gpio_init(LED1, GPO, GPIO_HIGH, GPO_PUSH_PULL);                             // 初始化 LED1 输出 默认高电平 推挽输出模式
    if(wireless_uart_init())                                                    // 判断是否通过初始化
    {
        while(1)                                                                // 初始化失败就在这进入死循环
        {
            gpio_toggle_level(LED1);                                            // 翻转 LED 引脚输出电平 控制 LED 亮灭
            system_delay_ms(100);                                               // 短延时快速闪灯表示异常
        }
    }
    wireless_uart_send_byte('\r');
    wireless_uart_send_byte('\n');
    wireless_uart_send_string("SEEKFREE wireless uart demo.\r\n");              // 初始化正常 输出测试信息

    // 此处编写用户代码 例如外设初始化代码等
    while(true)
    {
        // 此处编写需要循环执行的代码

        data_len = (uint8)wireless_uart_read_buffer(data_buffer, 32);             // 查看是否有消息 默认缓冲区是 WIRELESS_UART_BUFFER_SIZE 总共 64 字节
        if(data_len != 0)                                                       // 收到了消息 读取函数会返回实际读取到的数据个数
        {
            wireless_uart_send_buffer(data_buffer, data_len);                     // 将收到的消息发送回去
            memset(data_buffer, 0, 32);
            func_uint_to_str((char *)data_buffer, data_len);
            wireless_uart_send_string("\r\ndata len:");                                 // 显示实际收到的数据信息
            wireless_uart_send_buffer(data_buffer, strlen((const char *)data_buffer));    // 显示收到的数据个数
            wireless_uart_send_string(".\r\n");
        }
        if(count ++ > 10)
        {
            count = 0;
            gpio_toggle_level(LED1);                                            // 翻转 LED 引脚输出电平 控制 LED 亮灭
        }
        system_delay_ms(50);

        // 此处编写需要循环执行的代码
    } 
}

void zrun_test_cam(void){
    // 设置逐飞助手使用DEBUG串口进行收发
    seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_DEBUG_UART);
    // 此处编写用户代码 例如外设初始化代码等

    gpio_init(LED1, GPO, GPIO_HIGH, GPO_PUSH_PULL);                             // 初始化 LED1 输出 默认高电平 推挽输出模式

#if(0 != INCLUDE_BOUNDARY_TYPE)
    int32 i=0;
#endif

#if(3 <= INCLUDE_BOUNDARY_TYPE)
    int32 j=0;
#endif

    // 此处编写用户代码 例如外设初始化代码等
    while(1)
    {
        if(mt9v03x_init())
            gpio_toggle_level(LED1);                                            // 翻转 LED 引脚输出电平 控制 LED 亮灭 初始化出错这个灯会闪的很慢
        else
            break;
        system_delay_ms(500);                                                   // 闪灯表示异常
    }

#if(0 == INCLUDE_BOUNDARY_TYPE)
    // 发送总钻风图像信息(仅包含原始图像信息)
    seekfree_assistant_camera_information_config(SEEKFREE_ASSISTANT_MT9V03X, image_copy[0], MT9V03X_W, MT9V03X_H);

#elif(1 == INCLUDE_BOUNDARY_TYPE)
    // 发送总钻风图像信息(并且包含三条边界信息，边界信息只含有横轴坐标，纵轴坐标由图像高度得到，意味着每个边界在一行中只会有一个点)
    // 对边界数组写入数据
    for(i = 0; i < MT9V03X_H; i++)
    {
        x1_boundary[i] = 70 - (70 - 20) * i / MT9V03X_H;
        x2_boundary[i] = MT9V03X_W / 2;
        x3_boundary[i] = 118 + (168 - 118) * i / MT9V03X_H;
    }
    seekfree_assistant_camera_information_config(SEEKFREE_ASSISTANT_MT9V03X, image_copy[0], MT9V03X_W, MT9V03X_H);
    seekfree_assistant_camera_boundary_config(X_BOUNDARY, MT9V03X_H, x1_boundary, x2_boundary, x3_boundary, NULL, NULL ,NULL);


#elif(2 == INCLUDE_BOUNDARY_TYPE)
    // 发送总钻风图像信息(并且包含三条边界信息，边界信息只含有纵轴坐标，横轴坐标由图像宽度得到，意味着每个边界在一列中只会有一个点)
    // 通常很少有这样的使用需求
    // 对边界数组写入数据
    for(i = 0; i < MT9V03X_W; i++)
    {
        y1_boundary[i] = i * MT9V03X_H / MT9V03X_W;
        y2_boundary[i] = MT9V03X_H / 2;
        y3_boundary[i] = (MT9V03X_W - i) * MT9V03X_H / MT9V03X_W;
    }
    seekfree_assistant_camera_information_config(SEEKFREE_ASSISTANT_MT9V03X, image_copy[0], MT9V03X_W, MT9V03X_H);
    seekfree_assistant_camera_boundary_config(Y_BOUNDARY, MT9V03X_W, NULL, NULL ,NULL, y1_boundary, y2_boundary, y3_boundary);


#elif(3 == INCLUDE_BOUNDARY_TYPE)
    // 发送总钻风图像信息(并且包含三条边界信息，边界信息含有横纵轴坐标)
    // 这样的方式可以实现对于有回弯的边界显示
    j = 0;
    for(i = MT9V03X_H - 1; i >= MT9V03X_H / 2; i--)
    {
        // 直线部分
        xy_x1_boundary[j] = 34;
        xy_y1_boundary[j] = (uint8)i;

        xy_x2_boundary[j] = 47;
        xy_y2_boundary[j] = (uint8)i;

        xy_x3_boundary[j] = 60;
        xy_y3_boundary[j] = (uint8)i;
        j++;
    }

    for(i = MT9V03X_H / 2 - 1; i >= 0; i--)
    {
        // 直线连接弯道部分
        xy_x1_boundary[j] = 34 + (MT9V03X_H / 2 - i) * (MT9V03X_W / 2 - 34) / (MT9V03X_H / 2);
        xy_y1_boundary[j] = (uint8)i;

        xy_x2_boundary[j] = 47 + (MT9V03X_H / 2 - i) * (MT9V03X_W / 2 - 47) / (MT9V03X_H / 2);
        xy_y2_boundary[j] = 15 + i * 3 / 4;

        xy_x3_boundary[j] = 60 + (MT9V03X_H / 2 - i) * (MT9V03X_W / 2 - 60) / (MT9V03X_H / 2);
        xy_y3_boundary[j] = 30 + i / 2;
        j++;
    }

    for(i = 0; i < MT9V03X_H / 2; i++)
    {
        // 回弯部分
        xy_x1_boundary[j] = MT9V03X_W / 2 + i * (138 - MT9V03X_W / 2) / (MT9V03X_H / 2);
        xy_y1_boundary[j] = (uint8)i;

        xy_x2_boundary[j] = MT9V03X_W / 2 + i * (133 - MT9V03X_W / 2) / (MT9V03X_H / 2);
        xy_y2_boundary[j] = 15 + i * 3 / 4;

        xy_x3_boundary[j] = MT9V03X_W / 2 + i * (128 - MT9V03X_W / 2) / (MT9V03X_H / 2);
        xy_y3_boundary[j] = 30 + i / 2;
        j++;
    }
    seekfree_assistant_camera_information_config(SEEKFREE_ASSISTANT_MT9V03X, image_copy[0], MT9V03X_W, MT9V03X_H);
    seekfree_assistant_camera_boundary_config(XY_BOUNDARY, BOUNDARY_NUM, xy_x1_boundary, xy_x2_boundary, xy_x3_boundary, xy_y1_boundary, xy_y2_boundary, xy_y3_boundary);


#elif(4 == INCLUDE_BOUNDARY_TYPE)
    // 发送总钻风图像信息(并且包含三条边界信息，边界信息只含有横轴坐标，纵轴坐标由图像高度得到，意味着每个边界在一行中只会有一个点)
    // 对边界数组写入数据
    for(i = 0; i < MT9V03X_H; i++)
    {
        x1_boundary[i] = 70 - (70 - 20) * i / MT9V03X_H;
        x2_boundary[i] = MT9V03X_W / 2;
        x3_boundary[i] = 118 + (168 - 118) * i / MT9V03X_H;
    }
    seekfree_assistant_camera_information_config(SEEKFREE_ASSISTANT_MT9V03X, NULL, MT9V03X_W, MT9V03X_H);
    seekfree_assistant_camera_boundary_config(X_BOUNDARY, MT9V03X_H, x1_boundary, x2_boundary, x3_boundary, NULL, NULL ,NULL);


#endif
    
    
    // 此处编写用户代码 例如外设初始化代码等
    while(true)
    {
        // 此处编写需要循环执行的代码

        if(mt9v03x_finish_flag)
        {
            mt9v03x_finish_flag = 0;

            // 在发送前将图像备份再进行发送，这样可以避免图像出现撕裂的问题
            memcpy(image_copy[0], mt9v03x_image[0], MT9V03X_IMAGE_SIZE);

            // 发送图像
            seekfree_assistant_camera_send();
        }
      
      
        // 此处编写需要循环执行的代码
    }
}

void zrun_test_airprintf(void){
    serial_optimizer_init(); //串口初始化
    while(true){
        air_printf("This is a test of air_printf function: %d, %f, %s\r\n", 12345, 3.14159, "Hello, World!");
        printf("This is a test of debug printf function: %d, %f, %s\r\n", 12345, 3.14159, "Hello, World!");
        system_delay_ms(1000);
    }
}

void zrun_test_motor_read_speed(void){
    motor_control_init();               // 电机控制初始化
    while(true){
        small_driver_get_speed();   //右轮的Speed是负的
        printf("left_speed:%d,right_speed:%d\r\n", motor_value.receive_left_speed_data, motor_value.receive_right_speed_data);
        system_delay_ms(500);
    }
}

void zrun_test_controler(void){
    led_init();
    cascade_init();
    serial_optimizer_init(); //串口初始化
    while(true){
        printf("mechanical_offset:%d\r\n", cascade_value.cascade_common_value.mechanical_offset);
        printf("angular_speed_cycle.kp:%f\r\n", cascade_value.angular_speed_cycle.kp);
        printf("angular_speed_cycle.kd:%f\r\n", cascade_value.angular_speed_cycle.kd);
        printf("angle_cycle.kp:%f\r\n", cascade_value.angle_cycle.kp);
        printf("angle_cycle.kd:%f\r\n", cascade_value.angle_cycle.kd);
        system_delay_ms(1000);
    }
}

void zrun_test_servo(void){
    servo_init();
    while(true){
        for(float angle = -90; angle <= 90; angle += 1){
            servo_set_angle(R1, angle);
            servo_set_angle(R2, angle);
            servo_set_angle(L1, angle);
            servo_set_angle(L2, angle);

            system_delay_ms(50);
        }
        for(float angle = 90; angle >= -90; angle -= 10){
            servo_set_angle(R1, angle);
            servo_set_angle(R2, angle);
            servo_set_angle(L1, angle);
            servo_set_angle(L2, angle);
            system_delay_ms(50);
        }
    } 
    
}

void zrun_test_led(void){
    led_init();
    button_init();
    while(true){
        if(button_press(UP)){
            led(on);
        }
        else if(button_press(DOWN)){
            led(off);
        }
        else if(button_press(LEFT)){
            led(toggle);
        }
        system_delay_ms(100);
    }
}

void zrun_test_balance(void)
{
    //clock_init(SYSTEM_CLOCK_250M);                              // 时钟初始化
    //debug_init();                                               // debug 串口初始化
    button_init();
    motor_control_init();               // 电机控制初始化
    cascade_init();                     // 滤波链初始化
    //serial_optimizer_init();           // 串口初始化

    while(1)
    {
        if(imu660ra_init())
        {
           printf("\r\n imu660ra init error.");        // imu660ra 初始化失败
        }
        else
        {
           break;
        }
    }
    pit_ms_init( PIT_CH10, 1 );          // pit通道10初始化，周期中断时间1ms
    while(true){
        //printf("%d,%d,%d,%d,%d,%d,%f\r\n", imu660ra_gyro_x, imu660ra_gyro_y, imu660ra_gyro_z, imu660ra_acc_x, imu660ra_acc_y, imu660ra_acc_z,cascade_value.cascade_common_value.filtered_value);
        air_printf("speed:%d target:%f\r\n", car_speed, target_speed);
        printf("speed:%d target:%f\r\n", car_speed, target_speed);
        //printf("%d,%d\r\n", motor_value.receive_left_speed_data, motor_value.receive_right_speed_data);
        if(button_press(UP)){
            run_flag = true;
            air_printf("UP\r\n");  
        }
        else if(button_press(DOWN)){
            target_speed = 0.0f;
            air_printf("DOWN\r\n");
        }
        else if(button_press(LEFT)){
            target_speed = 50.0f;
            air_printf("LEFT\r\n");
        }
        else if(button_press(NE)){
            target_speed = -50.0f;
            air_printf("NE\r\n");
        }
        system_delay_ms(100); 

    }
}

void zrun_test_balance_wireless_uart(void)
{
    //clock_init(SYSTEM_CLOCK_250M);                              // 时钟初始化
    //debug_init();                                               // debug 串口初始化
    servo_init();
    servo_set_angle(R1, 0);
    servo_set_angle(R2, 0); 
    servo_set_angle(L1, 0);
    servo_set_angle(L2, 0);
    button_init();
    motor_control_init();               // 电机控制初始化
    cascade_init();                     // 滤波链初始化
    wireless_uart_init();                 // 无线串口初始化
    //serial_optimizer_init();           // 串口初始化

    while(1)
    {
        if(imu660ra_init())
        {
           printf("\r\n imu660ra init error.");        // imu660ra 初始化失败
        }
        else
        {
           break;
        }
    }
    pit_ms_init( PIT_CH10, 1 );          // pit通道10初始化，周期中断时间1ms
    while(true){
        //air_printf("%d,%d,%d,%d,%d,%d,%f\r\n", imu660ra_gyro_x, imu660ra_gyro_y, imu660ra_gyro_z, imu660ra_acc_x, imu660ra_acc_y, imu660ra_acc_z,cascade_value.cascade_common_value.filtered_value);
        air_printf("speed:%d target:%f\r\n mechanical_offset:%d", car_speed, target_speed, cascade_value.cascade_common_value.mechanical_offset);
        printf("speed:%d target:%f\r\n", car_speed, target_speed);
        //printf("%d,%d\r\n", motor_value.receive_left_speed_data, motor_value.receive_right_speed_data);
        if(button_press(UP)){
            run_flag = true;
            air_printf("UP\r\n");  
        }
        else if(button_press(DOWN)){
            target_speed = 0.0f;
            air_printf("DOWN\r\n");
        }
        else if(button_press(LEFT)){
            target_speed = 50.0f;
            air_printf("LEFT\r\n");
        }
        else if(button_press(SE)){
            target_speed = -50.0f;
            air_printf("SE\r\n");
        }
        system_delay_ms(100); 

    }
}