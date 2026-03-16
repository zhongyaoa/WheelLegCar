#include "test_zrun.h"
#include "controler.h"
#include "posture_control.h"

// **************************** 变量定义区域 ****************************
#define SERVO_MOTOR_PWM1            (TCPWM_CH13_P00_3)                          // 定义主板上舵机对应引脚
#define SERVO_MOTOR_PWM2            (TCPWM_CH12_P01_0)                          // 定义主板上舵机对应引脚
#define SERVO_MOTOR_PWM3            (TCPWM_CH11_P01_1)                          // 定义主板上舵机对应引脚
#define SERVO_MOTOR_PWM4            (TCPWM_CH20_P08_1)                          // 主板上的有刷电机2引脚飞线
#define SERVO_MOTOR_FREQ            (50 )                                       // 定义主板上舵机频率  请务必注意范围 50-300

#define SERVO_MOTOR_L_MAX           (50 )                                       // 定义主板上舵机活动范围 角度
#define SERVO_MOTOR_R_MAX           (150)                                       // 定义主板上舵机活动范围 角度

#define SERVO_MOTOR_DUTY(x)         ((float)PWM_DUTY_MAX/(1000.0/(float)SERVO_MOTOR_FREQ)*(0.5+(float)(x)/90.0))

#if (SERVO_MOTOR_FREQ<50 || SERVO_MOTOR_FREQ>300)
    #error "SERVO_MOTOR_FREQ ERROE!"
#endif

float servo_motor_duty = 90.0;                                                  // 舵机动作角度
float servo_motor_dir = 1;                                                      // 舵机动作状态

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
//**************************************/
#define WIFI_SSID_TEST          "SeekFree"
#define WIFI_PASSWORD_TEST      "SEEKFREE"
#define UDP_TARGET_IP           "192.168.137.1"             // 连接目标的 IP
#define UDP_TARGET_PORT         "8086"                      // 连接目标的端口
#define WIFI__LOCAL_PORT        "6666"                      // 本机的端口 0：随机  可设置范围2048-65535  默认 6666

uint8 wifi_spi_test_buffer[] = "this is wifi spi test buffer";
uint8 wifi_spi_get_data_buffer[256];
uint32 data_length = 0;
// **************************** test_wireless_uart ****************************
#define LED1                    (P19_0)
uint8 data_buffer[32];
uint8 data_len;


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

void zrun_test_wifi(void)
{



    while(wifi_spi_init(WIFI_SSID_TEST, WIFI_PASSWORD_TEST))
    {
        printf("\r\n connect wifi failed. \r\n");
        system_delay_ms(100);                                                   // 初始化失败 等待 100ms
    }

    printf("\r\n module version:%s",wifi_spi_version);                          // 模块固件版本
    printf("\r\n module mac    :%s",wifi_spi_mac_addr);                         // 模块 MAC 信息
    printf("\r\n module ip     :%s",wifi_spi_ip_addr_port);                     // 模块 IP 地址

    // zf_device_wifi_spi.h 文件内的宏定义可以更改模块连接(建立) WIFI 之后，是否自动连接 TCP 服务器、创建 UDP 连接
    if(0 == WIFI_SPI_AUTO_CONNECT)                                              // 如果没有开启自动连接 就需要手动连接目标 IP
    {
        while(wifi_spi_socket_connect(                                          // 向指定目标 IP 的端口建立 UDP 连接
            "UDP",                                                              // 指定使用UDP方式通讯
            UDP_TARGET_IP,                                                      // 指定远端的IP地址，填写上位机的IP地址
            UDP_TARGET_PORT,                                                    // 指定远端的端口号，填写上位机的端口号，通常上位机默认是8080
            WIFI__LOCAL_PORT))                                                  // 指定本机的端口号
        {
            // 如果一直建立失败 考虑一下是不是没有接硬件复位
            printf("\r\n Connect UDP Servers error, try again.");
            system_delay_ms(100);                                               // 建立连接失败 等待 100ms
        }
    }


    // 发送测试数据至服务器
    data_length = wifi_spi_send_buffer(wifi_spi_test_buffer, sizeof(wifi_spi_test_buffer));
    wifi_spi_udp_send_now();
    if(!data_length)
    {
        printf("\r\n send success.");
    }
    else
    {
        printf("\r\n %d bytes data send failed.", data_length);
    }

    // 此处编写用户代码 例如外设初始化代码等
    while(true)
    {
        // 此处编写需要循环执行的代码
        data_length = wifi_spi_read_buffer(wifi_spi_get_data_buffer, sizeof(wifi_spi_get_data_buffer));
        if(data_length)                                                         // 如果接收到数据 则进行数据类型判断
        {
            printf("\r\n Get data: <%s>.", wifi_spi_get_data_buffer);
            if(!wifi_spi_send_buffer(wifi_spi_get_data_buffer, data_length))
            {
                wifi_spi_udp_send_now();
                printf("\r\n send success.");
                memset(wifi_spi_get_data_buffer, 0, data_length);          // 数据发送完成 清空数据
            }
            else
            {
                printf("\r\n %d bytes data send failed.", data_length);
            }
        }
        system_delay_ms(100);

        // 此处编写需要循环执行的代码
    }
}

void zrun_test_cam(void){
    seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_CUSTOM);
    led_init();
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
            led(toggle);                                            // 翻转 LED 引脚输出电平 控制 LED 亮灭 初始化出错这个灯会闪的很慢
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
    button_init();
    while(true){
        if(button_press(bt1)){
            break;
        }
        system_delay_ms(10);
    }
    //pwm_init(SERVO_MOTOR_PWM1, SERVO_MOTOR_FREQ, 300);
    //pwm_init(SERVO_MOTOR_PWM2, SERVO_MOTOR_FREQ, 300);
    pwm_init(SERVO_MOTOR_PWM4, SERVO_MOTOR_FREQ, 300);
    while(true)
    {
        // 此处编写需要循环执行的代码

        if(servo_motor_dir)
        {
            servo_motor_duty ++;
            if(servo_motor_duty >= SERVO_MOTOR_R_MAX)
            {
                servo_motor_dir = 0x00;
            }
        }
        else
        {
            servo_motor_duty --;
            if(servo_motor_duty <= SERVO_MOTOR_L_MAX)
            {
                servo_motor_dir = 0x01;
            }
        }
        
        //pwm_set_duty(SERVO_MOTOR_PWM1, (uint16)SERVO_MOTOR_DUTY(servo_motor_duty));
        //pwm_set_duty(SERVO_MOTOR_PWM2, (uint16)SERVO_MOTOR_DUTY(200-servo_motor_duty));
        pwm_set_duty(SERVO_MOTOR_PWM4, (uint16)SERVO_MOTOR_DUTY(200-servo_motor_duty));
        system_delay_ms(15);                                                   // 延时
      
      
      
        // 此处编写需要循环执行的代码a
    }
}

void zrun_test_led(void){
    led_init();
    button_init();
    while(true){
        if(button_press(bt1)){
            led(on);
        }
        else if(button_press(bt2)){
            led(off);
        }
        else if(button_press(bt3)){
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
        if(button_press(bt1)){
            run_flag = true;
            air_printf("bt1\r\n");  
        }
        else if(button_press(bt2)){
            target_speed = 0.0f;
            air_printf("bt2\r\n");
        }
        else if(button_press(bt3)){
            target_speed = 50.0f;
            air_printf("bt3\r\n");
        }
        else if(button_press(bt4)){
            target_speed = -50.0f;
            air_printf("bt4\r\n");
        }
        system_delay_ms(100); 

    }
}

void zrun_test_balance_wireless_uart(void)
{
    //clock_init(SYSTEM_CLOCK_250M);                              // 时钟初始化
    //debug_init();                                               // debug 串口初始化
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
        //printf("%d,%d,%d,%d,%d,%d,%f\r\n", imu660ra_gyro_x, imu660ra_gyro_y, imu660ra_gyro_z, imu660ra_acc_x, imu660ra_acc_y, imu660ra_acc_z,cascade_value.cascade_common_value.filtered_value);
        air_printf("speed:%d target:%f\r\n mechanical_offset:%d", car_speed, target_speed, cascade_value.cascade_common_value.mechanical_offset);
        printf("speed:%d target:%f\r\n", car_speed, target_speed);
        //printf("%d,%d\r\n", motor_value.receive_left_speed_data, motor_value.receive_right_speed_data);
        if(button_press(bt1)){
            run_flag = true;
            air_printf("bt1\r\n");  
        }
        else if(button_press(bt2)){
            target_speed = 0.0f;
            air_printf("bt2\r\n");
        }
        else if(button_press(bt3)){
            target_speed = 50.0f;
            air_printf("bt3\r\n");
        }
        else if(button_press(bt4)){
            target_speed = -50.0f;
            air_printf("bt4\r\n");
        }
        system_delay_ms(100); 

    }
}