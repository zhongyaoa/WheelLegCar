#include "posture_control.h"

uint32 system_count = 0;//系统计数器
bool run_flag = false;
cascade_value_struct cascade_value;
int16 car_speed = 0;
float target_speed = 0.0f; // 目标速度，可以通过串口调参修改        

//******串级控制器初始化 */
void cascade_init(void){
    //一阶互补滤波
    cascade_value.cascade_common_value.gyro_raw_data_l = &imu660ra_gyro_x;
    cascade_value.cascade_common_value.acc_raw_data_l = &imu660ra_acc_y;
    cascade_value.cascade_common_value.gyro_ration = -4;   // 角速度置信度
    cascade_value.cascade_common_value.acc_ration = 4;    // 加速度置信度
    cascade_value.cascade_common_value.filtered_value = 0;  // 互补滤波后的值
    cascade_value.cascade_common_value.dt = 0.005f;          // 采样时间间隔
    cascade_value.cascade_common_value.mechanical_offset = 545;
    /*机械偏置：
    舵机角度    机械偏置
    0           600

    */

    //角速度闭环控制结构体
    cascade_value.angular_speed_cycle.kp = 0.5f;//0.43f
    cascade_value.angular_speed_cycle.ki = 0.0f;
    cascade_value.angular_speed_cycle.kd = 0.0f;

    //角度闭环控制结构体
    cascade_value.angle_cycle.kp = 10.4f;//10.4f
    cascade_value.angle_cycle.ki = 0.0f;
    cascade_value.angle_cycle.kd = 0.0f;

    //速度闭环控制结构体
    cascade_value.speed_cycle.kp = 2.0f; //0.2f
    cascade_value.speed_cycle.ki = 0.0f;//0.0f
    cascade_value.speed_cycle.kd = 0.0f;

}

//获取IMU数据,并初步处理
void imu_data_get(void)
{
    imu660ra_get_acc();
    imu660ra_get_gyro();
    
    imu660ra_gyro_x = imu660ra_gyro_x - 4;//陀螺仪x零偏校准
    imu660ra_gyro_y = imu660ra_gyro_y + 6;//陀螺仪y零偏校准
    //imu660ra_gyro_z = imu660ra_gyro_z + 6;

    if(func_abs(imu660ra_gyro_x) <= 5)
    {
        imu660ra_gyro_x = 0;
    }
    if(func_abs(imu660ra_gyro_y) <= 5)
    {
        imu660ra_gyro_y = 0;
    }
    if(func_abs(imu660ra_gyro_z) <= 5)
    {
        imu660ra_gyro_z = 0;
    }
}

//动态电机控制函数
void dynamic_motor_control(void)
{
    int16 left_motor_duty;
    int16 right_motor_duty;
    if(run_flag){
        if(cascade_value.cascade_common_value.filtered_value > 2000 || cascade_value.cascade_common_value.filtered_value < -2000){
            small_driver_set_duty(0,0);
            run_flag = false;
        }
        left_motor_duty = func_limit_ab(cascade_value.angular_speed_cycle.out, -10000, 10000);
        right_motor_duty = func_limit_ab(cascade_value.angular_speed_cycle.out, -10000, 10000);
        small_driver_set_duty(-left_motor_duty,right_motor_duty); //4/010修改：电机反转
    }
    else{
        small_driver_set_duty(0,0);
    }      
}

//1ms中断回调函数:pit_ch10
void pit_isr_callback(void)
{
    // PIT中断回调函数的实现
    system_count++;
    imu_data_get();

    if(system_count % 20 ==0){
        small_driver_get_speed();
        car_speed = (motor_value.receive_left_speed_data + motor_value.receive_right_speed_data)/2;
        //pid_control_pd(&cascade_value.speed_cycle,0.0f,car_speed);
        pid_control_pd(&cascade_value.speed_cycle,target_speed,car_speed);
    }


    if(system_count % 5 ==0){//每5ms执行一次互补滤波和角度pid
        first_order_complementary_filter(&cascade_value.cascade_common_value,*(cascade_value.cascade_common_value.gyro_raw_data_l),*(cascade_value.cascade_common_value.acc_raw_data_l));
        pid_control_pd(&cascade_value.angle_cycle,-cascade_value.speed_cycle.out,cascade_value.cascade_common_value.filtered_value);
    }
    pid_control_pd(&cascade_value.angular_speed_cycle,(-cascade_value.angle_cycle.out),*(cascade_value.cascade_common_value.gyro_raw_data_l));
    dynamic_motor_control();
}

//*********一阶互补滤波*************
void first_order_complementary_filter(cascade_common_value_struct* filter,int16 gyro_raw_data,int16 acc_raw_data){
    float gyro_temp;
    float acc_temp;

    gyro_temp = gyro_raw_data * filter->gyro_ration; //角速度*角速度置信度
    acc_temp = (acc_raw_data-filter->temp_value) * filter->acc_ration;    //加速度*加速度置信度gyro
    filter->temp_value += ((gyro_temp + acc_temp) * filter->dt); //互补滤波计算
    filter->filtered_value = filter->temp_value + filter->mechanical_offset; //加上机械偏置
}


//**********PID控制器-PD************
void pid_control_pd(pid_cycle_struct* pid_cycle,float target_value,float current_value){
    float error;
    float p_value;
    float d_value;
    
    // 计算误差
    error = target_value - current_value;
    
    // 计算比例项
    p_value = pid_cycle->kp * error;
    
    // 计算微分项（基于比例项的变化）
    d_value = pid_cycle->kd * (p_value - pid_cycle->p_value_last);
    
    // 保存当前比例项用于下次微分计算
    pid_cycle->p_value_last = p_value;
    
    // 计算输出（PD控制器只有P和D项，没有I项）
    pid_cycle->out = p_value + d_value;
}
