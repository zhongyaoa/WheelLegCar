#include "nav_heading.h"
#include "zf_device_imu660ra.h"

// 对外航向角
float nav_heading_angle = 0.0f;

// 内部变量
static float gyro_z_bias  = 0.0f;   // 陀螺仪 Z 轴零偏（LSB 单位）
static uint8 heading_ready = 0;      // 初始化完成标志

//=============================================================================
// nav_heading_init — 采集零偏（静止时调用）
//=============================================================================
void nav_heading_init(void)
{
    // 采集 NAV_HEADING_BIAS_SAMPLES 次 gyro_z，每次间隔 2ms
    float sum = 0.0f;
    for (int i = 0; i < NAV_HEADING_BIAS_SAMPLES; i++)
    {
        imu660ra_get_gyro();
        sum += (float)imu660ra_gyro_z;
        system_delay_ms(2);
    }
    gyro_z_bias = sum / NAV_HEADING_BIAS_SAMPLES;

    nav_heading_angle = 0.0f;
    heading_ready = 1;

    printf("nav_heading: gyro_z_bias=%.2f\r\n", gyro_z_bias);
}

//=============================================================================
// nav_heading_update_1ms — 在 pit_call_back() 末尾每 1ms 调用
//=============================================================================
void nav_heading_update_1ms(void)
{
    if (!heading_ready) return;

    // 去零偏，对 10 取整消除抖动（博客方法）
    float gz_raw = (float)imu660ra_gyro_z - gyro_z_bias;
    float gz = (float)((int)(gz_raw / 10.0f)) * 10.0f;

    // 注意：balance_control.h 定义 GYRO_DATA_Z = -imu660ra_gyro_z
    // 即原始 gyro_z 正值 = 右转，对应航向角应增大（顺时针）
    // 所以航向角增量 = -gz / 16.384 * 0.001
    // （-是因为 imu660ra_gyro_z 右转为正，而我们用 -gyro_z 做平衡，
    //   这里直接用原始 imu660ra_gyro_z，右转=正，对应方位角增大）
    float delta_angle = gz / 16.384f * 0.001f;  // 单位：度

    nav_heading_angle += delta_angle;

    // 归一化到 0~360
    if (nav_heading_angle >= 360.0f) nav_heading_angle -= 360.0f;
    if (nav_heading_angle <    0.0f) nav_heading_angle += 360.0f;
}

//=============================================================================
// nav_heading_sync_gps — GPS 互补修正
// gps_direction: GPS 地面航向，0~360°，北为 0，顺时针
// 仅在车速 > 0.3 km/h 时调用（低速 GPS 方向无意义）
//=============================================================================
void nav_heading_sync_gps(float gps_direction)
{
    if (!heading_ready) return;

    // 计算角度差，归一化到 -180~180
    float diff = gps_direction - nav_heading_angle;
    if (diff >  180.0f) diff -= 360.0f;
    if (diff < -180.0f) diff += 360.0f;

    // 互补修正：每次拉近一小步
    if (diff > 0.0f)
        nav_heading_angle += NAV_HEADING_COMP_STEP;
    else if (diff < 0.0f)
        nav_heading_angle -= NAV_HEADING_COMP_STEP;

    // 归一化
    if (nav_heading_angle >= 360.0f) nav_heading_angle -= 360.0f;
    if (nav_heading_angle <    0.0f) nav_heading_angle += 360.0f;
}
