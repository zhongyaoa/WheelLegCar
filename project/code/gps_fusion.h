/*
    GPS 辅助惯性导航融合模块 (GPS-Aided INS Fusion)

    集成到 subject1_ui 的方式：
      1. 采集阶段：subject1_ui.c 在记录第一个点后调用 gps_fusion_start_heading_cal()，
                   车辆行驶采集过程中 GPS direction 持续校正 inav_heading_ref。
      2. 发车阶段：subject1_ui.c 发车前调用 gps_fusion_get_corrected_heading() 获取
                   GPS 校正后的航向参考值（若 GPS 未校正成功则返回原值），
                   以及 gps_fusion_start_tracking() 锁定起点 GPS 坐标。
      3. 循迹阶段：到达路径点时调用 gps_fusion_correct_position() 修正 inav_x/y。
      4. 降级保护：GPS 无效时所有接口均静默跳过，不影响纯惯性导航流程。

    主循环（50ms）：调用 gps_fusion_update() 解析 GPS 并更新航向校正滤波器。
*/
#ifndef _GPS_FUSION_H_
#define _GPS_FUSION_H_

#include "zf_common_typedef.h"

// ─── 可调参数 ────────────────────────────────────────────────────────────────

// 航向校正：GPS direction 可信的最低车速（km/h）
// 静止时 GPS 方向角噪声大，低于此阈值不采信
#define GPS_FUSION_MIN_SPEED_KPH        0.5f

// 航向校正：单帧 GPS direction 与滤波值差异超过此值认为跳变，丢弃
#define GPS_FUSION_MAX_HEADING_JUMP_DEG 30.0f

// 航向校正：低通滤波系数（越大越平滑，响应越慢）
#define GPS_FUSION_HEADING_ALPHA        0.10f

// 航向校正：积累多少帧有效 GPS direction 后才输出校正结果
#define GPS_FUSION_HEADING_WARMUP_CNT   5

// 位置修正：GPS 与 INS 位置差距超过此值才执行修正
#define GPS_FUSION_POS_CORRECT_THRESH_M 1.5f

// 位置修正：融合权重（0=不动，1=完全信任 GPS）
#define GPS_FUSION_POS_WEIGHT           0.5f

// ─── 外部接口 ────────────────────────────────────────────────────────────────

// 初始化（在 main_cm7_0 中 gnss 使用前调用）
void  gps_fusion_init(void);

// 50ms 周期更新，解析 GPS 数据并持续更新航向滤波
// 需在 subject1_ui_poll() / ins_tracker_update() 之前调用
void  gps_fusion_update(void);

// 采集阶段：记录第一个点后调用，启动 GPS 航向校正
// origin_imu_heading_deg: 记录第一个点时存入的 quat_yaw_deg
void  gps_fusion_start_heading_cal(float origin_imu_heading_deg);

// 查询 GPS 航向校正是否已成功（积累足够帧）
uint8 gps_fusion_heading_calibrated(void);

// 获取 GPS 校正后的初始航向参考值（°，IMU 坐标系）
// 若尚未校正成功则原样返回 origin_imu_heading_deg
float gps_fusion_get_corrected_heading(float origin_imu_heading_deg);

// 发车时：锁定起点 GPS 经纬度，切换到循迹位置修正模式
void  gps_fusion_start_tracking(void);

// 循迹阶段：到达路径点时调用，用 GPS 位置修正 inav_x / inav_y
// 返回 1=完成修正，0=GPS 无效或差距过小未修正
uint8 gps_fusion_correct_position(void);

// 将 GPS 经纬度转为相对起点的平面坐标（北→Y，东→X，单位 m）
void  gps_latlon_to_xy(double lat, double lon, float *out_x, float *out_y);

#endif
