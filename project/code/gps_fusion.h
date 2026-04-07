/*
    GPS 辅助惯性导航融合模块 (GPS-Aided INS Fusion)

    核心功能 —— GPS 静止采样：
      按键采点时车辆静止，连续采集 GPS_SAMPLER_TOTAL_COUNT 帧 GPS 数据，
      做野值剔除（距均值 > GPS_SAMPLER_REJECT_THRESH 的帧丢弃），
      取均值作为该点的高精度 GPS 坐标，替代 INS 积分坐标。

    坐标转换：
      发车时，利用"起点→调头点"GPS 方位角确定初始车头朝向（真北角度），
      将所有采集点的 GPS 经纬度转换为以起点为原点、车头方向为 Y 轴的 inav XY（m），
      直接注入 ins_tracker，无需 INS 积分辅助。

    运行时位置修正（可选）：
      循迹过程中到达每个路径点时，仍可用实时 GPS 对 inav_x/y 做修正。
*/
#ifndef _GPS_FUSION_H_
#define _GPS_FUSION_H_

#include "zf_common_typedef.h"

// ─── GPS 静止采样参数 ─────────────────────────────────────────────────────────

// 每个点位的总采样帧数（GPS 10Hz → 30帧 ≈ 3秒）
#define GPS_SAMPLER_TOTAL_COUNT    30

// 野值剔除阈值（m）：与第一轮均值偏差超过此值的帧被丢弃
#define GPS_SAMPLER_REJECT_THRESH  2.0f

// GPS 有效判据：卫星数不低于此值才采信
#define GPS_SAMPLER_MIN_SATS       4

// ─── 位置卡尔曼滤波参数 ───────────────────────────────────────────────────────

// 过程噪声（INS积分方差，单位 m²/s，越小越信任INS）
// INS 主导：设为较小值（轮式里程计 + 陀螺航向，短距积分误差小）
#define KF_PROCESS_NOISE_Q               0.002f

// 观测噪声基准（GPS方差基准，单位 m²，越大越不信任GPS）
// 开放天空好天气约 0.09（误差 0.3m），阴天/遮挡时自动乘以卫星数惩罚因子
#define KF_MEAS_NOISE_R_BASE             0.09f

// GPS卫星数惩罚：卫星数每少于 8 颗，R 乘以此因子（降低GPS权重）
#define KF_SAT_PENALTY_FACTOR            1.5f
#define KF_SAT_GOOD_COUNT                8

// GPS 跳变保护：单次更新最大允许修正量（m）；超出时按比例压缩，防止野值拉偏
#define KF_GPS_MAX_JUMP_M                3.0f

// 卡尔曼滤波启用门控：GPS 卫星数低于此值时跳过观测更新（完全依靠INS）
#define KF_MIN_SATS_FOR_UPDATE           4

// ─── GPS 采样状态 ─────────────────────────────────────────────────────────────

typedef enum
{
    GPS_SAMPLER_IDLE    = 0,  // 未启动
    GPS_SAMPLER_BUSY    = 1,  // 采集中
    GPS_SAMPLER_DONE    = 2,  // 完成，结果可读
    GPS_SAMPLER_FAIL    = 3,  // 失败（有效帧数不足）
} gps_sampler_state_enum;

extern gps_sampler_state_enum gps_sampler_state;

// ─── 接口 ─────────────────────────────────────────────────────────────────────

// 初始化（在 main 中调用一次，初始化 GNSS 驱动）
void  gps_fusion_init(void);

// 50ms 周期更新：解析新到的 GPS 帧，驱动采样状态机
// 需在 subject1_ui_poll() 之前调用
void  gps_fusion_update(void);

// ── GPS 静止采样 ──────────────────────────────────────────────────────────────

// 启动一次采样（重置计数器，开始收集 GPS_SAMPLER_TOTAL_COUNT 帧）
void  gps_sampler_start(void);

// 查询当前已收到的有效帧数（用于 UI 进度条）
uint8 gps_sampler_count(void);

// 查询采样是否完成（DONE 或 FAIL）
uint8 gps_sampler_ready(void);

// 获取采样结果（仅在 gps_sampler_state == GPS_SAMPLER_DONE 时有效）
// lat_out / lon_out：经过均值+野值剔除后的高精度经纬度
void  gps_sampler_get_result(double *lat_out, double *lon_out);

// ── 坐标转换 ──────────────────────────────────────────────────────────────────

// 将 GPS 经纬度转换为相对"起点"的 inav XY（m）
//   起点 GPS 坐标、初始航向（真北顺时针 °）由 gps_fusion_set_origin() 设置
//   inav Y = 车头方向（前进），inav X = 车头右侧
void  gps_fusion_set_origin(double origin_lat, double origin_lon,
                             float initial_bearing_deg);

void  gps_fusion_latlon_to_inav(double lat, double lon,
                                 float *inav_x_out, float *inav_y_out);

// ── 循迹运行时位置修正（卡尔曼滤波） ─────────────────────────────────────────

// 初始化卡尔曼滤波器（在 gps_fusion_set_origin 后、循迹启动前调用）
void  gps_fusion_kf_init(void);

// INS 预测步骤：每 1ms 由 pit_call_back 中 inav 积分代码调用
// dx/dy：本毫秒 inav 坐标增量（m）；dt：积分步长（s，固定 0.001）
void  gps_fusion_kf_predict(float dx, float dy, float dt);

// 到达路径点时调用：用实时 GPS 修正 inav_x/y（卡尔曼观测更新）
// 返回 1=修正成功，0=跳过
uint8 gps_fusion_correct_position(void);

#endif
