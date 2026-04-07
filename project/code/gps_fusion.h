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

// ─── 循迹运行时位置修正参数 ──────────────────────────────────────────────────

// GPS 与 INS 位置差超过此距离才执行修正（m）
#define GPS_FUSION_POS_CORRECT_THRESH_M  1.5f

// 位置融合权重（0=不动，1=完全信任 GPS）
#define GPS_FUSION_POS_WEIGHT            0.5f

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

// ── 循迹运行时位置修正 ────────────────────────────────────────────────────────

// 到达路径点时调用：用实时 GPS 修正 inav_x/y
// 返回 1=修正成功，0=跳过
uint8 gps_fusion_correct_position(void);

#endif
