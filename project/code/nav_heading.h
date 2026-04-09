/*
 * nav_heading — 导航航向角模块
 *
 * 原理（按逐飞博客方法）：
 *   1. 在 1ms PIT 中断里对 gyro_z 积分，得到高频连续的航向角
 *   2. 先采样静止零偏并消除（博客推荐方法：取 100 次平均，然后对 10 取整）
 *   3. GPS 有效时（速度 > 0.3 km/h）做互补滤波慢速拉向 GPS 方向角
 *   4. 对外暴露 nav_heading_angle（0~360°，顺时针，北为 0）
 *
 * 使用：
 *   - nav_heading_init()：上电静止时调用，采集零偏（需等待约 200ms）
 *   - nav_heading_update_1ms()：在 pit_call_back() 里调用（1ms 周期）
 *   - nav_heading_sync_gps()：GPS 有新数据时调用（约 100ms 周期）
 */

#ifndef _NAV_HEADING_H_
#define _NAV_HEADING_H_

#include "zf_common_headfile.h"

// 互补滤波修正步长（每次修正的角度，度）
// 越大越快跟 GPS，越小越平滑；博客建议调试确定，初始值 0.03
#define NAV_HEADING_COMP_STEP   0.03f

// 零偏采样次数
#define NAV_HEADING_BIAS_SAMPLES 100

// 对外航向角（0~360°，北为 0，顺时针）
extern float nav_heading_angle;

// 初始化：采集陀螺仪零偏（调用时车必须静止，约需 200ms）
void nav_heading_init(void);

// 1ms 积分更新（在 pit_call_back 末尾调用）
void nav_heading_update_1ms(void);

// GPS 互补修正（在 task_slalom_update 中检测到新 GPS 数据时调用）
void nav_heading_sync_gps(float gps_direction);

#endif
