#include "ui.h"
#include "zf_device_ips200.h"
#include "zf_device_gnss.h"
#include "balance_control.h"
#include "posture_control.h"
#include "task_slalom.h"

// ---- 布局常量 ----
#define ROW(n)      ((n) * 16)      // 行 y 坐标（8x16 字体，行高 16px）
#define COL(n)      ((n) * 8)       // 列 x 坐标（字宽 8px）

// 颜色语义
#define C_LABEL     RGB565_BLUE
#define C_VALUE     RGB565_BLACK
#define C_OK        RGB565_GREEN
#define C_WARN      RGB565_RED
#define C_BG        RGB565_WHITE
#define C_SEP       (0x8410)        // 灰色分隔线

// 区块 y 坐标
#define Y_STATE     ROW(0)
#define Y_GPS       ROW(2)
#define Y_LAT       ROW(3)
#define Y_LON       ROW(4)
#define Y_HEAD      ROW(5)
#define Y_IMU       ROW(7)
#define Y_IMU2      ROW(8)
#define Y_NAV       ROW(10)
#define Y_NAV2      ROW(11)
#define Y_MTR       ROW(13)
#define Y_MTR2      ROW(14)

// ---- 状态名字符串 ----
static const char *state_name(slalom_state_enum s)
{
    switch (s)
    {
        case SLALOM_IDLE:        return "IDLE       ";
        case SLALOM_GPS_WAIT:    return "GPS WAIT   ";
        case SLALOM_RECORD_MODE: return "RECORD MODE";
        case SLALOM_STRAIGHT_GO: return "STRAIGHT   ";
        case SLALOM_U_TURN:      return "U-TURN     ";
        case SLALOM_RETURN:      return "SLALOM     ";
        case SLALOM_STOP:        return "STOP  OK   ";
        default:                 return "???        ";
    }
}

// ---- 绘制水平分隔线 ----
static void draw_sep(uint16 y)
{
    ips200_draw_line(0, y, 319, y, C_SEP);
}

//=============================================================================
// ui_init — 清屏并绘制静态标签和分隔线框架
//=============================================================================
void ui_init(void)
{
    ips200_init(IPS200_TYPE_SPI);
    ips200_set_dir(IPS200_CROSSWISE);
    ips200_set_font(IPS200_8X16_FONT);
    ips200_set_color(C_VALUE, C_BG);
    ips200_clear();

    // 静态标签（蓝色）
    ips200_set_color(C_LABEL, C_BG);
    ips200_show_string(0,       Y_STATE, "STATE:");
    ips200_show_string(0,       Y_GPS,   "GPS SAT:");
    ips200_show_string(COL(11), Y_GPS,   "FIX");
    ips200_show_string(COL(16), Y_GPS,   "SPD:");
    ips200_show_string(COL(20), Y_GPS,   "km/h");
    ips200_show_string(0,       Y_LAT,   "LAT:");
    ips200_show_string(0,       Y_LON,   "LON:");
    ips200_show_string(0,       Y_HEAD,  "HDG:");
    ips200_show_string(COL(14), Y_HEAD,  "DIST:");
    ips200_show_string(COL(24), Y_HEAD,  "m");

    ips200_show_string(0,       Y_IMU,   "IMU ROL:");
    ips200_show_string(COL(16), Y_IMU,   "PIT:");
    ips200_show_string(COL(4),  Y_IMU2,  "YAW:");
    ips200_show_string(COL(16), Y_IMU2,  "SPD:");

    ips200_show_string(0,       Y_NAV,   "NAV CONE:");
    ips200_show_string(COL(12), Y_NAV,   "/");
    ips200_show_string(COL(16), Y_NAV,   "OUT:");
    ips200_show_string(COL(4),  Y_NAV2,  "TGT HDG:");

    ips200_show_string(0,       Y_MTR,   "MTR L:");
    ips200_show_string(COL(14), Y_MTR,   "R:");
    ips200_show_string(0,       Y_MTR2,  "RUN:");
    ips200_show_string(COL(7),  Y_MTR2,  "JMP:");
    ips200_show_string(COL(15), Y_MTR2,  "BMAX:");

    // 分隔线
    draw_sep(ROW(1));
    draw_sep(ROW(6)  + 8);
    draw_sep(ROW(9)  + 8);
    draw_sep(ROW(12) + 8);
}

//=============================================================================
// ui_update — 刷新所有动态数值（约 100ms 调用一次）
//=============================================================================
void ui_update(void)
{
    // ---- 状态栏 ----
    ips200_set_color(slalom_state == SLALOM_STOP ? C_OK : C_VALUE, C_BG);
    ips200_show_string(COL(6), Y_STATE, (char *)state_name(slalom_state));

    // ---- GPS 区 ----
    ips200_set_color(C_VALUE, C_BG);
    ips200_show_uint(COL(8), Y_GPS, gnss.satellite_used, 2);

    ips200_set_color(gnss.state == 1 ? C_OK : C_WARN, C_BG);
    ips200_show_string(COL(14), Y_GPS, gnss.state == 1 ? "Y" : "N");

    ips200_set_color(C_VALUE, C_BG);
    ips200_show_float(COL(20), Y_GPS, gnss.speed, 3, 1);

    ips200_show_float(COL(4),  Y_LAT, gnss.latitude,  3, 6);
    ips200_show_float(COL(4),  Y_LON, gnss.longitude, 4, 6);

    // 当前地面航向
    ips200_show_float(COL(4), Y_HEAD, gnss.direction, 5, 1);

    // 到当前目标锥桶距离
    double dist = 0.0;
    if (slalom_state == SLALOM_STRAIGHT_GO)
    {
        // turnaround_point 是 static，用 heading_target 大致显示
        dist = 0.0;   // 无法直接获取，留0（可根据需要导出）
    }
    else if (slalom_state == SLALOM_RETURN && current_cone_idx < slalom_cone_count)
    {
        // 无法直接读锥桶坐标（static），显示 nav_yaw_output 代替
        dist = 0.0;
    }
    ips200_show_float(COL(19), Y_HEAD, dist, 4, 1);

    // ---- IMU 区 ----
    ips200_show_float(COL(8),  Y_IMU,  roll_balance_cascade.posture_value.rol, 4, 1);
    ips200_show_float(COL(20), Y_IMU,  roll_balance_cascade.posture_value.pit, 4, 1);
    ips200_show_float(COL(8),  Y_IMU2, roll_balance_cascade.posture_value.yaw, 5, 1);
    ips200_show_int(COL(20),   Y_IMU2, car_speed, 5);

    // ---- NAV 区 ----
    ips200_show_uint(COL(9),  Y_NAV, current_cone_idx,   2);   // 当前进度
    ips200_show_uint(COL(13), Y_NAV, slalom_cone_count,  2);   // 总锥桶数
    ips200_show_int(COL(20),  Y_NAV, nav_yaw_output,     5);   // 航向叠加输出

    ips200_show_float(COL(12), Y_NAV2, heading_target, 5, 1);  // 目标航向

    // ---- 电机区 ----
    ips200_show_int(COL(6),  Y_MTR, left_motor_duty,  5);
    ips200_show_int(COL(16), Y_MTR, right_motor_duty, 5);

    ips200_show_uint(COL(4),  Y_MTR2, run_state,        1);
    ips200_show_uint(COL(11), Y_MTR2, jump_flag,        1);
    ips200_show_int(COL(20),  Y_MTR2, balance_duty_max, 5);
}
