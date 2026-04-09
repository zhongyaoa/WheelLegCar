/*
 * 科目一：行进绕桩导航模块
 *
 * 流程：
 *   IDLE → GPS_WAIT → STRAIGHT_GO → U_TURN → SLALOM_RETURN → STOP
 *
 * 赛前录坐标：
 *   按键 A (P09_0) 短按 → 记录当前 GPS 坐标为下一个锥桶位置
 *   按键 B (P08_3) 短按 → 完成录入，进入等待启动状态
 *   按键 UP (P10_4) 短按 → 启动科目（run_state 已为 1 时）
 *
 * 导航：GPS 提供位置目标，IMU yaw 提供高频航向补偿
 */

#ifndef _TASK_SLALOM_H_
#define _TASK_SLALOM_H_

#include "zf_common_headfile.h"

//=============================================================================
// 场地参数（根据实际场地调整）
//=============================================================================
#define SLALOM_STRAIGHT_DIST        12.0        // 直行到掉头区距离（米）
#define SLALOM_CONE_ARRIVE_DIST     0.8         // 到达目标点判定半径（米）
#define SLALOM_FORWARD_SPEED        200.0f      // 直行目标速度（编码器单位）
#define SLALOM_SLALOM_SPEED         150.0f      // 绕桩目标速度
#define SLALOM_UTURN_SPEED          100.0f      // 掉头速度（前进速度置0，纯差速）
#define SLALOM_UTURN_ANGLE          175.0f      // 掉头完成角度差阈值（度）
#define SLALOM_MAX_CONES            10          // 最大锥桶数量

//=============================================================================
// 航向 PID 参数
//=============================================================================
#define SLALOM_HEADING_KP           40.0f       // 航向比例系数
#define SLALOM_HEADING_KD           3.0f        // 航向微分系数
#define SLALOM_HEADING_OUT_MAX      1500        // 航向输出限幅（叠加到差速）
#define SLALOM_UTURN_KP             8.0f        // 掉头旋转比例系数（剩余角度→输出）

//=============================================================================
// 按键引脚
//=============================================================================
#define KEY_A_PIN                   (P09_0)
#define KEY_B_PIN                   (P08_3)
#define KEY_UP_PIN                  (P10_4)

//=============================================================================
// 状态机
//=============================================================================
typedef enum
{
    SLALOM_IDLE,            // 等待启动指令
    SLALOM_GPS_WAIT,        // 等待 GPS 定位有效
    SLALOM_RECORD_MODE,     // 录坐标模式（赛前手动走一遍）
    SLALOM_STRAIGHT_GO,     // 直行到掉头区
    SLALOM_U_TURN,          // 掉头（原地旋转 180°）
    SLALOM_RETURN,          // 返回并穿越锥桶间隙
    SLALOM_STOP,            // 完成停止
} slalom_state_enum;

//=============================================================================
// 坐标点结构体
//=============================================================================
typedef struct
{
    double latitude;
    double longitude;
} gps_point_struct;

//=============================================================================
// 模块状态（供外部查看）
//=============================================================================
extern slalom_state_enum    slalom_state;
extern int16                nav_yaw_output;     // 导航航向差速叠加量（posture_control 中使用）
extern uint8                slalom_cone_count;  // 已录入锥桶数量
extern uint8                current_cone_idx;   // 当前目标锥桶索引（UI 读取）
extern float                heading_target;     // 当前目标航向角（UI 读取）

//=============================================================================
// 接口函数
//=============================================================================

// 初始化：初始化 GPS、按键 GPIO，设置初始状态为 RECORD_MODE
void task_slalom_init(void);

// 主循环调用（约 20ms 周期，在 while(1) 中调用）
void task_slalom_update(void);

#endif
