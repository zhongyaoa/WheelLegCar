#include "gps_fusion.h"
#include "posture_control.h"
#include "zf_device_gnss.h"
#include "math.h"

// ─── 内部状态 ────────────────────────────────────────────────────────────────

// 采集阶段：记录第一个点时存入的 IMU 航向（°）
static float  s_imu_heading_at_origin = 0.0f;

// GPS direction 低通滤波值（真北为 0°，顺时针 0~360）；-1 表示尚未初始化
static float  s_gps_dir_filtered = -1.0f;

// 有效帧计数（热身用）
static uint8  s_heading_valid_cnt = 0;

// 航向校正是否已完成
static uint8  s_heading_calibrated = 0;

// 是否处于航向校正激活状态
static uint8  s_heading_cal_active = 0;

// 起点 GPS 经纬度（发车时锁定）
static double s_origin_lat   = 0.0;
static double s_origin_lon   = 0.0;
static uint8  s_origin_valid = 0;

// 是否处于循迹位置修正模式
static uint8  s_tracking_active = 0;

// GPS 校正后的初始航向在真北坐标系下的方向（°，顺时针自真北）
// 即：发车时 inav Y 轴（前进方向）距真北的角度
static float  s_gps_dir_at_origin = 0.0f;

// ─── 内部工具函数 ─────────────────────────────────────────────────────────────

static float normalize_360(float a)
{
    while(a >= 360.0f) a -= 360.0f;
    while(a <    0.0f) a += 360.0f;
    return a;
}

static float normalize_180(float a)
{
    while(a >  180.0f) a -= 360.0f;
    while(a < -180.0f) a += 360.0f;
    return a;
}

// ─── 公开接口实现 ─────────────────────────────────────────────────────────────

void gps_fusion_init(void)
{
    gnss_init(TAU1201);  // UART2（P10_0/P10_1），115200
    s_gps_dir_filtered   = -1.0f;
    s_heading_valid_cnt  = 0;
    s_heading_calibrated = 0;
    s_heading_cal_active = 0;
    s_origin_valid       = 0;
    s_tracking_active    = 0;
    s_imu_heading_at_origin = 0.0f;
    s_gps_dir_at_origin  = 0.0f;
}

void gps_fusion_start_heading_cal(float origin_imu_heading_deg)
{
    s_imu_heading_at_origin = origin_imu_heading_deg;
    s_gps_dir_filtered      = -1.0f;
    s_heading_valid_cnt     = 0;
    s_heading_calibrated    = 0;
    s_heading_cal_active    = 1;
}

uint8 gps_fusion_heading_calibrated(void)
{
    return s_heading_calibrated;
}

float gps_fusion_get_corrected_heading(float origin_imu_heading_deg)
{
    if(!s_heading_calibrated)
        return origin_imu_heading_deg;

    // GPS 校正后，inav Y 轴（前进）距真北 s_gps_dir_at_origin 度
    // 但 inav_heading_ref 需要的是 IMU 坐标系下的值（quat_yaw_deg 偏置）
    // 关系：inav_heading_ref = quat_yaw_deg（记录点时）= s_imu_heading_at_origin
    // GPS 校正只修正坐标系旋转，不改变 IMU 参考值本身
    // 实际上 inav_heading_ref 保持不变，坐标旋转已在 update() 中直接应用到 inav_x/y
    // 此函数主要供外部查询"是否已校正"并获取原值
    return origin_imu_heading_deg;
}

void gps_fusion_start_tracking(void)
{
    s_tracking_active = 1;
    s_origin_valid    = 0;

    if(gnss.state && gnss.satellite_used >= 4)
    {
        s_origin_lat   = gnss.latitude;
        s_origin_lon   = gnss.longitude;
        s_origin_valid = 1;
        wireless_printf("[GPS] Origin locked: lat=%.7f lon=%.7f\r\n",
                        s_origin_lat, s_origin_lon);
    }
    else
    {
        wireless_printf("[GPS] WARNING: GPS not valid at launch, pos-correction disabled.\r\n");
    }
}

void gps_latlon_to_xy(double lat, double lon, float *out_x, float *out_y)
{
    const double R = 6378137.0;
    double dlat = lat - s_origin_lat;
    double dlon = lon - s_origin_lon;
    // 北向（Y 轴）
    *out_y = (float)(dlat * (GNSS_PI / 180.0) * R);
    // 东向（X 轴）
    *out_x = (float)(dlon * (GNSS_PI / 180.0) * R * cos(s_origin_lat * GNSS_PI / 180.0));
}

uint8 gps_fusion_correct_position(void)
{
    if(!s_tracking_active || !s_origin_valid)
        return 0;
    if(!gnss.state || gnss.satellite_used < 4)
    {
        wireless_printf("[GPS] Pos correct skipped: GPS invalid.\r\n");
        return 0;
    }
    if(!s_heading_calibrated)
    {
        wireless_printf("[GPS] Pos correct skipped: heading not calibrated.\r\n");
        return 0;
    }

    // GPS NE（北东）→ inav XY 坐标旋转
    // inav Y 轴距真北 s_gps_dir_at_origin 度（顺时针）
    // 旋转矩阵（将 NE 坐标投影到 inav 坐标系）：
    //   inav_x =  gps_e * cos(θ) + gps_n * sin(θ)
    //   inav_y = -gps_e * sin(θ) + gps_n * cos(θ)
    float gps_n, gps_e;
    gps_latlon_to_xy(gnss.latitude, gnss.longitude, &gps_e, &gps_n);

    float theta = s_gps_dir_at_origin * (3.14159265f / 180.0f);
    float sinT  = sinf(theta);
    float cosT  = cosf(theta);

    float gps_inav_x =  gps_e * cosT + gps_n * sinT;
    float gps_inav_y = -gps_e * sinT + gps_n * cosT;

    float dx   = gps_inav_x - inav_x;
    float dy   = gps_inav_y - inav_y;
    float dist = sqrtf(dx * dx + dy * dy);

    if(dist < GPS_FUSION_POS_CORRECT_THRESH_M)
    {
        wireless_printf("[GPS] Pos correct skipped: diff=%.2fm < %.1fm\r\n",
                        dist, GPS_FUSION_POS_CORRECT_THRESH_M);
        return 0;
    }

    float old_x = inav_x, old_y = inav_y;
    inav_x += GPS_FUSION_POS_WEIGHT * dx;
    inav_y += GPS_FUSION_POS_WEIGHT * dy;

    wireless_printf("[GPS] Pos corrected: ins(%.2f,%.2f)->gps(%.2f,%.2f) diff=%.2fm\r\n",
                    old_x, old_y, gps_inav_x, gps_inav_y, dist);
    return 1;
}

void gps_fusion_update(void)
{
    // 解析 GPS 数据
    if(gnss_flag)
    {
        gnss_data_parse();
        gnss_flag = 0;
    }

    // 尝试补充锁定起点坐标（发车时 GPS 还未就绪的情况）
    if(s_tracking_active && !s_origin_valid)
    {
        if(gnss.state && gnss.satellite_used >= 4)
        {
            s_origin_lat   = gnss.latitude;
            s_origin_lon   = gnss.longitude;
            s_origin_valid = 1;
            wireless_printf("[GPS] Origin late-locked: lat=%.7f lon=%.7f\r\n",
                            s_origin_lat, s_origin_lon);
        }
    }

    // 航向校正：仅在激活且尚未完成时运行
    if(!s_heading_cal_active)
        return;

    // GPS 有效性检查
    if(!gnss.state || gnss.satellite_used < 4)
        return;

    // 速度过低时 GPS direction 不可靠
    if(gnss.speed < GPS_FUSION_MIN_SPEED_KPH)
        return;

    float gps_dir = gnss.direction;  // 真北为 0°，顺时针 0~360

    // 异常跳变过滤
    if(s_gps_dir_filtered >= 0.0f)
    {
        float diff = normalize_180(gps_dir - s_gps_dir_filtered);
        if(fabsf(diff) > GPS_FUSION_MAX_HEADING_JUMP_DEG)
            return;
        s_gps_dir_filtered += GPS_FUSION_HEADING_ALPHA * diff;
        s_gps_dir_filtered  = normalize_360(s_gps_dir_filtered);
    }
    else
    {
        s_gps_dir_filtered = gps_dir;
    }

    s_heading_valid_cnt++;
    if(s_heading_valid_cnt < GPS_FUSION_HEADING_WARMUP_CNT)
        return;

    // ─── 核心：反推起点时刻的 inav Y 轴在真北坐标系中的角度 ───
    //
    // 当前时刻：
    //   GPS direction = s_gps_dir_filtered（车头距真北角度，顺时针）
    //   IMU 相对起点的偏转 = quat_yaw_deg - s_imu_heading_at_origin
    // 因此起点时刻的 inav Y 轴（前进方向）距真北：
    //   gps_dir_at_origin = s_gps_dir_filtered - (quat_yaw_deg - s_imu_heading_at_origin)
    float imu_delta = normalize_180(quat_yaw_deg - s_imu_heading_at_origin);
    float new_gps_dir_at_origin = normalize_360(s_gps_dir_filtered - imu_delta);

    if(!s_heading_calibrated)
    {
        // 首次校正成功：直接设置，无需旋转历史坐标（采集阶段 inav 数据不用于循迹）
        s_gps_dir_at_origin  = new_gps_dir_at_origin;
        s_heading_calibrated = 1;
        wireless_printf("[GPS] Heading calibrated: gps_dir=%.1f imu_delta=%.1f origin_dir=%.1f cnt=%d\r\n",
                        s_gps_dir_filtered, imu_delta, s_gps_dir_at_origin, s_heading_valid_cnt);
        return;
    }

    // 后续持续校正：计算角度差，旋转 inav_x/y
    float angle_diff = normalize_180(new_gps_dir_at_origin - s_gps_dir_at_origin);
    if(fabsf(angle_diff) < 0.5f)
        return;  // 差距太小，跳过

    float rad   = angle_diff * (3.14159265f / 180.0f);
    float cos_r = cosf(rad);
    float sin_r = sinf(rad);
    float new_x = inav_x * cos_r - inav_y * sin_r;
    float new_y = inav_x * sin_r + inav_y * cos_r;
    inav_x = new_x;
    inav_y = new_y;

    s_gps_dir_at_origin = new_gps_dir_at_origin;

    wireless_printf("[GPS] Heading updated: origin_dir=%.1f diff=%.1f\r\n",
                    s_gps_dir_at_origin, angle_diff);
}
