#include "gps_fusion.h"
#include "posture_control.h"
#include "zf_device_gnss.h"
#include "math.h"

// ─── GPS 采样器内部状态 ───────────────────────────────────────────────────────

gps_sampler_state_enum gps_sampler_state = GPS_SAMPLER_IDLE;

// 原始样本缓冲区
static double s_lat_buf[GPS_SAMPLER_TOTAL_COUNT];
static double s_lon_buf[GPS_SAMPLER_TOTAL_COUNT];
static uint8  s_sample_cnt  = 0;   // 已收到的有效帧数

// 最终结果
static double s_result_lat = 0.0;
static double s_result_lon = 0.0;

// ─── 坐标系参数（发车时设置） ─────────────────────────────────────────────────

static double s_origin_lat     = 0.0;
static double s_origin_lon     = 0.0;
static float  s_bearing_deg    = 0.0f;  // 初始车头朝向距真北的角度（顺时针，°）
static uint8  s_origin_valid   = 0;

// ─── 位置卡尔曼滤波器内部状态 ────────────────────────────────────────────────
// 标量独立 2 轴：X 和 Y 各维护一个协方差 P，共享同一套 Q/R 参数
// 状态：inav_x / inav_y（直接使用 posture_control 中的全局变量）
// 协方差矩阵简化为对角：Px, Py

static float kf_Px = 1.0f;    // X 轴位置估计方差（m²）
static float kf_Py = 1.0f;    // Y 轴位置估计方差（m²）
static uint8 kf_init_done = 0; // 已初始化标志

// 上一帧GPS数据，用于判断是否收到新帧（避免重复更新）
static double s_last_gps_lat = 0.0;
static double s_last_gps_lon = 0.0;

// ─── 工具函数 ─────────────────────────────────────────────────────────────────

// 两点间距离（Haversine，返回 m）
static double latlon_dist_m(double lat1, double lon1, double lat2, double lon2)
{
    return get_two_points_distance(lat1, lon1, lat2, lon2);
}

// ─── 公开接口实现 ─────────────────────────────────────────────────────────────

void gps_fusion_init(void)
{
    gnss_init(TAU1201);  // UART2（P10_0/P10_1），与 GPS 模块通信
    gps_sampler_state = GPS_SAMPLER_IDLE;
    s_sample_cnt      = 0;
    s_origin_valid    = 0;
    kf_init_done      = 0;
}

// ─── GPS 采样器 ───────────────────────────────────────────────────────────────

void gps_sampler_start(void)
{
    s_sample_cnt      = 0;
    s_result_lat      = 0.0;
    s_result_lon      = 0.0;
    gps_sampler_state = GPS_SAMPLER_BUSY;
}

uint8 gps_sampler_count(void)
{
    return s_sample_cnt;
}

uint8 gps_sampler_ready(void)
{
    return (gps_sampler_state == GPS_SAMPLER_DONE ||
            gps_sampler_state == GPS_SAMPLER_FAIL);
}

void gps_sampler_get_result(double *lat_out, double *lon_out)
{
    *lat_out = s_result_lat;
    *lon_out = s_result_lon;
}

// 内部：采集完成后做均值+野值剔除，计算最终结果
static void sampler_finalize(void)
{
    uint8 i;
    uint8 n = s_sample_cnt;

    if(n == 0)
    {
        gps_sampler_state = GPS_SAMPLER_FAIL;
        return;
    }

    // ── 第一轮：计算粗均值 ──
    double sum_lat = 0.0, sum_lon = 0.0;
    for(i = 0; i < n; i++)
    {
        sum_lat += s_lat_buf[i];
        sum_lon += s_lon_buf[i];
    }
    double mean_lat = sum_lat / n;
    double mean_lon = sum_lon / n;

    // ── 第二轮：剔除距粗均值 > GPS_SAMPLER_REJECT_THRESH 的野值 ──
    sum_lat = 0.0;
    sum_lon = 0.0;
    uint8 valid_cnt = 0;
    for(i = 0; i < n; i++)
    {
        double dist = latlon_dist_m(s_lat_buf[i], s_lon_buf[i], mean_lat, mean_lon);
        if(dist <= (double)GPS_SAMPLER_REJECT_THRESH)
        {
            sum_lat += s_lat_buf[i];
            sum_lon += s_lon_buf[i];
            valid_cnt++;
        }
    }

    if(valid_cnt == 0)
    {
        // 全部被剔除（极端情况）：退回粗均值
        s_result_lat = mean_lat;
        s_result_lon = mean_lon;
        wireless_printf("[GPS] sampler: all rejected, use raw mean\r\n");
    }
    else
    {
        s_result_lat = sum_lat / valid_cnt;
        s_result_lon = sum_lon / valid_cnt;
        wireless_printf("[GPS] sampler done: %d/%d valid, lat=%.7f lon=%.7f\r\n",
                        valid_cnt, n, s_result_lat, s_result_lon);
    }

    gps_sampler_state = GPS_SAMPLER_DONE;
}

// ─── 坐标转换 ─────────────────────────────────────────────────────────────────

void gps_fusion_set_origin(double origin_lat, double origin_lon,
                            float initial_bearing_deg)
{
    s_origin_lat   = origin_lat;
    s_origin_lon   = origin_lon;
    s_bearing_deg  = initial_bearing_deg;
    s_origin_valid = 1;
    wireless_printf("[GPS] origin set: lat=%.7f lon=%.7f bearing=%.1f\r\n",
                    s_origin_lat, s_origin_lon, s_bearing_deg);
}

void gps_fusion_latlon_to_inav(double lat, double lon,
                                float *inav_x_out, float *inav_y_out)
{
    // 1. 计算相对起点的北向（N）和东向（E）位移（m）
    //    使用小角度平面近似，场地尺度 < 200m 误差可忽略
    const double R = 6378137.0;
    double dlat = lat - s_origin_lat;
    double dlon = lon - s_origin_lon;
    float ne_n = (float)(dlat * (GNSS_PI / 180.0) * R);          // 北向（m）
    float ne_e = (float)(dlon * (GNSS_PI / 180.0) * R
                         * cos(s_origin_lat * GNSS_PI / 180.0)); // 东向（m）

    // 2. 将 NE 坐标旋转到 inav 坐标系
    //    inav Y 轴 = 车头方向（距真北 s_bearing_deg 度，顺时针）
    //    inav X 轴 = 车头右侧
    //    旋转公式（θ = s_bearing_deg）：
    //      inav_y =  ne_n * cos(θ) + ne_e * sin(θ)
    //      inav_x = -ne_n * sin(θ) + ne_e * cos(θ)
    float theta = s_bearing_deg * (3.14159265f / 180.0f);
    *inav_y_out =  ne_n * cosf(theta) + ne_e * sinf(theta);
    *inav_x_out = -ne_n * sinf(theta) + ne_e * cosf(theta);
}

// ─── 位置卡尔曼滤波器 ─────────────────────────────────────────────────────────

void gps_fusion_kf_init(void)
{
    // 以当前 inav_x/y 作为初始状态，方差设为较大值（不确定性高）
    kf_Px        = 1.0f;
    kf_Py        = 1.0f;
    kf_init_done = 1;
    s_last_gps_lat = 0.0;
    s_last_gps_lon = 0.0;
    wireless_printf("[KF] position KF initialized. x=%.2f y=%.2f\r\n", inav_x, inav_y);
}

// 预测步骤：由 pit_call_back 中 INS 积分代码调用（每 1ms）
// dx/dy：本毫秒 inav 坐标增量（m）
void gps_fusion_kf_predict(float dx, float dy, float dt)
{
    if(!kf_init_done) return;

    // 状态预测：INS 积分已直接更新了 inav_x/y，这里只更新方差
    // P_k|k-1 = P_k-1 + Q * dt²（位置误差方差按速度噪声传播）
    float q_dt2 = KF_PROCESS_NOISE_Q * dt * dt;
    kf_Px += q_dt2;
    kf_Py += q_dt2;

    (void)dx; (void)dy;  // 状态已由调用方更新，此处不重复
}

// 内部：执行卡尔曼观测更新（GPS观测 → 修正 inav_x/y）
static void kf_do_gps_update(float gps_x, float gps_y, float R_meas)
{
    // 卡尔曼增益：K = P / (P + R)
    float Kx = kf_Px / (kf_Px + R_meas);
    float Ky = kf_Py / (kf_Py + R_meas);

    // 新息（Innovation）
    float innov_x = gps_x - inav_x;
    float innov_y = gps_y - inav_y;

    // 跳变保护：若新息过大（GPS野值），压缩修正量
    float innov_dist = sqrtf(innov_x * innov_x + innov_y * innov_y);
    if(innov_dist > KF_GPS_MAX_JUMP_M)
    {
        float scale = KF_GPS_MAX_JUMP_M / innov_dist;
        innov_x *= scale;
        innov_y *= scale;
        wireless_printf("[KF] GPS jump clipped: %.2fm -> %.2fm\r\n",
                        innov_dist, KF_GPS_MAX_JUMP_M);
    }

    float old_x = inav_x, old_y = inav_y;

    // 状态更新
    inav_x += Kx * innov_x;
    inav_y += Ky * innov_y;

    // 方差更新：P = (1 - K) * P
    kf_Px *= (1.0f - Kx);
    kf_Py *= (1.0f - Ky);

    wireless_printf("[KF] GPS update: ins(%.2f,%.2f)->fused(%.2f,%.2f) K=(%.3f,%.3f) R=%.2f\r\n",
                    old_x, old_y, inav_x, inav_y, Kx, Ky, R_meas);
}

// ─── 循迹运行时位置修正（对外接口，保持兼容） ─────────────────────────────────

uint8 gps_fusion_correct_position(void)
{
    if(!s_origin_valid || !kf_init_done)
        return 0;
    if(!gnss.state || gnss.satellite_used < KF_MIN_SATS_FOR_UPDATE)
        return 0;

    float gps_ix, gps_iy;
    gps_fusion_latlon_to_inav(gnss.latitude, gnss.longitude, &gps_ix, &gps_iy);

    // 根据卫星数动态调整观测噪声（卫星少 → R 更大 → 更不信任GPS）
    float R_meas = KF_MEAS_NOISE_R_BASE;
    if(gnss.satellite_used < KF_SAT_GOOD_COUNT)
    {
        uint8 sat_deficit = KF_SAT_GOOD_COUNT - gnss.satellite_used;
        float penalty = 1.0f;
        uint8 i;
        for(i = 0; i < sat_deficit; i++)
            penalty *= KF_SAT_PENALTY_FACTOR;
        R_meas *= penalty;
    }

    kf_do_gps_update(gps_ix, gps_iy, R_meas);
    return 1;
}

// ─── 主循环更新（50ms） ───────────────────────────────────────────────────────

void gps_fusion_update(void)
{
    // 解析 GPS 串口数据
    if(gnss_flag)
    {
        gnss_data_parse();
        gnss_flag = 0;

        // 如果循迹中且有有效GPS，立即做卡尔曼观测更新（连续修正，不等路径点）
        if(inav_active && kf_init_done && s_origin_valid)
        {
            if(gnss.state && gnss.satellite_used >= KF_MIN_SATS_FOR_UPDATE)
            {
                // 判断是否是新的GPS位置（避免同一帧数据重复更新）
                if(gnss.latitude != s_last_gps_lat || gnss.longitude != s_last_gps_lon)
                {
                    s_last_gps_lat = gnss.latitude;
                    s_last_gps_lon = gnss.longitude;

                    float gps_ix, gps_iy;
                    gps_fusion_latlon_to_inav(gnss.latitude, gnss.longitude,
                                              &gps_ix, &gps_iy);

                    // 根据卫星数动态调整观测噪声
                    float R_meas = KF_MEAS_NOISE_R_BASE;
                    if(gnss.satellite_used < KF_SAT_GOOD_COUNT)
                    {
                        uint8 sat_deficit = KF_SAT_GOOD_COUNT - gnss.satellite_used;
                        float penalty = 1.0f;
                        uint8 i;
                        for(i = 0; i < sat_deficit; i++)
                            penalty *= KF_SAT_PENALTY_FACTOR;
                        R_meas *= penalty;
                    }

                    kf_do_gps_update(gps_ix, gps_iy, R_meas);
                }
            }
        }
    }

    // 驱动采样状态机
    if(gps_sampler_state != GPS_SAMPLER_BUSY)
        return;

    // 当前帧是否有效
    if(!gnss.state || gnss.satellite_used < GPS_SAMPLER_MIN_SATS)
        return;

    // 存入缓冲区
    if(s_sample_cnt < GPS_SAMPLER_TOTAL_COUNT)
    {
        s_lat_buf[s_sample_cnt] = gnss.latitude;
        s_lon_buf[s_sample_cnt] = gnss.longitude;
        s_sample_cnt++;
    }

    // 达到目标帧数，执行后处理
    if(s_sample_cnt >= GPS_SAMPLER_TOTAL_COUNT)
    {
        sampler_finalize();
    }
}

