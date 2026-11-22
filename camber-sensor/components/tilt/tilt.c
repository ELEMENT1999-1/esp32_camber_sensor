#include "tilt.h"

#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#define RAD_TO_DEG (180.0f / (float)M_PI)
#define ACC_TRUST_BAND 0.03f      /* 平均矢量 |g-1| 小于此值信任加速度计 */
#define ACC_REJECT_BAND 0.10f     /* 超过此值只用陀螺积分 */
#define INNOVATION_GATE_DEG 15.0f /* 平均角度与预测偏差过大时视为线加速度干扰 */
#define GYRO_STATIC_MAX 1.5f      /* |ω| 小于此值视为静止，直接信任加速度计 */
#define GYRO_ONLY_ALPHA 0.9995f
#define BIAS_ADAPT_GAIN 0.0005f   /* 静止时零偏慢速跟踪（约 4s 时间常数） */
#define BIAS_ADAPT_GYRO_MAX 5.0f  /* |ω-bias| 小于此值才允许更新零偏 */
#define BIAS_ADAPT_INNOV_MAX 2.0f /* 角度差小于此时才允许更新零偏 */
#define UXY_MIN 0.05f             /* 面内分量过小时陀螺融合退化保护 */

static float wrap180(float a)
{
    while (a > 180.0f) a -= 360.0f;
    while (a < -180.0f) a += 360.0f;
    return a;
}

void tilt_init(tilt_t *t, float alpha)
{
    memset(t, 0, sizeof(*t));
    t->alpha = (alpha > 0.5f && alpha < 1.0f) ? alpha : 0.99f;
    t->alpha_dyn = t->alpha;
    t->cal_scale = 1.0f;
    t->gravity_ok = true;
}

void tilt_set_offset(tilt_t *t, float offset_deg)
{
    t->offset_deg = offset_deg;
}

void tilt_set_cal(tilt_t *t, float cal_offset_deg, float cal_scale)
{
    t->cal_offset_deg = cal_offset_deg;
    t->cal_scale = (cal_scale > 0.5f && cal_scale < 1.5f) ? cal_scale : 1.0f;
}

float tilt_corrected(const tilt_t *t)
{
    return wrap180((t->angle_deg - t->cal_offset_deg) * t->cal_scale);
}

float tilt_get_roll(const tilt_t *t)
{
    return t->roll_deg;
}

void tilt_invert(tilt_t *t, bool inverted)
{
    t->inverted = inverted;
}

void tilt_reset_filter(tilt_t *t)
{
    t->primed = false;
    t->win_idx = 0;
    t->win_count = 0;
    t->sum_ax = 0.0f;
    t->sum_ay = 0.0f;
    t->sum_az = 0.0f;
}

void tilt_start_bias_cal(tilt_t *t)
{
    t->bias_primed = false;
    t->bias_count = 0;
    t->bias_sum_x = 0.0f;
    t->bias_sum_y = 0.0f;
    t->primed = false;
}

static void win_push(tilt_t *t, float ax, float ay, float az)
{
    if (t->win_count == TILT_WIN) {
        t->sum_ax -= t->buf_ax[t->win_idx];
        t->sum_ay -= t->buf_ay[t->win_idx];
        t->sum_az -= t->buf_az[t->win_idx];
    } else {
        t->win_count++;
    }
    t->buf_ax[t->win_idx] = ax;
    t->buf_ay[t->win_idx] = ay;
    t->buf_az[t->win_idx] = az;
    t->sum_ax += ax;
    t->sum_ay += ay;
    t->sum_az += az;
    t->win_idx = (uint16_t)((t->win_idx + 1) % TILT_WIN);
}

float tilt_update(tilt_t *t, float ax, float ay, float az,
                  float gyro_x_dps, float gyro_y_dps, float dt)
{
    if (dt <= 0.0f || dt > 0.1f) {
        dt = 0.002f;
    }

    t->gyro_x_dps = gyro_x_dps;
    t->gyro_y_dps = gyro_y_dps;

    /* 先做窗口矢量平均再解角度：周期振动（零均值线加速度）在矢量域相互抵消 */
    win_push(t, ax, ay, az);

    float n = (float)t->win_count;
    float mx = t->sum_ax / n;
    float my = t->sum_ay / n;
    float mz = t->sum_az / n;

    float mag = sqrtf(mx * mx + my * my + mz * mz);
    t->gravity_g = mag;

    /* 板面倾角 = Z 轴相对水平面的倾角 = asin(az/|g|)，三轴合成、面内旋转无关 */
    float uxy = sqrtf(mx * mx + my * my);
    float accel_angle = atan2f(mz, uxy) * RAD_TO_DEG;
    t->accel_angle_deg = accel_angle;

    /* 左右倾斜（绕 Z 面内侧倾）：上沿右倾为正，窗口矢量直接解算 */
    t->roll_deg = atan2f(-mx, my) * RAD_TO_DEG;

    /* 启动零偏标定：前 TILT_BIAS_SAMPLES 拍保持静止，期间只跟随加速度角度 */
    if (!t->bias_primed) {
        t->bias_sum_x += gyro_x_dps;
        t->bias_sum_y += gyro_y_dps;
        t->bias_count++;
        if (t->bias_count >= TILT_BIAS_SAMPLES) {
            t->gyro_bias_x_dps = t->bias_sum_x / (float)t->bias_count;
            t->gyro_bias_y_dps = t->bias_sum_y / (float)t->bias_count;
            t->bias_primed = true;
            t->primed = false;
        }
        t->angle_deg = accel_angle;
        t->alpha_dyn = t->alpha;
        t->gravity_ok = true;
        return tilt_get_angle(t);
    }

    float gx = gyro_x_dps - t->gyro_bias_x_dps;
    float gy = gyro_y_dps - t->gyro_bias_y_dps;

    /* 运动学配对：du_z/dt = ωy·ux - ωx·uy，dθ/dt = du_z/dt / u_xy
       使任意面内朝向下陀螺积分方向都与三轴角度一致 */
    float gyro = 0.0f;
    if (mag > 0.1f && uxy > UXY_MIN) {
        float ux = mx / mag;
        float uy = my / mag;
        gyro = (gy * ux - gx * uy) * (mag / uxy);
    }

    if (!t->primed || t->win_count < TILT_WIN) {
        t->angle_deg = accel_angle;
        t->primed = (t->win_count >= TILT_WIN);
        t->alpha_dyn = t->alpha;
        t->gravity_ok = true;
    } else {
        float dev = fabsf(mag - 1.0f);
        float w;
        if (dev <= ACC_TRUST_BAND) {
            w = 1.0f;
        } else if (dev >= ACC_REJECT_BAND) {
            w = 0.0f;
        } else {
            w = 1.0f - (dev - ACC_TRUST_BAND) / (ACC_REJECT_BAND - ACC_TRUST_BAND);
        }

        float predicted = wrap180(t->angle_deg + gyro * dt);
        float diff = wrap180(accel_angle - predicted);

        /* 新息门限：动态时角度偏差过大 → 线加速度，不采用。
           静止时不启用门限，防止大偏差后死锁无法收敛 */
        bool is_static = (fabsf(gx) < GYRO_STATIC_MAX && fabsf(gy) < GYRO_STATIC_MAX &&
                          dev <= ACC_TRUST_BAND);
        if (!is_static && fabsf(diff) > INNOVATION_GATE_DEG) {
            w = 0.0f;
        }

        t->gravity_ok = (w > 0.5f);
        t->alpha_dyn = GYRO_ONLY_ALPHA + (t->alpha - GYRO_ONLY_ALPHA) * w;

        t->angle_deg = wrap180(predicted + (1.0f - t->alpha_dyn) * diff);

        /* 静止且重力可信时慢速跟踪零偏温漂 */
        if (t->gravity_ok && fabsf(gx) < BIAS_ADAPT_GYRO_MAX &&
            fabsf(gy) < BIAS_ADAPT_GYRO_MAX && fabsf(diff) < BIAS_ADAPT_INNOV_MAX) {
            t->gyro_bias_x_dps += (gyro_x_dps - t->gyro_bias_x_dps) * BIAS_ADAPT_GAIN;
            t->gyro_bias_y_dps += (gyro_y_dps - t->gyro_bias_y_dps) * BIAS_ADAPT_GAIN;
        }
    }

    return tilt_get_angle(t);
}

float tilt_get_angle(const tilt_t *t)
{
    float a = wrap180(tilt_corrected(t) - t->offset_deg);
    return t->inverted ? -a : a;
}
