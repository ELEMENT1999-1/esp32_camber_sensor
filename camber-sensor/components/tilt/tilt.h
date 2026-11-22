#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 板面倾角解算（三轴合成，竖直放置 = 0°）
 *
 * 加速度计静态角度：theta = atan2(az, sqrt(ax²+ay²)) = asin(az/|g|)
 *   板面竖直（Z 轴水平）：az≈0 -> 0°，与板面内旋转方向无关（真·三轴角度）
 *   板面水平（Z 轴朝天）：-> +90°
 *
 * 抗振动策略：
 *   1) 对加速度矢量做 TILT_WIN 拍滑动平均后再解 atan2，
 *      周期振动在矢量域抵消，消除逐拍解角的系统性偏差；
 *   2) 陀螺 X/Y 轴按运动学 du_z/dt = ωy·ux - ωx·uy 融合
 *      （含双轴零偏标定），任意面内朝向一致；
 *   3) 重力模长偏离 1g 或新息过大时自动切到陀螺主导。
 */

#define TILT_WIN 128
#define TILT_BIAS_SAMPLES 500   /* 启动零偏标定拍数（500Hz 下 1s，需保持静止） */

typedef struct {
    float angle_deg;          /* 滤波后主倾角（板面倾角，绕 X，未扣校准/偏置） */
    float roll_deg;           /* 左右倾斜（绕 Z 面内侧倾，窗口矢量直接解算，供刻度尺） */
    float accel_angle_deg;    /* 窗口平均后的加速度计角度 */
    float offset_deg;         /* 归零偏置 */
    float cal_offset_deg;     /* 两点校准零点（PC 端写入） */
    float cal_scale;          /* 两点校准比例系数（PC 端写入，默认 1） */
    float alpha;              /* 互补滤波系数（默认 0.99） */
    float alpha_dyn;          /* 根据置信度动态调整后的实际系数 */
    float gyro_x_dps;         /* 最近一拍陀螺 X 轴角速度 °/s（含零偏） */
    float gyro_y_dps;         /* 最近一拍陀螺 Y 轴角速度 °/s（含零偏） */
    float gyro_bias_x_dps;    /* 陀螺 X 轴零偏估计 °/s */
    float gyro_bias_y_dps;    /* 陀螺 Y 轴零偏估计 °/s */
    float gravity_g;          /* 窗口平均加速度模长 / g */
    bool gravity_ok;          /* 重力模长可信 */
    bool inverted;            /* 显示符号取反（左右轮镜像安装） */
    bool primed;              /* 窗口已填满并完成初始化 */
    bool bias_primed;         /* 零偏标定完成 */
    /* 零偏标定 */
    float bias_sum_x, bias_sum_y;
    uint32_t bias_count;
    /* 矢量平均窗口 */
    float buf_ax[TILT_WIN];
    float buf_ay[TILT_WIN];
    float buf_az[TILT_WIN];
    float sum_ax, sum_ay, sum_az;
    uint16_t win_idx;
    uint16_t win_count;
} tilt_t;

void tilt_init(tilt_t *t, float alpha);
void tilt_set_offset(tilt_t *t, float offset_deg);
void tilt_invert(tilt_t *t, bool inverted);
void tilt_reset_filter(tilt_t *t);
/* 重新启动陀螺零偏标定（需静止约 1s） */
void tilt_start_bias_cal(tilt_t *t);
/* 两点校准：θ_corrected = (θ_raw - offset) * scale */
void tilt_set_cal(tilt_t *t, float cal_offset_deg, float cal_scale);
/* 校准后的主倾角（未扣归零偏置） */
float tilt_corrected(const tilt_t *t);
/* 左右倾斜（绕 Z，面内侧倾）：上沿右倾为正 */
float tilt_get_roll(const tilt_t *t);

/* 传入加速度（单位统一，模长≈1g）与陀螺 X/Y 轴 °/s，dt 为采样间隔秒 */
float tilt_update(tilt_t *t, float ax, float ay, float az,
                  float gyro_x_dps, float gyro_y_dps, float dt);

/* 当前显示角度 = (angle - offset) * (inverted ? -1 : +1) */
float tilt_get_angle(const tilt_t *t);

#ifdef __cplusplus
}
#endif
