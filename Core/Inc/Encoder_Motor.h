#ifndef __ENCODER_MOTOR_H__
#define __ENCODER_MOTOR_H__

#include "tim.h"

/* ================= 机械/编码器参数（请按实际硬件修改） ================= */
#define ENCODER_PPR   13    /* 编码器每圈脉冲数（线数/PPR）：MG513 = 13 */
#define GEAR_RATIO    30    /* 减速比：输出轴转1圈 = 电机轴转30圈（MG513 = 1:30） */

/* ---- 以下仅用于把"编码器计数"换算成速度/距离，控制环不使用 ---- */
#define WHEEL_DIA_MM  65.0f /* 车轮直径 mm（实测 = 65mm） */
/* 每米计数 = PPR × 4(四倍频) × GEAR_RATIO ÷ (π × 轮径m)
 *          = 13 × 4 × 30 ÷ (π × 0.065) ≈ 7639 计数/米
 *
 * == 换算参考：把 CH2 读数换算成实际车速 ==
 *   车速(m/s) = CH2读数(计数/ms) × 1000 ÷ 7639
 *   例: CH2 = 5 计数/ms  →  5 × 1000 ÷ 7639 ≈ 0.65 m/s
 *
 * == 硬件变更历史（影响速度环增益的量级）==
 *   MG310 1:20 + 48mm  →  6896 计数/米   （旧）
 *   MG513 1:30 + 65mm  →  7639 计数/米   （现）
 *   两者量级接近（约 1.11 倍），速度环增益仍沿用同一数量级，
 *   但因为换了电机和减速比，必须从小值重新标定。
 *
 * 注意: 该差异只影响"速度换算成 m/s"，不影响平衡控制（控制用原始计数）。 */

void Set_Motor_A_Speed(int speed);
void Set_Motor_B_Speed(int speed);

/* 读取编码器自上次读取以来的增量计数（自动清零，带方向符号） */
int16_t Encoder_A_GetCount(void);
int16_t Encoder_B_GetCount(void);

/* 测速：dt_ms 为两次读取间隔(ms)，返回【输出轴】转速，单位 RPM */
float Encoder_A_Get_RPM(float dt_ms);
float Encoder_B_Get_RPM(float dt_ms);

#endif
