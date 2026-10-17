/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "g356_app.h"
#include "oled.h"
#include "encoder_motor.h"
#include <string.h>   /* memcpy 用于协议数据读写 */
#include "Ganwei.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* 控制输出与速度环参数 */
#define PWM_LIMIT     7199    /* PWM 最大计数值，与定时器周期一致 */
#define SPEED_ILIM    50.0f   /* 速度误差积分限幅 */
#define SPEED_WINDOW  50      /* 每 50 个 1 ms 控制周期更新一次速度环 */
#define GRAY_SCAN_MS  10u     /* 主循环每 10 ms 读取一次八路灰度 */
#define OLED_UPDATE_MS 50u   /* OLED 每 50 ms 刷新一次 */
#define LINE_FOLLOW_ENABLE 1u /* 1：根据灰度调整转向；0：只做原有平衡控制 */
#define LINE_TURN_KP 70.0f    /* 每单位横向误差对应的差速 PWM */
#define LINE_TURN_LIMIT 500.0f /* 限制循迹转向对平衡的干扰 */
#define LINE_DATA_TIMEOUT_MS 100u /* 灰度数据过期后停止按原误差前进 */
#define LINE_LOST_MEMORY_MS 2500u /* 丢线后最多沿最后方向寻找 2500 ms */
#define LINE_SEARCH_PWM 475     /* 丢线搜索时的固定差速 PWM */
#define LINE_RECENTER_FRAMES 1u /* 搜索后 CH2～CH7 任一路检出 1 次即恢复前进 */
#define LINE_RIGHT_HINT_MS 150u /* 右侧边缘见线后短时保留右转方向 */
#define MOTOR_START_DELAY_MS 15000u /* 上电后等待 15 秒再允许启动电机 */
#define ROUNDABOUT_DONE_DEG 280.0f /* 环岛累计转角达到 280 度直接进入 OK */
#define ROUNDABOUT_FIRST_EXIT_CLEAR_MS 50u /* 两侧边缘离线且中间四路见线 50 ms 后再识别入口 */
#define ROUNDABOUT_ENTRY_TARGET_DEG 20.0f /* 两侧入口固定差速转到 20 度后找线 */
#define ROUNDABOUT_YAW_SIGN_LOCK_DEG 2.0f /* 用实际转动的前 2 度确定 yaw 正方向 */
#define ROUNDABOUT_ENTRY_MAX_MS 3000u /* yaw 异常时 3 秒结束强制转弯 */
#define ROUNDABOUT_LINE_CATCH_MS 2500u /* 入岛后最多主动找线 2.5 秒 */
#define ROUNDABOUT_LINE_CATCH_PWM 220.0f /* 找线阶段降低差速 */
#define ROUNDABOUT_W1_ENTRY_SPEED_TARGET -0.9f /* W1 循迹及 IN 转向的速度目标 */
#define ROUNDABOUT_CATCH_SPEED_TARGET -0.9f /* OR 找线阶段的速度目标 */
#define ROUNDABOUT_DONE_REARM_MS 10000u /* OK 后 10 秒内不重新识别路口，保持 OK 而不进入 W1 */
#define ROUNDABOUT_EDGE_ERROR_DELAY_MS 5000u /* OK 后按入岛方向屏蔽部分探头 5 秒 */
#define ROUNDABOUT_ENTRY_PWM 350.0f /* IN 全程朝选定一侧转向的固定差速 PWM */
#define MAP_LAP_YAW_DEG 720.0f /* 外圈与内圈各转一周，合计 720 度 */
#define MAP_LAP_YAW_TOLERANCE_DEG 30.0f /* 到达 690 度后等待灰度居中及角速度稳定 */
#define MAP_LAP_QUARTER_DEG 90.0f /* 净转角每满 90 度增加一个进度 */
#define MAP_LAP_LOOP_SWITCH_DEG 345.0f /* 第一圈接近 360 度后直接开始第二圈 */
#define MAP_LAP_STABLE_GYRO_DPS 15.0f /* 计圈前允许的最大偏航角速度 */
#define MAP_LAP_STABLE_MS 150u /* 角速度稳定且灰度居中需连续保持 150 ms */
#define MAP_LAP_STOP_DELAY_MS 4000u /* 最后一圈确认后继续行驶 4 秒 */

/* 速度环输出的方向系数。若实测纠偏方向相反，应调整该符号。 */
#define SPEED_SIGN    1.0f
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* 角度环参数，可通过 USART1 调参协议在线修改。角度单位为度。 */
static volatile float  BALANCE_KP   = 460.0f;
static volatile float  BALANCE_KD   = 28.0f;
static volatile float  BALANCE_ZERO = -0.85f;   /* 车体直立时的前倾角设定值 */
static volatile uint8_t s_ctrl_enable = 1;     /* 0：停止输出；1：启用平衡控制 */

/* USART1 调参协议接收状态 */
static uint8_t  s_wf_buf[32];
static uint8_t  s_wf_idx = 0;
static uint32_t s_wf_period    = 20;   /* 遥测上报周期，单位 ms */
static uint32_t s_wf_last_tick = 0;

/* 速度环状态 */
static int32_t  s_enc_sum = 0;
static uint16_t s_enc_cnt = 0;
static float    s_speed_integral = 0.0f;
static float    s_speed_out      = 0.0f;  /* 速度环输出，单位：度 */
static float    s_speed_meas     = 0.0f;  /* 平均编码器计数/控制周期 */

/* 速度环参数，可通过调参协议 CH2 在线修改。 */
static volatile float SPEED_KP     = 3.0f;
static volatile float SPEED_KI     = 0.02f;
static volatile float SPEED_TARGET = -0.9f;    /* 目标编码器计数/控制周期 */

/* 主循环发布循迹结果，控制中断只读取，不进行阻塞式 ADC 采样。 */
static volatile int16_t s_line_turn_pwm = 0;
static volatile uint8_t s_line_valid = 0u;
static volatile uint8_t s_line_search = 0u;
static volatile uint32_t s_line_last_tick = 0u;
static volatile uint32_t s_line_search_start_tick = 0u;

/* 灰度状态由主循环判定，控制中断只消费已计算好的行驶命令。 */
typedef enum {
    ROUND_WAIT_FIRST_EXIT = 0, /* 沿原线首次经过出口 */
    ROUND_CLEAR_FIRST_EXIT,    /* 等待离开首次出口 */
    ROUND_WAIT_ENTRY,          /* 等待首次出口同侧的入口特征 */
    ROUND_ENTRY_TURN,          /* 固定差速向选定方向转至目标角 */
    ROUND_ORBIT,               /* 绕行并累计 yaw */
    ROUND_DONE                 /* 转到 280 度后普通循迹，等待再次入岛 */
} RoundaboutState_t;

typedef enum {
    ROUND_DRIVE_LINE = 0, /* 普通循迹转向 */
    ROUND_DRIVE_ENTRY,    /* IN 阶段的强制入圈转向 */
    ROUND_DRIVE_CATCH,    /* 转到目标角后主动找线 */
    ROUND_DRIVE_ORBIT,    /* 环岛内以灰度循迹为主 */
} RoundaboutDrive_t;

/* 环岛阶段与当前转向命令：主循环更新，定时器中断读取带 volatile 的输出。 */
static volatile RoundaboutState_t s_round_state = ROUND_WAIT_FIRST_EXIT; /* W0/X0/W1/IN/OR/OK 阶段 */
static volatile uint8_t s_round_drive = ROUND_DRIVE_LINE; /* 当前使用哪一种转向控制 */
static volatile int16_t s_round_turn_pwm = 0; /* 环岛动作输出的带方向差速 PWM */
static volatile int8_t s_round_direction = 0; /* 已进入 IN 的方向：-1 左，1 右 */
static int8_t s_round_expected_direction = 0; /* 首次出口锁定的入口侧：-1 左，1 右 */

/* 入圈 yaw：用相邻两帧之差累计，初始 2 度用于确定传感器的正负方向。 */
static uint8_t s_round_turn_tracking = 0u; /* 是否已有上一帧 yaw 可用于求差 */
static float s_round_previous_yaw = 0.0f; /* 上一帧 yaw */
static float s_round_turn_deg = 0.0f; /* 本次入圈累计转角，OLED 显示为 T */
static float s_round_yaw_probe = 0.0f; /* 确定 yaw 正负方向前累计的转角 */
static int8_t s_round_yaw_sign = 0; /* 0 未锁定，±1 表示实际入圈转动的 yaw 符号 */

/* 转到目标角后的找线记录。 */
static uint8_t s_round_orbit_line_latched = 0u; /* IN/OR 中间四路见线后置 1 */
static uint8_t s_round_orbit_catch_enabled = 0u; /* 是否允许定时主动找线 */
static uint32_t s_round_orbit_catch_tick = 0u; /* 主动找线起始时刻，单位 ms */

/* HAL_GetTick() 计时点：分别用于 IN 超时、X0 确认和 OK 后重新识别。 */
static uint32_t s_round_state_tick = 0u; /* IN 开始时刻 */
static uint32_t s_round_clear_tick = 0u; /* X0 满足离开首次出口条件的起始时刻 */
static uint32_t s_round_done_tick = 0u; /* 进入 OK 的时刻 */
static uint32_t s_round_edge_error_hold_tick = 0u; /* OK 后探头屏蔽的起始时刻 */
static uint8_t s_round_edge_error_hold_active = 0u; /* OK 后是否仍需屏蔽部分探头 */
static int8_t s_round_edge_error_hold_direction = 0; /* OK 时保存入圈方向，决定屏蔽哪侧探头 */
static uint8_t s_round_imu_paused = 0u; /* 标记 IMU 数据过期，恢复后补偿部分阶段计时 */
static uint32_t s_round_imu_pause_tick = 0u; /* IMU 暂停起始时刻 */

/* 蓝牙发送 ASCII '1'～'5' 设置整图圈数；命令由 USART1 中断交给主循环。 */
static volatile uint8_t s_lap_pending_target = 0u; /* 中断收到的新目标，主循环取走后清零 */
static volatile uint8_t s_lap_running = 1u; /* 1：继续行驶；0：圈数已到，前进和转向归零 */
static uint8_t s_lap_target = 0u; /* 0：不限圈数；1～5：目标整图圈数 */
static uint8_t s_lap_completed = 0u; /* 已确认完成的整图圈数 */

/* 整图一圈由两段净 yaw 组成；每段反向摇摆会抵消，不按瞬时转角累加。 */
static uint8_t s_lap_yaw_valid = 0u; /* 是否已记录有效的计圈 yaw 起点 */
static float s_lap_previous_yaw = 0.0f; /* 上一帧计圈 yaw */
static float s_lap_phase_yaw = 0.0f; /* 当前段的有符号净转角 */
static float s_lap_first_loop_yaw = 0.0f; /* 第一段结束时保存的转角绝对值 */
static float s_lap_progress_deg = 0.0f; /* OLED 的 A：按 90 度显示的计圈进度 */
static uint8_t s_lap_second_loop = 0u; /* 0：第一段；1：已切换到第二段 */
static uint8_t s_lap_angle_reached = 0u; /* 两段合计达到 690 度门槛 */
static uint32_t s_lap_stable_since = 0u; /* 灰度居中且 yaw 角速度稳定的起始时刻 */
static uint8_t s_lap_stop_pending = 0u; /* 已到目标圈数，正在执行最后 4 秒行驶 */
static uint32_t s_lap_stop_tick = 0u; /* 最后 4 秒行驶的起始时刻 */

/* IMU 初始化期间将 PWM 引脚保持为低电平，防止电机误动作。 */
static void Motor_PwmPins_SetSafeLow(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET); /* TIM1_CH2 / PWMB */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET); /* TIM4_CH3 / PWMA */

    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

    GPIO_InitStruct.Pin = GPIO_PIN_9;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = GPIO_PIN_8;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

static void Motor_PwmPins_EnableTimerOutput(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

    GPIO_InitStruct.Pin = GPIO_PIN_9;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = GPIO_PIN_8;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* 将传感器 yaw 变化限制在 -180～180 度。 */
static float Roundabout_WrapAngle(float angle)
{
    if (angle > 180.0f) angle -= 360.0f;
    if (angle < -180.0f) angle += 360.0f;
    return angle;
}

/* 计圈确认用原始八路状态：CH4/CH5 至少一路见线、CH1/CH8 均离线，
 * 且所有见线探头的平均位置处于中心 ±1；此判断不使用循迹时的探头屏蔽。 */
static uint8_t MapLap_LineCentered(uint8_t state)
{
    static const int8_t positions[8] = {-12, -7, -3, -1, 1, 3, 7, 12};
    int16_t position_sum = 0;
    uint8_t active_count = 0u;

    if ((state & 0x18u) == 0u || (state & 0x81u) != 0u) return 0u;
    for (uint8_t i = 0u; i < 8u; i++) {
        if ((state & (1u << i)) != 0u) {
            position_sum += positions[i];
            active_count++;
        }
    }
    return position_sum >= -(int16_t)active_count &&
           position_sum <= (int16_t)active_count;
}

/* 收到新的 1～5 圈命令时，从下一帧有效 yaw 重新计数；行驶中不重置环岛阶段。 */
static void MapLap_Start(uint8_t target)
{
    uint8_t was_running = s_lap_running;
    s_lap_target = target;
    s_lap_completed = 0u;
    s_lap_yaw_valid = 0u; /* 下一帧有效 yaw 作为新的计圈起点。 */
    s_lap_previous_yaw = 0.0f;
    s_lap_phase_yaw = 0.0f;
    s_lap_first_loop_yaw = 0.0f;
    s_lap_progress_deg = 0.0f;
    s_lap_second_loop = 0u;
    s_lap_angle_reached = 0u;
    s_lap_stable_since = 0u;
    s_lap_stop_pending = 0u;
    s_lap_stop_tick = 0u;
    /* 蓝牙命令从当前时刻重新累计整图圈数。 */
    if (!was_running) {
        /* 已完成上一任务后重新启动，才重置环岛识别；行驶中设圈数不打断入圈。 */
        s_round_state = ROUND_WAIT_FIRST_EXIT;
        s_round_drive = ROUND_DRIVE_LINE;
        s_round_turn_pwm = 0;
        s_round_direction = 0;
        s_round_expected_direction = 0;
        s_round_turn_tracking = 0u;
        s_round_imu_paused = 0u;
        s_round_turn_deg = 0.0f;
        s_round_edge_error_hold_active = 0u;
        s_round_edge_error_hold_direction = 0;
    }
    s_lap_running = 1u;
}

/* 一张地图按两段净 yaw 计角；两段允许转向相反，段内反向摆动会抵消。
 * 达到 690 度后，还需灰度居中和偏航角速度连续稳定 150 ms 才记一圈。 */
static void MapLap_Update(uint8_t gray_state)
{
    G356_Data_t imu;
    float delta;
    float progress;
    float phase_abs;
    uint8_t phase_quarters;
    /* 达到目标圈数后继续按原控制行驶 4 秒，再将前进目标和转向归零；
     * 角度环及目标为零的速度环仍继续运行。 */
    if (s_lap_stop_pending) {
        if (HAL_GetTick() - s_lap_stop_tick >= MAP_LAP_STOP_DELAY_MS) {
            s_lap_stop_pending = 0u;
            s_lap_running = 0u; /* 速度目标和转向归零，角度环继续保持平衡。 */
            s_round_drive = ROUND_DRIVE_LINE;
            s_round_turn_pwm = 0;
        }
        return;
    }
    if (s_lap_target != 0u && !s_lap_running) return;
    if (!G356_App_DataFresh(150u)) {
        s_lap_yaw_valid = 0u;
        s_lap_stable_since = 0u;
        return;
    }
    G356_App_CopyData(&imu);
    if (!s_lap_yaw_valid) {
        s_lap_previous_yaw = imu.yaw;
        s_lap_yaw_valid = 1u;
        return;
    }
    delta = Roundabout_WrapAngle(imu.yaw - s_lap_previous_yaw);
    s_lap_previous_yaw = imu.yaw;
    if (delta <= -45.0f || delta >= 45.0f) {
        s_lap_stable_since = 0u;
        return; /* 忽略异常跳变。 */
    }
    s_lap_phase_yaw += delta;
    phase_abs = s_lap_phase_yaw < 0.0f ? -s_lap_phase_yaw : s_lap_phase_yaw;
    if (!s_lap_second_loop && phase_abs >= MAP_LAP_LOOP_SWITCH_DEG) {
        /* 第一段净转角达到 345 度时保存进度，并从当前航向开始第二段。 */
        s_lap_first_loop_yaw = phase_abs;
        s_lap_second_loop = 1u;
        s_lap_phase_yaw = 0.0f;
        phase_abs = 0.0f;
    }
    phase_quarters = phase_abs >= 4.0f * MAP_LAP_QUARTER_DEG -
                                  MAP_LAP_YAW_TOLERANCE_DEG ?
                     4u : (uint8_t)(phase_abs / MAP_LAP_QUARTER_DEG);
    s_lap_progress_deg = (float)((s_lap_second_loop ? 4u : 0u) +
                                 phase_quarters) * MAP_LAP_QUARTER_DEG;
    progress = s_lap_first_loop_yaw + phase_abs;

    if (!s_lap_angle_reached && s_lap_second_loop &&
        progress >= MAP_LAP_YAW_DEG - MAP_LAP_YAW_TOLERANCE_DEG) {
        s_lap_angle_reached = 1u;
        s_lap_stable_since = 0u; /* 从到达目标角后才开始计稳定时间。 */
    }
    if (s_lap_angle_reached &&
        MapLap_LineCentered(gray_state) &&
        imu.gyro_z > -MAP_LAP_STABLE_GYRO_DPS &&
        imu.gyro_z < MAP_LAP_STABLE_GYRO_DPS) {
        if (s_lap_stable_since == 0u) s_lap_stable_since = HAL_GetTick();
    } else {
        s_lap_stable_since = 0u;
    }

    /* 角度门槛一旦达到便保持置位；即使随后转过 720 度，仍可等待稳定条件。 */
    if (s_lap_running &&
        s_lap_angle_reached &&
        s_lap_stable_since != 0u &&
        HAL_GetTick() - s_lap_stable_since >= MAP_LAP_STABLE_MS) {
        if (s_lap_completed < 9u) s_lap_completed++;
        s_lap_phase_yaw = 0.0f;
        s_lap_first_loop_yaw = 0.0f;
        s_lap_progress_deg = 0.0f;
        s_lap_second_loop = 0u;
        s_lap_angle_reached = 0u;
        s_lap_stable_since = 0u;
        s_lap_previous_yaw = imu.yaw; /* 从当前 yaw 开始累计下一圈。 */
        if (s_lap_target != 0u && s_lap_completed >= s_lap_target) {
            s_lap_stop_pending = 1u;
            s_lap_stop_tick = HAL_GetTick(); /* 保持原有行驶控制再运行 4 秒。 */
        }
    }
}

/* 入岛时从 0 度累计实际 yaw：先用前 2 度确定正方向，之后反向转动扣回。 */
static void Roundabout_CountTurn(float yaw)
{
    if (s_round_turn_tracking) {
        float delta = Roundabout_WrapAngle(yaw - s_round_previous_yaw);
        if (delta > -45.0f && delta < 45.0f) {
            /* 从实际转动方向学习 yaw 符号，反向转动仍会扣回累计角。 */
            if (s_round_yaw_sign == 0) {
                s_round_yaw_probe += delta;
                if (s_round_yaw_probe >= ROUNDABOUT_YAW_SIGN_LOCK_DEG) {
                    s_round_yaw_sign = 1;
                    s_round_turn_deg = s_round_yaw_probe;
                } else if (s_round_yaw_probe <= -ROUNDABOUT_YAW_SIGN_LOCK_DEG) {
                    s_round_yaw_sign = -1;
                    s_round_turn_deg = -s_round_yaw_probe;
                }
            } else {
                s_round_turn_deg += delta * (float)s_round_yaw_sign;
            }
        }
    }
    s_round_previous_yaw = yaw;
    s_round_turn_tracking = 1u;
}

static int16_t Roundabout_EntryPwm(void)
{
    return (int16_t)((float)s_round_direction * ROUNDABOUT_ENTRY_PWM);
}

static void Roundabout_StartEntry(int8_t direction, uint32_t now,
                                  const G356_Data_t *imu)
{
    /* 新一轮入岛不沿用上一轮 OK 后的出岛探头屏蔽。 */
    s_round_edge_error_hold_active = 0u;
    s_round_edge_error_hold_direction = 0;
    s_round_direction = direction;
    s_round_previous_yaw = imu->yaw;
    s_round_turn_tracking = 1u;
    s_round_turn_deg = 0.0f;
    s_round_yaw_probe = 0.0f;
    s_round_yaw_sign = 0;
    s_round_orbit_line_latched = 0u;
    s_round_orbit_catch_enabled = 0u;
    s_round_state_tick = now;
    s_round_state = ROUND_ENTRY_TURN;
    s_round_drive = ROUND_DRIVE_ENTRY;
    s_round_turn_pwm = Roundabout_EntryPwm();
}

/* state 的 bit0～bit7 分别代表 CH1～CH8；黑线为 1。
 * 首次出口左侧要求 CH1=1、CH2/CH3=0、CH4～CH6 至少一路见线；
 * 右侧镜像要求 CH8=1、CH7/CH6=0、CH3～CH5 至少一路见线。
 * W1 左侧入口只要求 CH1/CH2 至少一路见线；右侧镜像只要求
 * CH7/CH8 至少一路见线。
 * 阶段顺序：W0 首次出口 → X0 离开出口 → W1 等入口 → IN 转向 →
 * OR 绕岛 → OK 普通循迹；入口方向在 W0 首次出口时锁定。 */
static void Roundabout_Update(uint8_t state, uint32_t now)
{
    uint8_t center_four = (state & 0x3Cu) != 0u; /* CH3～CH6 任一路。 */
    uint8_t left_middle = (state & 0x38u) != 0u; /* CH4、CH5、CH6 任一路。 */
    uint8_t right_middle = (state & 0x1Cu) != 0u; /* CH3、CH4、CH5 任一路。 */
    uint8_t first_left = left_middle && (state & 0x04u) == 0u &&
                         (state & 0x03u) == 0x01u;
    uint8_t first_right = right_middle && (state & 0x20u) == 0u &&
                          (state & 0xC0u) == 0x80u;
    uint8_t left_entry = (state & 0x03u) != 0u;
    uint8_t right_entry = (state & 0xC0u) != 0u;
    G356_Data_t imu;

    s_round_drive = ROUND_DRIVE_LINE;
    s_round_turn_pwm = 0;

    if (!G356_App_DataFresh(150u)) {
        if (!s_round_imu_paused) {
            s_round_imu_pause_tick = now;
            s_round_imu_paused = 1u;
        }
        s_round_turn_tracking = 0u;
        return;
    }
    if (s_round_imu_paused) {
        /* IMU 暂停期间不消耗入口等待及 OK 后重新识别的间隔。 */
        uint32_t pause_ms = now - s_round_imu_pause_tick;
        if (s_round_state >= ROUND_ENTRY_TURN && s_round_state <= ROUND_ORBIT) {
            s_round_state_tick += pause_ms;
        }
        if (s_round_state == ROUND_DONE) {
            s_round_done_tick += pause_ms;
        }
        if (s_round_state == ROUND_ORBIT && s_round_orbit_catch_enabled &&
            !s_round_orbit_line_latched) {
            s_round_orbit_catch_tick += pause_ms;
        }
        s_round_imu_paused = 0u;
    }
    G356_App_CopyData(&imu);

    if (s_round_state >= ROUND_ENTRY_TURN &&
        s_round_state <= ROUND_ORBIT) {
        Roundabout_CountTurn(imu.yaw);
    }

    switch (s_round_state) {
    case ROUND_WAIT_FIRST_EXIT:
    case ROUND_DONE:
        if (s_round_state == ROUND_DONE &&
            now - s_round_done_tick < ROUNDABOUT_DONE_REARM_MS) {
            break;
        }
        /* 从任一方向接近时，首次检出边缘支线就按出口跳过。 */
        if (first_left != first_right) {
            s_round_expected_direction = first_left ? -1 : 1;
            s_round_state = ROUND_CLEAR_FIRST_EXIT;
            s_round_clear_tick = 0u;
        }
        break;
    case ROUND_CLEAR_FIRST_EXIT:
        /* CH1/CH2/CH7/CH8 均离线，CH3～CH6 任一路见线持续 50 ms，才开放 W1。 */
        if ((state & 0xC3u) == 0u && center_four) {
            if (s_round_clear_tick == 0u) s_round_clear_tick = now;
            if (now - s_round_clear_tick >= ROUNDABOUT_FIRST_EXIT_CLEAR_MS) {
                s_round_state = ROUND_WAIT_ENTRY;
            }
        } else {
            s_round_clear_tick = 0u;
        }
        break;
    case ROUND_WAIT_ENTRY:
        /* 方向由首次出口锁定；同侧边缘任一路单次见线就进入 IN。 */
        if ((s_round_expected_direction < 0 && left_entry) ||
            (s_round_expected_direction > 0 && right_entry)) {
            Roundabout_StartEntry(s_round_expected_direction, now, &imu);
        }
        break;
    case ROUND_ENTRY_TURN:
        /* 固定 350 PWM 转到 20 度；超时路径不启用定时找线。 */
        if (s_round_turn_deg >= ROUNDABOUT_ENTRY_TARGET_DEG ||
            now - s_round_state_tick >= ROUNDABOUT_ENTRY_MAX_MS) {
            /* 中间四路见线就直接循迹，否则沿入圈方向主动找线。 */
            s_round_state = ROUND_ORBIT;
            s_round_orbit_catch_tick = now;
            s_round_orbit_catch_enabled =
                s_round_turn_deg >= ROUNDABOUT_ENTRY_TARGET_DEG;
            s_round_orbit_line_latched =
                s_round_orbit_catch_enabled && s_line_valid &&
                (state & 0x3Cu) != 0u && state != 0xFFu;
            s_round_drive = s_round_orbit_catch_enabled && !s_round_orbit_line_latched ?
                ROUND_DRIVE_CATCH : ROUND_DRIVE_ORBIT;
            s_round_turn_pwm = (int16_t)((float)s_round_direction * ROUNDABOUT_LINE_CATCH_PWM);
        } else {
            s_round_drive = ROUND_DRIVE_ENTRY;
            s_round_turn_pwm = Roundabout_EntryPwm();
        }
        break;
    case ROUND_ORBIT:
        /* 转过 20 度后 CH3～CH6 任一路见线即交给循迹。 */
        if (!s_round_orbit_line_latched && s_line_valid &&
            (state & 0x3Cu) != 0u && state != 0xFFu) {
            s_round_orbit_line_latched = 1u;
        }
        if (!s_round_orbit_line_latched && s_round_orbit_catch_enabled &&
            now - s_round_orbit_catch_tick < ROUNDABOUT_LINE_CATCH_MS) {
            s_round_drive = ROUND_DRIVE_CATCH;
            s_round_turn_pwm = (int16_t)((float)s_round_direction * ROUNDABOUT_LINE_CATCH_PWM);
        } else {
            s_round_drive = ROUND_DRIVE_ORBIT;
        }
        if (s_round_turn_deg >= ROUNDABOUT_DONE_DEG) {
            /* 不等待出口灰度特征，直接恢复普通循迹。 */
            s_round_state = ROUND_DONE;
            s_round_done_tick = now;
            s_round_edge_error_hold_tick = now;
            s_round_edge_error_hold_direction = s_round_direction;
            s_round_edge_error_hold_active = s_round_direction != 0;
            s_round_drive = ROUND_DRIVE_LINE;
        }
        break;
    default:
        break;
    }
}

/* 右侧显示阶段和累计转角；第四行仍由灰度模块显示 CH1～CH8。 */
static void Roundabout_Show(void)
{
    const char *label = "--";
    switch (s_round_state) {
    case ROUND_WAIT_FIRST_EXIT: label = "W0"; break;
    case ROUND_CLEAR_FIRST_EXIT: label = "X0"; break;
    case ROUND_WAIT_ENTRY: label = "W1"; break;
    case ROUND_ENTRY_TURN: label = "IN"; break;
    case ROUND_ORBIT: label = "OR"; break;
    case ROUND_DONE: label = "OK"; break;
    default: break;
    }
    uint8_t lap_text[5] = {
        (uint8_t)('0' + s_lap_completed), '/',
        s_lap_target == 0u ? '-' : (uint8_t)('0' + s_lap_target), ' ', 0
    };
    OLED_ShowString(88,0,lap_text,16,1); /* 已完成圈数/蓝牙设定圈数。 */
    OLED_ShowString(96,16,(uint8_t *)label,16,1);
    OLED_ShowString(80,32,(uint8_t *)"T:",8,1);
    OLED_ShowFloat(92,32,s_round_turn_deg,0,6,8,1);
    OLED_ShowString(80,40,(uint8_t *)"A:",8,1);
    OLED_ShowFloat(92,40,s_lap_progress_deg,0,6,8,1);
}

/* 用探头中心位置的加权平均计算横向偏差：左负、右正。 */
static void Line_Follow_Update(uint8_t state, uint32_t now)
{
    static const int8_t positions[8] = {-12, -7, -3, -1, 1, 3, 7, 12};
    static int8_t last_direction = 0; /* -1：上次在左侧；1：上次在右侧。 */
    static uint8_t mode = 0u; /* 0：跟线；1：锁定方向搜索；2：超时后等待中央回线。 */
    static uint8_t center_frames = 0u;
    static uint8_t right_edge_seen = 0u;
    static uint32_t right_edge_tick = 0u;
    uint8_t follow_state = state;
    int16_t position_sum = 0;
    uint8_t active_count = 0u;

    /* X0/W1 仅用 CH4/CH5；IN 固定转向期间用 CH3～CH6 准备接管循迹。 */
    if (s_round_state >= ROUND_CLEAR_FIRST_EXIT &&
        s_round_state < ROUND_ENTRY_TURN) {
        follow_state &= 0x18u;
    } else if (s_round_state == ROUND_ENTRY_TURN) {
        follow_state &= 0x3Cu;
    } else if (s_round_edge_error_hold_active &&
               now - s_round_edge_error_hold_tick < ROUNDABOUT_EDGE_ERROR_DELAY_MS) {
        /* 右进圈：去掉 CH1/CH2/CH8；左进圈：镜像去掉 CH1/CH7/CH8。 */
        follow_state &= s_round_edge_error_hold_direction > 0 ? 0x7Cu : 0x3Eu;
    } else if (s_round_state == ROUND_DONE) {
        /* 延迟结束后，CH1～CH8 都可以参与出岛后的循迹误差。 */
    } else if ((state & 0x18u) != 0u && (state & 0x81u) != 0u) {
        /* 其他阶段沿用原规则：出入口处的 CH1/CH8 不拉偏误差。 */
        follow_state &= 0x7Eu;
    }
    if (s_round_state >= ROUND_ENTRY_TURN && s_round_state <= ROUND_ORBIT) {
        /* 右侧入圈忽略 CH1；左侧入圈镜像忽略 CH8。 */
        if (s_round_direction > 0) {
            follow_state &= 0xFEu;
        } else if (s_round_direction < 0) {
            follow_state &= 0x7Fu;
        }
    }
    if (s_round_edge_error_hold_active &&
        now - s_round_edge_error_hold_tick >= ROUNDABOUT_EDGE_ERROR_DELAY_MS) {
        s_round_edge_error_hold_active = 0u;
        s_round_edge_error_hold_direction = 0;
    }

    for (uint8_t i = 0u; i < 8u; i++) {
        if ((follow_state & (1u << i)) != 0u) {
            position_sum += positions[i];
            active_count++;
        }
    }

    float line_error = active_count ? (float)position_sum / (float)active_count : 0.0f;
    float abs_error = line_error < 0.0f ? -line_error : line_error;
    uint8_t centered = ((state & 0x18u) != 0u) && (state != 0xFFu) &&
                       (abs_error <= 1.0f);
    /* 丢线恢复看原始 CH2～CH7，不使用上面为当前阶段屏蔽过的 follow_state。 */
    uint8_t line_reacquired = ((state & 0x7Eu) != 0u) && (state != 0xFFu);

    if (mode == 1u) { /* 搜索时忽略边缘探头，不覆盖已经锁定的方向。 */
        if (now - s_line_search_start_tick >= LINE_LOST_MEMORY_MS) {
            mode = 2u;
            center_frames = 0u;
            last_direction = 0;
            right_edge_seen = 0u;
        } else {
            center_frames = line_reacquired ? (uint8_t)(center_frames + 1u) : 0u;
            if (center_frames < LINE_RECENTER_FRAMES) {
                s_line_valid = 0u; /* 搜索中目标前进速度为零。 */
                s_line_turn_pwm = (int16_t)(last_direction * LINE_SEARCH_PWM);
                s_line_search = 1u;
                return;
            }
            mode = 0u;
            center_frames = 0u;
        }
    }

    if (mode == 2u) { /* 超时后不因边缘碰线重新启动，等待中央回线。 */
        center_frames = line_reacquired ? (uint8_t)(center_frames + 1u) : 0u;
        if (center_frames < LINE_RECENTER_FRAMES) {
            s_line_valid = 0u;
            s_line_search = 0u;
            s_line_turn_pwm = 0;
            return;
        }
        mode = 0u;
        center_frames = 0u;
    }

    if (active_count == 0u) {
        int8_t search_direction = last_direction;
        if (right_edge_seen && (now - right_edge_tick <= LINE_RIGHT_HINT_MS)) {
            search_direction = 1; /* 右侧刚见线时，抵抗最后一帧的反向误判。 */
        }
        s_line_valid = 0u;
        if (search_direction != 0 &&
            (now - s_line_last_tick <= LINE_DATA_TIMEOUT_MS)) {
            s_line_search_start_tick = now;
            last_direction = search_direction; /* 搜索期间锁定方向。 */
            s_line_turn_pwm = (int16_t)(search_direction * LINE_SEARCH_PWM);
            s_line_search = 1u;
            mode = 1u;
        } else {
            s_line_turn_pwm = 0;
            s_line_search = 0u;
            last_direction = 0;
            right_edge_seen = 0u;
            mode = 2u;
        }
        return;
    }

    /* 只补强右转：右侧 CH6～CH8 占优时记录，明确偏左时取消。 */
    if ((follow_state & 0xE0u) != 0u && (follow_state & 0x07u) == 0u && line_error > 0.0f) {
        right_edge_tick = now;
        right_edge_seen = 1u;
    } else if ((follow_state & 0x07u) != 0u && line_error < 0.0f) {
        right_edge_seen = 0u;
    }

    if (centered) {
        last_direction = 0; /* 回到中央后立即清除上次的转弯方向。 */
    } else {
        if (line_error < 0.0f) {
            last_direction = -1;
        } else if (line_error > 0.0f) {
            last_direction = 1;
        } else {
            last_direction = 0;
        }
    }
    float turn_pwm = LINE_TURN_KP * line_error;
    if (turn_pwm > LINE_TURN_LIMIT) turn_pwm = LINE_TURN_LIMIT;
    if (turn_pwm < -LINE_TURN_LIMIT) turn_pwm = -LINE_TURN_LIMIT;

    s_line_turn_pwm = (int16_t)turn_pwm;
    s_line_last_tick = now;
    s_line_search = 0u;
    s_line_valid = 1u;
}

/* USART1 调参协议：小端序；校验和为包头至最后一个参数的 8 位累加和。
 * CH1 为角度环，CH2 为速度环；角度遥测值为前倾角乘 100。 */

/* 发送一帧调参协议数据。 */
static void WF_SendFrame(uint8_t cmd, uint8_t ch, const uint8_t *data, uint8_t num)
{
    uint8_t frame[32];
    uint8_t fi = 0, sum = 0, i;
    uint8_t numB = (uint8_t)(num * 4);
    uint32_t len = 0x0B + numB;   /* 总帧长，包含校验字节 */

    frame[fi++] = 0x53; frame[fi++] = 0x5A; frame[fi++] = 0x48; frame[fi++] = 0x59;
    frame[fi++] = ch;
    frame[fi++] = (uint8_t)(len & 0xFF);
    frame[fi++] = (uint8_t)((len >> 8) & 0xFF);
    frame[fi++] = (uint8_t)((len >> 16) & 0xFF);
    frame[fi++] = (uint8_t)((len >> 24) & 0xFF);
    frame[fi++] = cmd;
    for (i = 0; i < numB; i++) frame[fi++] = data[i];
    for (i = 0; i < fi; i++) sum += frame[i];   /* 累加校验 */
    frame[fi++] = sum;

    HAL_UART_Transmit(&huart1, frame, fi, 10);
}

/* 发送实际值（0x02）：CH1 为角度×100，CH2 为速度×10000。 */
static void WF_UploadActual(uint8_t ch, int value) { WF_SendFrame(0x02, ch, (uint8_t *)&value, 1); }
/* 发送目标值（0x01）。 */
static void WF_UploadTarget(uint8_t ch, int value) { WF_SendFrame(0x01, ch, (uint8_t *)&value, 1); }
/* 发送 PID 参数（0x03）：CH1 回传角度环 P/D，CH2 回传速度环 P/I。 */
static void WF_UploadPID(uint8_t ch)
{
    float p, i, d;
    if (ch == 0x02) {
        p = SPEED_KP; i = SPEED_KI; d = 0.0f;
    } else {
        p = BALANCE_KP; i = 0.0f; d = BALANCE_KD;
    }
    uint8_t data[12];
    memcpy(&data[0], &p, 4);
    memcpy(&data[4], &i, 4);
    memcpy(&data[8], &d, 4);
    WF_SendFrame(0x03, ch, data, 3);
}

/* 解析并执行上位机命令。当前接收层只校验总帧长与校验和，
 * 这里没有按命令检查参数长度；短帧可能把校验字节或缓存旧字节当参数。 */
static void WF_Dispatch(const uint8_t *f, uint16_t len)
{
    uint8_t cmd = f[9];
    uint8_t ch  = f[4];
    (void)len;
    switch (cmd) {
        case 0x10: { /* 设置 P/I/D，数据偏移分别为 10、14、18。 */
            float p, i, d;
            memcpy(&p, &f[10], 4);
            memcpy(&i, &f[14], 4);
            memcpy(&d, &f[18], 4);
            if (ch == 0x02) {
                SPEED_KP = p; SPEED_KI = i;
            } else {
                BALANCE_KP = p; BALANCE_KD = d;
            }
            WF_UploadPID(ch); /* 回传当前通道参数。 */
            break;
        }
        case 0x11: { /* 设置目标值。 */
            int t;
            memcpy(&t, &f[10], 4);
            if (ch == 0x02) {
                SPEED_TARGET = (float)t / 10000.0f;
            } else {
                BALANCE_ZERO = (float)t / 100.0f;
            }
            WF_UploadTarget(ch, t);
            break;
        }
        case 0x12: s_ctrl_enable = 1; break;       /* 启动平衡控制。 */
        case 0x13: s_ctrl_enable = 0; break;       /* 停止并将电机输出归零。 */
        case 0x14: HAL_NVIC_SystemReset(); break;  /* 复位 MCU。 */
        case 0x15: { /* 设置遥测周期，单位 ms，范围 5～500。 */
            uint32_t t;
            memcpy(&t, &f[10], 4);
            if (t >= 5 && t <= 500) s_wf_period = t;
            break;
        }
        default: break;
    }
}

/* USART1 接收回调逐字节调用本函数；找到完整帧后校验并分发命令。 */
void PID_Cmd_OnByte(uint8_t ch)
{
    uint8_t i;
    uint32_t hdr;
    uint32_t len;
    uint8_t sum;

    /* HC-05 与调参协议共用 USART1；只在没有二进制帧时接收单字节圈数。 */
    if (s_wf_idx == 0u) {
        if (ch >= '1' && ch <= '5') {
            s_lap_pending_target = (uint8_t)(ch - '0');
            return;
        }
        if (ch == '\r' || ch == '\n') return;
        if (ch != 0x53u) return; /* 二进制调参帧以 0x53 开头。 */
    }

    if (s_wf_idx >= sizeof(s_wf_buf)) s_wf_idx = 0;  /* 丢弃异常缓存状态。 */
    s_wf_buf[s_wf_idx++] = ch;

    while (s_wf_idx >= 11) {   /* 最短帧长为 11 字节。 */
        hdr = (uint32_t)s_wf_buf[0] | ((uint32_t)s_wf_buf[1] << 8) |
              ((uint32_t)s_wf_buf[2] << 16) | ((uint32_t)s_wf_buf[3] << 24);
        if (hdr != 0x59485A53u) {   /* 移除一个字节并重新搜索帧头。 */
            for (i = 1; i < s_wf_idx; i++) s_wf_buf[i-1] = s_wf_buf[i];
            s_wf_idx--;
            continue;
        }
        len = (uint32_t)s_wf_buf[5] | ((uint32_t)s_wf_buf[6] << 8) |
              ((uint32_t)s_wf_buf[7] << 16) | ((uint32_t)s_wf_buf[8] << 24);
        if (len < 11 || len > sizeof(s_wf_buf)) {   /* 长度超出接收缓存范围。 */
            for (i = 1; i < s_wf_idx; i++) s_wf_buf[i-1] = s_wf_buf[i];
            s_wf_idx--;
            continue;
        }
        if (s_wf_idx < len) return;   /* 等待剩余字节。 */
        sum = 0;
        for (i = 0; i < len - 1; i++) sum += s_wf_buf[i];
        if (sum != s_wf_buf[len-1]) {   /* 校验失败，移除一个字节后重新同步。 */
            for (i = 1; i < s_wf_idx; i++) s_wf_buf[i-1] = s_wf_buf[i];
            s_wf_idx--;
            continue;
        }
        WF_Dispatch(s_wf_buf, len);
        for (i = len; i < s_wf_idx; i++) s_wf_buf[i-len] = s_wf_buf[i];
        s_wf_idx -= len;
    }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  uint32_t motor_delay_start_tick = 0u;
  uint8_t motors_started = 0u;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  motor_delay_start_tick = HAL_GetTick();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART3_UART_Init();
  MX_USART1_UART_Init();
  MX_TIM4_Init();
  /* 定时器初始化会配置 PWM 复用脚；IMU 初始化期间将其恢复为低电平输出。 */
  Motor_PwmPins_SetSafeLow();
  /* USER CODE BEGIN 2 */

  /* ADC 初始化后校准灰度采样，电机仍保持关闭。 */
  Sensor_Init();

  OLED_Init();                        
  OLED_Refresh();

  /* 启动 G356 的 USART3 接收。 */
  G356_App_Init();

  /* 启动 USART1 在线调参接收。 */
  UART_Cmd_Init();

  /* 向调参上位机发送角度环初始设定。 */
  WF_UploadTarget(0x01, (int)(BALANCE_ZERO * 100.0f));
  WF_UploadPID(0x01);
  /* 发送速度环初始设定。 */
  WF_UploadTarget(0x02, (int)(SPEED_TARGET * 10000.0f));
  WF_UploadPID(0x02);

  /* 先确认 IMU 已经开始输出有效数据，启动失败时重新下发输出命令。 */
  uint8_t startup_ok = 0u;
  uint8_t startup_retry;

  for (startup_retry = 0u; startup_retry < 3u && !startup_ok; startup_retry++) {
    G356_App_EnableUartOutput();
    startup_ok = G356_App_WaitReady(5000u);
    if (!startup_ok) {
      HAL_Delay(500u);
    }
  }

  if (startup_ok) {
    /* 立即完成传感器校准；15 秒电机等待由主循环非阻塞计时。 */
    startup_ok = 0u;
    for (startup_retry = 0u; startup_retry < 3u && !startup_ok; startup_retry++) {
      startup_ok = G356_App_StartupCalibration();
      if (!startup_ok) {
        G356_App_EnableUartOutput();
        HAL_Delay(500);   /* 重试前短暂等待。 */
      }
    }
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint32_t gray_last_tick = 0u;
  uint32_t oled_last_tick = 0u;
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  
    uint32_t now = HAL_GetTick();

    if (s_lap_pending_target != 0u) {
        uint8_t target = s_lap_pending_target;
        s_lap_pending_target = 0u;
        MapLap_Start(target);
    }

    /* 非阻塞等待：OLED、灰度和串口在 15 秒内继续正常运行。
     * 到时后仍需确认 IMU 最近收到新数据，避免用停住的旧角度启动电机。 */
    if (!motors_started && startup_ok &&
        (now - motor_delay_start_tick >= MOTOR_START_DELAY_MS) &&
        G356_App_DataFresh(150u)) {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
        Motor_PwmPins_EnableTimerOutput();
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
        HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
        HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
        HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
        HAL_TIM_Base_Start_IT(&htim4);
        motors_started = 1u;
    }

    if (now - gray_last_tick >= GRAY_SCAN_MS) {
        gray_last_tick = now;
        uint8_t gray_state = Scan_Sensor_Byte();
        /* 先按当前环岛阶段计算循迹，再用本次灰度更新阶段；新阶段下次扫描生效。 */
        Line_Follow_Update(gray_state, HAL_GetTick());
        if (motors_started && s_lap_running) {
            Roundabout_Update(gray_state, HAL_GetTick());
        }
        MapLap_Update(gray_state);
    }

    /* 按设定周期上传前倾角和平均轮速。 */
    if (now - s_wf_last_tick >= s_wf_period) {
        s_wf_last_tick = now;
        G356_Data_t g;
        G356_App_CopyData(&g);
        WF_UploadActual(0x01, (int)((-g.roll) * 100.0f));
        WF_UploadActual(0x02, (int)(s_speed_meas * 10000.0f));
    }

    if (now - oled_last_tick < OLED_UPDATE_MS) {
        continue;
    }
    oled_last_tick = now;
    G356_Data_t g;
    G356_App_CopyData(&g);

    OLED_ShowString(0,0,(uint8_t *)"P:",16,1);      /* 前倾角，前倾为正。 */
    OLED_ShowFloat(24,0,-g.roll,1,6,16,1);
    OLED_ShowString(0,16,(uint8_t *)"R:",16,1);    /* 横向倾角。 */
    OLED_ShowFloat(24,16,g.pitch,1,6,16,1);
    OLED_ShowString(0,32,(uint8_t *)"Y:",16,1);
    OLED_ShowFloat(24,32,g.yaw,1,6,16,1);
    int8_t search_direction = 0;
    if (s_line_search && HAL_GetTick() - s_line_search_start_tick < LINE_LOST_MEMORY_MS) {
        if (s_line_turn_pwm > 0) search_direction = 1;
        if (s_line_turn_pwm < 0) search_direction = -1;
    }
    if (s_round_state >= ROUND_ENTRY_TURN && s_round_state <= ROUND_ORBIT) {
        search_direction = s_round_direction; /* 环岛期间显示已选定的入口侧。 */
    }
    Roundabout_Show();
    OLED_ShowChar(80,16,
                  search_direction > 0 ? 'R' : (search_direction < 0 ? 'L' : ' '),
                  16,1);
    Send_State_Buffer_Get_OLED();
    
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* TIM4 更新中断负责控制；OLED 刷新留在主循环中，不阻塞控制计算。 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance != TIM4) {
        return;
    }

    static uint8_t tick_div = 0;
    if (++tick_div < 10) {   /* TIM4 更新频率 10 kHz，控制频率 1 kHz。 */
        return;
    }
    tick_div = 0;

    if (!G356_App_DataFresh(150u)) { /* IMU 数据超过 150 ms 未更新时停止电机。 */
        Set_Motor_A_Speed(0);
        Set_Motor_B_Speed(0);
        s_speed_integral = 0.0f;
        s_speed_out = 0.0f;
        return;
    }

    /* 读取最近一帧姿态数据。 */
    G356_Data_t g;
    G356_App_CopyData(&g);
    float fwd_tilt_raw = -g.roll;  /* 前倾角，单位度，前倾为正。 */
    float fwd_rate     = -g.gyro_x; /* 前倾角速度，单位度/秒。 */
    /* 读取本控制周期内的编码器计数增量。 */
    int16_t enc_a = Encoder_A_GetCount();
    int16_t enc_b = Encoder_B_GetCount();

    /* 两侧电机镜像安装，编码器方向相反，因此将 B 侧计数取反后合并。 */
    s_enc_sum += (int32_t)enc_a - (int32_t)enc_b;
    uint8_t line_active = LINE_FOLLOW_ENABLE && s_line_valid &&
                          (HAL_GetTick() - s_line_last_tick <= LINE_DATA_TIMEOUT_MS);
    uint8_t line_search = LINE_FOLLOW_ENABLE && s_line_search &&
                          (HAL_GetTick() - s_line_search_start_tick < LINE_LOST_MEMORY_MS);
    uint8_t round_drive = s_round_drive;
    uint8_t round_speed = round_drive == ROUND_DRIVE_ENTRY ||
                          round_drive == ROUND_DRIVE_CATCH;
    /* IN 强制转向和 OR 主动找线可在尚未咬线时前进；其他阶段无有效线则目标速度归零。 */
    uint8_t speed_active = line_active || round_speed;
    if (++s_enc_cnt >= SPEED_WINDOW) {
        float speed_meas = (float)s_enc_sum / (float)SPEED_WINDOW;
        s_speed_meas = speed_meas;
        /* 正常循迹、W1、IN 和 OR 找线的默认行驶目标均为 -0.9。 */
        float speed_target = (round_drive == ROUND_DRIVE_ENTRY ||
                              (s_round_state == ROUND_WAIT_ENTRY && line_active)) ?
                             ROUNDABOUT_W1_ENTRY_SPEED_TARGET :
                             (round_drive == ROUND_DRIVE_CATCH ?
                              ROUNDABOUT_CATCH_SPEED_TARGET :
                              ((LINE_FOLLOW_ENABLE && !speed_active) ?
                               0.0f : SPEED_TARGET));
        if (!s_lap_running) speed_target = 0.0f;
        if (!s_lap_running || (LINE_FOLLOW_ENABLE && !speed_active)) {
            s_speed_integral = 0.0f; /* 清除之前累积的速度误差。 */
        }
        float speed_err  = speed_target - speed_meas;
        s_speed_out = SPEED_KP * speed_err + SPEED_KI * s_speed_integral;
        /* 输出到达限幅且误差仍推动输出继续饱和时，暂停积分。 */
        if (s_lap_running && (!LINE_FOLLOW_ENABLE || speed_active) &&
            ((s_speed_out <  25.0f && s_speed_out > -25.0f) ||
             (s_speed_out >= 25.0f && speed_err < 0.0f)     ||
             (s_speed_out <= -25.0f && speed_err > 0.0f))) {
            s_speed_integral += speed_err;
             if (s_speed_integral >  SPEED_ILIM) s_speed_integral =  SPEED_ILIM;
            if (s_speed_integral < -SPEED_ILIM) s_speed_integral = -SPEED_ILIM;
        }
        s_speed_out = SPEED_KP * speed_err + SPEED_KI * s_speed_integral;
        /* 限制速度环施加到平衡点的角度偏移。 */
        if (s_speed_out >  25.0f) s_speed_out =  25.0f;
        if (s_speed_out < -25.0f) s_speed_out = -25.0f;
        s_enc_sum = 0;
        s_enc_cnt = 0;
    }

    /* 速度环输出修正平衡点；角度环据此计算直立控制量。 */
    float fwd_tilt  = fwd_tilt_raw - BALANCE_ZERO + SPEED_SIGN * s_speed_out;
    float angle_out = (BALANCE_KP * fwd_tilt) + (BALANCE_KD * fwd_rate);

    float turn_out;
    if (round_drive == ROUND_DRIVE_ENTRY ||
        round_drive == ROUND_DRIVE_CATCH) {
        turn_out = (float)s_round_turn_pwm;
    } else if (round_drive == ROUND_DRIVE_ORBIT && !line_active) {
        /* 沿用现有的 2.5 秒丢线超时，只将搜索方向固定为入岛方向。 */
        turn_out = line_search ? (float)s_round_direction * LINE_SEARCH_PWM : 0.0f;
    } else {
        turn_out = (line_active || line_search) ? (float)s_line_turn_pwm : 0.0f;
    }
    if (!s_lap_running) turn_out = 0.0f;

    /* 直立控制量与转向控制量合成为左右轮 PWM。 */
    int pwm_a = (int)(angle_out + turn_out);
    int pwm_b = (int)(angle_out - turn_out);

    /* 停止命令会将电机归零并清除速度环积分。 */
    if (!s_ctrl_enable) {
        pwm_a = 0; pwm_b = 0;
        s_speed_integral = 0.0f; s_speed_out = 0.0f;
    }
    /* 限幅后更新电机 PWM。 */
    if (pwm_a >  PWM_LIMIT) pwm_a =  PWM_LIMIT;
    if (pwm_a < -PWM_LIMIT) pwm_a = -PWM_LIMIT;
    if (pwm_b >  PWM_LIMIT) pwm_b =  PWM_LIMIT;
    if (pwm_b < -PWM_LIMIT) pwm_b = -PWM_LIMIT;
    Set_Motor_A_Speed(pwm_a);
    Set_Motor_B_Speed(pwm_b);
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
