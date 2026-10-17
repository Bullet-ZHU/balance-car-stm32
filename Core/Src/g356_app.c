/*
 * G356 六轴陀螺仪 应用层实现 (USART3 中断接收)
 * ------------------------------------------------------------------
 * 本文件是 G356 的唯一封装层: 变量、帧解析、HAL 回调、调试打印都在这里,
 * main.c 只需要调用 G356_App_* 接口。
 */

#include "g356_app.h"

#include "usart.h"   /* huart3 (G356 链路), huart1 (调试打印) */
#include <string.h>

/* ==================== 模块配置 ==================== */
/* 启动就绪判定 —— 采用【漏桶计数】: 好帧 +1 分, 坏帧 -1 分, 达到阈值即认为模块稳定。
 *
 * 原实现是"连续 N 帧有效", 一帧坏就把计数清零。在电气噪声环境下(电机/编码器
 * 干扰耦合到 IMU 的 UART), 坏帧会偶发出现, 导致计数永远到不了阈值 →
 * 初始化必然超时失败。
 *   ✓✓✓✓✓✓✓✓✓✗  (旧: 第10帧坏 → 归零 → 永远等不到)
 *   ✓✓✓✓✓✓✓✓✓✗✓✓✓✓✓  (新: 坏帧只 -1 分 → 攒够 10 分即通过)
 * 可容忍约 50% 的偶发坏帧。
 * 注意: 坏帧的数据本身仍然被丢弃, 不影响进入控制环的数据质量。 */
#define G356_APP_READY_CONSECUTIVE_FRAMES 10u   /* 就绪阈值: 漏桶分数达到 10 分 (~100ms) */
#define G356_APP_READY_SCORE_MAX          20u   /* 漏桶分数上限, 防止长期累积后瞬间通过 */

/* USART1 串口实时调参逐字节回调 (实现在 main.c, 这里中转) */
void PID_Cmd_OnByte(uint8_t ch);

/* ==================== 模块内部变量 (全部封装在本文件内) ==================== */
static uint8_t  s_rx_byte;                  /* USART3 逐字节接收缓冲 */
static uint8_t  s_uart1_rx_byte;            /* USART1 逐字节接收缓冲 (调参命令) */
static uint8_t  s_frame_buf[G356_FRAME_SIZE]; /* 校验通过的完整 56 字节帧 */
static G356_Data_t s_data[2];               /* 双缓冲: 接收解析与控制读取分离 */
static volatile uint8_t s_data_index;       /* 当前已发布的数据缓冲区 */
static volatile uint8_t s_new_data;         /* 新帧标志: 中断置1, 主循环消费后清0 */
static volatile uint8_t  s_ready;           /* 主循环与串口中断共享的启动门控 */
static volatile uint32_t s_valid_run;       /* 启动阶段有效帧漏桶计数 */
static volatile G356_AppStats_t s_stats;    /* 接收诊断计数 */
static volatile uint32_t s_last_valid_tick; /* 最近一次有效帧的系统时间 */

/* ==================== 对外接口 ==================== */

void G356_App_Init(void)
{
    s_new_data = 0;
    s_ready = 0;
    s_valid_run = 0;
    s_data_index = 0;
    memset(s_data, 0, sizeof(s_data));
    s_last_valid_tick = 0;
    s_stats.rx_bytes = 0;
    s_stats.valid_frames = 0;
    s_stats.invalid_frames = 0;
    s_stats.uart_errors = 0;
    s_stats.last_error = 0;
    /* 启动 USART3 中断接收: 每收到 1 字节回调一次 HAL_UART_RxCpltCallback */
    HAL_UART_Receive_IT(&huart3, &s_rx_byte, 1);
}

/* 启动 USART1 中断接收, 用于串口实时调参 (kp=/kd=/z=/show).
 * 需在 USART1 的 NVIC 中断已使能(MX_USART1 里)后调用. */
void UART_Cmd_Init(void)
{
    s_uart1_rx_byte = 0;
    HAL_UART_Receive_IT(&huart1, &s_uart1_rx_byte, 1);
}

uint8_t G356_App_NewData(void)
{
    return s_new_data;
}

void G356_App_ClearNewData(void)
{
    s_new_data = 0;
}

const G356_Data_t *G356_App_GetData(void)
{
    return &s_data[s_data_index];
}

void G356_App_CopyData(G356_Data_t *data)
{
    uint8_t index;
    if (data == 0) {
        return;
    }
    index = s_data_index;
    memcpy(data, &s_data[index], sizeof(*data));
}

uint8_t G356_App_DataFresh(uint32_t timeout_ms)
{
    if (!s_ready || s_last_valid_tick == 0u) {
        return 0u;
    }
    return ((HAL_GetTick() - s_last_valid_tick) <= timeout_ms) ? 1u : 0u;
}

void G356_App_GetStats(G356_AppStats_t *stats)
{
    if (stats == 0) {
        return;
    }

    stats->rx_bytes = s_stats.rx_bytes;
    stats->valid_frames = s_stats.valid_frames;
    stats->invalid_frames = s_stats.invalid_frames;
    stats->uart_errors = s_stats.uart_errors;
    stats->last_error = s_stats.last_error;
}

uint8_t G356_App_Ready(void)
{
    return s_ready;
}

uint8_t G356_App_WaitReady(uint32_t timeout_ms)
{
    uint32_t start = HAL_GetTick();
    while (!s_ready) {
        if (timeout_ms > 0u && (HAL_GetTick() - start) >= timeout_ms) {
            return 0u;
        }
    }
    return 1u;
}

uint8_t G356_App_WaitFreshFrames(uint32_t required_frames, uint32_t timeout_ms)
{
    uint32_t start_tick = HAL_GetTick();
    uint32_t start_count = s_stats.valid_frames;

    if (required_frames == 0u) {
        required_frames = 1u;
    }

    while ((uint32_t)(s_stats.valid_frames - start_count) < required_frames) {
        if (timeout_ms > 0u && (HAL_GetTick() - start_tick) >= timeout_ms) {
            return 0u;
        }
    }

    return G356_App_DataFresh(150u);
}

/* ==================== 下行命令 (发送方向) ==================== */

/* CRC16-XMODEM (多项式 0x1021, 初始值 0x0000), 与 G356 固件一致 */
static uint16_t G356_App_Crc16Xmodem(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0x0000u;
    uint16_t i;
    for (i = 0; i < len; i++) {
        uint8_t bit;
        crc ^= (uint16_t)data[i] << 8;
        for (bit = 0; bit < 8u; bit++) {
            if ((crc & 0x8000u) != 0u) {
                crc = (uint16_t)((crc << 1) ^ 0x1021u);
            } else {
                crc = (uint16_t)(crc << 1);
            }
        }
    }
    return crc;
}

uint8_t G356_App_SendCommand(uint8_t cmd, uint8_t param)
{
    uint8_t packet[7];
    uint8_t crc_in[2];
    uint16_t crc;

    /* 组帧: AA AF cmd param crcL crcH 5A (CRC16-XMODEM 覆盖 cmd+param) */
    crc_in[0] = cmd;
    crc_in[1] = param;
    crc = G356_App_Crc16Xmodem(crc_in, 2);

    packet[0] = 0xAAu;
    packet[1] = 0xAFu;
    packet[2] = cmd;
    packet[3] = param;
    packet[4] = (uint8_t)(crc & 0xFFu);
    packet[5] = (uint8_t)(crc >> 8);
    packet[6] = 0x5Au;

    /* 阻塞发送 7 字节 (115200 下约 0.6ms), 不影响已在运行的 RX 中断接收 */
    if (HAL_UART_Transmit(&huart3, packet, sizeof(packet), 100) != HAL_OK) {
        return 0;
    }
    return 1;
}

static void G356_App_SendCommandRepeated(uint8_t cmd, uint8_t param)
{
    G356_App_SendCommand(cmd, param);
    HAL_Delay(60);
    G356_App_SendCommand(cmd, param);
}

void G356_App_EnableUartOutput(void)
{
    G356_App_SendCommandRepeated(G356_CMD_SET_OUTPUT_CHANNEL, 0x01u);
}

uint8_t G356_App_StartupCalibration(void)
{
    s_new_data = 0;
    s_ready = 0;
    s_valid_run = 0;

    /* 确保模块当前通过 UART 连续输出遥测帧 */
    G356_App_EnableUartOutput();
    HAL_Delay(100);

    /* 1. 传感器复位: 让模块重新初始化内部传感器 */
    G356_App_SendCommandRepeated(G356_CMD_SENSOR_RESET, 0);
    HAL_Delay(300);

    /* 传感器复位可能恢复默认输出配置，必须再次打开 UART 连续输出。 */
    G356_App_EnableUartOutput();
    HAL_Delay(100);

    /* 复位后必须重新等到有效帧到位，不能用固定延时直接开始校准。
     * 超时放宽到 5 秒: 电气噪声会造成偶发坏帧, 门控需要更多时间攒够分数。 */
    if (!G356_App_WaitReady(5000u)) {
        return 0u;
    }

    /* 2. 陀螺仪零偏校准 — ⚠️ 此时模块必须保持静止, 否则零偏会校歪 */
    G356_App_SendCommandRepeated(G356_CMD_GYRO_CALIBRATE, 0);
    HAL_Delay(300);

    /* 3. Yaw 归零: 把当前朝向设为 0° */
    G356_App_SendCommandRepeated(G356_CMD_ZERO_YAW, 0);
    HAL_Delay(100);

    /* 校准命令执行后再次确认 UART 输出已开启。 */
    G356_App_EnableUartOutput();

    /* 不能沿用校准前的 ready 状态，必须确认校准后仍在连续输出新帧。 */
    return G356_App_WaitFreshFrames(G356_APP_READY_CONSECUTIVE_FRAMES, 5000u);
}

/* ==================== HAL 回调 (USART3 逐字节中断) ==================== */

/*
 * 覆盖 HAL 弱函数 HAL_UART_RxCpltCallback。
 * USART3 接收姿态帧，USART1 接收蓝牙圈数及调参命令；
 * 依靠 huart->Instance 将收到的字节交给各自的处理函数。
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3) {
        s_stats.rx_bytes++;
        G356_FrameStatus st = G356_FeedByte(s_rx_byte, s_frame_buf);
        if (st == G356_FRAME_VALID) {
            uint8_t write_index = (uint8_t)(s_data_index ^ 1u);
            s_stats.valid_frames++;
            G356_ParseData(s_frame_buf, &s_data[write_index]);
            s_data_index = write_index;
            s_last_valid_tick = HAL_GetTick();
            if (s_ready) {
                s_new_data = 1;   /* 就绪后: 每帧有效都置新帧标志 */
            } else {
                /* 启动门控(漏桶): 好帧 +1 分。达到阈值即认为模块解算稳定,
                 * 防止开机初期姿态未收敛/校验不稳的数据进入控制环。 */
                if (s_valid_run < G356_APP_READY_SCORE_MAX) {
                    s_valid_run++;
                }
                if (s_valid_run >= G356_APP_READY_CONSECUTIVE_FRAMES) {
                    s_ready = 1;
                    s_valid_run = 0;
                    s_new_data = 1;
                }
            }
        } else if (st == G356_FRAME_INVALID) {
            s_stats.invalid_frames++;
            if (!s_ready) {
                /* 漏桶: 坏帧只 -1 分,【不清零】, 从而容忍偶发坏帧。
                 * (原实现是 s_valid_run = 0, 有噪声时会导致永远无法就绪) */
                if (s_valid_run > 0u) {
                    s_valid_run--;
                }
            }
        }
        HAL_UART_Receive_IT(&huart3, &s_rx_byte, 1); /* 重新武装, 继续收下一字节 */
    } else if (huart->Instance == USART1) {
        /* USART1: 逐字节解析蓝牙/调参命令；完整调参帧可能在此中断内同步回传。 */
        PID_Cmd_OnByte(s_uart1_rx_byte);
        HAL_UART_Receive_IT(&huart1, &s_uart1_rx_byte, 1); /* 重新武装 */
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3) {
        s_stats.uart_errors++;
        s_stats.last_error = huart->ErrorCode;
        HAL_UART_Receive_IT(&huart3, &s_rx_byte, 1);
    } else if (huart->Instance == USART1) {
        HAL_UART_Receive_IT(&huart1, &s_uart1_rx_byte, 1);
    }
}

/* ==================== 调试打印 (USART1 / HC-05, 仅验证用) ==================== */

static void PrintString(const char *str)
{
    while (*str) {
        HAL_UART_Transmit(&huart1, (uint8_t *)str, 1, 10);
        str++;
    }
}

static void PrintUint(uint32_t val)
{
    char buf[12];
    int i = 11;
    buf[i] = '\0';
    if (val == 0) {
        PrintString("0");
        return;
    }
    while (val > 0) {
        i--;
        buf[i] = (val % 10) + '0';
        val /= 10;
    }
    PrintString(&buf[i]);
}

static void PrintFloat(float val, int decimals)
{
    if (val < 0.0f) {
        PrintString("-");
        val = -val;
    }
    float rounding = 0.5f;
    for (int d = 0; d < decimals; d++) {
        rounding /= 10.0f;
    }
    val += rounding;
    uint32_t int_part = (uint32_t)val;
    PrintUint(int_part);
    if (decimals > 0) {
        PrintString(".");
        float frac = val - (float)int_part;
        for (int d = 0; d < decimals; d++) {
            frac *= 10.0f;
            uint32_t digit = (uint32_t)frac;
            PrintUint(digit);
            frac -= (float)digit;
        }
    }
}

void G356_App_PrintData(void)
{
    static uint32_t cnt = 0;
    G356_Data_t data;
    if (++cnt % 50 != 0) {   /* 每 50 帧 (~0.5s) 打印一次, 防止串口拥堵 */
        return;
    }

    G356_App_CopyData(&data);

    PrintString("ANG[R:");
    PrintFloat(data.roll, 2);
    PrintString(", P:");
    PrintFloat(data.pitch, 2);
    PrintString(", Y:");
    PrintFloat(data.yaw, 2);
    PrintString("] deg | ACC[");
    PrintFloat(data.accel_x, 3);
    PrintString(", ");
    PrintFloat(data.accel_y, 3);
    PrintString(", ");
    PrintFloat(data.accel_z, 3);
    PrintString("] g | GYR[");
    PrintFloat(data.gyro_x, 1);
    PrintString(", ");
    PrintFloat(data.gyro_y, 1);
    PrintString(", ");
    PrintFloat(data.gyro_z, 1);
    PrintString("] dps | Temp:");
    PrintFloat(data.temp, 1);
    PrintString(" C\r\n");
}
