#ifndef G356_APP_H
#define G356_APP_H

/*
 * G356 六轴陀螺仪 应用层封装 (USART3 中断接收)
 * ------------------------------------------------------------------
 * 模块化原则: 所有与 G356 相关的变量 (接收缓冲/帧缓冲/解析结果/新帧标志)
 * 全部声明在 g356_app.c 内部 (static), main.c 只能通过下面的接口读写,
 * 不直接接触驱动内部状态。
 *
 * 接线 (STM32F103C8T6, CubeMX 中 USART3, 115200 8N1):
 *   G356 VIN  -> 5V (推荐, 3.3~12V 均可)
 *   G356 GND  -> GND (必须共地)
 *   G356 TXD  -> PB11 (USART3_RX)   模块 -> MCU 遥测数据
 *   G356 RXD  -> PB10 (USART3_TX)   仅需下发命令时才接
 *
 * 依赖:
 *   - g356_driver.h/.c (纯字节流帧解析, 无 HAL 依赖)
 *   - CubeMX 生成的 USART3 (huart3) 及其中断服务函数 USART3_IRQHandler
 *   - 本模块在 g356_app.c 中定义了 HAL_UART_RxCpltCallback (覆盖 HAL 弱函数)
 */

#include "g356_driver.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 模块初始化: 外设初始化完成后调用一次, 启动 USART3 逐字节中断接收 */
void G356_App_Init(void);

/* 启动 USART1 中断接收, 用于串口实时调参 (kp=/kd=/z=/show), 需在 USART1 NVIC 使能后调用 */
void UART_Cmd_Init(void);

/* 查询是否有新一帧解析完成: 返回 1 表示有新数据 (约 100Hz), 0 表示无 */
uint8_t G356_App_NewData(void);

/* 主循环消费完新帧后调用, 清除新帧标志 */
void G356_App_ClearNewData(void);

/* 获取最新一帧解析结果 (只读指针, 工程单位: accel=g / gyro=dps / 姿态=deg / temp=C) */
const G356_Data_t *G356_App_GetData(void);

typedef struct {
    uint32_t rx_bytes;       /* USART3 已收到的字节数 */
    uint32_t valid_frames;   /* 校验通过的数据帧数 */
    uint32_t invalid_frames; /* 凑满但校验/类型/帧尾失败的帧数 */
    uint32_t uart_errors;    /* USART3 HAL 错误次数 */
    uint32_t last_error;     /* 最近一次 huart3.ErrorCode */
} G356_AppStats_t;

/* 获取接收诊断计数, 用于区分没收到字节/协议不匹配/串口错误 */
void G356_App_GetStats(G356_AppStats_t *stats);

/* 查询模块是否就绪: 1=已连续收到多帧有效数据 (解算稳定), 0=还在启动阶段。
 * 就绪前 G356_App_NewData() 不会置位, 控制环可据此屏蔽开机初期的不可靠数据 */
uint8_t G356_App_Ready(void);

/* 阻塞等待模块就绪。timeout_ms 为超时时间 (0=无限等待);
 * 模块上电启动约 1s + 连续有效帧约 100ms 后返回, 超时也会返回 */
uint8_t G356_App_WaitReady(uint32_t timeout_ms);

/* 从调用时刻起等待 required_frames 个新的有效帧，防止旧的 ready 状态误判。 */
uint8_t G356_App_WaitFreshFrames(uint32_t required_frames, uint32_t timeout_ms);

/* 判断最近一帧有效数据是否在 timeout_ms 内收到。 */
uint8_t G356_App_DataFresh(uint32_t timeout_ms);

/* 将当前姿态复制到调用方缓存，避免主循环/控制中断直接读取接收缓冲。 */
void G356_App_CopyData(G356_Data_t *data);

/* ==================== 下行命令 (向 G356 发送配置/校准命令) ==================== */

/* 常用命令码 (7 字节帧: AA AF cmd param crc16 5A) */
#define G356_CMD_GYRO_CALIBRATE      0x10u   /* 陀螺仪零偏校准, 校准期间模块必须静止 */
#define G356_CMD_ACCEL_LEVEL_CALIB   0x11u   /* 加速度计水平校准, 要求模块绝对水平放置 */
#define G356_CMD_SENSOR_RESET        0x12u   /* 传感器复位 (重新初始化内部传感器) */
#define G356_CMD_SET_OUTPUT_CHANNEL  0x14u   /* 输出通道: bit0=UART, bit1=SPI */
#define G356_CMD_ZERO_YAW            0x18u   /* Yaw 归零: 当前朝向设为 0° */
#define G356_CMD_RESTORE_FACTORY     0x1Eu   /* 恢复出厂设置 */

/* 发送一条 7 字节下行命令, 返回 1=发送成功, 0=失败 */
uint8_t G356_App_SendCommand(uint8_t cmd, uint8_t param);

/* 按手册建议同一命令发送两次, 确保模块通过 UART 输出遥测帧 */
void G356_App_EnableUartOutput(void);

/* 上电启动归零序列: 传感器复位 → 陀螺仪校准 → Yaw 归零。
 * 调用时机: G356_App_WaitReady() 之后、正式使用数据之前。
 * ⚠️ 陀螺仪校准 (0x10) 执行期间模块必须保持静止, 否则零偏会校歪 */
uint8_t G356_App_StartupCalibration(void);

/* 调试: 通过 USART1 (HC-05, 115200,8N1) 每 ~0.5s 打印一次姿态, 验证数据用, 可删除 */
void G356_App_PrintData(void);

#ifdef __cplusplus
}
#endif

#endif /* G356_APP_H */
