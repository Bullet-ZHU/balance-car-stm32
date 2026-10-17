#ifndef __Ganwei_H
#define __Ganwei_H

#include "main.h" // 包含 HAL 库和引脚宏定义

// ================= 全局变量声明 =================
// 使用 extern 声明，告诉编译器这些变量在 .c 文件中定义，外部可以直接读取
extern volatile uint8_t Sensor_State_Buffer;
extern uint16_t Analog[8];
extern uint16_t white[8];
extern uint16_t black[8];

// ================= 函数接口声明 =================
void Sensor_Init(void);                // 传感器初始化 (包含ADC校准)
void Read_All_Analog(void);            // 读取8路模拟量
uint8_t Scan_Sensor_Byte(void);        // 扫描并计算数字状态(滞回比较)
void Send_Analog_Data_UART(void);      // 串口打印模拟量
void Send_State_Buffer_UART(void);     // 串口打印数字状态
void Send_State_Buffer_Get_OLED(void);

#endif
