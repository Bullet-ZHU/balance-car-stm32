/**
 ******************************************************************************
 * @file    Ganwei.c
 * @brief   感为八路灰度传感器驱动源文件
 * @details 包含底层硬件延时、ADC 轮询读取、滞回比较器数字量转换及串口调试打印功能
 ******************************************************************************
 */

#include "Ganwei.h"
#include <stdio.h>
#include <string.h>
#include "main.h"
#include "oled.h"

// ================= 外部硬件句柄 =================
// 引用 main.c 中初始化的外设句柄
extern ADC_HandleTypeDef hadc1;
extern UART_HandleTypeDef huart3;

// ================= 全局变量定义 =================
volatile uint8_t Sensor_State_Buffer = 0x00;  // 8位数字状态缓冲区 (1: 黑线, 0: 白底)
uint16_t Analog[8] = {0};            // 8路模拟量原始数据缓冲区

// 传感器基准校准值：白底取截图后段较稳定的读数，黑底取各路代表读数。
uint16_t white[8] = {345, 1960, 380, 2140, 1720, 1230, 910, 610};
uint16_t black[8] = {150, 600, 155, 720, 510, 285, 200, 160};

// ================= 内部静态函数 =================

/**
 * @brief   微秒级硬件延时函数
 * @note    基于系统主频 (HCLK) 进行粗略延时，用于等待硬件电平稳定。
 *          为阻塞式延时，内部使用 __NOP() 消耗 CPU 周期。
 * @param   us: 需要延时的微秒数
 * @retval  无
 */
static void delay_us(uint32_t us) 
{
    uint32_t ticks = us * (HAL_RCC_GetHCLKFreq() / 1000000);
    uint32_t start = DWT->CYCCNT; // 获取当前周期计数值
    // 等待经过指定的时钟周期数
    while ((DWT->CYCCNT - start) < ticks);
}

// ================= 对外接口实现 =================

/**
 * @brief   灰度传感器底层初始化
 * @note    主要用于执行 ADC 上电后的内部硬件校准，消除零点偏移误差。
 *          必须在 MX_ADC1_Init() 之后调用。
 * @param   无
 * @retval  无
 */
void Sensor_Init(void)
{
    // 1. 开启 DWT 循环计数器功能 (用于精确 delay_us)
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; 
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    // 2. 开启 ADC 硬件校准
    HAL_ADCEx_Calibration_Start(&hadc1);
}

/**
 * @brief   读取全部 8 路探头的模拟量数据
 * @note    通过 GPIO 控制 74HC4051 多路复用器的 AD0-AD2 引脚切换通道，
 *          并在延时稳定后触发单次 ADC 轮询转换，将结果存入 Analog 数组。
 * @param   无
 * @retval  无
 */
void Read_All_Analog(void)
{
    for(uint8_t i = 0; i < 8; i++)
    {
        // 设置多路复用器地址线，选中第 i 个通道
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, (i & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, ((i >> 1) & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, ((i >> 2) & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);

        // 硬件延时，等待通道切换及模拟电压稳定
        delay_us(40); 

        // 启动 ADC 转换并阻塞等待结果 (单路超时时间 10ms)
        HAL_ADC_Start(&hadc1); 
        if(HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)
        {
            Analog[i] = HAL_ADC_GetValue(&hadc1); 
        }
        /* 超时时 Analog[i] 保持旧值（首次采样前为 0）；本次扫描仍会用它判定黑白。
         * 这是现有行为，不代表该路已取得新的有效采样。 */
    }
}

/**
 * @brief   扫描传感器阵列并输出 8 位数字状态 (核心滞回比较算法)
 * @note    基于预设的 black[] 和 white[] 数组计算动态双阈值 (GrayW, GrayB)。
 *          结合上次的探测状态进行惯性滤波，有效抵抗赛道噪点和距离微变带来的干扰。
 * @param   无
 * @retval  uint8_t: 8位传感器状态字节 (例如 0x18 代表中间两路压线)
 */
uint8_t Scan_Sensor_Byte(void)
{
    Read_All_Analog(); 
    uint8_t state_buffer = 0x00;

    for(uint8_t i = 0; i < 8; i++)
    {
        // 动态计算该通道的灰白阈值 (GrayW) 与灰黑阈值 (GrayB)
        uint16_t GrayW = ((uint32_t)black[i] + 2 * (uint32_t)white[i]) / 3; 
        uint16_t GrayB = (2 * (uint32_t)black[i] + (uint32_t)white[i]) / 3; 
        
        // 获取该通道在上一次循环中的探测状态
        uint8_t last_state = (Sensor_State_Buffer >> i) & 0x01; 

        /* 保留手册的双阈值公式，并按实际标定值选择比较方向。 */
        uint8_t is_black;
        if (black[i] < white[i]) {
            is_black = last_state ? (Analog[i] <= GrayW) : (Analog[i] < GrayB);
        } else {
            is_black = last_state ? (Analog[i] >= GrayW) : (Analog[i] > GrayB);
        }
        if (is_black) {
            state_buffer |= (1u << i);
        }
    }
    
    // 更新全局状态缓冲区，供下一次滞回判断使用
    Sensor_State_Buffer = state_buffer; 
    return state_buffer; 
}

/**
 * @brief   通过串口发送 8 路探头的原始模拟量数据
 * @note    用于前期环境标定及上位机波形观测，采用格式化字符串发送。
 * @param   无
 * @retval  无
 */
void Send_Analog_Data_UART(void)
{
    char tx_buffer[128];
    sprintf(tx_buffer, "CH1:%4d CH2:%4d CH3:%4d CH4:%4d CH5:%4d CH6:%4d CH7:%4d CH8:%4d\r\n", 
            Analog[0], Analog[1], Analog[2], Analog[3], 
            Analog[4], Analog[5], Analog[6], Analog[7]);
            
    HAL_UART_Transmit(&huart3, (uint8_t *)tx_buffer, strlen(tx_buffer), HAL_MAX_DELAY);
}
/**
 * @brief   通过串口发送 8 位传感器的数字状态
 * @note    将 state_buffer 转换为 8 位二进制字符串 (如 "00011000") 打印，
 *          便于直观监控小车偏离中心线的情况。
 * @param   无
 * @retval  无
 */
void Send_State_Buffer_UART(void)
{
    char tx_buffer[64];
    char binary_str[9]; 
    
    // 将 8 位整型转换为二进制字符串字符 (由高位向低位转换)
    for(int i = 7; i >= 0; i--) 
    {
        binary_str[7 - i] = (Sensor_State_Buffer & (1 << i)) ? '1' : '0';
    }
    binary_str[8] = '\0'; // 字符串终止符
    
    sprintf(tx_buffer, "State: %s\r\n", binary_str);
    HAL_UART_Transmit(&huart3, (uint8_t *)tx_buffer, strlen(tx_buffer), HAL_MAX_DELAY);
}

void Send_State_Buffer_Get_OLED(void)
{
    /* 第四行显示 CH1～CH8 的状态，末尾空格清除旧方向标记。 */
    uint8_t oled_text[17] = "State:00000000  ";
    uint8_t state = Sensor_State_Buffer;

    for (uint8_t i = 0u; i < 8u; i++) {
        oled_text[6u + i] = (state & (1u << i)) ? '1' : '0';
    }
    OLED_ShowString(0, 48, oled_text, 16, 1);
	OLED_Refresh();
}
	
