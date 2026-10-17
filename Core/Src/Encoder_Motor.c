#include "main.h"
#include "tim.h"
#include "gpio.h"
#include "Encoder_Motor.h"

// ================= 控制电机 A =================
// 参数说明：speed 的有效范围是 -7199 到 7199
// 正数代表正转，负数代表反转，0 代表停止
void Set_Motor_A_Speed(int speed)
{
    if (speed > 0) 
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);   // AIN1 置高
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET); // AIN2 置低
    } 
    else if (speed < 0) 
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET); // AIN1 置低
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);   // AIN2 置高
        speed = -speed; // 将负数速度值转换为正数，供 PWM 寄存器使用
    } 
    else 
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET); // 刹车/滑行
    }
    
    // 修改 PWMA (TIM4_CH3 / PB8) 的比较寄存器
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, speed); 
}

// ================= 控制电机 B =================
// 参数说明：speed 的有效范围是 -7199 到 7199
void Set_Motor_B_Speed(int speed)
{
    if (speed > 0) 
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);   // BIN1 置高
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET); // BIN2 置低
    } 
    else if (speed < 0) 
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET); // BIN1 置低
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);   // BIN2 置高
        speed = -speed; 
    } 
    else 
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
    }

    // 修改 PWMB (TIM1_CH2) 的比较寄存器
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, speed); 
}

// ================= 编码器读数（测速） =================
// 返回自上次读取以来的计数增量并清零。
// TIM2/TIM3 是 16 位计数器，转成 int16_t 后：
// 正数=正转、负数=反转，只要两次读取间隔内增量不超过 ±32767 就不会出错。
int16_t Encoder_A_GetCount(void)
{
    int16_t cnt = (int16_t)__HAL_TIM_GET_COUNTER(&htim2);
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    return cnt;
}

int16_t Encoder_B_GetCount(void)
{
    int16_t cnt = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);
    __HAL_TIM_SET_COUNTER(&htim3, 0);
    return cnt;
}

// 输出轴转速(RPM) = 电机轴转速 / 减速比
// 电机轴转速 = 计数 / 每圈脉冲数 / 时间(分钟)
float Encoder_A_Get_RPM(float dt_ms)
{
    float rpm_motor;
    if (dt_ms <= 0.0f)
        return 0.0f;
    rpm_motor = (float)Encoder_A_GetCount() / ENCODER_PPR * 60000.0f / dt_ms;
    return rpm_motor / GEAR_RATIO;
}

float Encoder_B_Get_RPM(float dt_ms)
{
    float rpm_motor;
    if (dt_ms <= 0.0f)
        return 0.0f;
    rpm_motor = (float)Encoder_B_GetCount() / ENCODER_PPR * 60000.0f / dt_ms;
    return rpm_motor / GEAR_RATIO;
}
