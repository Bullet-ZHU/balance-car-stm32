# STM32 两轮平衡循迹小车

基于 STM32F103C8T6 的两轮平衡车工程，包含角度环、速度环、八路灰度循迹、双向环岛识别和蓝牙设置整图圈数。

## 打开工程

- 使用 Keil MDK 打开 `MDK-ARM/Balabce_Car.uvprojx`。工程配置引用 `Keil.STM32F1xx_DFP.2.2.0`。
- STM32CubeMX 外设配置保存在 `Balabce_Car.ioc`。
- 应用代码位于 `Core/Src` 和 `Core/Inc`，HAL/CMSIS 驱动位于 `Drivers`。

## 常用参数

- 主控制逻辑和循迹、环岛、圈数参数：`Core/Src/main.c`
- 八路灰度白底与黑底校准值：`Core/Src/Ganwei.c`
- 蓝牙通过 USART1 接收 ASCII `1`～`5`，设置整图圈数；未设置时继续沿路线行驶。

编译输出和本机调试文件由 `.gitignore` 排除；请在目标硬件上根据实际传感器读数重新检查灰度校准值。
