/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : 主程序
 ******************************************************************************
 * @attention
 *
 * 版权所有 (c) 2024 STMicroelectronics.
 * 保留所有权利。
 *
 * 本软件按“原样”提供，不提供任何形式的明示或暗示担保。
 *
 * 2025-02-07    flagset      TianFu
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "hrtim.h"
#include "i2c.h"
#include "iwdg.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "OLED.h"
#include "function.h"
#include "Key.h"
#include "PID.h"
#include "W25Q64.h"

/* Private variables ---------------------------------------------------------*/
volatile uint16_t ms_cnt_1 = 0; // 毫秒计时变量1
volatile uint16_t ms_cnt_2 = 0; // 毫秒计时变量2
volatile uint16_t ms_cnt_3 = 0; // 毫秒计时变量3
volatile uint16_t ms_cnt_4 = 0; // 毫秒计时变量4

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Init_Peripherals(void);
void Process_Tasks(void);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

/**
 * @brief  应用程序入口
 * @retval int
 */
int main(void)
{
    HAL_Init();           // 初始化HAL库
    SystemClock_Config(); // 配置系统时钟
    Init_Peripherals();   // 初始化外设
    Process_Tasks();      // 主任务处理
    return 0;
}

/**
 * @brief 初始化外设
 */
void Init_Peripherals(void)
{
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_TIM2_Init();
    MX_I2C3_Init();
    MX_ADC1_Init();
    MX_ADC2_Init();
    MX_USART1_UART_Init();
    MX_TIM8_Init();
    MX_ADC5_Init();
    MX_HRTIM1_Init();
    MX_TIM3_Init();
    MX_IWDG_Init();
    MX_TIM4_Init();
    MX_SPI3_Init();

    FlagSet.SMFlag = Init;                    // 初始化状态机
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3); // 启动PWM输出
    FAN_PWM_set(100);                         // 设置风扇转速为100%
    OLED_Init();                              // OLED初始化
    OLED_Clear();                             // 清除OLED显示
    OLED_ShowChinese(40, 24, "启动中");       // 显示“启动中”
    OLED_Update();                            // 更新OLED显示
    HAL_TIM_Base_Start_IT(&htim2);            // 启动定时器2（1kHz）
    HAL_TIM_Base_Start_IT(&htim3);            // 启动定时器3（200Hz）
    HAL_TIM_Base_Start_IT(&htim4);            // 启动定时器4（100Hz）
    Key_Init();                               // 按键初始化
    PID_Init();                               // PID初始化
    Init_Flash();                             // Flash初始化
    Read_Flash();                             // 读取Flash数据

    HAL_Delay(200);                                        // 延时等待供电稳定
    HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED); // 校准ADC1
    HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED); // 校准ADC2
    HAL_ADCEx_Calibration_Start(&hadc5, ADC_SINGLE_ENDED); // 校准ADC5
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)ADC1_RESULT, 4); // 启动ADC1 DMA采样
    HAL_ADC_Start(&hadc2);                                 // 启动ADC2采样
    HAL_ADC_Start(&hadc5);                                 // 启动ADC5采样

    HAL_HRTIM_WaveformCountStart(&hhrtim1, HRTIM_TIMERID_TIMER_D);                     // 启动HRTIM波形计数器
    HAL_HRTIM_WaveformCountStart(&hhrtim1, HRTIM_TIMERID_TIMER_F);                     // 启动HRTIM波形计数器
    __HAL_HRTIM_TIMER_ENABLE_IT(&hhrtim1, HRTIM_TIMERINDEX_TIMER_D, HRTIM_TIM_IT_REP); // 开启HRTIM定时器D中断

    HAL_GPIO_WritePin(GPIOC, LED_G_Pin | LED_R_Pin, GPIO_PIN_RESET); // 关闭LED
    HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET); // 关闭蜂鸣器
    FAN_PWM_set(0);                                                  // 设置风扇转速为0
}

/**
 * @brief 主任务处理
 */
void Process_Tasks(void)
{
    while (1) {
        Encoder(); // 编码器信号处理

        if (ms_cnt_3 >= 10) { // 10ms任务
            ms_cnt_3 = 0;
            BUZZER_Short();  // 蜂鸣器短鸣
            ADC_calculate(); // ADC采样计算
        }

        if (ms_cnt_4 >= 50) { // 50ms任务
            ms_cnt_4 = 0;
            BUZZER_Middle(); // 蜂鸣器中鸣

            HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, (FlagSet.SMFlag == Rise || FlagSet.SMFlag == Run) ? GPIO_PIN_SET : GPIO_PIN_RESET); // LED_G状态控制

            powerEfficiency = (IOUT >= 0.1) ? (VOUT * IOUT) / (VIN * IIN) * 100.0 : 0;                                                            // 计算效率
            USART1_Printf("%.3f,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,%d\n", VIN, IIN, VOUT, IOUT, MainBoard_TEMP, CPU_TEMP, powerEfficiency, CVCC_Mode); // 串口输出
        }

        if (ms_cnt_2 >= 100) { // 100ms任务
            ms_cnt_2 = 0;
            OLED_Display(); // 刷新OLED显示
            Auto_FAN();     // 自动风扇控制
        }

        if (ms_cnt_1 >= 500) { // 500ms任务
            ms_cnt_1 = 0;
            HAL_GPIO_TogglePin(LED_R_GPIO_Port, LED_R_Pin); // LED_R翻转
            Update_Flash();                                 // 更新Flash
        }

        HAL_IWDG_Refresh(&hiwdg); // 喂狗
    }
}

/**
 * @brief 系统时钟配置
 */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI | RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState       = RCC_HSE_ON;
    RCC_OscInitStruct.LSIState       = RCC_LSI_ON;
    RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM       = RCC_PLLM_DIV5;
    RCC_OscInitStruct.PLL.PLLN       = 68;
    RCC_OscInitStruct.PLL.PLLP       = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ       = RCC_PLLQ_DIV2;
    RCC_OscInitStruct.PLL.PLLR       = RCC_PLLR_DIV2;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
        Error_Handler();
    }

    HAL_RCC_EnableCSS();
}

/**
 * @brief 定时器中断回调函数
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2) { // 1ms任务
        ms_cnt_1++;
        ms_cnt_2++;
        ms_cnt_3++;
        ms_cnt_4++;
    }
    if (htim->Instance == TIM3) { // 5ms任务
        ADCSample();              // ADC采样滤波
        ShortOff();               // 短路保护
        OTP();                    // 过温保护
        OVP();                    // 过压保护
        OCP();                    // 过流保护
        StateMachine();           // 状态机
        BBMode();                 // 运行模式判断
    }
    if (htim->Instance == TIM4) { // 10ms任务
        KEY_Scan(1, KEY1);        // 按键1扫描
        KEY_Scan(2, KEY2);        // 按键2扫描
        KEY_Scan(3, Encoder_KEY); // 编码器按键扫描
        Key_Process();            // 按键处理
    }
}

/**
 * @brief 错误处理函数
 */
void Error_Handler(void)
{
    __disable_irq();
    while (1) {
    }
}

#ifdef USE_FULL_ASSERT
/**
 * @brief 断言失败处理函数
 */
void assert_failed(uint8_t *file, uint32_t line)
{
    /* USER CODE BEGIN 6 */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */