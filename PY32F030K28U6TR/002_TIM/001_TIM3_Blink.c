/**------------------------------------------------------
 \date  29.01.2025
 *
 *
 | ** The code is written in the Keil uVision5 IDE   ** |
 *
 *  PY32F030K28U6TR
 *   ------------
 *  |            |
 *  |            |
 *  |            |
 *  |       PA.11|  ---->  LED PA.11  _|¯|_  Period=500ms +Width=250ms -Width=250ms
 *  |            |
 *  |       PA.08|  ---->  Вывод MCO (Master Clock Output)  _|¯|_  24MHz
 *  |            |
 *  |       PA.10|  <---   PA10 - OSC_IN
 *  |       PA.09|  --->   PA9 - OSC_OUT
 *  |            |
 *  |      +3.3V |
 *  |        GND |
 *
 * OSC_IN —|Кварц 24 МГц|— OSC_OUT
 *             |  |
 *             C1 C2 ( 20 пФ)
 *             |  |
 *            GND GND
 *
 | ** https://github.com/OpenPuya/PY32F0xx_Firmware
 */
/******************************************************************************/
#include "main.h"
#include "stdbool.h"

#define TIMER  TIM3
volatile static TIM_HandleTypeDef HTIMx;
volatile static uint32_t gu32_ticks = 0;

static void GpioConfig_Init(void);
static void APP_SystemClockConfig(void);
void TimerDelay_Init(void);
void delay_ms(volatile uint16_t au16_ms);
void delay_us(volatile uint16_t au16_us);

int main(void)
{
  HAL_Init();                                  
  GpioConfig_Init();
  /* Configure PA08 pin as MCO1 function to output the system clock */
  HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_SYSCLK, RCC_MCODIV_1);
  APP_SystemClockConfig();
  TimerDelay_Init();
  
  while (1) {    
    //HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_11);
	//delay_ms(10);
	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_11);
	delay_us(250);
  }
}
/******************************************************************************/
static void GpioConfig_Init(void)
{
  GPIO_InitTypeDef  GPIO_InitStruct = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();

  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH; 
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}
/******************************************************************************/
void TimerDelay_Init(void)
{
	 __HAL_RCC_TIM3_CLK_ENABLE();
	gu32_ticks = (HAL_RCC_GetHCLKFreq() / 1000000);
	HTIMx.Instance = TIMER;

    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    HTIMx.Init.Prescaler = gu32_ticks-1;
    HTIMx.Init.CounterMode = TIM_COUNTERMODE_UP;
    HTIMx.Init.Period = 65535;
    HTIMx.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HTIMx.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_Base_Init(&HTIMx) != HAL_OK)
    {
      APP_ErrorHandler();
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&HTIMx, &sClockSourceConfig) != HAL_OK)
    {
      APP_ErrorHandler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&HTIMx, &sMasterConfig) != HAL_OK)
    {
     APP_ErrorHandler();
    }

    HAL_TIM_Base_Start(&HTIMx);
}
/******************************************************************************/
static void APP_SystemClockConfig(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /* Oscillator configuration */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE | RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_LSI | RCC_OSCILLATORTYPE_LSE; /* Select oscillators HSE, HSI, LSI, LSE */
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;                          /* Enable HSI */
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;                          /* HSI not divided */
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_8MHz;  /* Configure HSI clock as 8MHz */
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;                          /* Enable HSE */
  RCC_OscInitStruct.HSEFreq = RCC_HSE_16_32MHz;
  RCC_OscInitStruct.LSIState = RCC_LSI_OFF;                         /* Disable LSI */
  RCC_OscInitStruct.LSEState = RCC_LSE_OFF;                         /* Disable LSE */
  /*RCC_OscInitStruct.LSEDriver = RCC_LSEDRIVE_MEDIUM;*/
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_OFF;                     /* Disable PLL */
  /*RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;*/          /* Select HSI as PLL source */
  /* Configure oscillators */
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    APP_ErrorHandler();
  }
  /* Initialize CPU, AHB, and APB bus clocks */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1;/* RCC system clock types */
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;                                        /* SYSCLK source select as HSE */
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;                                            /* AHB clock not divided */
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;                                             /* APB clock divided by 1 */
  /* Initialize RCC system clock (FLASH_LATENCY_0=24M or below; FLASH_LATENCY_1=48M) */
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    APP_ErrorHandler();
  }
}
/******************************************************************************/
void delay_ms(volatile uint16_t au16_ms)
{
	while(au16_ms > 0)
	{
		HTIMx.Instance->CNT = 0;
		au16_ms--;
		while (HTIMx.Instance->CNT < 1000);
	}
}
/******************************************************************************/
void delay_us(volatile uint16_t au16_us)
{
	HTIMx.Instance->CNT = 0;
	while (HTIMx.Instance->CNT < au16_us);
}

/******************************************************************************/
void APP_ErrorHandler(void)
{
  while (1)
  {
  }
}
/******************************************************************************/
#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  while (1)
  {
  }
}
#endif 

/*****************************END OF FILE**************************************/