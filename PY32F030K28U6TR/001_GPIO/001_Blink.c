/**-----------------------------------------------------------------------
 \date  27.01.2025
 *
 | ** The first example is blinking the LED.            ** |
 | ** The code is written in the Keil uVision5 IDE      ** |
 *
 *  PY32F030K28U6TR
 *   ------------
 *  |            |
 *  |            |
 *  |            |
 *  |       PA.11| ---->  LED
 *  |            |
 *  |            |
 *  |      +3.3V |
 *  |        GND |
 *
 | ** https://github.com/OpenPuya/PY32F0xx_Firmware
 */
/******************************* The first example ****************************/
#include "main.h"
#include "stdbool.h"

static void GpioConfig_Init(void);
volatile bool i = 0;

int main(void) {
  HAL_Init();
  GpioConfig_Init();

  while (1) {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, (i == 0) ? (i = GPIO_PIN_SET) : (i = GPIO_PIN_RESET));
    HAL_Delay(250);
  }
}
/******************************************************************************/
static void GpioConfig_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();

  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}
/******************************************************************************/
void APP_ErrorHandler(void) {
  while (1) {
  }
}
/******************************************************************************/
#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) {
  while (1) {
  }
}
#endif
/*****************************END OF FILE**************************************/
/*
 *  PY32F030K28U6TR
 *   ------------
 *  |            |
 *  |            |
 *  |            |
 *  |       PA.11| ---->  LED
 *  |            |
 *  |            |
 *  |      +3.3V |
 *  |        GND |
 *
 */
/****************************** The second example ****************************/

#include "main.h"
/******************************************************************************/
static void APP_SystemClockConfig(void);
static void APP_GpioConfig(void);
/******************************************************************************/
int main(void)
{
  APP_GpioConfig();  
  APP_SystemClockConfig();

  while (1)
  {
	LL_mDelay(500);
    //LL_GPIO_TogglePin(GPIOA, LL_GPIO_PIN_11);
	WRITE_REG(GPIOA->ODR, READ_REG(GPIOA->ODR) ^ LL_GPIO_PIN_11);
  }
}
/******************************************************************************/
static void APP_SystemClockConfig(void)
{
  // Enable HSI 
  LL_RCC_HSI_Enable();
  while(LL_RCC_HSI_IsReady() != 1)
  {
  }
  /* Set AHB prescaler 
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  // Configure HSISYS as system clock source */
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSISYS);
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSISYS)
  {
  }
  // Set APB1 prescaler
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_Init1msTick(8000000);
  // Update system clock global variable SystemCoreClock
  LL_SetSystemCoreClock(8000000);
}
/******************************************************************************/
static void APP_GpioConfig(void)
{
  /* Enable clock */
  SET_BIT(RCC->IOPENR, LL_IOP_GRP1_PERIPH_GPIOA);
  /* Configure LED pin as output */
  LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_11, LL_GPIO_MODE_OUTPUT);
  LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_11, LL_GPIO_PULL_NO);// Select I/O no pull 
}
/******************************************************************************/
void APP_ErrorHandler(void)
{
  while (1)
  {
  }
}
#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  while (1)
  {
  }
}
#endif
/*****************************END OF FILE**************************************/
