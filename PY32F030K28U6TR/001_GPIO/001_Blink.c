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
