/**-----------------------------------------------------------------------
 \date  27.01.2025
 *
 | ** The second example: when the button is pressed, the LED lights up.            ** |
 | ** The code is written in the Keil uVision5 IDE                                  ** |
 *
 *  PY32F030K28U6TR
 *   ------------
 *  |            |
 *  |            |
 *  |            |
 *  |       PA.11| ---->  LED
 *  |       PB.1 | <----  Button
 *  |            |
 *  |            |
 *  |      +3.3V |
 *  |        GND |
 *
 | ** https://github.com/OpenPuya/PY32F0xx_Firmware
 | ** I use the TTP223 capacitive touch button module as a button.
 | ** https://www.chipdip.ru/product/ttp223
 */
/******************************* The first example ****************************/
#include "main.h"
#include "stdbool.h"

static void GpioConfig_Init(void);
volatile bool condition  = false;

int main(void)
{
  HAL_Init();                                  
  GpioConfig_Init();

  while (1)
  {
   condition = HAL_GPIO_ReadPin (GPIOB, GPIO_PIN_1);	
	 (condition == true)?(HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET)):(HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET));
  }
}
/******************************************************************************/
static void GpioConfig_Init(void)
{
  GPIO_InitTypeDef  GPIO_InitStruct = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH; 
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);   

  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct); 	
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
