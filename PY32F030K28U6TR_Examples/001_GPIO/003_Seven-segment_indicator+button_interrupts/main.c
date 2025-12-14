/**-----------------------------------------------------------------------
 \date  27.01.2025
 *
 * В проекте две кнопки + сегментный индикатор. Одна кнопка при нажатии увеличивает значение от 0 до 9
 * на семи сегментном индикаторе, вторая кнопка уменьшает от 9 до 0.
 *
 * The code is written in the Keil uVision5 IDE
 *
 *  PY32F030K28U6TR
 *   ------------
 *  |            |
 *  |            |
 *  |        PA.0| ---->  ---
 *  |        PA.1| ----> |   |
 *  |        PA.2| ----> |   |
 *  |        PA.3| ---->  ---
 *  |        PA.4| ----> |   |
 *  |        PA.5| ----> |   |
 *  |        PA.6| ---->  ---
 *  |            |
 *  |            |
 *  |        PB.7| <---- up button
 *  |        PB.6| <---- down button
 *
 *
 | ** https://github.com/OpenPuya/PY32F0xx_Firmware
 | ** I use the TTP223 capacitive touch button module as a button.
 | ** https://www.chipdip.ru/product/ttp223
 */
/******************************* The first example ****************************/
#include "main.h"
#include "stdbool.h"

typedef uint8_t u8;
u8 CW[10] = {0x3F, 0X6, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x7, 0x7F, 0x67};

static void GpioConfig_Init(void);
volatile bool i = 0;
__IO u8 Count = 1;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  if (GPIO_Pin == GPIO_PIN_6) {
    i = false;
    Count--;
    if (Count <= 0)
      Count = 1;
  }
  if (GPIO_Pin == GPIO_PIN_7) {
    i = false;
    Count++;
    if (Count > 10)
      Count = 10;
  }
}

int main(void) {
  HAL_Init();
  GpioConfig_Init();

  while (1) {
    switch (Count) {
    case 1:
      GPIOA->ODR = CW[0];
      i = true;
      break;
    case 2:
      GPIOA->ODR = CW[1];
      i = true;
      break;
    case 3:
      GPIOA->ODR = CW[2];
      i = true;
      break;
    case 4:
      GPIOA->ODR = CW[3];
      i = true;
      break;
    case 5:
      GPIOA->ODR = CW[4];
      i = true;
      break;
    case 6:
      GPIOA->ODR = CW[5];
      i = true;
      break;
    case 7:
      GPIOA->ODR = CW[6];
      i = true;
      break;
    case 8:
      GPIOA->ODR = CW[7];
      i = true;
      break;
    case 9:
      GPIOA->ODR = CW[8];
      i = true;
      break;
    case 10:
      GPIOA->ODR = CW[9];
      i = true;
      break;
    }
    while (i) {
    }
  }
}
/******************************************************************************/
static void GpioConfig_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
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