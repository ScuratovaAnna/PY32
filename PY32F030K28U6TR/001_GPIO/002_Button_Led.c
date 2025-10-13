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

/*
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
 *
 */
/****************************** The second example ****************************/
#include "main.h"
#include "stdbool.h"

volatile bool condition  = false;

static void APP_SystemClockConfig(void);
static void APP_GpioConfig(void);
/******************************************************************************/
int main(void)
{
  APP_GpioConfig();  
  APP_SystemClockConfig();

  while (1)
  {
    condition =  LL_GPIO_IsInputPinSet(GPIOB, LL_GPIO_PIN_1);	
	 (condition == true)?(LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_11)):(LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_11));
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
  // Set AHB prescaler
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  // Configure HSISYS as system clock source 
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
	
  SET_BIT(RCC->IOPENR, LL_IOP_GRP1_PERIPH_GPIOB);
  LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_1, LL_GPIO_MODE_INPUT);
  LL_GPIO_SetPinPull(GPIOB, LL_GPIO_PIN_1, LL_GPIO_PULL_NO);// Select I/O no pull 
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
/*
 *  PY32F030K28U6TR
 *   ------------
 *  |            |
 *  |            |
 *  |            |
 *  |       PA.2 | ---->  LED
 *  |       PA.3 | ---->  LED
 *  |       PA.4 | ---->  LED
 *  |            |
 *  |       PB.7 | <----  Button
 *  |            |
 *  |      +3.3V |
 *  |        GND |
 *
 */
/****************************** The third example *****************************/
#include "py32f030x8.h"

#define SET_BIT(REG, BIT) ((REG) |= (BIT))
#define CLEAR_BIT(REG, BIT) ((REG) &= ~(BIT))
#define READ_BIT(REG, BIT) ((REG) & (BIT))
#define CLEAR_REG(REG) ((REG) = (0x0))
#define WRITE_REG(REG, VAL) ((REG) = (VAL))
#define READ_REG(REG) ((REG))
#define MODIFY_REG(REG, CLEARMASK, SETMASK) WRITE_REG((REG), (((READ_REG(REG)) & (~(CLEARMASK))) | (SETMASK)))

#define GPIO_Pin_0  ((uint16_t)0x0001)
#define GPIO_Pin_1  ((uint16_t)0x0002)
#define GPIO_Pin_2  ((uint16_t)0x0004)
#define GPIO_Pin_3  ((uint16_t)0x0008)
#define GPIO_Pin_4  ((uint16_t)0x0010)
#define GPIO_Pin_5  ((uint16_t)0x0020)
#define GPIO_Pin_6  ((uint16_t)0x0040)
#define GPIO_Pin_7  ((uint16_t)0x0080)
#define GPIO_Pin_8  ((uint16_t)0x0100)
#define GPIO_Pin_9  ((uint16_t)0x0200)
#define GPIO_Pin_10 ((uint16_t)0x0400)
#define GPIO_Pin_11 ((uint16_t)0x0800)
#define GPIO_Pin_12 ((uint16_t)0x1000)
#define GPIO_Pin_13 ((uint16_t)0x2000)
#define GPIO_Pin_14 ((uint16_t)0x4000)
#define GPIO_Pin_15 ((uint16_t)0x8000)

#define Input_mode (0x0UL)
#define General_purpose_output_mode (0x1UL)
#define Alternate_function_mode (0x2UL)
#define Analog_mode (0x3UL)

#define Output_push_pull (0x0UL)
#define Output_open_drain (0x1UL)

#define Very_low_speed (0x0UL)
#define Low_speed (0x1UL)
#define High_speed (0x2UL)
#define Very_high_speed (0x3UL)

#define No_pull_up_No_pull_down (0x0UL)
#define Pull_up (0x1UL)
#define Pull_down (0x2UL)

void Led_PORTA_2_INIT(void);
void Button_PORTB_7_INIT(void);

int main(void)
{
  Led_PORTA_2_INIT();
  Button_PORTB_7_INIT();
  while (1)
  {
    ((GPIOB->IDR & GPIO_Pin_7)==0x0) ? (SET_BIT(GPIOA->BSRR, GPIO_BSRR_BS2|GPIO_BSRR_BS3|GPIO_BSRR_BS4)) : (SET_BIT(GPIOA->BSRR, GPIO_BSRR_BR2|GPIO_BSRR_BR3|GPIO_BSRR_BR4));
  }
}

void Led_PORTA_2_INIT(void)
{
  SET_BIT(RCC->IOPENR, RCC_IOPENR_GPIOAEN);
  MODIFY_REG(GPIOA->MODER, GPIO_MODER_MODE2, General_purpose_output_mode << GPIO_MODER_MODE2_Pos); // Настройка GPIOA пин 2 на выход (output mode)
  MODIFY_REG(GPIOA->OTYPER, GPIO_OTYPER_OT2, Output_push_pull << GPIO_OTYPER_OT2_Pos);             // Настройка GPIOA пин 2 в режим Push-Pull
  MODIFY_REG(GPIOA->OSPEEDR, GPIO_OSPEEDR_OSPEED2, High_speed << GPIO_OSPEEDR_OSPEED2_Pos);        // Настройка GPIOA пин 2 в режим High_speed
  MODIFY_REG(GPIOA->PUPDR, GPIO_PUPDR_PUPD2, No_pull_up_No_pull_down << GPIO_PUPDR_PUPD2_Pos);     // Настройка GPIOA пин 2 в режим No_pull_up_No_pull_down
  // ===================================================================
  MODIFY_REG(GPIOA->MODER, GPIO_MODER_MODE3, General_purpose_output_mode << GPIO_MODER_MODE3_Pos); // Настройка GPIOA пин 3 на выход (output mode)
  MODIFY_REG(GPIOA->OTYPER, GPIO_OTYPER_OT3, Output_push_pull << GPIO_OTYPER_OT3_Pos);             // Настройка GPIOA пин 3 в режим Push-Pull
  MODIFY_REG(GPIOA->OSPEEDR, GPIO_OSPEEDR_OSPEED3, High_speed << GPIO_OSPEEDR_OSPEED3_Pos);        // Настройка GPIOA пин 3 в режим High_speed
  MODIFY_REG(GPIOA->PUPDR, GPIO_PUPDR_PUPD3, No_pull_up_No_pull_down << GPIO_PUPDR_PUPD3_Pos);     // Настройка GPIOA пин 3 в режим No_pull_up_No_pull_down
  // ===================================================================
  MODIFY_REG(GPIOA->MODER, GPIO_MODER_MODE4, General_purpose_output_mode << GPIO_MODER_MODE4_Pos); // Настройка GPIOA пин 4 на выход (output mode)
  MODIFY_REG(GPIOA->OTYPER, GPIO_OTYPER_OT4, Output_push_pull << GPIO_OTYPER_OT4_Pos);             // Настройка GPIOA пин 4 в режим Push-Pull
  MODIFY_REG(GPIOA->OSPEEDR, GPIO_OSPEEDR_OSPEED4, High_speed << GPIO_OSPEEDR_OSPEED4_Pos);        // Настройка GPIOA пин 4 в режим High_speed
  MODIFY_REG(GPIOA->PUPDR, GPIO_PUPDR_PUPD4, No_pull_up_No_pull_down << GPIO_PUPDR_PUPD4_Pos);     // Настройка GPIOA пин 4 в режим No_pull_up_No_pull_down
}

void Button_PORTB_7_INIT(void)
{
 SET_BIT(RCC->IOPENR, RCC_IOPENR_GPIOBEN);
 MODIFY_REG(GPIOB->MODER, GPIO_MODER_MODE7, Input_mode << GPIO_MODER_MODE7_Pos);                  // Настройка GPIOB пин 7 на выход (Input_mode)
 MODIFY_REG(GPIOB->PUPDR, GPIO_PUPDR_PUPD7, No_pull_up_No_pull_down << GPIO_PUPDR_PUPD7_Pos);     // Настройка GPIOB пин 7 в режим No_pull_up_No_pull_down
}
/*****************************END OF FILE**************************************/
