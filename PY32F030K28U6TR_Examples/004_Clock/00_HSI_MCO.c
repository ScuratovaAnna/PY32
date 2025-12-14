/**-----------------------------------------------------------------------
 \date  28.10.2025
 *
 *
 *  PY32F030K28U6TR
 *   ------------
 *  |            |
 *  |            |
 *  |            |
 *  |       PA.11| ---->  LED
 *  |            |
 *  |            |
 *  |        PA.8| ----> MCO _|¯|_ 8MHz HSI out
 *  |            |
 *  |      +3.3V |
 *  |        GND |
 *
 *
 *
/******************************* The first example ****************************/
#include "py32f030x8.h"

#define SET_BIT(REG, BIT) ((REG) |= (BIT))
#define CLEAR_BIT(REG, BIT) ((REG) &= ~(BIT))
#define READ_BIT(REG, BIT) ((REG) & (BIT))
#define CLEAR_REG(REG) ((REG) = (0x0))
#define WRITE_REG(REG, VAL) ((REG) = (VAL))
#define READ_REG(REG) ((REG))
#define MODIFY_REG(REG, CLEARMASK, SETMASK) WRITE_REG((REG), (((READ_REG(REG)) & (~(CLEARMASK))) | (SETMASK)))

#define Input_mode                  (0x0UL)
#define General_purpose_output_mode (0x1UL)
#define Alternate_function_mode     (0x2UL)
#define Analog_mode                 (0x3UL)

#define Output_push_pull            (0x0UL)
#define Output_open_drain           (0x1UL)

#define Very_low_speed              (0x0UL)
#define Low_speed                   (0x1UL)
#define High_speed                  (0x2UL)
#define Very_high_speed             (0x3UL)

#define No_pull_up_No_pull_down     (0x0UL)
#define Pull_up                     (0x1UL)
#define Pull_down                   (0x2UL)

#define PA8_MCO_AF5                 (0x5UL)

#define RCC_SYSCLK_DIV_1            0x00000000U
#define RCC_SYSCLK_DIV_2            (RCC_CFGR_HPRE_3)                                                       // SYSCLK not divided 
#define RCC_SYSCLK_DIV_4            (RCC_CFGR_HPRE_3 | RCC_CFGR_HPRE_0)                                     // SYSCLK divided by 4
#define RCC_SYSCLK_DIV_8            (RCC_CFGR_HPRE_3 | RCC_CFGR_HPRE_1)                                     // SYSCLK divided by 8
#define RCC_SYSCLK_DIV_16           (RCC_CFGR_HPRE_3 | RCC_CFGR_HPRE_1 | RCC_CFGR_HPRE_0)                   // SYSCLK divided by 16
#define RCC_SYSCLK_DIV_64           (RCC_CFGR_HPRE_3 | RCC_CFGR_HPRE_2)                                     // SYSCLK divided by 64
#define RCC_SYSCLK_DIV_128          (RCC_CFGR_HPRE_3 | RCC_CFGR_HPRE_2 | RCC_CFGR_HPRE_0)                   // SYSCLK divided by 128
#define RCC_SYSCLK_DIV_256          (RCC_CFGR_HPRE_3 | RCC_CFGR_HPRE_2 | RCC_CFGR_HPRE_1)                   // SYSCLK divided by 256
#define RCC_SYSCLK_DIV_512          (RCC_CFGR_HPRE_3 | RCC_CFGR_HPRE_2 | RCC_CFGR_HPRE_1 | RCC_CFGR_HPRE_0) //SYSCLK divided by 512

#define RCC_APB1_DIV_1              0x00000000U                                           // HCLK not divided
#define RCC_APB1_DIV_2              (RCC_CFGR_PPRE_2)                                     // HCLK divided by 2
#define RCC_APB1_DIV_4              (RCC_CFGR_PPRE_2 | RCC_CFGR_PPRE_0)                   // HCLK divided by 4
#define RCC_APB1_DIV_8              (RCC_CFGR_PPRE_2 | RCC_CFGR_PPRE_1)                   // HCLK divided by 8
#define RCC_APB1_DIV_16             (RCC_CFGR_PPRE_2 | RCC_CFGR_PPRE_1 | RCC_CFGR_PPRE_0) // HCLK divided by 16

#define RCC_SYS_CLKSOURCE_HSISYS    0x00000000U                          // HSISYS selection as system clock 
#define RCC_SYS_CLKSOURCE_HSE       (RCC_CFGR_SW_0)                      // HSE selection as system clock 
#define RCC_SYS_CLKSOURCE_PLL       (RCC_CFGR_SW_1)                      // PLL selection as system clock 
#define RCC_SYS_CLKSOURCE_LSI       (RCC_CFGR_SW_1 | RCC_CFGR_SW_0)      // LSI selection used as system clock 
#define RCC_SYS_CLKSOURCE_LSE       (RCC_CFGR_SW_2)                      //LSE selection used as system clock

#define RCC_SYS_CLKSOURCE_STATUS_HSISYS   0x00000000U                         // HSISYS used as system clock 
#define RCC_SYS_CLKSOURCE_STATUS_HSE      RCC_CFGR_SWS_0                      // HSE used as system clock 
#define RCC_SYS_CLKSOURCE_STATUS_PLL      RCC_CFGR_SWS_1                      // PLL used as system clock 
#define RCC_SYS_CLKSOURCE_STATUS_LSI      (RCC_CFGR_SWS_1 | RCC_CFGR_SWS_0)   // LSI used as system clock 
#define RCC_SYS_CLKSOURCE_STATUS_LSE      RCC_CFGR_SWS_2                      // LSE used as system clock 

#define RCC_MCO1SOURCE_NOCLOCK            0x00000000U                            // MCO output disabled, no clock on MCO
#define RCC_MCO1SOURCE_SYSCLK             RCC_CFGR_MCOSEL_0                      // SYSCLK selection as MCO1 source
#define RCC_MCO1SOURCE_HSI                (RCC_CFGR_MCOSEL_0| RCC_CFGR_MCOSEL_1) // HSI selection as MCO1 source
#define RCC_MCO1SOURCE_HSE                RCC_CFGR_MCOSEL_2                      // HSE selection as MCO1 source
#define RCC_MCO1SOURCE_PLLCLK             (RCC_CFGR_MCOSEL_0|RCC_CFGR_MCOSEL_2)  // Main PLL selection as MCO1 source
#define RCC_MCO1SOURCE_LSI                (RCC_CFGR_MCOSEL_1|RCC_CFGR_MCOSEL_2)  // LSI selection as MCO1 source
#define RCC_MCO1SOURCE_LSE                (RCC_CFGR_MCOSEL_0|RCC_CFGR_MCOSEL_1|RCC_CFGR_MCOSEL_2) // LSE selection as MCO1 source

#define RCC_MCO1_DIV_1                  0x00000000U                                                 // MCO1 not divided
#define RCC_MCO1_DIV_2                  RCC_CFGR_MCOPRE_0                                           // MCO1 divided by 2
#define RCC_MCO1_DIV_4                  RCC_CFGR_MCOPRE_1                                           // MCO1 divided by 4
#define RCC_MCO1_DIV_8                  (RCC_CFGR_MCOPRE_1 | RCC_CFGR_MCOPRE_0)                     // MCO1 divided by 8
#define RCC_MCO1_DIV_16                 RCC_CFGR_MCOPRE_2                                           // MCO1 divided by 16
#define RCC_MCO1_DIV_32                 (RCC_CFGR_MCOPRE_2 | RCC_CFGR_MCOPRE_0)                     // MCO1 divided by 32
#define RCC_MCO1_DIV_64                 (RCC_CFGR_MCOPRE_2 | RCC_CFGR_MCOPRE_1)                     // MCO1 divided by 64
#define RCC_MCO1_DIV_128                (RCC_CFGR_MCOPRE_2 | RCC_CFGR_MCOPRE_1 | RCC_CFGR_MCOPRE_0) // MCO1 divided by 128

void delay_ms(uint32_t ms);
void System_ClockConfig(void);
void InitTick(uint32_t HCLKFrequency, uint32_t Ticks);
void Gpio_Config(void);
void MCO_GPIOConfig(void);

int main(void)
{
  System_ClockConfig();
  Gpio_Config();
  MCO_GPIOConfig();

  // LL_RCC_ConfigMCO(LL_RCC_MCO1SOURCE_SYSCLK,LL_RCC_MCO1_DIV_1);
  // MODIFY_REG(RCC->CFGR, RCC_CFGR_MCOSEL | RCC_CFGR_MCOPRE, MCOxSource | MCOxPrescaler);
  //MODIFY_REG(RCC->CFGR, RCC_CFGR_MCOSEL | RCC_CFGR_MCOPRE, LL_RCC_MCO1SOURCE_SYSCLK | LL_RCC_MCO1_DIV_1);
  MODIFY_REG(RCC->CFGR, RCC_CFGR_MCOSEL, RCC_MCO1SOURCE_SYSCLK);
  MODIFY_REG(RCC->CFGR,RCC_CFGR_MCOPRE, RCC_MCO1_DIV_1);

  while (1)
  {
    delay_ms(300);
    WRITE_REG(GPIOA->ODR, READ_REG(GPIOA->ODR) ^ GPIO_BSRR_BS11);
  }
}

void delay_ms(uint32_t ms) {
  for (uint32_t i = 0; i < ms * 1000; i++) {
    __NOP(); // No operation instruction
  }
}

void System_ClockConfig(void)
{
  // Enable HSI
  //LL_RCC_HSI_Enable();
  SET_BIT(RCC->CR, RCC_CR_HSION);

  while(READ_BIT(RCC->CR, RCC_CR_HSIRDY == 0)){}
  //while(LL_RCC_HSI_IsReady() != 1){}

  // Set AHB prescaler
  //LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  MODIFY_REG(RCC->CFGR, RCC_CFGR_HPRE, RCC_SYSCLK_DIV_1);

  // Configure HSISYS as system clock source
  //LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSISYS);
  MODIFY_REG(RCC->CFGR, RCC_CFGR_SW, RCC_SYS_CLKSOURCE_HSISYS);

  //while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSISYS){}
  while(READ_BIT(RCC->CFGR, RCC_CFGR_SWS != RCC_SYS_CLKSOURCE_STATUS_HSISYS)){}

  // Set APB1 prescaler
  //LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE, RCC_APB1_DIV_1);

  // LL_Init1msTick(8000000);
  //LL_InitTick(HCLKFrequency, 1000U);
  InitTick( 8000000,1000U);

  // Update system clock global variable SystemCoreClock (can also be updated by calling SystemCoreClockUpdate function) 
  //LL_SetSystemCoreClock(8000000);
  //LL_SetSystemCoreClock(uint32_t HCLKFrequency)
  SystemCoreClock = 8000000;
}

void InitTick(uint32_t HCLKFrequency, uint32_t Ticks)
{
  /* Configure the SysTick to have interrupt in 1ms time base */
  SysTick->LOAD  = (uint32_t)((HCLKFrequency / Ticks) - 1UL);  /* set reload register */
  SysTick->VAL   = 0UL;                                       /* Load the SysTick Counter Value */
  SysTick->CTRL  = SysTick_CTRL_CLKSOURCE_Msk |
                   SysTick_CTRL_ENABLE_Msk;                   /* Enable the Systick Timer */
}

void Gpio_Config(void)
{
  SET_BIT(RCC->IOPENR, RCC_IOPENR_GPIOAEN);

  MODIFY_REG(GPIOA->MODER, GPIO_MODER_MODE11, General_purpose_output_mode << GPIO_MODER_MODE11_Pos); // Настройка GPIOA пин 2 на выход (output mode)
  MODIFY_REG(GPIOA->OTYPER, GPIO_OTYPER_OT11, Output_push_pull << GPIO_OTYPER_OT11_Pos);             // Настройка GPIOA пин 2 в режим Push-Pull
  MODIFY_REG(GPIOA->OSPEEDR, GPIO_OSPEEDR_OSPEED11, High_speed << GPIO_OSPEEDR_OSPEED11_Pos);        // Настройка GPIOA пин 2 в режим High_speed
  MODIFY_REG(GPIOA->PUPDR, GPIO_PUPDR_PUPD11, No_pull_up_No_pull_down << GPIO_PUPDR_PUPD11_Pos);     // Настройка GPIOA пин 2 в режим No_pull_up_No_pull_down
}

 void MCO_GPIOConfig(void)
{
  MODIFY_REG(GPIOA->MODER, GPIO_MODER_MODE8, Alternate_function_mode << GPIO_MODER_MODE8_Pos);       
  MODIFY_REG(GPIOA->AFR[1], GPIO_AFRH_AFSEL8, PA8_MCO_AF5  << GPIO_AFRH_AFSEL8_Pos );
  //AFR[0] (AFRL - Low): Управляет пинами PA0 - PA7.
  //AFR[1] (AFRH - High): Управляет пинами PA8 - PA15
  MODIFY_REG(GPIOA->OTYPER, GPIO_OTYPER_OT8, Output_push_pull << GPIO_OTYPER_OT8_Pos);
  MODIFY_REG(GPIOA->OSPEEDR, GPIO_OSPEEDR_OSPEED8, Very_high_speed << GPIO_OSPEEDR_OSPEED8_Pos);
  MODIFY_REG(GPIOA->PUPDR, GPIO_PUPDR_PUPD8, No_pull_up_No_pull_down << GPIO_PUPDR_PUPD8_Pos);
}
/*****************************END OF FILE**************************************/

