/**
  ******************************************************************************
  * @file    gpio_driver.c
  * @author  Vu Trung Hieu
  * @version V2.0
  * @date    30-August-2017
  * @brief   This file contains functions for LED, Button and DI
  *
 @verbatim
 ===============================================================================
                      ##### How to use this driver #####
 ===============================================================================
 @endverbatim
  *
  ******************************************************************************
  * @attention
  * (#) Use STM32F4xx_StdPeriph_Driver
  *
  ******************************************************************************
  */
  
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "gpio_driver.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static PULSE_HANDLE_T pv_EL_Home_Rising_Handler = 0;
static PULSE_HANDLE_T pv_EL_Home_Falling_Handler = 0;
static PULSE_HANDLE_T pv_AZ_Home_Rising_Handler = 0;
static PULSE_HANDLE_T pv_AZ_Home_Falling_Handler = 0;
static PULSE_HANDLE_T pv_EL_Limit_Rising_Handler = 0;
static PULSE_HANDLE_T pv_EL_Limit_Falling_Handler = 0;

/* Private function prototypes -----------------------------------------------*/
static void v_LED_Init(void);
static void v_Button_Init(void);
void v_DI_Init(void);
static void v_Home_Pulse_Init(void);
static void v_EL_Limit_Init(void);
static void v_DO_Init(void);

/* Private functions ---------------------------------------------------------*/

/** @defgroup Init Function
 *  @brief    ...
 *
 @verbatim
 ===============================================================================
                       ##### GPIO Driver Init Function #####
 ===============================================================================  

 @endverbatim
  * @{
  */

/**
  * @brief  GPIO Driver Init Function
  * @note   Call function first if using this driver
  * @param  none
  * @retval none
  */
void v_GPIO_Init(void)
{
  v_LED_Init();
  v_Led_Reset(LED0_PIN);
  v_Led_Reset(LED1_PIN);
  v_Led_Reset(LED2_PIN);
  
  v_Button_Init();
  
  //v_DI_Init();
  v_Home_Pulse_Init();
  v_EL_Limit_Init();
  
  v_DO_Init();
  v_DO_Reset(DO0_PIN);
  v_DO_Reset(DO1_PIN);
}

/**
  * @}
  */

/** @defgroup LED DRIVER
 *  @brief    ...
 *
 @verbatim
 ===============================================================================
                             ##### LED DRIVER #####
 ===============================================================================  

 @endverbatim
  * @{
  */

/**
  * @brief  Init 3 ouputs for 3 led RGB
  * @note   ...
  * @param  none
  * @retval none
  */
static void v_LED_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  /* GPIOD Peripheral clock enable */
  RCC_AHB1PeriphClockCmd(LED_PERIPH_GPIO, ENABLE);
  
  /* Configure LED1, LED2, LED3 in output pushpull mode */
  GPIO_InitStructure.GPIO_Pin   = LED0_PIN | LED1_PIN | LED2_PIN;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
  GPIO_Init(LED_GPIO, &GPIO_InitStructure);
}

/**
  * @brief  LED Set, Reset, Toggle
  * @note   ...
  * @param  LEDx_Pin: specifies the LED pin to write
  * @retval none
  */
void v_Led_Set(uint16_t LEDx_Pin)
{
  GPIO_SetBits(LED_GPIO, LEDx_Pin);
}

void v_Led_Reset(uint16_t LEDx_Pin)
{
  GPIO_ResetBits(LED_GPIO, LEDx_Pin);
}

void v_Led_Toggle(uint16_t LEDx_Pin)
{
  GPIO_ToggleBits(LED_GPIO, LEDx_Pin);
}
/**
  * @}
  */

/** @defgroup Button DRIVER
 *  @brief    ...
 *
 @verbatim
 ===============================================================================
                            ##### Button DRIVER #####
 ===============================================================================  

 @endverbatim
  * @{
  */

/**
  * @brief  Init for 2 buttons
  * @note   ...
  * @param  none
  * @retval none
  */
static void v_Button_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  /* GPIOD Peripheral clock enable */
  RCC_AHB1PeriphClockCmd(BTN_PERIPH_GPIO, ENABLE);
  
  /* Configure LED1, LED2, LED3 in output pushpull mode */
  GPIO_InitStructure.GPIO_Pin   = BTN0_PIN | BTN1_PIN;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
  GPIO_Init(BTN_GPIO, &GPIO_InitStructure);
}

/**
  * @}
  */

/** @defgroup DI DRIVER
 *  @brief    ...
 *
 @verbatim
 ===============================================================================
                               ##### DI DRIVER #####
 ===============================================================================  

 @endverbatim
  * @{
  */

/**
  * @brief  Init for 4 Digital Inputs
  * @note   ...
  * @param  none
  * @retval none
  */
void v_DI_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  /* GPIOD Peripheral clock enable */
  RCC_AHB1PeriphClockCmd(DI_PERIPH_GPIO, ENABLE);
  
  /* Configure LED1, LED2, LED3 in output pushpull mode */
  GPIO_InitStructure.GPIO_Pin   = DI0_PIN | DI1_PIN | DI2_PIN | DI3_PIN;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
  GPIO_Init(DI_GPIO, &GPIO_InitStructure);
}

/**
  * @brief  Read Input Pin
  * @note   It works right if enum_Pin 0 -> 7
  * @param  enum_Pin: Specifies the port bit to read
  * @retval The input port pin value
  */
uint8_t u8_DI_Read_Pin(ENUM_DI_PIN_T enum_Pin)
{
  return GPIO_ReadInputDataBit(DI_GPIO, 1 << enum_Pin);
}

/**
  * @brief  Init for 2 Home Pulses (Digital Input)
  * @note   AZ and EL
  * @param  none
  * @retval none
  */
static void v_Home_Pulse_Init(void)
{
  EXTI_InitTypeDef   EXTI_InitStructure;
  GPIO_InitTypeDef   GPIO_InitStructure;
  NVIC_InitTypeDef   NVIC_InitStructure;
  /* Config EL Home Pulse*/
  /* Enable GPIO clock */
  RCC_AHB1PeriphClockCmd(EL_HOME_PULSE_PORT_CLK, ENABLE);
  /* Enable SYSCFG clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
  /* Configure pin as input */
  GPIO_InitStructure.GPIO_Pin   = EL_HOME_PULSE_PIN;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
  GPIO_Init(EL_HOME_PULSE_PORT, &GPIO_InitStructure);
  /* Connect EXTI Line2 to EL pin */
  SYSCFG_EXTILineConfig(EL_EXTI_PORT_SOURCE, EL_HOME_PULSE_PIN_SOURCE);
  /* Configure EXTI EL - rising & falling when home */
  EXTI_InitStructure.EXTI_Line    = EL_EXTI_LINE;
  EXTI_InitStructure.EXTI_Mode    = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);
  /* Enable and set EXTI EL Interrupt to the lowest priority */
  NVIC_InitStructure.NVIC_IRQChannel = EL_EXTI_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority  = 2;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  /* Config AZ Home Pulse*/
  /* Enable GPIO clock */
  RCC_AHB1PeriphClockCmd(AZ_HOME_PULSE_PORT_CLK, ENABLE);
  /* Enable SYSCFG clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
  /* Configure pin as input */
  GPIO_InitStructure.GPIO_Pin   = AZ_HOME_PULSE_PIN;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
  GPIO_Init(AZ_HOME_PULSE_PORT, &GPIO_InitStructure);
  /* Connect EXTI Line1 to EL pin */
  SYSCFG_EXTILineConfig(AZ_EXTI_PORT_SOURCE, AZ_HOME_PULSE_PIN_SOURCE);
  /* Configure EXTI EL - rising & falling when home */
  EXTI_InitStructure.EXTI_Line    = AZ_EXTI_LINE;
  EXTI_InitStructure.EXTI_Mode    = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);
  /* Enable and set EXTI EL Interrupt to the lowest priority */
  NVIC_InitStructure.NVIC_IRQChannel = AZ_EXTI_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);  
}

/**
  * @brief  Functions to register & unregister handle exti
  * @note   ...
  * @param  p_Function: Pointer to function
  * @retval none
  */
void v_EL_Home_Rising_Register(PULSE_HANDLE_T pv_Function)
{
  pv_EL_Home_Rising_Handler = pv_Function;
}
void v_EL_Home_Rising_Unregister(void)
{
  pv_EL_Home_Rising_Handler = 0;
}
void v_EL_Home_Falling_Register(PULSE_HANDLE_T pv_Function)
{
  pv_EL_Home_Falling_Handler = pv_Function;
}
void v_EL_Home_Falling_Unregister(void)
{
  pv_EL_Home_Falling_Handler = 0;
}
void v_AZ_Home_Rising_Register(PULSE_HANDLE_T pv_Function)
{
  pv_AZ_Home_Rising_Handler = pv_Function;
}
void v_AZ_Home_Rising_Unregister(void)
{
  pv_AZ_Home_Rising_Handler = 0;
}
void v_AZ_Home_Falling_Register(PULSE_HANDLE_T pv_Function)
{
  pv_AZ_Home_Falling_Handler = pv_Function;
}
void v_AZ_Home_Falling_Unregister(void)
{
  pv_AZ_Home_Falling_Handler = 0;
}

/**
  * @brief  Interrupt EL & AZ handler
  * @note   ...
  * @param  none
  * @retval none
  */
void EL_EXTI_IRQn_Handler(void)
{
  if(EXTI_GetITStatus(EL_EXTI_LINE) != RESET)
  {
    EXTI_ClearITPendingBit(EL_EXTI_LINE);
    if (GPIO_ReadInputDataBit(EL_HOME_PULSE_PORT, EL_HOME_PULSE_PIN))
    {
      if (pv_EL_Home_Rising_Handler != 0)
        pv_EL_Home_Rising_Handler();
    }
    else
    {
      if (pv_EL_Home_Falling_Handler != 0)
        pv_EL_Home_Falling_Handler();
    }
  }
}

void AZ_EXTI_IRQn_Handler(void)
{
  if (EXTI_GetITStatus(AZ_EXTI_LINE) != RESET)
  {
    EXTI_ClearITPendingBit(AZ_EXTI_LINE);
    if (GPIO_ReadInputDataBit(AZ_HOME_PULSE_PORT, AZ_HOME_PULSE_PIN))
    {
      if (pv_AZ_Home_Rising_Handler != 0)
        pv_AZ_Home_Rising_Handler();
    }
    else
    {
      if (pv_AZ_Home_Falling_Handler != 0)
        pv_AZ_Home_Falling_Handler();
    }
  }
}

/**
  * @brief  Init EL Limit pulse
  * @note   ...
  * @param  p_Function: Pointer to function
  * @retval none
  */
static void v_EL_Limit_Init(void)
{
  EXTI_InitTypeDef   EXTI_InitStructure;
  GPIO_InitTypeDef   GPIO_InitStructure;
  NVIC_InitTypeDef   NVIC_InitStructure;
  /* Config EL Limit Pulse */
  /* Enable GPIO clock */
  RCC_AHB1PeriphClockCmd(EL_LIMIT_PULSE_PORT_CLK, ENABLE);
  /* Enable SYSCFG clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
  /* Configure pin as input floating */
  GPIO_InitStructure.GPIO_Pin   = EL_LIMIT_PULSE_PIN;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
  GPIO_Init(EL_LIMIT_PULSE_PORT, &GPIO_InitStructure);
  /* Connect EXTI Line2 to EL Limit pin */
  SYSCFG_EXTILineConfig(EL_LIMIT_EXTI_PORT_SOURCE, EL_LIMIT_PULSE_PIN_SOURCE);
  /* Configure EXTI EL Limit - rising & falling */
  EXTI_InitStructure.EXTI_Line    = EL_LIMIT_EXTI_LINE;
  EXTI_InitStructure.EXTI_Mode    = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);
  /* Enable and set EXTI EL Limit Interrupt to the lowest priority */
  NVIC_InitStructure.NVIC_IRQChannel = EL_LIMIT_EXTI_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

/**
  * @brief  Functions to register & unregister handle exti
  * @note   ...
  * @param  with register: PULSE_HANDLE_T f (handler function)
  * @retval none
  */
void v_EL_Limit_Rising_Register(PULSE_HANDLE_T pv_Function)
{
  pv_EL_Limit_Rising_Handler = pv_Function;
}
void v_EL_Limit_Rising_Unregister(void)
{
  pv_EL_Limit_Rising_Handler = 0;
}
void v_EL_Limit_Falling_Register(PULSE_HANDLE_T pv_Function)
{
  pv_EL_Limit_Falling_Handler = pv_Function;
}
void v_EL_Limit_Falling_Unregister(void)
{
  pv_EL_Limit_Falling_Handler = 0;
}

/**
  * @brief  Interrupt EL Limit handler
  * @note   ...
  * @param  none
  * @retval none
  */
void EL_LIMIT_EXTI_IRQn_Handler(void)
{
  if (EXTI_GetITStatus(EL_LIMIT_EXTI_LINE) != RESET)
  {
    EXTI_ClearITPendingBit(EL_LIMIT_EXTI_LINE);
    if (GPIO_ReadInputDataBit(EL_LIMIT_PULSE_PORT, EL_LIMIT_PULSE_PIN))
    {
      if (pv_EL_Limit_Rising_Handler != 0)
        pv_EL_Limit_Rising_Handler();
    }
    else
    {
      if (pv_EL_Limit_Falling_Handler != 0)
        pv_EL_Limit_Falling_Handler();
    }
  }
}

/**
  * @}
  */

/** @defgroup DO DRIVER
 *  @brief    ...
 *
 @verbatim
 ===============================================================================
                               ##### DO DRIVER #####
 ===============================================================================  

 @endverbatim
  * @{
  */

/**
  * @brief  Init for 2 Digital Outputs
  * @note   ...
  * @param  none
  * @retval none
  */
static void v_DO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  /* GPIO Peripheral clock enable */
  RCC_AHB1PeriphClockCmd(DO_PERIPH_GPIO, ENABLE);
  
  /* Configure DO0, DO1 in output pushpull mode */
  GPIO_InitStructure.GPIO_Pin   = DO0_PIN | DO1_PIN;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
  GPIO_Init(DO_GPIO, &GPIO_InitStructure);
}

/**
  * @brief  DO Set, Reset, Toggle
  * @note   ...
  * @param  DOx_Pin: specifies the DO pin to write
  * @retval none
  */
void v_DO_Set(uint16_t DOx_Pin)
{
  GPIO_SetBits(DO_GPIO, DOx_Pin);
}

void v_DO_Reset(uint16_t DOx_Pin)
{
  GPIO_ResetBits(DO_GPIO, DOx_Pin);
}

void v_DO_Toggle(uint16_t DOx_Pin)
{
  GPIO_ToggleBits(DO_GPIO, DOx_Pin);
}

/**
  * @}
  */
/*********************************END OF FILE**********************************/
