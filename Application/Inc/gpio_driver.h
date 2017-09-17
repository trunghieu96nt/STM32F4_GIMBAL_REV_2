/**
  ******************************************************************************
  * @file    gpio_driver.h
  * @author  Vu Trung Hieu
  * @version V2.0
  * @date    30-August-2017
  * @brief   This file contains all the functions prototypes for gpio_driver.c
  ******************************************************************************
  * @attention
  * (#) Use STM32F4xx_StdPeriph_Driver
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GPIO_DRIVER_H
#define __GPIO_DRIVER_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"

/* Exported types ------------------------------------------------------------*/
typedef void (*PULSE_HANDLE_T)(void);
typedef enum
{
  DI_PIN_0          = 0,
  DI_PIN_1          = 1,
  DI_PIN_2          = 2,
  DI_PIN_3          = 3,
  DI_PIN_EL_LIMIT   = 1,
  DI_PIN_AZ_HOME    = 2,
  DI_PIN_EL_HOME    = 3
} ENUM_DI_PIN_T;
/* Exported constants --------------------------------------------------------*/

/** @defgroup LED Peripheral
  * @{
  */
#define LED_PERIPH_GPIO                   RCC_AHB1Periph_GPIOC
#define LED_GPIO                          GPIOC
#define LED0_PIN                          GPIO_Pin_6
#define LED1_PIN                          GPIO_Pin_7
#define LED2_PIN                          GPIO_Pin_8
/**
  * @}
  */

/** @defgroup Button Peripheral
  * @{
  */
#define BTN_PERIPH_GPIO                   RCC_AHB1Periph_GPIOC
#define BTN_GPIO                          GPIOC
#define BTN0_PIN                          GPIO_Pin_10
#define BTN1_PIN                          GPIO_Pin_11
/**
  * @}
  */

/** @defgroup DI Peripheral
  * @{
  */
/* DI Pin */
#define DI_PERIPH_GPIO                    RCC_AHB1Periph_GPIOC
#define DI_GPIO                           GPIOC
#define DI0_PIN                           GPIO_Pin_0
#define DI1_PIN                           GPIO_Pin_1
#define DI2_PIN                           GPIO_Pin_2
#define DI3_PIN                           GPIO_Pin_3

/* Home Pulse */
#define EL_HOME_PULSE_PORT                GPIOC
#define EL_HOME_PULSE_PORT_CLK            RCC_AHB1Periph_GPIOC
#define EL_HOME_PULSE_PIN                 GPIO_Pin_3
#define EL_HOME_PULSE_PIN_SOURCE          GPIO_PinSource3
#define EL_EXTI_LINE                      EXTI_Line3
#define EL_EXTI_PORT_SOURCE               EXTI_PortSourceGPIOC
#define EL_EXTI_IRQn                      EXTI3_IRQn
#define EL_EXTI_IRQn_Handler              EXTI3_IRQHandler

#define AZ_HOME_PULSE_PORT                GPIOC
#define AZ_HOME_PULSE_PORT_CLK            RCC_AHB1Periph_GPIOC
#define AZ_HOME_PULSE_PIN                 GPIO_Pin_2
#define AZ_HOME_PULSE_PIN_SOURCE          GPIO_PinSource2
#define AZ_EXTI_LINE                      EXTI_Line2
#define AZ_EXTI_PORT_SOURCE               EXTI_PortSourceGPIOC
#define AZ_EXTI_IRQn                      EXTI2_IRQn
#define AZ_EXTI_IRQn_Handler              EXTI2_IRQHandler

/* EL  Limit */
#define EL_LIMIT_PULSE_PORT               GPIOC
#define EL_LIMIT_PULSE_PORT_CLK           RCC_AHB1Periph_GPIOC
#define EL_LIMIT_PULSE_PIN                GPIO_Pin_1
#define EL_LIMIT_PULSE_PIN_SOURCE         GPIO_PinSource1
#define EL_LIMIT_EXTI_LINE                EXTI_Line1
#define EL_LIMIT_EXTI_PORT_SOURCE         EXTI_PortSourceGPIOC
#define EL_LIMIT_EXTI_IRQn                EXTI1_IRQn
#define EL_LIMIT_EXTI_IRQn_Handler        EXTI1_IRQHandler

/**
  * @}
  */

/** @defgroup DO Peripheral
  * @{
  */
#define DO_PERIPH_GPIO                    RCC_AHB1Periph_GPIOC
#define DO_GPIO                           GPIOC
#define DO0_PIN                           GPIO_Pin_4
#define DO1_PIN                           GPIO_Pin_5
/**
  * @}
  */

/* Exported macro ------------------------------------------------------------*/

/** @defgroup LED Peripheral
  * @{
  */
#define v_Red_On()                        v_Led_Set(LED0_PIN)
#define v_Red_Off()                       v_Led_Reset(LED0_PIN)
#define v_Red_Toggle()                    v_Led_Toggle(LED0_PIN)

#define v_Green_On()                      v_Led_Set(LED1_PIN)
#define v_Green_Off()                     v_Led_Reset(LED1_PIN)
#define v_Green_Toggle()                  v_Led_Toggle(LED1_PIN)

#define v_Blue_On()                       v_Led_Set(LED2_PIN)
#define v_Blue_Off()                      v_Led_Reset(LED2_PIN)
#define v_Blue_Toggle()                   v_Led_Toggle(LED2_PIN)
/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/

/* GPIO Initial ***************************************************************/
void v_GPIO_Init(void);

/* LED functions **************************************************************/
void v_Led_Set(uint16_t LEDx_Pin);
void v_Led_Reset(uint16_t LEDx_Pin);
void v_Led_Toggle(uint16_t LEDx_Pin);

/* DI functions ***************************************************************/
uint8_t u8_DI_Read_Pin(ENUM_DI_PIN_T enum_Pin);

/* Register & Unregister interrupt handler ************************************/
void v_EL_Home_Rising_Register(PULSE_HANDLE_T pv_Function);
void v_EL_Home_Rising_Unregister(void);
void v_EL_Home_Falling_Register(PULSE_HANDLE_T pv_Function);
void v_EL_Home_Falling_Unregister(void);
void v_AZ_Home_Rising_Register(PULSE_HANDLE_T pv_Function);
void v_AZ_Home_Rising_Unregister(void);
void v_AZ_Home_Falling_Register(PULSE_HANDLE_T pv_Function);
void v_AZ_Home_Falling_Unregister(void);
void v_EL_Limit_Rising_Register(PULSE_HANDLE_T pv_Function);
void v_EL_Limit_Rising_Unregister(void);
void v_EL_Limit_Falling_Register(PULSE_HANDLE_T pv_Function);
void v_EL_Limit_Falling_Unregister(void);

/* DO functions ***************************************************************/
void v_DO_Set(uint16_t DOx_Pin);
void v_DO_Reset(uint16_t DOx_Pin);
void v_DO_Toggle(uint16_t DOx_Pin);

#ifdef __cplusplus
}
#endif

#endif /* __GPIO_DRIVER_H */

/*********************************END OF FILE**********************************/
