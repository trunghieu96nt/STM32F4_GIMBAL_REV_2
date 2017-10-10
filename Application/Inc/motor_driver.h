/**
  ******************************************************************************
  * @file    motor_driver.h
  * @author  Vu Trung Hieu
  * @version V2.0
  * @date    05-September-2017
  * @brief   This file contains all the functions prototypes for motor_driver.c
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MOTOR_DRIVER_H
#define __MOTOR_DRIVER_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/** @defgroup Encoder Information
  * @{
  */
#define NUMBER_OF_ENCODER               3
/**
  * @}
  */

/** @defgroup Encoder 0
  * @{
  */
#define ENC0_TIM_POS                    TIM1 
#define ENC0_TIM_CLK_POS                RCC_APB2Periph_TIM1
#define ENC0_PERIPH_PORT_POS            RCC_AHB1Periph_GPIOA
#define ENC0_PORT_POS                   GPIOA
#define ENC0_A_POS                      GPIO_Pin_8
#define ENC0_A_SOURCE_POS               GPIO_PinSource8
#define ENC0_B_POS                      GPIO_Pin_9
#define ENC0_B_SOURCE_POS               GPIO_PinSource9
#define ENC0_AF_POS                     GPIO_AF_TIM1
#define ENC0_ANGLE_SCALE                0.0018f //360 / (1000 * 50 * 4)
/**
  * @}
  */

/** @defgroup Encoder 1
  * @{
  */
#define ENC1_TIM_POS                    TIM3
#define ENC1_TIM_CLK_POS                RCC_APB1Periph_TIM3
#define ENC1_PERIPH_PORT_POS            RCC_AHB1Periph_GPIOB
#define ENC1_PORT_POS                   GPIOB
#define ENC1_A_POS                      GPIO_Pin_4
#define ENC1_A_SOURCE_POS               GPIO_PinSource4
#define ENC1_B_POS                      GPIO_Pin_5
#define ENC1_B_SOURCE_POS               GPIO_PinSource5
#define ENC1_AF_POS                     GPIO_AF_TIM3
#define ENC1_ANGLE_SCALE                0.0018f //360 / (1000 * 50 * 4)
/**
  * @}
  */

/** @defgroup Encoder 2
  * @{
  */
#define ENC2_TIM_POS                    TIM2
#define ENC2_TIM_CLK_POS                RCC_APB1Periph_TIM2
#define ENC2_PERIPH_PORT_1_POS          RCC_AHB1Periph_GPIOA
#define ENC2_PERIPH_PORT_2_POS          RCC_AHB1Periph_GPIOB
#define ENC2_PORT_1_POS                 GPIOA
#define ENC2_PORT_2_POS                 GPIOB
#define ENC2_A_POS                      GPIO_Pin_15
#define ENC2_A_SOURCE_POS               GPIO_PinSource15
#define ENC2_B_POS                      GPIO_Pin_3
#define ENC2_B_SOURCE_POS               GPIO_PinSource3
#define ENC2_AF_POS                     GPIO_AF_TIM2
#define ENC2_ANGLE_SCALE                0.0018f //360 / (1000 * 50 * 4)
/**
  * @}
  */

/** @defgroup PWM Information
  * @{
  */
#define PWM0_USE_EN_PIN           0
#define PWM1_USE_EN_PIN           0
/**
  * @}
  */

/** @defgroup PWM 0
  * @{
  */
#define PWM0_TIM                  TIM12 // Channel 1
#define PWM0_TIM_CLK              RCC_APB1Periph_TIM12
#define PWM0_PULSE_PORT_CLK       RCC_AHB1Periph_GPIOB
#define PWM0_PULSE_PORT           GPIOB
#define PWM0_PULSE                GPIO_Pin_14
#define PWM0_PULSE_SOURCE         GPIO_PinSource14
#define PWM0_PULSE_AF             GPIO_AF_TIM12
#define PWM0_DIR_PORT_CLK         RCC_AHB1Periph_GPIOB
#define PWM0_DIR_PORT             GPIOB
#define PWM0_DIR                  GPIO_Pin_15
   
#if PWM0_USE_EN_PIN
#define PWM0_EN_PORT_CLK          RCC_AHB1Periph_GPIOD
#define PWM0_EN_PORT              GPIOD
#define PWM0_EN                   GPIO_Pin_10
#endif

#define PWM0_PRESCALER            12 // 84M/12=7M
#define PWM0_PERIOD               700 // 7M/700=10k
#define PWM0_DUTY                 0
/**
  * @}
  */

/** @defgroup PWM 1
  * @{
  */
#define PWM1_TIM                  TIM4 // Channel 1
#define PWM1_TIM_CLK              RCC_APB1Periph_TIM4
#define PWM1_PULSE_PORT_CLK       RCC_AHB1Periph_GPIOB
#define PWM1_PULSE_PORT           GPIOB
#define PWM1_PULSE                GPIO_Pin_6
#define PWM1_PULSE_SOURCE         GPIO_PinSource6
#define PWM1_PULSE_AF             GPIO_AF_TIM4
#define PWM1_DIR_PORT_CLK         RCC_AHB1Periph_GPIOB
#define PWM1_DIR_PORT             GPIOB
#define PWM1_DIR                  GPIO_Pin_12

#if PWM1_USE_EN_PIN
#define PWM1_EN_PORT_CLK          RCC_AHB1Periph_GPIOC
#define PWM1_EN_PORT              GPIOC
#define PWM1_EN                   GPIO_Pin_7
#endif

#define PWM1_PRESCALER            12 // 84M/12=7M
#define PWM1_PERIOD               700 // 7M/700=10k
#define PWM1_DUTY                 0
/**
  * @}
  */

/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

/* Initialization and Configuration functions *********************************/
void v_Motor_Init(void);

/* Encoder Functions **********************************************************/
int32_t s32_ENC0_Get_Pos(void);
int32_t s32_ENC1_Get_Pos(void);
int32_t s32_ENC2_Get_Pos(void);
float flt_ENC0_Get_Angle(void);
float flt_ENC1_Get_Angle(void);
float flt_ENC2_Get_Angle(void);
void v_ENC0_Reset(void);
void v_ENC1_Reset(void);
void v_ENC2_Reset(void);

/* PWM Functions **************************************************************/
void v_PWM0_Set_Freq(uint32_t u32_Frequency);
void v_PWM1_Set_Freq(uint32_t u32_Frequency);
void v_PWM0_Set_Duty(int16_t s16_Duty);
void v_PWM1_Set_Duty(int16_t s16_Duty);

/* Gimbal Port Functions ******************************************************/
int32_t s32_EL_ENC_Get_Pos(void);
int32_t s32_AZ_ENC_Get_Pos(void);

float flt_EL_ENC_Get_Angle(void);
float flt_AZ_ENC_Get_Angle(void);

void v_EL_ENC_Reset(void);
void v_AZ_ENC_Reset(void);

void v_EL_PWM_Set_Freq(uint32_t u32_Frequency);
void v_AZ_PWM_Set_Freq(uint32_t u32_Frequency);

void v_EL_PWM_Set_Duty(int16_t s16_Duty);
void v_AZ_PWM_Set_Duty(int16_t s16_Duty);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_DRIVER_H */

/*********************************END OF FILE**********************************/
