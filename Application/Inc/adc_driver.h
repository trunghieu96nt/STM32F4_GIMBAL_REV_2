/**
  ******************************************************************************
  * @file    adc_driver.h
  * @author  Vu Trung Hieu
  * @version V1.0
  * @date    18-January-2018
  * @brief   This file contains all the functions prototypes for the adc_driver
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ADC_DRIVER_H
#define __ADC_DRIVER_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"

/* Exported types ------------------------------------------------------------*/
typedef enum
{
  ADC_ID_INVALID_L      = -1,
  ADC_ID_0              = 0,
  ADC_ID_1              = 1,
  ADC_ID_INVALID_H      = 2,
} ENUM_ADC_ID_T;

/* Exported constants --------------------------------------------------------*/
/** @defgroup ADC Peripheral
  * @{
  */
/* DI Pin */
#define ADC_PERIPH_GPIO                     RCC_AHB1Periph_GPIOB
#define ADC_GPIO                            GPIOB
#define ADC_PIN_0                           GPIO_Pin_0
#define ADC_PIN_1                           GPIO_Pin_1
#define ADC_PERIPH                          RCC_APB2Periph_ADC1
#define ADC_MODULE                          ADC1
#define ADC_NUM_CHANNEL                     2
#define ADC_DMA_CHANNEL                     DMA_Channel_0
#define ADC_DMA_STREAM                      DMA2_Stream0
#define ADC_DR_ADDRESS                      (uint32_t)ADC_MODULE + 0x4C
/**
  * @}
  */
  
/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
void v_ADC_Init(void);
uint16_t u16_ADC_Get_Raw_Value(ENUM_ADC_ID_T enum_adc_id);

#ifdef __cplusplus
}
#endif

#endif /* __MODULE_TEMPLATE_H */

/*********************************END OF FILE**********************************/
