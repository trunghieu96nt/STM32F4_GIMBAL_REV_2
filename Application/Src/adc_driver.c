/**
  ******************************************************************************
  * @file    adc_driver.c
  * @author  Vu Trung Hieu
  * @version V1.0
  * @date    18-January-2018
  * @brief   This file contains functions for analog input
  *
 @verbatim
 ===============================================================================
                      ##### How to use this driver #####
 ===============================================================================
 @endverbatim
  *
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */
  
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "adc_driver.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static volatile uint16_t au16_result_adc[ADC_NUM_CHANNEL];

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/** @defgroup ADC Function
 *  @brief   ...
 *
 @verbatim
 ===============================================================================
                        ##### ADC Function #####
 ===============================================================================  

 @endverbatim
  * @{
  */

/**
  * @brief  ADC1 Channel 8,9 Init 
  * @note   ...
  * @param  none
  * @retval none
  */
void v_ADC_Init(void)
{
  GPIO_InitTypeDef        GPIO_InitStructure;
  ADC_CommonInitTypeDef   ADC_CommonInitStructure;
  ADC_InitTypeDef         ADC_InitStructure;
  DMA_InitTypeDef         DMA_InitStructure;
  
  /* Enable ADCx, DMA and GPIO clocks */ 
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
  RCC_AHB1PeriphClockCmd(ADC_PERIPH_GPIO, ENABLE);  
  RCC_APB2PeriphClockCmd(ADC_PERIPH, ENABLE);
  
  /* Configure ADC1 Channel8,9 pin as analog input */
  GPIO_InitStructure.GPIO_Pin   = ADC_PIN_0 | ADC_PIN_1;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL ;
  GPIO_Init(ADC_GPIO, &GPIO_InitStructure);
  
  /* DMA2 Stream0 channel0 configuration **************************************/
  DMA_InitStructure.DMA_Channel             = ADC_DMA_CHANNEL;  
  DMA_InitStructure.DMA_PeripheralBaseAddr  = ADC_DR_ADDRESS;//ADC_DR_ADDRESS;
  DMA_InitStructure.DMA_Memory0BaseAddr     = (uint32_t)&au16_result_adc;
  DMA_InitStructure.DMA_DIR                 = DMA_DIR_PeripheralToMemory;
  DMA_InitStructure.DMA_BufferSize          = ADC_NUM_CHANNEL;
  DMA_InitStructure.DMA_PeripheralInc       = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc           = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize  = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStructure.DMA_MemoryDataSize      = DMA_MemoryDataSize_HalfWord;
  DMA_InitStructure.DMA_Mode                = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority            = DMA_Priority_High;
  DMA_InitStructure.DMA_FIFOMode            = DMA_FIFOMode_Disable;
  DMA_InitStructure.DMA_FIFOThreshold       = DMA_FIFOThreshold_HalfFull;
  DMA_InitStructure.DMA_MemoryBurst         = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_PeripheralBurst     = DMA_PeripheralBurst_Single;
  DMA_Init(ADC_DMA_STREAM, &DMA_InitStructure);
  DMA_Cmd(ADC_DMA_STREAM, ENABLE);
  
  /* ADC Common Init */
  ADC_CommonInitStructure.ADC_Mode              = ADC_Mode_Independent;
  ADC_CommonInitStructure.ADC_Prescaler         = ADC_Prescaler_Div2;
  ADC_CommonInitStructure.ADC_DMAAccessMode     = ADC_DMAAccessMode_Disabled;
  ADC_CommonInitStructure.ADC_TwoSamplingDelay  = ADC_TwoSamplingDelay_5Cycles;
  ADC_CommonInit(&ADC_CommonInitStructure);
  
  /* ADC Init */
  ADC_InitStructure.ADC_Resolution            = ADC_Resolution_12b;
  ADC_InitStructure.ADC_ScanConvMode          = ENABLE;
  ADC_InitStructure.ADC_ContinuousConvMode    = ENABLE;
  ADC_InitStructure.ADC_ExternalTrigConvEdge  = ADC_ExternalTrigConvEdge_None;
  ADC_InitStructure.ADC_ExternalTrigConv      = ADC_ExternalTrigConv_T1_CC1;
  ADC_InitStructure.ADC_DataAlign             = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfConversion       = ADC_NUM_CHANNEL;
  ADC_Init(ADC_MODULE, &ADC_InitStructure);
  
  /* ADC regular channel8,9 configuration */
  ADC_RegularChannelConfig(ADC_MODULE, ADC_Channel_8, 1, ADC_SampleTime_480Cycles);
  ADC_RegularChannelConfig(ADC_MODULE, ADC_Channel_9, 2, ADC_SampleTime_480Cycles);
  
  //Enable ADC
  ADC_Cmd(ADC_MODULE, ENABLE);
  
  //Enable DMA for ADC
  ADC_DMACmd(ADC_MODULE, ENABLE);
  
  /* Enable DMA request after last transfer (Single-ADC mode) */
  ADC_DMARequestAfterLastTransferCmd(ADC_MODULE, ENABLE);

  /* Start ADC Software Conversion */ 
  ADC_SoftwareStartConv(ADC_MODULE);
}

/**
  * @brief  Get ADC Raw Value
  * @note   ...
  * @param  enum_adc_id: ID of ADC Channel
  * @retval ADC Raw Value
  */
uint16_t u16_ADC_Get_Raw_Value(ENUM_ADC_ID_T enum_adc_id)
{
  if ((enum_adc_id >= ADC_ID_INVALID_H) || (enum_adc_id <= ADC_ID_INVALID_L))
    return 0xFFFF;
  else 
    return au16_result_adc[enum_adc_id];
}

/**
  * @}
  */

/*********************************END OF FILE**********************************/
