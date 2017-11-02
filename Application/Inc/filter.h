/**
  ******************************************************************************
  * @file    filter.h
  * @author  Vu Trung Hieu
  * @version V1.0
  * @date    25-October-2017
  * @brief   This file contains all the functions prototypes for filter.c
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __FILTER_H
#define __FILTER_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
/* Exported types ------------------------------------------------------------*/
typedef struct
{
  uint8_t enable; //easy to save
  uint8_t n;      //order
  float a[10];    //denominator
  float b[10];    //numerator
  float w[10];    //mediate
  float y;        //output
} STRU_IIR_FILTER_T;

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
/* IIR functions **************************************************************/
void v_IIR_Filter_Init(STRU_IIR_FILTER_T *pstru_iir_filter, uint8_t u8_order, 
  float aflt_denominator[10], float aflt_numerator[10]);

float flt_IIR_Filter_Calc(STRU_IIR_FILTER_T *pstru_iir_filter, float x);

void v_IIR_Filter_Reset(STRU_IIR_FILTER_T *pstru_iir_filter);

void v_IIR_Filter_Set_Enable(STRU_IIR_FILTER_T *pstru_iir_filter, uint8_t u8_enable);
uint8_t v_IIR_Filter_Get_Enable(STRU_IIR_FILTER_T *pstru_iir_filter);

#ifdef __cplusplus
}
#endif

#endif /* __FILTER_H */

/*********************************END OF FILE**********************************/
