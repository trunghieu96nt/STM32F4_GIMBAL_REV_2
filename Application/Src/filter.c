/**
  ******************************************************************************
  * @file    filter.c
  * @author  Vu Trung Hieu
  * @version V1.0
  * @date    25-October-2017
  * @brief   This file contains functions for digital filter
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
#include "filter.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/** @defgroup IIR Filter
 *  @brief   Refer to https://www.mathworks.com/help/signal/ref/dfilt.df2.html
 *           Direct form II implement
 *
 @verbatim
 ===============================================================================
                              ##### IIR Filter #####
 ===============================================================================  

 @endverbatim
  * @{
  */

/**
  * @brief  Init PID
  * @note   H(z) = num/den
  *         When Init the filter is enable
  * @param  STRU_IIR_FILTER_T: Pointer to struct IIR Filter
  * @param  u8_order: Order of filter
  * @param  aflt_denominator[10]: Array of denominator (size must be greater than or equal to order)
  * @param  aflt_numerator[10]: Array of numerator (size must be greater than or equal to order)
  * @retval none
  */
void v_IIR_Filter_Init(STRU_IIR_FILTER_T *pstru_iir_filter, uint8_t u8_order, float aflt_denominator[10], float aflt_numerator[10])
{
  uint32_t u32_idx;
  
  pstru_iir_filter->enable = 0; //By default, filter is enable when initialize
  pstru_iir_filter->n = u8_order;
  
  for (u32_idx = 0; u32_idx <= u8_order; u32_idx++)
  {
    pstru_iir_filter->a[u32_idx] = aflt_denominator[u32_idx];
    pstru_iir_filter->b[u32_idx] = aflt_numerator[u32_idx];
  }
}

/**
  * @brief  Calculate IIR
  * @note   ...
  * @param  pstru_iir_filter: Pointer to struct IIR
  * @param  x: input
  * @retval pstru_iir_filter->y (result)
  */
float flt_IIR_Filter_Calc(STRU_IIR_FILTER_T *pstru_iir_filter, float x)
{
  uint32_t u32_idx;
  
  if (pstru_iir_filter->enable != 0) //enable
  {
    /* w[0] = x[0] - a[1]*w[-1] - a[2]*w[-2] - ... */
    pstru_iir_filter->w[0] = x;
    for (u32_idx = 1; u32_idx <= pstru_iir_filter->n; u32_idx++)
    {
      pstru_iir_filter->w[0] -= pstru_iir_filter->a[u32_idx] * pstru_iir_filter->w[u32_idx];
    }
    
    /* y[0] = b[0]*w[0] + b[1]*w[-1] + ... */
    pstru_iir_filter->y = 0;
    for (u32_idx = 0; u32_idx <= pstru_iir_filter->n; u32_idx++)
    {
      pstru_iir_filter->y += pstru_iir_filter->b[u32_idx] * pstru_iir_filter->w[u32_idx];
    }
    
    /* update w */
    for (u32_idx = pstru_iir_filter->n; u32_idx > 0; u32_idx--)
    {
      pstru_iir_filter->w[u32_idx] = pstru_iir_filter->w[u32_idx - 1];
    }
  }
  else //disable
  {
    pstru_iir_filter->y = x;
  }
  return pstru_iir_filter->y;
}

/**
  * @brief  Reset IIR Filter
  * @note   ...
  * @param  pstru_iir_filter: Pointer to struct IIR Filter
  * @retval none
  */
void v_IIR_Filter_Reset(STRU_IIR_FILTER_T *pstru_iir_filter)
{
  uint32_t u32_idx;
  
  for (u32_idx = 0; u32_idx <= pstru_iir_filter->n; u32_idx++)
  {
    pstru_iir_filter->w[u32_idx] = 0;
  }
}

/**
  * @brief  Set Enable/Disable (Get)
  * @note   Disable when init function (By default)
  * @param  pstru_pid: Pointer to struct IIR Filter
  * @param  u8_enable: Desired Enable Value (1: Enable, 0: Disable)
  * @retval none (pstru_iir_filter->enable)
  */
void v_IIR_Filter_Set_Enable(STRU_IIR_FILTER_T *pstru_iir_filter, uint8_t u8_enable)
{
  pstru_iir_filter->enable = u8_enable;
}

uint8_t v_IIR_Filter_Get_Enable(STRU_IIR_FILTER_T *pstru_iir_filter)
{
  return pstru_iir_filter->enable;
}

/**
  * @}
  */

/*********************************END OF FILE**********************************/
