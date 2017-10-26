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
  * @param  u8_Order: Order of filter
  * @param  flt_Denominator[10]: Array of denominator (size must be greater than or equal to order)
  * @param  flt_Numerator[10]: Array of numerator (size must be greater than or equal to order)
  * @retval none
  */
void v_IIR_Filter_Init(STRU_IIR_FILTER_T *pstru_IIR_Filter, uint8_t u8_Order, float flt_Denominator[10], float flt_Numerator[10])
{
  uint32_t u32_Idx;
  
  pstru_IIR_Filter->enable = 0; //By default, filter is enable when initialize
  pstru_IIR_Filter->n = u8_Order;
  
  for (u32_Idx = 0; u32_Idx <= u8_Order; u32_Idx++)
  {
    pstru_IIR_Filter->a[u32_Idx] = flt_Denominator[u32_Idx];
    pstru_IIR_Filter->b[u32_Idx] = flt_Numerator[u32_Idx];
  }
}

/**
  * @brief  Calculate IIR
  * @note   ...
  * @param  pstru_IIR_Filter: Pointer to struct IIR
  * @param  x: input
  * @retval pstru_IIR_Filter->y (result)
  */
float flt_IIR_Filter_Calc(STRU_IIR_FILTER_T *pstru_IIR_Filter, float x)
{
  uint32_t u32_Idx;
  
  if (pstru_IIR_Filter->enable != 0) //enable
  {
    /* w[0] = x[0] - a[1]*w[-1] - a[2]*w[-2] - ... */
    pstru_IIR_Filter->w[0] = x;
    for (u32_Idx = 1; u32_Idx <= pstru_IIR_Filter->n; u32_Idx++)
    {
      pstru_IIR_Filter->w[0] -= pstru_IIR_Filter->a[u32_Idx] * pstru_IIR_Filter->w[u32_Idx];
    }
    
    /* y[0] = b[0]*w[0] + b[1]*w[-1] + ... */
    pstru_IIR_Filter->y = 0;
    for (u32_Idx = 0; u32_Idx <= pstru_IIR_Filter->n; u32_Idx++)
    {
      pstru_IIR_Filter->y += pstru_IIR_Filter->b[u32_Idx] * pstru_IIR_Filter->w[u32_Idx];
    }
    
    /* update w */
    for (u32_Idx = pstru_IIR_Filter->n; u32_Idx > 0; u32_Idx--)
    {
      pstru_IIR_Filter->w[u32_Idx] = pstru_IIR_Filter->w[u32_Idx - 1];
    }
  }
  else //disable
  {
    pstru_IIR_Filter->y = x;
  }
  return pstru_IIR_Filter->y;
}

/**
  * @brief  Reset IIR Filter
  * @note   ...
  * @param  pstru_IIR_Filter: Pointer to struct IIR Filter
  * @retval none
  */
void v_IIR_Filter_Reset(STRU_IIR_FILTER_T *pstru_IIR_Filter)
{
  uint32_t u32_Idx;
  
  for (u32_Idx = 0; u32_Idx <= pstru_IIR_Filter->n; u32_Idx++)
  {
    pstru_IIR_Filter->w[u32_Idx] = 0;
  }
}

/**
  * @brief  Set Enable/Disable (Get)
  * @note   Disable when init function (By default)
  * @param  pstru_PID: Pointer to struct IIR Filter
  * @param  u8_Enable: Desired Enable Value (1: Enable, 0: Disable)
  * @retval none (pstru_IIR_Filter->enable)
  */
void v_IIR_Filter_Set_Enable(STRU_IIR_FILTER_T *pstru_IIR_Filter, uint8_t u8_Enable)
{
  pstru_IIR_Filter->enable = u8_Enable;
}

uint8_t v_IIR_Filter_Get_Enable(STRU_IIR_FILTER_T *pstru_IIR_Filter)
{
  return pstru_IIR_Filter->enable;
}

/**
  * @}
  */

/*********************************END OF FILE**********************************/
