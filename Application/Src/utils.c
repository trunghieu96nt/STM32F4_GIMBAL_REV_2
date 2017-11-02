/**
  ******************************************************************************
  * @file    utils.c
  * @author  Vu Trung Hieu
  * @version V2.0
  * @date    21-October-2017
  * @brief   This file contains functions for general purpose.
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
#include "utils.h"
#include "stdio.h"
#include "math.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/** @defgroup Utilities
 *  @brief   ...
 *
 @verbatim
 ===============================================================================
                              ##### Utilities #####
 ===============================================================================  

 @endverbatim
  * @{
  */

/**
  * @brief  IntToStrN
  * @note   convert int to str with n char, the first char is sign
            If n < lenOfStr(number), final len will be lenOfStr(number) + 1
            Example IntToStrN(10, str, 5)   -> str: 0010
                    IntToStrN(-10, str, 5)  -> str:-0010
                    IntToStrN(-0, str, 6)   -> str: 00000
                    IntToStrN(3000, str, 2) -> str: 3000
                    IntToStrN(-200, str, 3) -> str:-200
  * @param  s32_number: input number
  * @param  *pu8_str: pointer to stored string
  * @param  u32_n: Length of output string
  * @retval none
  */
void v_Int_To_Str_N(int32_t s32_number, uint8_t *pu8_str, uint32_t u32_n)
{
  uint8_t u8_mask[10];
  if(s32_number < 0)
  {
    pu8_str[0] = '-';
    s32_number = -s32_number;
  }
  else
  {
    pu8_str[0] = ' ';
  }
  sprintf((char *)u8_mask, "%%0%dd", u32_n - 1);
  sprintf((char *)(pu8_str + 1), (char *)u8_mask, s32_number);
}

/**
  * @}
  */

/** @brief Convert euler rate to body rate
 *  @param[in]  euler_angle   Array of roll, pitch, yaw angles
 *  @param[in]  euler_rate    Array of roll, pitch, yaw angle rate
 *  @param[out] body_rate     Array of body rates
 *  @return void
 */
void v_Euler_To_Body_Rate(float flt_euler_angle[3], float flt_euler_rate[3], float flt_body_rate[3])
{
  flt_body_rate[0] = flt_euler_rate[0] - sin(flt_euler_angle[1]) * flt_euler_rate[2];
  flt_body_rate[1] = cos(flt_euler_angle[0]) * flt_euler_rate[1] + sin(flt_euler_angle[0]) * cos(flt_euler_angle[1]) * flt_euler_rate[2];
  flt_body_rate[2] = -sin(flt_euler_angle[0]) * flt_euler_rate[1] + cos(flt_euler_angle[0]) * cos(flt_euler_angle[1]) * flt_euler_rate[2];
}

/*********************************END OF FILE**********************************/
