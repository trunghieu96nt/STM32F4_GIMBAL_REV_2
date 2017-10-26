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
  * @param  s32_Number: input number
  * @param  *pu8_Str: pointer to stored string
  * @param  u32_N: Length of output string
  * @retval none
  */
void v_Int_To_Str_N(int32_t s32_Number, uint8_t *pu8_Str, uint32_t u32_N)
{
  uint8_t u8_Mask[10];
  if(s32_Number < 0)
  {
    pu8_Str[0] = '-';
    s32_Number = -s32_Number;
  }
  else
  {
    pu8_Str[0] = ' ';
  }
  sprintf((char *)u8_Mask, "%%0%dd", u32_N - 1);
  sprintf((char *)(pu8_Str + 1), (char *)u8_Mask, s32_Number);
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
void v_Euler_To_Body_Rate(float flt_Euler_Angle[3], float flt_Euler_Rate[3], float flt_Body_Rate[3])
{
  flt_Body_Rate[0] = flt_Euler_Rate[0] - sin(flt_Euler_Angle[1]) * flt_Euler_Rate[2];
  flt_Body_Rate[1] = cos(flt_Euler_Angle[0]) * flt_Euler_Rate[1] + sin(flt_Euler_Angle[0]) * cos(flt_Euler_Angle[1]) * flt_Euler_Rate[2];
  flt_Body_Rate[2] = -sin(flt_Euler_Angle[0]) * flt_Euler_Rate[1] + cos(flt_Euler_Angle[0]) * cos(flt_Euler_Angle[1]) * flt_Euler_Rate[2];
}

/*********************************END OF FILE**********************************/
