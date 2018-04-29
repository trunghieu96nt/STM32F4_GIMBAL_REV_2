/**
  ******************************************************************************
  * @file    utils.h
  * @author  Vu Trung Hieu
  * @version V2.0
  * @date    21-Octobor-2017
  * @brief   This file contains all the functions prototypes for utils.c
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __UTILS_H
#define __UTILS_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/** @defgroup Const in Math
  * @{
  */
#ifndef PI
#define PI (3.141592653589793)
#endif
#define RAD_TO_DEGREE       (180 / PI)
#define DEGREE_TO_RAD       (PI / 180)
#define DEGREE_TO_MRAD      (PI / 0.18)
#define DEGREE_TO_MDEGREE   (1000)
/**
  * @}
  */

/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
void v_Int_To_Str_N(int32_t s32_number, uint8_t *pu8_str, uint32_t u32_n);
void v_Euler_To_Body_Rate(float flt_euler_angle[3], float flt_euler_rate[3], float flt_body_rate[3]);

#ifdef __cplusplus
}
#endif

#endif /* __UTILS_H */

/*********************************END OF FILE**********************************/
