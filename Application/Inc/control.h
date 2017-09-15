/**
  ******************************************************************************
  * @file    control.h
  * @author  Vu Trung Hieu
  * @version V2.0
  * @date    15-September-2017
  * @brief   This file contains all the functions prototypes for control.c
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CONTROL_H
#define __CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "stdbool.h"

/* Exported types ------------------------------------------------------------*/
typedef enum
{
  AXIS_INVALID = -1,
  AXIS_A,
  AXIS_E,
  AXIS_BOTH,
} ENUM_AXIS;

/* Exported constants --------------------------------------------------------*/
/** @defgroup Group_1
  * @{
  */
//#define somethings
/**
  * @}
  */

/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

/* Initialization and Configuration functions *********************************/

/* GPIO Read and Write functions **********************************************/
#ifdef __cplusplus
}
#endif

#endif /* __MODULE_TEMPLATE_H */

/*********************************END OF FILE**********************************/
