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
  AXIS_INVALID_L = 0,
  AXIS_AZ,
  AXIS_EL,
  AXIS_BOTH,
  AXIS_INVALID_H,
} ENUM_AXIS_T;

typedef enum
{
  PID_ID_INVALID_L      = 0,
  PID_ID_MANUAL         = 1,
  PID_ID_POINTING       = 2,
  PID_ID_TRACKING       = 3,
  PID_ID_VELOCITY       = 4,
  PID_ID_CURRENT        = 5,
  PID_ID_INVALID_H      = 6,
} ENUM_PID_ID_T;

typedef enum {
  STATE_KEEP = -1,
  STATE_HOME,
  STATE_STOP,
  STATE_MANUAL,
  STATE_POINTING,
  STATE_TRACKING,
  STATE_SINE,
} ENUM_AXIS_STATE_T;

typedef enum {
  ROLL = 0,
  PITCH = 1,
  YAW = 2,
} ENUM_ROTATION_T;

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
/* Initialization and Configuration functions *********************************/
void v_Control_Init(void);

/* Control Functions **********************************************************/
void v_Control(void);
void v_Send_Data(void);
void v_Params_Load_All(void);

#ifdef __cplusplus
}
#endif

#endif /* __MODULE_TEMPLATE_H */

/*********************************END OF FILE**********************************/
