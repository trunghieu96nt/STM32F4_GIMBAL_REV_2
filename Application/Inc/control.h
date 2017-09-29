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
  AXIS_INVALID = 0,
  AXIS_AZ,
  AXIS_EL,
  AXIS_BOTH,
} ENUM_AXIS_T;

typedef enum
{
  PID_ID_INVALID = 0,
  PID_ID_MANUAL,
  PID_ID_STABILIZING_OUTER,
  PID_ID_STABILIZING_INNER,
} ENUM_PID_ID_T;

typedef enum {
  STATE_KEEP = -1,
  STATE_STOP,
  STATE_HOME,
  STATE_MANUAL,
  STATE_POINTING,
  STATE_TRACKING,
  STATE_SINE,
} ENUM_AXIS_STATE_T;

/* Exported constants --------------------------------------------------------*/
/** @defgroup Const in Math
  * @{
  */
#ifndef PI
#define PI (3.141592653589793)
#endif
#define RAD_TO_DEGREE       (180 / PI)
#define DEGREE_TO_RAD       (PI / 180)
/**
  * @}
  */

/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
/* Initialization and Configuration functions *********************************/
void v_Control_Init(void);

/* Control Functions **********************************************************/
void v_Control(void);
void v_Send_Data(void);
void v_Params_Save_Default(void);
void v_Params_Load_All(void);

#ifdef __cplusplus
}
#endif

#endif /* __MODULE_TEMPLATE_H */

/*********************************END OF FILE**********************************/
