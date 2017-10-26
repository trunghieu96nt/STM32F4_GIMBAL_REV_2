/**
  ******************************************************************************
  * @file    pid.h
  * @author  Vu Trung Hieu
  * @version V2.0
  * @date    16-September-2017
  * @brief   This file contains all the functions prototypes for pid.c
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __PID_H
#define __PID_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"

/* Exported types ------------------------------------------------------------*/
typedef struct
{
  /* Save to eeprom -> size: sizeof(float) * Num_Of_Float_Variable */
  float Kp;
  float Ki;
  float Kd;
  float Kff1;
  float Kff2;
  
  /* Considerate of saving to eeprom */
  uint8_t Use_Set_Point_Ramp; //easy to save
  float Max_Set_Point_Step;
  float Deadband;
  float dPart_Alpha; //for filtering
  float Ts;
  float Max_Response;
  
  /* No need to save to eeprom */
  float Set_Point;
  float Set_Point_Buff;
  float e;
  float e_;
  float e__;
  float pPart;
  float iPart;
  float dPart;
  float dPart_Raw;
  float Result;
} STRU_PID_T;
/* Exported constants --------------------------------------------------------*/
/** @defgroup PID
  * @{
  */
#define PID_METHOD_2
#define PID_DEFAULT_SYSTEM_TS             0.001f
#define PID_DEFAULT_D_PART_ALPHA          0.1f
#define PID_DEFAULT_MAX_RESPONSE          900
/**
  * @}
  */

/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
/* PID Functions **************************************************************/
void v_PID_Init(STRU_PID_T *pstru_PID);

float flt_PID_Calc(STRU_PID_T *pstru_PID, float flt_Feedback);
void v_PID_Reset(STRU_PID_T *pstru_PID);

void v_PID_Set_Use_Set_Point_Ramp(STRU_PID_T *pstru_PID, uint8_t u8_Use_Set_Point_Ramp);
void v_PID_Set_Set_Point(STRU_PID_T *pstru_PID, float flt_Set_Point, uint8_t u8_Use_Ramp);
void v_PID_Set_Max_Set_Point_Step(STRU_PID_T *pstru_PID, float flt_Max_Set_Point_Step);
void v_PID_Set_Deadband(STRU_PID_T *pstru_PID, float flt_Deadband);
void v_PID_Set_Kp(STRU_PID_T *pstru_PID, float flt_Kp);
void v_PID_Set_Ki(STRU_PID_T *pstru_PID, float flt_Ki);
void v_PID_Set_Kd(STRU_PID_T *pstru_PID, float flt_Kd);
void v_PID_Set_dPart_Alpha(STRU_PID_T *pstru_PID, float flt_dPart_Alpha);
void v_PID_Set_Ts(STRU_PID_T *pstru_PID, float flt_Ts);
void v_PID_Set_Max_Response(STRU_PID_T *pstru_PID, float flt_Max_Response);

uint8_t u8_PID_Get_Use_Set_Point_Ramp(STRU_PID_T *pstru_PID);
float flt_PID_Get_Set_Point(STRU_PID_T *pstru_PID);
float flt_PID_Get_Max_Set_Point_Step(STRU_PID_T *pstru_PID);
float flt_PID_Get_Deadband(STRU_PID_T *pstru_PID);
float flt_PID_Get_Kp(STRU_PID_T *pstru_PID);
float flt_PID_Get_Ki(STRU_PID_T *pstru_PID);
float flt_PID_Get_Kd(STRU_PID_T *pstru_PID);
void v_PID_Set_Kff1(STRU_PID_T *pstru_PID, float flt_Kff1);
void v_PID_Set_Kff2(STRU_PID_T *pstru_PID, float flt_Kff2);
float flt_PID_Get_Kff1(STRU_PID_T *pstru_PID);
float flt_PID_Get_Kff2(STRU_PID_T *pstru_PID);
float flt_PID_Get_dPart_Alpha(STRU_PID_T *pstru_PID);
float flt_PID_Get_Ts(STRU_PID_T *pstru_PID);
float flt_PID_Get_Max_Response(STRU_PID_T *pstru_PID);
float flt_PID_Get_Result(STRU_PID_T *pstru_PID);

#ifdef __cplusplus
}
#endif

#endif /* __PID_H */

/*********************************END OF FILE**********************************/
