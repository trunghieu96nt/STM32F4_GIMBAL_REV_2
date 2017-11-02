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
  uint8_t use_setpoint_ramp; //easy to save
  float max_setpoint_step;
  float deadband;
  float d_part_alpha; //for filtering
  float Ts;
  float max_response;
  
  /* No need to save to eeprom */
  float setpoint;
  float setpoint_buff;
  float e;
  float e_;
  float e__;
  float p_part;
  float i_part;
  float d_part;
  float d_part_raw;
  float result;
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
void v_PID_Init(STRU_PID_T *pstru_pid);

float flt_PID_Calc(STRU_PID_T *pstru_pid, float flt_feedback);
void v_PID_Reset(STRU_PID_T *pstru_pid);

void v_PID_Set_Use_Setpoint_Ramp(STRU_PID_T *pstru_pid, uint8_t u8_use_setpoint_ramp);
void v_PID_Set_Setpoint(STRU_PID_T *pstru_pid, float flt_setpoint, uint8_t u8_use_ramp);
void v_PID_Set_Max_Setpoint_Step(STRU_PID_T *pstru_pid, float flt_max_setpoint_step);
void v_PID_Set_Deadband(STRU_PID_T *pstru_pid, float flt_deadband);
void v_PID_Set_Kp(STRU_PID_T *pstru_pid, float flt_Kp);
void v_PID_Set_Ki(STRU_PID_T *pstru_pid, float flt_Ki);
void v_PID_Set_Kd(STRU_PID_T *pstru_pid, float flt_Kd);
void v_PID_Set_Kff1(STRU_PID_T *pstru_pid, float flt_Kff1);
void v_PID_Set_Kff2(STRU_PID_T *pstru_pid, float flt_Kff2);
void v_PID_Set_d_Part_Alpha(STRU_PID_T *pstru_pid, float flt_d_part_alpha);
void v_PID_Set_Ts(STRU_PID_T *pstru_pid, float flt_Ts);
void v_PID_Set_Max_Response(STRU_PID_T *pstru_pid, float flt_max_response);

uint8_t u8_PID_Get_Use_Setpoint_Ramp(STRU_PID_T *pstru_pid);
float flt_PID_Get_Setpoint(STRU_PID_T *pstru_pid);
float flt_PID_Get_Max_Setpoint_Step(STRU_PID_T *pstru_pid);
float flt_PID_Get_Deadband(STRU_PID_T *pstru_pid);
float flt_PID_Get_Kp(STRU_PID_T *pstru_pid);
float flt_PID_Get_Ki(STRU_PID_T *pstru_pid);
float flt_PID_Get_Kd(STRU_PID_T *pstru_pid);
float flt_PID_Get_Kff1(STRU_PID_T *pstru_pid);
float flt_PID_Get_Kff2(STRU_PID_T *pstru_pid);
float flt_PID_Get_d_Part_Alpha(STRU_PID_T *pstru_pid);
float flt_PID_Get_Ts(STRU_PID_T *pstru_pid);
float flt_PID_Get_Max_Response(STRU_PID_T *pstru_pid);
float flt_PID_Get_Result(STRU_PID_T *pstru_pid);

#ifdef __cplusplus
}
#endif

#endif /* __PID_H */

/*********************************END OF FILE**********************************/
