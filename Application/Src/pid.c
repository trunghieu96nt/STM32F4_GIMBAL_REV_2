/**
  ******************************************************************************
  * @file    pid.c
  * @author  Vu Trung Hieu
  * @version V2.0
  * @date    16-September-2017
  * @brief   This file contains functions for using pid
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
#include "pid.h"
#include "math.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/** @defgroup PID Functions
 *  @brief   ...
 *
 @verbatim
 ===============================================================================
                            ##### PID Functions #####
 ===============================================================================  

 @endverbatim
  * @{
  */
/**
  * @brief  Init PID
  * @note   Some params is set to default
  * @param  pstru_pid: Pointer to struct PID
  * @retval none
  */
void v_PID_Init(STRU_PID_T *pstru_pid)
{
  pstru_pid->Kp                   = 0.0f;
  pstru_pid->Ki                   = 0.0f;
  pstru_pid->Kd                   = 0.0f;
  pstru_pid->Kff1                 = 0.0f;
  pstru_pid->Kff2                 = 0.0f;
  
  pstru_pid->use_setpoint_ramp    = 0;
  pstru_pid->max_setpoint_step    = 0.0f;
  pstru_pid->deadband             = 0.0f;
  pstru_pid->d_part_alpha         = PID_DEFAULT_D_PART_ALPHA;
  pstru_pid->Ts                   = PID_DEFAULT_SYSTEM_TS;
  pstru_pid->max_response         = PID_DEFAULT_MAX_RESPONSE;
  
  pstru_pid->setpoint             = 0.0f;
  pstru_pid->setpoint_buff        = 0.0f;
  pstru_pid->e                    = 0.0f;
  pstru_pid->e_                   = 0.0f;
  pstru_pid->e__                  = 0.0f;
  pstru_pid->p_part               = 0.0f;
  pstru_pid->i_part               = 0.0f;
  pstru_pid->d_part               = 0.0f;
  pstru_pid->d_part_raw           = 0.0f;
  pstru_pid->result               = 0.0f;
}

/**
  * @brief  Calculate PID
  * @note   ...
  * @param  pstru_pid: Pointer to struct PID
  * @param  flt_feedback: feedback value
  * @retval pstru_pid->result
  */
float flt_PID_Calc(STRU_PID_T *pstru_pid, float flt_feedback)
{
#ifdef PID_METHOD_1
  float ke, ke_, ke__;
  
  if (pstru_pid->use_setpoint_ramp != 0) //true
  {
    if (pstru_pid->setpoint_buff > (pstru_pid->setpoint + pstru_pid->max_setpoint_step))
      pstru_pid->setpoint += pstru_pid->max_setpoint_step;
    else if (pstru_pid->setpoint_buff < (pstru_pid->setpoint - pstru_pid->max_setpoint_step))
      pstru_pid->setpoint -= pstru_pid->max_setpoint_step;
    else
      pstru_pid->setpoint = pstru_pid->setpoint_buff;
  }
  
  pstru_pid->e = pstru_pid->setpoint - flt_feedback;
  if (fabsf(pstru_pid->e) < pstru_pid->deadband) pstru_pid->e = 0;
  
  ke   = pstru_pid->Kp + (pstru_pid->Ki / 2) + pstru_pid->Kd;
  ke_  = -pstru_pid->Kp + (pstru_pid->Ki / 2) - (2 * pstru_pid->Kd);
  ke__ = pstru_pid->Kd;
  
  pstru_pid->result += (ke * pstru_pid->e) + (ke_ * pstru_pid->e_) + (ke__ * pstru_pid->e__);
  
  pstru_pid->e__ = pstru_pid->e_;
  pstru_pid->e_ =pstru_pid->e;
  
  if (pstru_pid->result > pstru_pid->max_response)
    pstru_pid->result = pstru_pid->max_response;
  else if (pstru_pid->result < -pstru_pid->max_response)
    pstru_pid->result = -pstru_pid->max_response;
  
  return pstru_pid->result;
#endif

#ifdef PID_METHOD_2
  if (pstru_pid->use_setpoint_ramp != 0) //true
  {
    if (pstru_pid->setpoint_buff > (pstru_pid->setpoint + pstru_pid->max_setpoint_step))
      pstru_pid->setpoint += pstru_pid->max_setpoint_step;
    else if (pstru_pid->setpoint_buff < (pstru_pid->setpoint - pstru_pid->max_setpoint_step))
      pstru_pid->setpoint -= pstru_pid->max_setpoint_step;
    else
      pstru_pid->setpoint = pstru_pid->setpoint_buff;
  }
  
  pstru_pid->e = pstru_pid->setpoint - flt_feedback;
  if (fabsf(pstru_pid->e) < pstru_pid->deadband) pstru_pid->e = 0;
  
  /* p_part */
  pstru_pid->p_part = pstru_pid->Kp * pstru_pid->e;
  
  /* i_part */
  pstru_pid->i_part += pstru_pid->Ki * (pstru_pid->e + pstru_pid->e_) / 2;
  
  if (pstru_pid->i_part > pstru_pid->max_response)
    pstru_pid->i_part = pstru_pid->max_response;
  else if (pstru_pid->i_part < -pstru_pid->max_response) 
    pstru_pid->i_part = -pstru_pid->max_response;
  
  /* d_part */
  pstru_pid->d_part_raw = pstru_pid->Kd * (pstru_pid->e - pstru_pid->e_);
  pstru_pid->d_part = pstru_pid->d_part + pstru_pid->d_part_alpha * (pstru_pid->d_part_raw - pstru_pid->d_part);
  
  if (pstru_pid->d_part > (pstru_pid->max_response / 2))
    pstru_pid->d_part = pstru_pid->max_response / 2;
  else if (pstru_pid->d_part < (-pstru_pid->max_response / 2))
    (*pstru_pid).d_part = -pstru_pid->max_response / 2;
  
  /* Save e_ */
  pstru_pid->e_ =pstru_pid->e;
  
  /* result */
  pstru_pid->result = pstru_pid->p_part + pstru_pid->i_part + pstru_pid->d_part;
  
  if (pstru_pid->result > pstru_pid->max_response)
    pstru_pid->result = pstru_pid->max_response;
  else if (pstru_pid->result < -pstru_pid->max_response)
    pstru_pid->result = -pstru_pid->max_response;
  
  return pstru_pid->result;
#endif
}

/**
  * @brief  Reset PID
  * @note   ...
  * @param  pstru_pid: Pointer to struct PID
  * @retval none
  */
void v_PID_Reset(STRU_PID_T *pstru_pid)
{
  pstru_pid->e            = 0.0f;
  pstru_pid->e_           = 0.0f;
  pstru_pid->e__          = 0.0f;
  pstru_pid->p_part       = 0.0f;
  pstru_pid->i_part       = 0.0f;
  pstru_pid->d_part       = 0.0f;
  pstru_pid->d_part_raw   = 0.0f;
  pstru_pid->result       = 0.0f;
}

/**
  * @brief  Set use_setpoint_ramp (Get)
  * @note   ...
  * @param  pstru_pid: Pointer to struct PID
  * @param  u8_use_setpoint_ramp: desired value
  * @retval none (pstru_pid->use_setpoint_ramp)
  */
void v_PID_Set_Use_Setpoint_Ramp(STRU_PID_T *pstru_pid, uint8_t u8_use_setpoint_ramp)
{
  pstru_pid->use_setpoint_ramp = u8_use_setpoint_ramp;
}

uint8_t u8_PID_Get_Use_Setpoint_Ramp(STRU_PID_T *pstru_pid)
{
  return pstru_pid->use_setpoint_ramp;
}

/**
  * @brief  Set setpoint (Get)
  * @note   ...
  * @param  pstru_pid: Pointer to struct PID
  * @param  flt_setpoint: Desired Set Point
  * @param  u8_use_ramp: if true -> set through setpoint_buff else -> set directly setpoint
  * @retval none (pstru_pid->setpoint)
  */
void v_PID_Set_Setpoint(STRU_PID_T *pstru_pid, float flt_setpoint, uint8_t u8_use_ramp)
{
  if (u8_use_ramp != 0) //true
    pstru_pid->setpoint_buff = flt_setpoint;
  else
    pstru_pid->setpoint = flt_setpoint;
}

float flt_PID_Get_Setpoint(STRU_PID_T *pstru_pid)
{
  return pstru_pid->setpoint;
}

/**
  * @brief  Set max_setpoint_step (get)
  * @note   This is only work if (use_setpoint_ramp == true)
  *         when call PID_SetPoint_Set
  * @param  pstru_pid: Pointer to struct PID
  * @param  flt_max_setpoint_step: Desired Value
  * @retval none (pstru_pid->max_setpoint_step)
  */
void v_PID_Set_Max_Setpoint_Step(STRU_PID_T *pstru_pid, float flt_max_setpoint_step)
{
  pstru_pid->max_setpoint_step = flt_max_setpoint_step;
}

float flt_PID_Get_Max_Setpoint_Step(STRU_PID_T *pstru_pid)
{
  return pstru_pid->max_setpoint_step;
}

/**
  * @brief  Set deadband (Get)
  * @note   deadband of e
  * @param  pstru_pid: Pointer to struct PID
  * @param  flt_deadband: Desired Value
  * @retval none (pstru_pid->deadband)
  */
void v_PID_Set_Deadband(STRU_PID_T *pstru_pid, float flt_deadband)
{
  pstru_pid->deadband = flt_deadband;
}

float flt_PID_Get_Deadband(STRU_PID_T *pstru_pid)
{
  return pstru_pid->deadband;
}

/**
  * @brief  Set Kp Ki Kd Kff1 Kff2(Get)
  * @note   Scale in Ki Kd
  * @param  pstru_pid: Pointer to struct PID
  * @param  flt_Kx (x = p, i, d): Desired Value
  * @retval none (pstru_pid->Kx)
  */
void v_PID_Set_Kp(STRU_PID_T *pstru_pid, float flt_Kp)
{
  pstru_pid->Kp = flt_Kp;
}

float flt_PID_Get_Kp(STRU_PID_T *pstru_pid)
{
  return pstru_pid->Kp;
}

void v_PID_Set_Ki(STRU_PID_T *pstru_pid, float flt_Ki)
{
  pstru_pid->Ki = flt_Ki * pstru_pid->Ts;
}

float flt_PID_Get_Ki(STRU_PID_T *pstru_pid)
{
  return pstru_pid->Ki / pstru_pid->Ts;
}

void v_PID_Set_Kd(STRU_PID_T *pstru_pid, float flt_Kd)
{
  pstru_pid->Kd = flt_Kd / pstru_pid->Ts;
}

float flt_PID_Get_Kd(STRU_PID_T *pstru_pid)
{
  return pstru_pid->Kd * pstru_pid->Ts;
}

void v_PID_Set_Kff1(STRU_PID_T *pstru_pid, float flt_Kff1)
{
  pstru_pid->Kff1 = flt_Kff1;
}

float flt_PID_Get_Kff1(STRU_PID_T *pstru_pid)
{
  return pstru_pid->Kff1;
}

void v_PID_Set_Kff2(STRU_PID_T *pstru_pid, float flt_Kff2)
{
  pstru_pid->Kff2 = flt_Kff2;
}

float flt_PID_Get_Kff2(STRU_PID_T *pstru_pid)
{
  return pstru_pid->Kff2;
}

/**
  * @brief  Set d part alpha (get)
  * @note   This is low filter for d_part
  * @param  pstru_pid: Pointer to struct PID
  * @param  flt_d_part_alpha: Desired Value
  * @retval none (pstru_pid->d_part_alpha)
  */
void v_PID_Set_d_Part_Alpha(STRU_PID_T *pstru_pid, float flt_d_part_alpha)
{
  pstru_pid->d_part_alpha = flt_d_part_alpha;
}

float flt_PID_Get_d_Part_Alpha(STRU_PID_T *pstru_pid)
{
  return pstru_pid->d_part_alpha;
}

/**
  * @brief  Set Ts (Get)
  * @note   ...
  * @param  pstru_pid: Pointer to struct PID
  * @param  float flt_Ts: Desired Value
  * @retval none (pstru_pid->Ts)
  */
void v_PID_Set_Ts(STRU_PID_T *pstru_pid, float flt_Ts)
{
  pstru_pid->Ts = flt_Ts;
}

float flt_PID_Get_Ts(STRU_PID_T *pstru_pid)
{
  return pstru_pid->Ts;
}

/**
  * @brief  Set Max Response (Get)
  * @note   Limit the pstru_pid->result
  * @param  pstru_pid: Pointer to struct PID
  * @param  flt_max_response: Desired Value
  * @retval none (pstru_pid->max_response)
  */
void v_PID_Set_Max_Response(STRU_PID_T *pstru_pid, float flt_max_response)
{
  pstru_pid->max_response = flt_max_response;
}

float flt_PID_Get_Max_Response(STRU_PID_T *pstru_pid)
{
  return pstru_pid->max_response;
}

/**
  * @brief  Get result
  * @note   ...
  * @param  pstru_pid: Pointer to struct PID
  * @retval pstru_pid->result
  */
float flt_PID_Get_Result(STRU_PID_T *pstru_pid)
{
  return pstru_pid->result;
}
/**
  * @}
  */
  
/*********************************END OF FILE**********************************/
