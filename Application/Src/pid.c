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
  * @param  pstru_PID: Pointer to struct PID
  * @retval none
  */
void v_PID_Init(STRU_PID_T *pstru_PID)
{
  pstru_PID->Kp                 = 0.0f;
  pstru_PID->Ki                 = 0.0f;
  pstru_PID->Kd                 = 0.0f;
  pstru_PID->Kff1               = 0.0f;
  pstru_PID->Kff2               = 0.0f;
  
  pstru_PID->Use_Set_Point_Ramp = 0;
  pstru_PID->Max_Set_Point_Step = 0.0f;
  pstru_PID->Deadband           = 0.0f;
  pstru_PID->dPart_Alpha        = PID_DEFAULT_D_PART_ALPHA;
  pstru_PID->Ts                 = PID_DEFAULT_SYSTEM_TS;
  pstru_PID->Max_Response       = PID_DEFAULT_MAX_RESPONSE;
  
  pstru_PID->Set_Point          = 0.0f;
  pstru_PID->Set_Point_Buff     = 0.0f;
  pstru_PID->e                  = 0.0f;
  pstru_PID->e_                 = 0.0f;
  pstru_PID->e__                = 0.0f;
  pstru_PID->pPart              = 0.0f;
  pstru_PID->iPart              = 0.0f;
  pstru_PID->dPart              = 0.0f;
  pstru_PID->dPart_Raw          = 0.0f;
  pstru_PID->Result             = 0.0f;
}

/**
  * @brief  Calculate PID
  * @note   ...
  * @param  pstru_PID: Pointer to struct PID
  * @param  flt_Feedback: feedback value
  * @retval pstru_PID->Result
  */
float flt_PID_Calc(STRU_PID_T *pstru_PID, float flt_Feedback)
{
#ifdef PID_METHOD_1
  float ke, ke_, ke__;
  
  if (pstru_PID->Use_Set_Point_Ramp != 0) //true
  {
    if (pstru_PID->Set_Point_Buff > (pstru_PID->Set_Point + pstru_PID->Max_Set_Point_Step))
      pstru_PID->Set_Point += pstru_PID->Max_Set_Point_Step;
    else if (pstru_PID->Set_Point_Buff < (pstru_PID->Set_Point - pstru_PID->Max_Set_Point_Step))
      pstru_PID->Set_Point -= pstru_PID->Max_Set_Point_Step;
    else
      pstru_PID->Set_Point = pstru_PID->Set_Point_Buff;
  }
  
  pstru_PID->e = pstru_PID->Set_Point - flt_Feedback;
  if (fabsf(pstru_PID->e) < pstru_PID->Deadband) pstru_PID->e = 0;
  
  ke   = pstru_PID->Kp + (pstru_PID->Ki / 2) + pstru_PID->Kd;
  ke_  = -pstru_PID->Kp + (pstru_PID->Ki / 2) - (2 * pstru_PID->Kd);
  ke__ = pstru_PID->Kd;
  
  pstru_PID->Result += (ke * pstru_PID->e) + (ke_ * pstru_PID->e_) + (ke__ * pstru_PID->e__);
  
  pstru_PID->e__ = pstru_PID->e_;
  pstru_PID->e_ =pstru_PID->e;
  
  if (pstru_PID->Result > pstru_PID->Max_Response)
    pstru_PID->Result = pstru_PID->Max_Response;
  else if (pstru_PID->Result < -pstru_PID->Max_Response)
    pstru_PID->Result = -pstru_PID->Max_Response;
  
  return pstru_PID->Result;
#endif

#ifdef PID_METHOD_2
  if (pstru_PID->Use_Set_Point_Ramp != 0) //true
  {
    if (pstru_PID->Set_Point_Buff > (pstru_PID->Set_Point + pstru_PID->Max_Set_Point_Step))
      pstru_PID->Set_Point += pstru_PID->Max_Set_Point_Step;
    else if (pstru_PID->Set_Point_Buff < (pstru_PID->Set_Point - pstru_PID->Max_Set_Point_Step))
      pstru_PID->Set_Point -= pstru_PID->Max_Set_Point_Step;
    else
      pstru_PID->Set_Point = pstru_PID->Set_Point_Buff;
  }
  
  pstru_PID->e = pstru_PID->Set_Point - flt_Feedback;
  if (fabsf(pstru_PID->e) < pstru_PID->Deadband) pstru_PID->e = 0;
  
  /* pPart */
  pstru_PID->pPart = pstru_PID->Kp * pstru_PID->e;
  
  /* iPart */
  pstru_PID->iPart += pstru_PID->Ki * (pstru_PID->e + pstru_PID->e_) / 2;
  
  if (pstru_PID->iPart > pstru_PID->Max_Response)
    pstru_PID->iPart = pstru_PID->Max_Response;
  else if (pstru_PID->iPart < -pstru_PID->Max_Response) 
    pstru_PID->iPart = -pstru_PID->Max_Response;
  
  /* dPart */
  pstru_PID->dPart_Raw = pstru_PID->Kd * (pstru_PID->e - pstru_PID->e_);
  pstru_PID->dPart = pstru_PID->dPart + pstru_PID->dPart_Alpha * (pstru_PID->dPart_Raw - pstru_PID->dPart);
  
  if (pstru_PID->dPart > (pstru_PID->Max_Response / 2))
    pstru_PID->dPart = pstru_PID->Max_Response / 2;
  else if (pstru_PID->dPart < (-pstru_PID->Max_Response / 2))
    (*pstru_PID).dPart = -pstru_PID->Max_Response / 2;
  
  /* Save e_ */
  pstru_PID->e_ =pstru_PID->e;
  
  /* Result */
  pstru_PID->Result = pstru_PID->pPart + pstru_PID->iPart + pstru_PID->dPart;
  
  if (pstru_PID->Result > pstru_PID->Max_Response)
    pstru_PID->Result = pstru_PID->Max_Response;
  else if (pstru_PID->Result < -pstru_PID->Max_Response)
    pstru_PID->Result = -pstru_PID->Max_Response;
  
  return pstru_PID->Result;
#endif
}

/**
  * @brief  Reset PID
  * @note   ...
  * @param  pstru_PID: Pointer to struct PID
  * @retval none
  */
void v_PID_Reset(STRU_PID_T *pstru_PID)
{
  pstru_PID->e          = 0.0f;
  pstru_PID->e_         = 0.0f;
  pstru_PID->e__        = 0.0f;
  pstru_PID->pPart      = 0.0f;
  pstru_PID->iPart      = 0.0f;
  pstru_PID->dPart      = 0.0f;
  pstru_PID->dPart_Raw  = 0.0f;
  pstru_PID->Result     = 0.0f;
}

/**
  * @brief  Set Use_Set_Point_Ramp (Get)
  * @note   ...
  * @param  pstru_PID: Pointer to struct PID
  * @param  u8_Use_Set_Point_Ramp: desired value
  * @retval none (pstru_PID->Use_Set_Point_Ramp)
  */
void v_PID_Set_Use_Set_Point_Ramp(STRU_PID_T *pstru_PID, uint8_t u8_Use_Set_Point_Ramp)
{
  pstru_PID->Use_Set_Point_Ramp = u8_Use_Set_Point_Ramp;
}

uint8_t u8_PID_Get_Use_Set_Point_Ramp(STRU_PID_T *pstru_PID)
{
  return pstru_PID->Use_Set_Point_Ramp;
}

/**
  * @brief  Set setpoint (Get)
  * @note   ...
  * @param  pstru_PID: Pointer to struct PID
  * @param  flt_Set_Point: Desired Set Point
  * @param  u8_Use_Ramp: if true -> set through Set_Point_Buff else -> set directly Set_Point
  * @retval none (pstru_PID->Set_Point)
  */
void v_PID_Set_Set_Point(STRU_PID_T *pstru_PID, float flt_Set_Point, uint8_t u8_Use_Ramp)
{
  if (u8_Use_Ramp != 0) //true
    pstru_PID->Set_Point_Buff = flt_Set_Point;
  else
    pstru_PID->Set_Point = flt_Set_Point;
}

float flt_PID_Get_Set_Point(STRU_PID_T *pstru_PID)
{
  return pstru_PID->Set_Point;
}

/**
  * @brief  Set Max_Set_Point_Step (get)
  * @note   This is only work if (Use_Set_Point_Ramp == true)
  *         when call PID_SetPoint_Set
  * @param  pstru_PID: Pointer to struct PID
  * @param  flt_Max_Set_Point_Step: Desired Value
  * @retval none (pstru_PID->Max_Set_Point_Step)
  */
void v_PID_Set_Max_Set_Point_Step(STRU_PID_T *pstru_PID, float flt_Max_Set_Point_Step)
{
  pstru_PID->Max_Set_Point_Step = flt_Max_Set_Point_Step;
}

float flt_PID_Get_Max_Set_Point_Step(STRU_PID_T *pstru_PID)
{
  return pstru_PID->Max_Set_Point_Step;
}

/**
  * @brief  Set Deadband (Get)
  * @note   Deadband of e
  * @param  pstru_PID: Pointer to struct PID
  * @param  flt_Deadband: Desired Value
  * @retval none (pstru_PID->Deadband)
  */
void v_PID_Set_Deadband(STRU_PID_T *pstru_PID, float flt_Deadband)
{
  pstru_PID->Deadband = flt_Deadband;
}

float flt_PID_Get_Deadband(STRU_PID_T *pstru_PID)
{
  return pstru_PID->Deadband;
}

/**
  * @brief  Set Kp Ki Kd Kff1 Kff2(Get)
  * @note   Scale in Ki Kd
  * @param  pstru_PID: Pointer to struct PID
  * @param  flt_Kx (x = p, i, d): Desired Value
  * @retval none (pstru_PID->Kx)
  */
void v_PID_Set_Kp(STRU_PID_T *pstru_PID, float flt_Kp)
{
  pstru_PID->Kp = flt_Kp;
}

float flt_PID_Get_Kp(STRU_PID_T *pstru_PID)
{
  return pstru_PID->Kp;
}

void v_PID_Set_Ki(STRU_PID_T *pstru_PID, float flt_Ki)
{
  pstru_PID->Ki = flt_Ki * pstru_PID->Ts;
}

float flt_PID_Get_Ki(STRU_PID_T *pstru_PID)
{
  return pstru_PID->Ki / pstru_PID->Ts;
}

void v_PID_Set_Kd(STRU_PID_T *pstru_PID, float flt_Kd)
{
  pstru_PID->Kd = flt_Kd / pstru_PID->Ts;
}

float flt_PID_Get_Kd(STRU_PID_T *pstru_PID)
{
  return pstru_PID->Kd * pstru_PID->Ts;
}

void v_PID_Set_Kff1(STRU_PID_T *pstru_PID, float flt_Kff1)
{
  pstru_PID->Kff1 = flt_Kff1;
}

float flt_PID_Get_Kff1(STRU_PID_T *pstru_PID)
{
  return pstru_PID->Kff1;
}

void v_PID_Set_Kff2(STRU_PID_T *pstru_PID, float flt_Kff2)
{
  pstru_PID->Kff2 = flt_Kff2;
}

float flt_PID_Get_Kff2(STRU_PID_T *pstru_PID)
{
  return pstru_PID->Kff2;
}

/**
  * @brief  Set d part alpha (get)
  * @note   This is low filter for dPart
  * @param  pstru_PID: Pointer to struct PID
  * @param  flt_dPart_Alpha: Desired Value
  * @retval none (pstru_PID->dPart_Alpha)
  */
void v_PID_Set_dPart_Alpha(STRU_PID_T *pstru_PID, float flt_dPart_Alpha)
{
  pstru_PID->dPart_Alpha = flt_dPart_Alpha;
}

float flt_PID_Get_dPart_Alpha(STRU_PID_T *pstru_PID)
{
  return pstru_PID->dPart_Alpha;
}

/**
  * @brief  Set Ts (Get)
  * @note   ...
  * @param  pstru_PID: Pointer to struct PID
  * @param  float flt_Ts: Desired Value
  * @retval none (pstru_PID->Ts)
  */
void v_PID_Set_Ts(STRU_PID_T *pstru_PID, float flt_Ts)
{
  pstru_PID->Ts = flt_Ts;
}

float flt_PID_Get_Ts(STRU_PID_T *pstru_PID)
{
  return pstru_PID->Ts;
}

/**
  * @brief  Set Max Response (Get)
  * @note   Limit the pstru_PID->Result
  * @param  pstru_PID: Pointer to struct PID
  * @param  flt_Max_Response: Desired Value
  * @retval none (pstru_PID->Max_Response)
  */
void v_PID_Set_Max_Response(STRU_PID_T *pstru_PID, float flt_Max_Response)
{
  pstru_PID->Max_Response = flt_Max_Response;
}

float flt_PID_Get_Max_Response(STRU_PID_T *pstru_PID)
{
  return pstru_PID->Max_Response;
}

/**
  * @brief  Get Result
  * @note   ...
  * @param  pstru_PID: Pointer to struct PID
  * @retval pstru_PID->Result
  */
float flt_PID_Get_Result(STRU_PID_T *pstru_PID)
{
  return pstru_PID->Result;
}
/**
  * @}
  */
  
/*********************************END OF FILE**********************************/
