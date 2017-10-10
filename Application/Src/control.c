/**
  ******************************************************************************
  * @file    control.c
  * @author  Vu Trung Hieu
  * @version V2.0
  * @date    15-September-2017
  * @brief   This file contains functions for controling gimbal
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
#include "control.h"
#include "motor_driver.h"
#include "gpio_driver.h"
#include "adis_sensor.h"
#include "uart_comm.h"
#include "i2c_comm.h"
#include "pid.h"
#include "math.h"
#include "stdio.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define MAX_SHORT_RES_PAYLOAD_LEN           8
#define MAX_LONG_RES_PAYLOAD_LEN            32
#define MAX_RES_MESSAGE_LEN                 64
#define PARAMS_SCALE                        1000000.0f
#define POS_VEL_SCALE                       100.0f

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static volatile ENUM_AXIS_STATE_T enum_AZ_State = STATE_HOME; //STATE_HOME STATE_SINE
static volatile ENUM_AXIS_STATE_T enum_EL_State = STATE_HOME; //STATE_STOP

static volatile bool bool_AZ_Going_Home = false;
static volatile bool bool_EL_Going_Home = false;

static volatile uint32_t u32_AZ_Sine_Idx = 0, u32_EL_Sine_Idx = 0;

static STRU_PID_T stru_PID_AZ_Manual_Pos;
static STRU_PID_T stru_PID_EL_Manual_Pos;

static STRU_PID_T stru_PID_AZ_Stabilizing_Outer;
static STRU_PID_T stru_PID_EL_Stabilizing_Outer;
static STRU_PID_T stru_PID_AZ_Stabilizing_Inner;
static STRU_PID_T stru_PID_EL_Stabilizing_Inner;

static STRU_PID_T *apstru_PID[3][4] = {
  {0, 0, 0, 0},
  {0, &stru_PID_AZ_Manual_Pos, &stru_PID_AZ_Stabilizing_Outer, &stru_PID_AZ_Stabilizing_Inner},
  {0, &stru_PID_EL_Manual_Pos, &stru_PID_EL_Stabilizing_Outer, &stru_PID_EL_Stabilizing_Inner}
};

static const uint8_t au8_Code_Version[2] = {10, 1}; //Major.Minor

/* Private function prototypes -----------------------------------------------*/
static void v_Control_Change_Mode(ENUM_AXIS_T enum_Axis, ENUM_AXIS_STATE_T enum_New_State);
static void v_Home_AZ_Handler(void);
static void v_Home_EL_Handler(void);
static void v_Limit_El_Handler(void);
static void v_Int_To_Str_N(int32_t s32_Number, uint8_t *pu8_Str, uint32_t u32_N);
static void v_Send_Respond_Msg(uint8_t u8_Msg_ID, uint8_t *pu8_Payload, uint32_t u32_Payload_Cnt);

/* Private functions ---------------------------------------------------------*/

/** @defgroup Main Control
 *  @brief    ...
 *
 @verbatim
 ===============================================================================
                            ##### Main Control #####
 ===============================================================================  

 @endverbatim
  * @{
  */

/**
  * @brief  Control Initialization
  * @note   Init some params
  * @param  none
  * @retval none
  */
void v_Control_Init(void)
{
  /* Limit EL*/
  if(u8_DI_Read_Pin(DI_PIN_EL_LIMIT) == 1)
    v_Limit_El_Handler();
  v_EL_Limit_Falling_Register(v_Limit_El_Handler);
  v_EL_Limit_Rising_Register(v_Limit_El_Handler);
  
  /* Manual mode */
  v_PID_Init(&stru_PID_AZ_Manual_Pos);
  v_PID_Set_Kp(&stru_PID_AZ_Manual_Pos, 40);
  v_PID_Set_Ki(&stru_PID_AZ_Manual_Pos, 0.3);
  v_PID_Set_Kd(&stru_PID_AZ_Manual_Pos, 0.01);
  v_PID_Set_Use_Set_Point_Ramp(&stru_PID_AZ_Manual_Pos, 1);
  v_PID_Set_Max_Set_Point_Step(&stru_PID_AZ_Manual_Pos, 0.01);
  
  v_PID_Init(&stru_PID_EL_Manual_Pos);
  v_PID_Set_Kp(&stru_PID_EL_Manual_Pos, 41);
  v_PID_Set_Ki(&stru_PID_EL_Manual_Pos, 0.5);
  v_PID_Set_Kd(&stru_PID_EL_Manual_Pos, 0.01);
  v_PID_Set_Use_Set_Point_Ramp(&stru_PID_EL_Manual_Pos, 1);
  v_PID_Set_Max_Set_Point_Step(&stru_PID_EL_Manual_Pos, 0.01);
  v_PID_Set_Max_Response(&stru_PID_EL_Manual_Pos, 500);
  
  /* Stabilizing mode */
  v_PID_Init(&stru_PID_AZ_Stabilizing_Inner);
  v_PID_Set_Kp(&stru_PID_AZ_Stabilizing_Inner, 0.10);
  v_PID_Set_Ki(&stru_PID_AZ_Stabilizing_Inner, 20);
  v_PID_Set_Kd(&stru_PID_AZ_Stabilizing_Inner, 0.0004);
  
  v_PID_Init(&stru_PID_AZ_Stabilizing_Outer);
}

/**
  * @brief  Control Funtion
  * @note   Call it with Ts in loop main (same Ts with PID)
  * @param  none
  * @retval none
  */
void v_Control(void)
{
  float flt_az_test;
  switch(enum_AZ_State)
  {
    case STATE_STOP:
    { 
      flt_az_test = flt_AZ_ENC_Get_Angle();
      if ((flt_az_test > 45.0f) || (flt_az_test < -45.0f))
        v_AZ_PWM_Set_Duty(0);
      break;
    }
    case STATE_HOME:
      if(bool_AZ_Going_Home == false)
      {
        bool_AZ_Going_Home = true;
        v_AZ_Home_Rising_Register(v_Home_AZ_Handler);
        v_AZ_Home_Falling_Register(v_Home_AZ_Handler);
        
        if(u8_DI_Read_Pin(DI_PIN_AZ_HOME) == 0)
          v_AZ_PWM_Set_Duty(-90);
        else
          v_AZ_PWM_Set_Duty(75);
      }
      break;
    case STATE_MANUAL:
      flt_PID_Calc(&stru_PID_AZ_Manual_Pos, flt_AZ_ENC_Get_Angle());
      
      //Output PWM 1 - AZ
      v_AZ_PWM_Set_Duty(stru_PID_AZ_Manual_Pos.Result + 0.5f);
      break;
    case STATE_POINTING:
      flt_PID_Calc(&stru_PID_AZ_Stabilizing_Inner, stru_Get_IMU_Data().flt_Gyro_z);
      //stru_PID_AZ_Pointing_Inner.Result = -stru_PID_AZ_Pointing_Inner.Kp * struIMUData.gyro_z;
      flt_PID_Calc(&stru_PID_AZ_Stabilizing_Inner, stru_Get_IMU_Data().flt_Gyro_z);
      //Output PWM 1 - AZ
      v_AZ_PWM_Set_Duty(stru_PID_AZ_Stabilizing_Inner.Result / cos(flt_AZ_ENC_Get_Angle() * DEGREE_TO_RAD) + 0.5f);
      break;
    case STATE_SINE:
      v_AZ_PWM_Set_Duty(300 * sin(2 * PI * u32_AZ_Sine_Idx / 5000));
      if(++u32_AZ_Sine_Idx == 5000) u32_AZ_Sine_Idx = 0;
      break;
    default:
      break;
  }
  
  switch(enum_EL_State)
  {
    case STATE_STOP:
      break;
    case STATE_HOME:
      if(bool_EL_Going_Home == false)
      {
        bool_EL_Going_Home = true;
        v_EL_Home_Rising_Register(v_Home_EL_Handler);
        v_EL_Home_Falling_Register(v_Home_EL_Handler);
        
        if(u8_DI_Read_Pin(DI_PIN_EL_HOME) == 0)
          v_EL_PWM_Set_Duty(90);
        else
          v_EL_PWM_Set_Duty(-75);
      }
      break;
    case STATE_MANUAL:
      flt_PID_Calc(&stru_PID_EL_Manual_Pos, flt_EL_ENC_Get_Angle());
      
      //Output PWM 0 - EL
      v_EL_PWM_Set_Duty(stru_PID_EL_Manual_Pos.Result + 0.5f);
      break;
    case STATE_SINE:
      v_EL_PWM_Set_Duty(200 * sin(2 * PI * u32_EL_Sine_Idx / 5000));
      if(++u32_EL_Sine_Idx == 5000) u32_EL_Sine_Idx = 0;
      break;
    default:
      break;
  }
}

/**
  * @brief  Change State Operation
  * @note   
  * @param  enum_Axis: Desired Axis
  * @param  enum_New_State: New State of Axis
  * @retval none
  */
static void v_Control_Change_Mode(ENUM_AXIS_T enum_Axis, ENUM_AXIS_STATE_T enum_New_State)
{
  if(enum_New_State != STATE_KEEP)
  {
    if ((enum_Axis == AXIS_AZ) || (enum_Axis == AXIS_BOTH))
    {
      switch(enum_New_State)
      {
        case STATE_STOP:
          v_AZ_PWM_Set_Duty(0);
          break;
        case STATE_HOME:
          v_AZ_PWM_Set_Duty(0);
          bool_AZ_Going_Home = false;
          break;
        case STATE_MANUAL:
          v_PID_Reset(&stru_PID_AZ_Manual_Pos);
          /* Set Set_Point_Buff */
          v_PID_Set_Set_Point(&stru_PID_AZ_Manual_Pos, flt_AZ_ENC_Get_Angle(), 1);
          /* Set Set_Point */
          v_PID_Set_Set_Point(&stru_PID_AZ_Manual_Pos, flt_AZ_ENC_Get_Angle(), 0);
          break;
        case STATE_POINTING:
          v_PID_Reset(&stru_PID_AZ_Stabilizing_Outer);
          v_PID_Reset(&stru_PID_AZ_Stabilizing_Inner);
          break;
        case STATE_TRACKING:
          break;
        case STATE_SINE:
          u32_AZ_Sine_Idx = 0;
          break;
        default:
          break;
      }
      enum_AZ_State = enum_New_State;
    }
    
    if((enum_Axis == AXIS_EL) || (enum_Axis == AXIS_BOTH))
    {
      switch(enum_New_State)
      {
        case STATE_STOP:
          v_EL_PWM_Set_Duty(0);
          break;
        case STATE_HOME:
          v_EL_PWM_Set_Duty(0);
          bool_EL_Going_Home = false;
          break;
        case STATE_MANUAL:
          v_PID_Reset(&stru_PID_EL_Manual_Pos);
          /* Set Set_Point_Buff */
          v_PID_Set_Set_Point(&stru_PID_EL_Manual_Pos, flt_EL_ENC_Get_Angle(), 1);
          /* Set Set_Point */
          v_PID_Set_Set_Point(&stru_PID_EL_Manual_Pos, flt_EL_ENC_Get_Angle(), 0);
          break;
        case STATE_POINTING:
          v_PID_Reset(&stru_PID_EL_Stabilizing_Outer);
          v_PID_Reset(&stru_PID_EL_Stabilizing_Inner);
          break;
        case STATE_TRACKING:
          break;
        case STATE_SINE:
          u32_EL_Sine_Idx = 0;
          break;
        default:
          break;
      }
      enum_EL_State = enum_New_State;
    }
  }
}
/**
  * @}
  */

/** @defgroup AZ - EL Interrupt Handler
 *  @brief    ...
 *
 @verbatim
 ===============================================================================
                       ##### AZ - EL Interrupt Handler #####
 ===============================================================================  

 @endverbatim
  * @{
  */

/**
  * @brief  DI Interrupt Handler
  * @note   AZ (Home), EL (Home, Limit)
  * @param  none
  * @retval none
  */
static void v_Home_AZ_Handler(void)
{
  v_AZ_PWM_Set_Duty(0);
  v_AZ_ENC_Reset();
  v_AZ_Home_Rising_Unregister();
  v_AZ_Home_Falling_Unregister();
  v_Control_Change_Mode(AXIS_AZ, STATE_MANUAL);
  bool_AZ_Going_Home = false;
}

static void v_Home_EL_Handler(void)
{
  v_EL_PWM_Set_Duty(0);
  v_EL_ENC_Reset();
  v_EL_Home_Rising_Unregister();
  v_EL_Home_Falling_Unregister();
  v_Control_Change_Mode(AXIS_EL, STATE_STOP);
  bool_EL_Going_Home = false;
}

static void v_Limit_El_Handler(void)
{
  //v_Control_Change_Mode(STATE_STOP, STATE_STOP);
  //while(true); //Loop forever
}
/**
  * @}
  */

/** @defgroup CMD Handler
 *  @brief   ...
 *
 @verbatim
 ===============================================================================
                           ##### CMD Handler #####
 ===============================================================================  

 @endverbatim
  * @{
  */

/**
  * @brief  CMD Handler (CMD received from UART)
  * @note   comm_driver.c need these funtions
  * @param  pu8_Data: Pointer to Data
  * @param  u32_Data_Cnt: Number of Data in bytes
  * @retval true if successful and vice versa
  */
bool bool_None_Handler(uint8_t u8_Msg_ID, uint8_t *pu8_Payload, uint32_t u32_Payload_Cnt)
{
  
  return true;
}

bool bool_Home_Handler(uint8_t u8_Msg_ID, uint8_t *pu8_Payload, uint32_t u32_Payload_Cnt)
{
  uint8_t au8_Respond_Payload[MAX_SHORT_RES_PAYLOAD_LEN];
  
  v_Control_Change_Mode((ENUM_AXIS_T)(*pu8_Payload), STATE_HOME);
  
  au8_Respond_Payload[0] = *pu8_Payload;
  au8_Respond_Payload[1] = 0x00; //Ok
  v_Send_Respond_Msg(u8_Msg_ID, au8_Respond_Payload, 2);
  return true;
}

bool bool_Stop_Handler(uint8_t u8_Msg_ID, uint8_t *pu8_Payload, uint32_t u32_Payload_Cnt)
{
  uint8_t au8_Respond_Payload[MAX_SHORT_RES_PAYLOAD_LEN];
  
  v_Control_Change_Mode((ENUM_AXIS_T)(*pu8_Payload), STATE_STOP);
  
  v_AZ_PWM_Set_Duty(75);
  
  au8_Respond_Payload[0] = *pu8_Payload;
  au8_Respond_Payload[1] = 0x00; //Ok
  v_Send_Respond_Msg(u8_Msg_ID, au8_Respond_Payload, 2);
  return true;
}

bool bool_Emergency_Stop_Handler(uint8_t u8_Msg_ID, uint8_t *pu8_Payload, uint32_t u32_Payload_Cnt)
{
  uint8_t au8_Respond_Payload[MAX_SHORT_RES_PAYLOAD_LEN];
  
  v_Limit_El_Handler();
  
  au8_Respond_Payload[0] = *pu8_Payload;
  au8_Respond_Payload[1] = 0x00; //Ok
  v_Send_Respond_Msg(u8_Msg_ID, au8_Respond_Payload, 2);
  return true;
}

bool bool_Stabilizing_Mode_Handler(uint8_t u8_Msg_ID, uint8_t *pu8_Payload, uint32_t u32_Payload_Cnt)
{
  uint8_t au8_Respond_Payload[MAX_SHORT_RES_PAYLOAD_LEN];
  
  v_Control_Change_Mode((ENUM_AXIS_T)(*pu8_Payload), STATE_STOP);
  v_AZ_PWM_Set_Duty(-90);
  
  au8_Respond_Payload[0] = *pu8_Payload;
  au8_Respond_Payload[1] = 0x00; //Ok
  v_Send_Respond_Msg(u8_Msg_ID, au8_Respond_Payload, 2);
  return true;
}

bool bool_Set_Pos_Handler(uint8_t u8_Msg_ID, uint8_t *pu8_Payload, uint32_t u32_Payload_Cnt)
{
  uint8_t au8_Respond_Payload[MAX_SHORT_RES_PAYLOAD_LEN];
  
  au8_Respond_Payload[0] = *pu8_Payload;
  au8_Respond_Payload[1] = 0x00; //Ok
  v_Send_Respond_Msg(u8_Msg_ID, au8_Respond_Payload, 2);
  return true;
}

bool bool_Set_Vel_Handler(uint8_t u8_Msg_ID, uint8_t *pu8_Payload, uint32_t u32_Payload_Cnt)
{
  uint8_t au8_Respond_Payload[MAX_SHORT_RES_PAYLOAD_LEN];
  
  au8_Respond_Payload[0] = *pu8_Payload;
  au8_Respond_Payload[1] = 0x00; //Ok
  v_Send_Respond_Msg(u8_Msg_ID, au8_Respond_Payload, 2);
  return true;
}

bool bool_Set_Pos_Vel_Handler(uint8_t u8_Msg_ID, uint8_t *pu8_Payload, uint32_t u32_Payload_Cnt)
{
  uint8_t au8_Respond_Payload[MAX_SHORT_RES_PAYLOAD_LEN];
  
  au8_Respond_Payload[0] = *pu8_Payload;
  au8_Respond_Payload[1] = 0x00; //Ok
  v_Send_Respond_Msg(u8_Msg_ID, au8_Respond_Payload, 2);
  return true;
}

bool bool_Get_Pos_Handler(uint8_t u8_Msg_ID, uint8_t *pu8_Payload, uint32_t u32_Payload_Cnt)
{
  uint8_t au8_Respond_Payload[MAX_SHORT_RES_PAYLOAD_LEN];
  int32_t i32_ENC_Angle;
  
  if (*pu8_Payload == AXIS_AZ)
    i32_ENC_Angle = (int32_t)(flt_AZ_ENC_Get_Angle() * POS_VEL_SCALE);
    //i32_ENC_Angle = (int32_t)(359.999f * 100.0f);
  else if (*pu8_Payload == AXIS_EL)
    i32_ENC_Angle = (int32_t)(flt_EL_ENC_Get_Angle() * POS_VEL_SCALE);
    //i32_ENC_Angle = (int32_t)(-230.12f * 100.0f);
  else return false;
  
  au8_Respond_Payload[0] = *pu8_Payload;
  au8_Respond_Payload[1] = (i32_ENC_Angle >> 24) & 0x0ff;
  au8_Respond_Payload[2] = (i32_ENC_Angle >> 16) & 0x0ff;
  au8_Respond_Payload[3] = (i32_ENC_Angle >> 8) & 0x0ff;
  au8_Respond_Payload[4] = i32_ENC_Angle & 0x0ff;
  v_Send_Respond_Msg(u8_Msg_ID, au8_Respond_Payload, 5);
  return true;
}

bool bool_Set_Kp_Handler(uint8_t u8_Msg_ID, uint8_t *pu8_Payload, uint32_t u32_Payload_Cnt)
{
  uint8_t au8_Respond_Payload[MAX_SHORT_RES_PAYLOAD_LEN];
  STRU_PID_T *pstru_Desired_PID;
  uint32_t u32_Desired_Value;
  
  if (*pu8_Payload <= AXIS_INVALID) return false;
  if (*pu8_Payload >= AXIS_BOTH) return false;
  if (*(pu8_Payload + 1) <= PID_ID_INVALID) return false;
  if (*(pu8_Payload + 1) > PID_ID_STABILIZING_INNER) return false;
  
  pstru_Desired_PID = apstru_PID[*pu8_Payload][*(pu8_Payload + 1)];
  
  u32_Desired_Value = (*(pu8_Payload + 2) << 24) & 0x0ff000000;
  u32_Desired_Value += (*(pu8_Payload + 3) << 16) & 0x0ff0000;
  u32_Desired_Value += (*(pu8_Payload + 4) << 8) & 0x0ff00;
  u32_Desired_Value += *(pu8_Payload + 5) & 0x0ff;
  
  v_PID_Set_Kp(pstru_Desired_PID, (float)u32_Desired_Value / PARAMS_SCALE);
  
  au8_Respond_Payload[0] = *pu8_Payload;
  au8_Respond_Payload[1] = 0x00; //Ok
  v_Send_Respond_Msg(u8_Msg_ID, au8_Respond_Payload, 2);
  return true;
}

bool bool_Set_Ki_Handler(uint8_t u8_Msg_ID, uint8_t *pu8_Payload, uint32_t u32_Payload_Cnt)
{
  uint8_t au8_Respond_Payload[MAX_SHORT_RES_PAYLOAD_LEN];
  STRU_PID_T *pstru_Desired_PID;
  uint32_t u32_Desired_Value;
  
  if (*pu8_Payload <= AXIS_INVALID) return false;
  if (*pu8_Payload >= AXIS_BOTH) return false;
  if (*(pu8_Payload + 1) <= PID_ID_INVALID) return false;
  if (*(pu8_Payload + 1) > PID_ID_STABILIZING_INNER) return false;
  
  pstru_Desired_PID = apstru_PID[*pu8_Payload][*(pu8_Payload + 1)];
  
  u32_Desired_Value = (*(pu8_Payload + 2) << 24) & 0x0ff000000;
  u32_Desired_Value += (*(pu8_Payload + 3) << 16) & 0x0ff0000;
  u32_Desired_Value += (*(pu8_Payload + 4) << 8) & 0x0ff00;
  u32_Desired_Value += *(pu8_Payload + 5) & 0x0ff;
  
  v_PID_Set_Ki(pstru_Desired_PID, (float)u32_Desired_Value / PARAMS_SCALE);
  
  au8_Respond_Payload[0] = *pu8_Payload;
  au8_Respond_Payload[1] = 0x00; //Ok
  v_Send_Respond_Msg(u8_Msg_ID, au8_Respond_Payload, 2);
  return true;
}

bool bool_Set_Kd_Handler(uint8_t u8_Msg_ID, uint8_t *pu8_Payload, uint32_t u32_Payload_Cnt)
{
  uint8_t au8_Respond_Payload[MAX_SHORT_RES_PAYLOAD_LEN];
  STRU_PID_T *pstru_Desired_PID;
  uint32_t u32_Desired_Value;
  
  if (*pu8_Payload <= AXIS_INVALID) return false;
  if (*pu8_Payload >= AXIS_BOTH) return false;
  if (*(pu8_Payload + 1) <= PID_ID_INVALID) return false;
  if (*(pu8_Payload + 1) > PID_ID_STABILIZING_INNER) return false;
  
  pstru_Desired_PID = apstru_PID[*pu8_Payload][*(pu8_Payload + 1)];
  
  u32_Desired_Value = (*(pu8_Payload + 2) << 24) & 0x0ff000000;
  u32_Desired_Value += (*(pu8_Payload + 3) << 16) & 0x0ff0000;
  u32_Desired_Value += (*(pu8_Payload + 4) << 8) & 0x0ff00;
  u32_Desired_Value += *(pu8_Payload + 5) & 0x0ff;
  
  v_PID_Set_Kd(pstru_Desired_PID, (float)u32_Desired_Value / PARAMS_SCALE);
  
  au8_Respond_Payload[0] = *pu8_Payload;
  au8_Respond_Payload[1] = 0x00; //Ok
  v_Send_Respond_Msg(u8_Msg_ID, au8_Respond_Payload, 2);
  return true;
}

bool bool_Set_Kff1_Handler(uint8_t u8_Msg_ID, uint8_t *pu8_Payload, uint32_t u32_Payload_Cnt)
{
  uint8_t au8_Respond_Payload[MAX_SHORT_RES_PAYLOAD_LEN];
  STRU_PID_T *pstru_Desired_PID;
  uint32_t u32_Desired_Value;
  
  if (*pu8_Payload <= AXIS_INVALID) return false;
  if (*pu8_Payload >= AXIS_BOTH) return false;
  if (*(pu8_Payload + 1) <= PID_ID_INVALID) return false;
  if (*(pu8_Payload + 1) > PID_ID_STABILIZING_INNER) return false;
  
  pstru_Desired_PID = apstru_PID[*pu8_Payload][*(pu8_Payload + 1)];
  
  u32_Desired_Value = (*(pu8_Payload + 2) << 24) & 0x0ff000000;
  u32_Desired_Value += (*(pu8_Payload + 3) << 16) & 0x0ff0000;
  u32_Desired_Value += (*(pu8_Payload + 4) << 8) & 0x0ff00;
  u32_Desired_Value += *(pu8_Payload + 5) & 0x0ff;
  
  v_PID_Set_Kff1(pstru_Desired_PID, (float)u32_Desired_Value / PARAMS_SCALE);
  
  au8_Respond_Payload[0] = *pu8_Payload;
  au8_Respond_Payload[1] = 0x00; //Ok
  v_Send_Respond_Msg(u8_Msg_ID, au8_Respond_Payload, 2);
  return true;
}

bool bool_Set_Kff2_Handler(uint8_t u8_Msg_ID, uint8_t *pu8_Payload, uint32_t u32_Payload_Cnt)
{
  uint8_t au8_Respond_Payload[MAX_SHORT_RES_PAYLOAD_LEN];
  STRU_PID_T *pstru_Desired_PID;
  uint32_t u32_Desired_Value;
  
  if (*pu8_Payload <= AXIS_INVALID) return false;
  if (*pu8_Payload >= AXIS_BOTH) return false;
  if (*(pu8_Payload + 1) <= PID_ID_INVALID) return false;
  if (*(pu8_Payload + 1) > PID_ID_STABILIZING_INNER) return false;
  
  pstru_Desired_PID = apstru_PID[*pu8_Payload][*(pu8_Payload + 1)];
  
  u32_Desired_Value = (*(pu8_Payload + 2) << 24) & 0x0ff000000;
  u32_Desired_Value += (*(pu8_Payload + 3) << 16) & 0x0ff0000;
  u32_Desired_Value += (*(pu8_Payload + 4) << 8) & 0x0ff00;
  u32_Desired_Value += *(pu8_Payload + 5) & 0x0ff;
  
  v_PID_Set_Kff2(pstru_Desired_PID, (float)u32_Desired_Value / PARAMS_SCALE);
  
  au8_Respond_Payload[0] = *pu8_Payload;
  au8_Respond_Payload[1] = 0x00; //Ok
  v_Send_Respond_Msg(u8_Msg_ID, au8_Respond_Payload, 2);
  return true;
}

bool bool_Get_Params_Handler(uint8_t u8_Msg_ID, uint8_t *pu8_Payload, uint32_t u32_Payload_Cnt)
{
  uint8_t au8_Respond_Payload[MAX_LONG_RES_PAYLOAD_LEN];
  uint32_t u32_Params, u32_Cnt;
  STRU_PID_T *pstru_Desired_PID;
  
  if (*pu8_Payload <= AXIS_INVALID) return false;
  if (*pu8_Payload >= AXIS_BOTH) return false;
  if (*(pu8_Payload + 1) <= PID_ID_INVALID) return false;
  if (*(pu8_Payload + 1) > PID_ID_STABILIZING_INNER) return false;
  
  pstru_Desired_PID = apstru_PID[*pu8_Payload][*(pu8_Payload + 1)];
  
  u32_Cnt = 0;
  au8_Respond_Payload[u32_Cnt++] = *pu8_Payload;
  
  u32_Params = (uint32_t)(flt_PID_Get_Kp(pstru_Desired_PID) * PARAMS_SCALE);
  au8_Respond_Payload[u32_Cnt++] = (u32_Params >> 24) & 0x0ff;
  au8_Respond_Payload[u32_Cnt++] = (u32_Params >> 16) & 0x0ff;
  au8_Respond_Payload[u32_Cnt++] = (u32_Params >> 8) & 0x0ff;
  au8_Respond_Payload[u32_Cnt++] = u32_Params & 0x0ff;
  
  u32_Params = (uint32_t)(flt_PID_Get_Ki(pstru_Desired_PID) * PARAMS_SCALE);
  au8_Respond_Payload[u32_Cnt++] = (u32_Params >> 24) & 0x0ff;
  au8_Respond_Payload[u32_Cnt++] = (u32_Params >> 16) & 0x0ff;
  au8_Respond_Payload[u32_Cnt++] = (u32_Params >> 8) & 0x0ff;
  au8_Respond_Payload[u32_Cnt++] = u32_Params & 0x0ff;
  
  u32_Params = (uint32_t)(flt_PID_Get_Kd(pstru_Desired_PID) * PARAMS_SCALE);
  au8_Respond_Payload[u32_Cnt++] = (u32_Params >> 24) & 0x0ff;
  au8_Respond_Payload[u32_Cnt++] = (u32_Params >> 16) & 0x0ff;
  au8_Respond_Payload[u32_Cnt++] = (u32_Params >> 8) & 0x0ff;
  au8_Respond_Payload[u32_Cnt++] = u32_Params & 0x0ff;
  
  u32_Params = (uint32_t)(flt_PID_Get_Kff1(pstru_Desired_PID) * PARAMS_SCALE);
  au8_Respond_Payload[u32_Cnt++] = (u32_Params >> 24) & 0x0ff;
  au8_Respond_Payload[u32_Cnt++] = (u32_Params >> 16) & 0x0ff;
  au8_Respond_Payload[u32_Cnt++] = (u32_Params >> 8) & 0x0ff;
  au8_Respond_Payload[u32_Cnt++] = u32_Params & 0x0ff;
  
  u32_Params = (uint32_t)(flt_PID_Get_Kff2(pstru_Desired_PID) * PARAMS_SCALE);
  au8_Respond_Payload[u32_Cnt++] = (u32_Params >> 24) & 0x0ff;
  au8_Respond_Payload[u32_Cnt++] = (u32_Params >> 16) & 0x0ff;
  au8_Respond_Payload[u32_Cnt++] = (u32_Params >> 8) & 0x0ff;
  au8_Respond_Payload[u32_Cnt++] = u32_Params & 0x0ff;
  
  v_Send_Respond_Msg(u8_Msg_ID, au8_Respond_Payload, u32_Cnt);
  return true;
}

static void v_Send_Respond_Msg(uint8_t u8_Msg_ID, uint8_t *pu8_Payload, uint32_t u32_Payload_Cnt)
{
  uint32_t u32_Idx, u32_Message_No_Checksum_Size;
  uint16_t u16_CRC_Check;
  static uint8_t au8_Respond_Message[MAX_RES_MESSAGE_LEN] = {0x47, 0x42, 0x01, 0x02, 0x00};
  
  /* Total Length */
  u32_Message_No_Checksum_Size = 7 + u32_Payload_Cnt;
  
  /* Length */
  au8_Respond_Message[5] = 1 + u32_Payload_Cnt + 2;
  
  /* MsgID */
  au8_Respond_Message[6] = u8_Msg_ID;
  
  /* Payload */
  for (u32_Idx = 0; u32_Idx < u32_Payload_Cnt; u32_Idx++)
  {
    au8_Respond_Message[7 + u32_Idx] = *(pu8_Payload + u32_Idx);
  }
  
  /* Checksum */
  u16_CRC_Check = 0;
  for (u32_Idx = 0; u32_Idx < u32_Message_No_Checksum_Size; u32_Idx++)
  {
    u16_CRC_Check += au8_Respond_Message[u32_Idx];
  }
  u16_CRC_Check = ~u16_CRC_Check;
  
  au8_Respond_Message[u32_Idx++] = (u16_CRC_Check >> 8) & 0x0ff;
  au8_Respond_Message[u32_Idx++] = u16_CRC_Check & 0x0ff;
  
  bool_CMD_Send(au8_Respond_Message, u32_Idx);
}
/**
  * @}
  */

/** @defgroup Send Params
 *  @brief    ...
 *
 @verbatim
 ===============================================================================
                            ##### Send Params #####
 ===============================================================================  

 @endverbatim
  * @{
  */

/**
  * @brief  Send Data for Collecting
  * @note   ...
  * @param  none
  * @retval none
  */
void v_Send_Data(void)
{
  uint8_t u8_Tx_Buff[DATA_TXBUFF_SIZE];
  uint32_t u32_Cnt = 0;
  int32_t s32_Temp = 0;
  
  u8_Tx_Buff[0] = 0x0a;
  u32_Cnt = 1;
  
  s32_Temp = (int32_t)(flt_AZ_ENC_Get_Angle() * 100);
  v_Int_To_Str_N(s32_Temp, &u8_Tx_Buff[u32_Cnt], 7);
  u32_Cnt += 7;
  u8_Tx_Buff[u32_Cnt++] = ' ';
  
  s32_Temp = (int32_t)(flt_EL_ENC_Get_Angle() * 100);
  v_Int_To_Str_N(s32_Temp, &u8_Tx_Buff[u32_Cnt], 7);
  u32_Cnt += 7;
  u8_Tx_Buff[u32_Cnt++] = ' ';
  
  u8_Tx_Buff[u32_Cnt++] = 0x0d;
  
  bool_DATA_Send(u8_Tx_Buff, u32_Cnt);
}

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
static void v_Int_To_Str_N(int32_t s32_Number, uint8_t *pu8_Str, uint32_t u32_N)
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

/** @defgroup Parameters Funtion
 *  @brief    ...
 *
 @verbatim
 ===============================================================================
                          ##### Parameters Funtion #####
 ===============================================================================  

 @endverbatim
  * @{
  */
/**
  * @brief  save default params
  * @note   
  * @param  none
  * @retval none
  */
void v_Params_Save_Default(void)
{
  bool_Params_Save(PARAMS_CODE_VERSION, au8_Code_Version);
  bool_Params_Save(PARAMS_PID_AZ_MANUAL_POS, (uint8_t *)&stru_PID_AZ_Manual_Pos);
  bool_Params_Save(PARAMS_PID_EL_MANUAL_POS, (uint8_t *)&stru_PID_EL_Manual_Pos);
}

/**
  * @brief  load all params
  * @note   
  * @param  none
  * @retval none
  */
void v_Params_Load_All(void)
{
  uint8_t au8_Loaded_Version[2] = {0, 0};
  
  bool_Params_Load(PARAMS_CODE_VERSION, au8_Loaded_Version);
  if((au8_Loaded_Version[0] != au8_Code_Version[0]) || (au8_Loaded_Version[1] != au8_Code_Version[1]))
  {
    v_Params_Save_Default();
  }
  
  bool_Params_Load(PARAMS_PID_AZ_MANUAL_POS, (uint8_t *)&stru_PID_AZ_Manual_Pos);
  bool_Params_Load(PARAMS_PID_EL_MANUAL_POS, (uint8_t *)&stru_PID_EL_Manual_Pos);
}
/**
  * @}
  */

/*********************************END OF FILE**********************************/
