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
#include "pid.h"
#include "math.h"
#include "stdio.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static volatile ENUM_AXIS_STATE_T enum_AZ_State = STATE_HOME; //STATE_HOME STATE_SINE
static volatile ENUM_AXIS_STATE_T enum_EL_State = STATE_HOME; //STATE_IDLE

static volatile bool bool_AZ_Going_Home = false;
static volatile bool bool_EL_Going_Home = false;

static volatile uint32_t u32_AZ_Sine_Idx = 0, u32_EL_Sine_Idx = 0;

static STRU_PID_T stru_PID_AZ_Manual_Pos;
static STRU_PID_T stru_PID_EL_Manual_Pos;

static STRU_PID_T stru_PID_AZ_Stabilizing_Outer;
static STRU_PID_T stru_PID_EL_Stabilizing_Outer;
static STRU_PID_T stru_PID_AZ_Stabilizing_Inner;
static STRU_PID_T stru_PID_EL_Stabilizing_Inner;

/* Private function prototypes -----------------------------------------------*/
static void v_Control_Change_Mode(ENUM_AXIS_STATE_T enum_AZ_New_State, ENUM_AXIS_STATE_T enum_EL_New_State);
static void v_Home_AZ_Handler(void);
static void v_Home_EL_Handler(void);
static void v_Limit_El_Handler(void);
static void v_Int_To_Str_N(int32_t s32_Number, uint8_t *pu8_Str, uint32_t u32_N);

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
  
  /* Pointing mode */
  v_PID_Init(&stru_PID_AZ_Stabilizing_Inner);
  v_PID_Set_Kp(&stru_PID_AZ_Stabilizing_Inner, 0.10);
  v_PID_Set_Ki(&stru_PID_AZ_Stabilizing_Inner, 20);
  v_PID_Set_Kd(&stru_PID_AZ_Stabilizing_Inner, 0.0004);
  
  /* Tracking mode */
  
}

/**
  * @brief  Control Funtion
  * @note   Call it with Ts in loop main (same Ts with PID)
  * @param  none
  * @retval none
  */
void v_Control(void)
{
  switch(enum_AZ_State)
  {
    case STATE_STOP:
    { 
      break;
    }
    case STATE_HOME:
      if(bool_AZ_Going_Home == false)
      {
        bool_AZ_Going_Home = true;
        v_AZ_Home_Rising_Register(v_Home_AZ_Handler);
        v_AZ_Home_Falling_Register(v_Home_AZ_Handler);
        
        if(u8_DI_Read_Pin(DI_PIN_AZ_HOME) == 0)
          v_AZ_PWM_Set_Duty(-75);
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
      
      //stru_PID_AZ_Pointing_Inner.Result = -stru_PID_AZ_Pointing_Inner.Kp * struIMUData.gyro_z;
      flt_PID_Calc(&stru_PID_AZ_Stabilizing_Inner, stru_ADIS_Data().flt_Gyro_z);
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
          v_EL_PWM_Set_Duty(110);
        else
          v_EL_PWM_Set_Duty(-110);
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
  * @param  enum_AZ_New_State: New State of Axis AZ
  * @param  enum_EL_New_State: New State of Axis EL
  * @retval none
  */
static void v_Control_Change_Mode(ENUM_AXIS_STATE_T enum_AZ_New_State, ENUM_AXIS_STATE_T enum_EL_New_State)
{
  if(enum_AZ_New_State != STATE_KEEP)
  {
    switch(enum_AZ_New_State)
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
    enum_AZ_State = enum_AZ_New_State;
  }
  
  if(enum_EL_New_State != STATE_KEEP)
  {
    switch(enum_EL_New_State)
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
    enum_EL_State = enum_EL_New_State;
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
  v_Control_Change_Mode(STATE_MANUAL, STATE_KEEP);
  bool_AZ_Going_Home = false;
}

static void v_Home_EL_Handler(void)
{
  v_EL_PWM_Set_Duty(0);
  v_EL_ENC_Reset();
  v_EL_Home_Rising_Unregister();
  v_EL_Home_Falling_Unregister();
  v_Control_Change_Mode(STATE_KEEP, STATE_STOP);
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
bool bool_None_Handler(uint8_t *pu8_Data, uint32_t u32_Data_Cnt)
{
  
  return true;
}

bool bool_Home_Handler(uint8_t *pu8_Data, uint32_t u32_Data_Cnt)
{
  
  return true;
}

bool bool_Stop_Handler(uint8_t *pu8_Data, uint32_t u32_Data_Cnt)
{
  
  return true;
}

bool bool_Emergency_Stop_Handler(uint8_t *pu8_Data, uint32_t u32_Data_Cnt)
{
  
  return true;
}

bool bool_Stabilizing_Mode_Handler(uint8_t *pu8_Data, uint32_t u32_Data_Cnt)
{
  
  return true;
}

bool bool_Set_Pos_Handler(uint8_t *pu8_Data, uint32_t u32_Data_Cnt)
{
  
  return true;
}

bool bool_Set_Vel_Handler(uint8_t *pu8_Data, uint32_t u32_Data_Cnt)
{
  
  return true;
}

bool bool_Set_Pos_Vel_Handler(uint8_t *pu8_Data, uint32_t u32_Data_Cnt)
{
  
  return true;
}

bool bool_Get_Pos_Handler(uint8_t *pu8_Data, uint32_t u32_Data_Cnt)
{
  
  return true;
}

bool bool_Set_Kp_Handler(uint8_t *pu8_Data, uint32_t u32_Data_Cnt)
{
  
  return true;
}

bool bool_Set_Ki_Handler(uint8_t *pu8_Data, uint32_t u32_Data_Cnt)
{
  
  return true;
}

bool bool_Set_Kd_Handler(uint8_t *pu8_Data, uint32_t u32_Data_Cnt)
{
  
  return true;
}

bool bool_Set_Kff1_Handler(uint8_t *pu8_Data, uint32_t u32_Data_Cnt)
{
  
  return true;
}

bool bool_Set_Kff2_Handler(uint8_t *pu8_Data, uint32_t u32_Data_Cnt)
{
  
  return true;
}

bool bool_Get_Params_Handler(uint8_t *pu8_Data, uint32_t u32_Data_Cnt)
{
  
  return true;
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

/*********************************END OF FILE**********************************/
