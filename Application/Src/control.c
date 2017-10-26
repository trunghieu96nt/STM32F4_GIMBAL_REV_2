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
#include "filter.h"
#include "math.h"
#include "stdio.h"
#include "utils.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define MAX_SHORT_RES_PAYLOAD_LEN           8
#define MAX_LONG_RES_PAYLOAD_LEN            32
#define MAX_RES_MESSAGE_LEN                 64
#define PARAMS_SCALE                        1000000.0f
#define POS_VEL_SCALE                       100.0f
#define DATA_LOG_AZ_VELOCITY_LOOP

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static const uint8_t au8_Code_Version[2] = {7, 7}; //Major.Minor

static volatile ENUM_AXIS_STATE_T enum_AZ_State = STATE_HOME; //STATE_HOME STATE_SINE
static volatile ENUM_AXIS_STATE_T enum_EL_State = STATE_HOME; //STATE_STOP

static bool bool_Active_AZ = false;
static bool bool_Active_EL = true;

static int16_t s16_AZ_PWM_Value = 0;
static int16_t s16_EL_PWM_Value = 0;
static float s16_AZ_PWM_Value_Raw = 0;
static float s16_EL_PWM_Value_Raw = 0;

static volatile bool bool_AZ_Going_Home = false;
static volatile bool bool_EL_Going_Home = false;

static uint32_t u32_AZ_Sine_Idx = 0;
static uint32_t u32_EL_Sine_Idx = 0;

static float flt_Euler_Angle[3], flt_Euler_Rate[3], flt_Body_Rate[3];
static float flt_Filtered_Body_Rate[3];


static STRU_PID_T stru_PID_AZ_Manual;
static STRU_PID_T stru_PID_AZ_Pointing;
//static STRU_PID_T stru_PID_AZ_Tracking;
static STRU_PID_T stru_PID_AZ_Velocity;
static STRU_PID_T stru_PID_AZ_Current;

static STRU_PID_T stru_PID_EL_Manual;
static STRU_PID_T stru_PID_EL_Pointing;
//static STRU_PID_T stru_PID_EL_Tracking;
static STRU_PID_T stru_PID_EL_Velocity;
static STRU_PID_T stru_PID_EL_Current;

static STRU_PID_T *apstru_PID[3][6] = {
  {0, 0, 0, 0, 0, 0},
  {0, &stru_PID_AZ_Manual, &stru_PID_AZ_Pointing, 0, &stru_PID_AZ_Velocity, &stru_PID_AZ_Current},
  {0, &stru_PID_EL_Manual, &stru_PID_EL_Pointing, 0, &stru_PID_EL_Velocity, &stru_PID_EL_Current}
};

static STRU_IIR_FILTER_T stru_IIR_AZ_Velocity_SP;
static STRU_IIR_FILTER_T stru_IIR_AZ_Velocity_PWM;

static STRU_IIR_FILTER_T stru_IIR_EL_Velocity_SP;
static STRU_IIR_FILTER_T stru_IIR_EL_Velocity_PWM;

/* Private function prototypes -----------------------------------------------*/
static void v_Control_Change_Mode(ENUM_AXIS_T enum_Axis, ENUM_AXIS_STATE_T enum_New_State);
static void v_Home_AZ_Handler(void);
static void v_Home_EL_Handler(void);
static void v_Limit_El_Handler(void);
static void v_Send_Response(uint8_t u8_Msg_ID, uint8_t *pu8_Payload, uint32_t u32_Payload_Cnt);

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
  float aflt_a[10], aflt_b[10];
  
  /* Limit EL*/
  if(u8_DI_Read_Pin(DI_PIN_EL_LIMIT) == 1)
    v_Limit_El_Handler();
  v_EL_Limit_Falling_Register(v_Limit_El_Handler);
  v_EL_Limit_Rising_Register(v_Limit_El_Handler);
  
  /* AZ Manual PID */
  v_PID_Init(&stru_PID_AZ_Manual);
  v_PID_Set_Kp(&stru_PID_AZ_Manual, 100);
  v_PID_Set_Ki(&stru_PID_AZ_Manual, 40);
  v_PID_Set_Kd(&stru_PID_AZ_Manual, 0.5);
  v_PID_Set_Use_Set_Point_Ramp(&stru_PID_AZ_Manual, 1);
  v_PID_Set_Max_Set_Point_Step(&stru_PID_AZ_Manual, 0.05);
  
  /* EL Manual PID */
  v_PID_Init(&stru_PID_EL_Manual);
  v_PID_Set_Kp(&stru_PID_EL_Manual, 100);
  v_PID_Set_Ki(&stru_PID_EL_Manual, 40);
  v_PID_Set_Kd(&stru_PID_EL_Manual, 0.5);
  v_PID_Set_Use_Set_Point_Ramp(&stru_PID_EL_Manual, 1);
  v_PID_Set_Max_Set_Point_Step(&stru_PID_EL_Manual, 0.02);
  v_PID_Set_Max_Response(&stru_PID_EL_Manual, 500);
  
  /* AZ Velocity PID */
  v_PID_Init(&stru_PID_AZ_Velocity);
  v_PID_Set_Kp(&stru_PID_AZ_Velocity, 0.10);
  v_PID_Set_Ki(&stru_PID_AZ_Velocity, 20);
  v_PID_Set_Kd(&stru_PID_AZ_Velocity, 0.0004);
  v_PID_Set_Use_Set_Point_Ramp(&stru_PID_AZ_Velocity, 0);
  v_PID_Set_Set_Point(&stru_PID_AZ_Velocity, 0.0f, 0);
  
  /* EL Velocity PID */
  v_PID_Init(&stru_PID_EL_Velocity);
  v_PID_Set_Kp(&stru_PID_EL_Velocity, 0.10);
  v_PID_Set_Ki(&stru_PID_EL_Velocity, 20);
  v_PID_Set_Kd(&stru_PID_EL_Velocity, 0.0004);
  v_PID_Set_Use_Set_Point_Ramp(&stru_PID_EL_Velocity, 0);
  v_PID_Set_Set_Point(&stru_PID_EL_Velocity, 0.0f, 0);
  
  /* AZ, EL Velocity IIR filter */
//  aflt_a[0] = 1.0f; // fs = 1000Hz, fc = 20Hz, Butterworth first order
//  aflt_a[1] = -0.881618592363189;
//  aflt_b[0] = 0.059190703818405;
//  aflt_b[1] = 0.059190703818405;
  
//  aflt_a[0] = 1.0f; // fs = 1000Hz, fc = 15Hz, Butterworth first order
//  aflt_a[1] = -0.909929988177738;
//  aflt_b[0] = 0.045035005911131;
//  aflt_b[1] = 0.045035005911131;
  
//  aflt_a[0] = 1.0f; // fs = 1000Hz, fc = 10Hz, Butterworth first order
//  aflt_a[1] = -0.939062505817492;
//  aflt_b[0] = 0.030468747091254;
//  aflt_b[1] = 0.030468747091254;
  
  aflt_a[0] = 1.0f; // fs = 1000Hz, fc = 5Hz, Butterworth first order
  aflt_a[1] = -0.969067417193793;
  aflt_b[0] = 0.015466291403103;
  aflt_b[1] = 0.015466291403103;
  
  v_IIR_Filter_Init(&stru_IIR_AZ_Velocity_SP, 1, aflt_a, aflt_b);
  v_IIR_Filter_Set_Enable(&stru_IIR_AZ_Velocity_SP, 1);
  
  v_IIR_Filter_Init(&stru_IIR_EL_Velocity_SP, 1, aflt_a, aflt_b);
  v_IIR_Filter_Set_Enable(&stru_IIR_EL_Velocity_SP, 1);
  
  aflt_a[0] = 1.0f; // fs = 1000Hz, fc = 5Hz, Butterworth first order
  aflt_a[1] = -0.969067417193793;
  aflt_b[0] = 0.015466291403103;
  aflt_b[1] = 0.015466291403103;
  
  v_IIR_Filter_Init(&stru_IIR_AZ_Velocity_PWM, 1, aflt_a, aflt_b);
  v_IIR_Filter_Set_Enable(&stru_IIR_AZ_Velocity_PWM, 0);
  
  v_IIR_Filter_Init(&stru_IIR_EL_Velocity_PWM, 1, aflt_a, aflt_b);
  v_IIR_Filter_Set_Enable(&stru_IIR_EL_Velocity_PWM, 0);
  
  /* Need to determine */
  v_PID_Init(&stru_PID_AZ_Pointing);
  //v_PID_Init(&stru_PID_AZ_Tracking);
  v_PID_Init(&stru_PID_AZ_Current);
  
  v_PID_Init(&stru_PID_EL_Pointing);
  //v_PID_Init(&stru_PID_EL_Tracking);
  v_PID_Init(&stru_PID_EL_Current);
  
}

/**
  * @brief  Control Funtion
  * @note   Call it with Ts in loop main (same Ts with PID)
  * @param  none
  * @retval none
  */
void v_Control(void)
{
  /* Position Loop */
  switch(enum_AZ_State)
  {
    case STATE_STOP:
    { 
      s16_AZ_PWM_Value = 0;
      break;
    }
    case STATE_HOME:
      if(bool_AZ_Going_Home == false)
      {
        bool_AZ_Going_Home = true;
        v_AZ_Home_Rising_Register(v_Home_AZ_Handler);
        v_AZ_Home_Falling_Register(v_Home_AZ_Handler);
        
        if(u8_DI_Read_Pin(DI_PIN_AZ_HOME) == 0)
          s16_AZ_PWM_Value = 75;
        else
          s16_AZ_PWM_Value = -75;
      }
      break;
    case STATE_MANUAL:
      s16_AZ_PWM_Value = 0.5f + flt_PID_Calc(&stru_PID_AZ_Manual, flt_AZ_ENC_Get_Angle());
      break;
    case STATE_POINTING:
      flt_Euler_Rate[YAW] = flt_PID_Calc(&stru_PID_AZ_Pointing, stru_Get_IMU_Data().flt_Euler_z);
      break;
    case STATE_TRACKING:
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
      s16_EL_PWM_Value = 0;
      break;
    case STATE_HOME:
      if(bool_EL_Going_Home == false)
      {
        bool_EL_Going_Home = true;
        v_EL_Home_Rising_Register(v_Home_EL_Handler);
        v_EL_Home_Falling_Register(v_Home_EL_Handler);
        
        if(u8_DI_Read_Pin(DI_PIN_EL_HOME) == 0)
          s16_EL_PWM_Value = -75;
        else
          s16_EL_PWM_Value = 75;
      }
      break;
    case STATE_MANUAL:
      s16_EL_PWM_Value = 0.5f + flt_PID_Calc(&stru_PID_EL_Manual, flt_EL_ENC_Get_Angle());
      break;
    case STATE_POINTING:
      flt_Euler_Rate[PITCH] = flt_PID_Calc(&stru_PID_EL_Pointing, stru_Get_IMU_Data().flt_Euler_y);
      break;
    case STATE_TRACKING:
      break;
    case STATE_SINE:
      v_EL_PWM_Set_Duty(200 * sin(2 * PI * u32_EL_Sine_Idx / 5000));
      if(++u32_EL_Sine_Idx == 5000) u32_EL_Sine_Idx = 0;
      break;
    default:
      break;
  }


  /* Velocity Loop */
  switch(enum_AZ_State)
  {
    case STATE_POINTING:
      flt_Euler_Angle[ROLL]   = stru_Get_IMU_Data().flt_Gyro_x;
      flt_Euler_Angle[PITCH]  = stru_Get_IMU_Data().flt_Gyro_y;
      flt_Euler_Angle[YAW]    = stru_Get_IMU_Data().flt_Gyro_z;
      v_Euler_To_Body_Rate(flt_Euler_Angle, flt_Euler_Rate, flt_Body_Rate);
    case STATE_TRACKING:
      flt_Filtered_Body_Rate[YAW] = flt_IIR_Filter_Calc(&stru_IIR_AZ_Velocity_SP, flt_Body_Rate[YAW]);
      v_PID_Set_Set_Point(&stru_PID_AZ_Velocity, flt_Filtered_Body_Rate[YAW], 0);
      
      s16_AZ_PWM_Value_Raw = flt_PID_Calc(&stru_PID_AZ_Velocity, stru_Get_IMU_Data().flt_Gyro_z) * cos(flt_AZ_ENC_Get_Angle() * DEGREE_TO_RAD);
      s16_AZ_PWM_Value = 0.5f + flt_IIR_Filter_Calc(&stru_IIR_AZ_Velocity_PWM, s16_AZ_PWM_Value_Raw);
      break;
    default:
      break;
  }
  
  switch(enum_EL_State)
  {
    case STATE_POINTING:
    case STATE_TRACKING:
      flt_Filtered_Body_Rate[PITCH] = flt_IIR_Filter_Calc(&stru_IIR_EL_Velocity_SP, flt_Body_Rate[PITCH]);
      v_PID_Set_Set_Point(&stru_PID_EL_Velocity, flt_Filtered_Body_Rate[PITCH], 0);
      
      s16_EL_PWM_Value_Raw = flt_PID_Calc(&stru_PID_EL_Velocity, stru_Get_IMU_Data().flt_Gyro_y);
      s16_EL_PWM_Value = 0.5f + flt_IIR_Filter_Calc(&stru_IIR_EL_Velocity_PWM, s16_EL_PWM_Value_Raw);
      break;
    default:
      break;
  }
  
  /* Write PWM */
  if (bool_Active_AZ == true)
    v_AZ_PWM_Set_Duty(s16_AZ_PWM_Value);
  else
    v_AZ_PWM_Set_Duty(0);
  
  if (bool_Active_EL == true)
    v_EL_PWM_Set_Duty(s16_EL_PWM_Value);
  else
    v_EL_PWM_Set_Duty(0);
  
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
      v_AZ_PWM_Set_Duty(0);
      
      if (enum_AZ_State == STATE_HOME) //current state is home
      {
        v_AZ_Home_Rising_Unregister();
        v_AZ_Home_Falling_Unregister();
      }
      
      switch(enum_New_State)
      {
        case STATE_STOP:
          break;
        case STATE_HOME:
          bool_AZ_Going_Home = false;
          break;
        case STATE_MANUAL:
          v_PID_Reset(&stru_PID_AZ_Manual);
          /* Set Set_Point_Buff */
          v_PID_Set_Set_Point(&stru_PID_AZ_Manual, flt_AZ_ENC_Get_Angle(), 1);
          /* Set Set_Point */
          v_PID_Set_Set_Point(&stru_PID_AZ_Manual, flt_AZ_ENC_Get_Angle(), 0);
          break;
        case STATE_POINTING:
          break;
        case STATE_TRACKING:
          v_IIR_Filter_Reset(&stru_IIR_AZ_Velocity_SP);
          v_IIR_Filter_Reset(&stru_IIR_AZ_Velocity_PWM);
          v_PID_Reset(&stru_PID_AZ_Velocity);
          flt_Body_Rate[YAW] = 0.0f; //setpoint
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
      v_EL_PWM_Set_Duty(0);
      
      if (enum_EL_State == STATE_HOME) //current state is home
      {
        v_EL_Home_Rising_Unregister();
        v_EL_Home_Falling_Unregister();
      }
      
      switch(enum_New_State)
      {
        case STATE_STOP:
          break;
        case STATE_HOME:
          bool_EL_Going_Home = false;
          break;
        case STATE_MANUAL:
          v_PID_Reset(&stru_PID_EL_Manual);
          /* Set Set_Point_Buff */
          v_PID_Set_Set_Point(&stru_PID_EL_Manual, flt_EL_ENC_Get_Angle(), 1);
          /* Set Set_Point */
          v_PID_Set_Set_Point(&stru_PID_EL_Manual, flt_EL_ENC_Get_Angle(), 0);
          break;
        case STATE_POINTING:
          break;
        case STATE_TRACKING:
          v_IIR_Filter_Reset(&stru_IIR_EL_Velocity_SP);
          v_IIR_Filter_Reset(&stru_IIR_EL_Velocity_PWM);
          v_PID_Reset(&stru_PID_EL_Velocity);
          flt_Body_Rate[PITCH] = 0.0f; //setpoint
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
  v_Control_Change_Mode(AXIS_EL, STATE_MANUAL);
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
  
  if (*pu8_Payload != 0x03) return false; //must be set 2 axis
  
  v_Control_Change_Mode(AXIS_BOTH, STATE_HOME);
  
  au8_Respond_Payload[0] = *pu8_Payload;
  au8_Respond_Payload[1] = 0x00; //Ok
  v_Send_Response(u8_Msg_ID, au8_Respond_Payload, 2);
  return true;
}

bool bool_Stop_Handler(uint8_t u8_Msg_ID, uint8_t *pu8_Payload, uint32_t u32_Payload_Cnt)
{
  uint8_t au8_Respond_Payload[MAX_SHORT_RES_PAYLOAD_LEN];
  
  if (*pu8_Payload != 0x03) return false; //must be set 2 axis
  
  v_Control_Change_Mode(AXIS_BOTH, STATE_STOP);
  
  au8_Respond_Payload[0] = *pu8_Payload;
  au8_Respond_Payload[1] = 0x00; //Ok
  v_Send_Response(u8_Msg_ID, au8_Respond_Payload, 2);
  return true;
}

bool bool_Emergency_Stop_Handler(uint8_t u8_Msg_ID, uint8_t *pu8_Payload, uint32_t u32_Payload_Cnt)
{
  uint8_t au8_Respond_Payload[MAX_SHORT_RES_PAYLOAD_LEN];
  
  v_Limit_El_Handler();
  
  au8_Respond_Payload[0] = *pu8_Payload;
  au8_Respond_Payload[1] = 0x00; //Ok
  v_Send_Response(u8_Msg_ID, au8_Respond_Payload, 2);
  return true;
}

bool bool_Stabilizing_Mode_Handler(uint8_t u8_Msg_ID, uint8_t *pu8_Payload, uint32_t u32_Payload_Cnt)
{
  uint8_t au8_Respond_Payload[MAX_SHORT_RES_PAYLOAD_LEN];
  
  if (*pu8_Payload != 0x03) return false; //must be set 2 axis
  
  if (*(pu8_Payload + 1) == 0x00)
    v_Control_Change_Mode(AXIS_BOTH, STATE_MANUAL);
  else if (*(pu8_Payload + 1) == 0x02)
    v_Control_Change_Mode(AXIS_BOTH, STATE_POINTING);
  else if (*(pu8_Payload + 1) == 0x01)
    v_Control_Change_Mode(AXIS_BOTH, STATE_TRACKING);
  
  au8_Respond_Payload[0] = *pu8_Payload;
  au8_Respond_Payload[1] = 0x00; //Ok
  v_Send_Response(u8_Msg_ID, au8_Respond_Payload, 2);
  return true;
}

bool bool_Get_Mode_Handler(uint8_t u8_Msg_ID, uint8_t *pu8_Payload, uint32_t u32_Payload_Cnt)
{
  uint8_t au8_Respond_Payload[MAX_SHORT_RES_PAYLOAD_LEN];
  
  au8_Respond_Payload[0] = *pu8_Payload;
  
  switch (enum_AZ_State)
  {
    case STATE_HOME:
      au8_Respond_Payload[1] = 0x01;
      au8_Respond_Payload[2] = 0x00;
      break;
    case STATE_STOP:
      au8_Respond_Payload[1] = 0x02;
      au8_Respond_Payload[2] = 0x00;
      break;
    case STATE_MANUAL:
      au8_Respond_Payload[1] = 0x04;
      au8_Respond_Payload[2] = 0x00;
      break;
    case STATE_POINTING:
      au8_Respond_Payload[1] = 0x04;
      au8_Respond_Payload[2] = 0x02;
      break;
    case STATE_TRACKING:
      au8_Respond_Payload[1] = 0x04;
      au8_Respond_Payload[2] = 0x01;
      break;
    default:
      break;
  }
  
  v_Send_Response(u8_Msg_ID, au8_Respond_Payload, 3);
  
  return true;
}

bool bool_Set_Pos_Handler(uint8_t u8_Msg_ID, uint8_t *pu8_Payload, uint32_t u32_Payload_Cnt)
{
  uint8_t au8_Respond_Payload[MAX_SHORT_RES_PAYLOAD_LEN];
  int32_t s32_Pos_Value;
  
  s32_Pos_Value = (*(pu8_Payload + 1) << 24) & 0x0ff000000;
  s32_Pos_Value += (*(pu8_Payload + 2) << 16) & 0x0ff0000;
  s32_Pos_Value += (*(pu8_Payload + 3) << 8) & 0x0ff00;
  s32_Pos_Value += *(pu8_Payload + 4) & 0x0ff;
  
  if (*pu8_Payload == 0x01)
  {
    switch (enum_AZ_State)
    {
      case STATE_MANUAL:
        v_PID_Set_Set_Point(&stru_PID_AZ_Manual, (float)s32_Pos_Value / 100.0f, 1);
        break;
      default:
        break;
    }
  }
  else if (*pu8_Payload == 0x02)
  {
    switch (enum_EL_State)
    {
      case STATE_MANUAL:
        v_PID_Set_Set_Point(&stru_PID_EL_Manual, (float)s32_Pos_Value / 100.0f, 1);
        break;
      default:
        break;
    }
  }
  
  au8_Respond_Payload[0] = *pu8_Payload;
  au8_Respond_Payload[1] = 0x00; //Ok
  v_Send_Response(u8_Msg_ID, au8_Respond_Payload, 2);
  return true;
}

bool bool_Set_Vel_Handler(uint8_t u8_Msg_ID, uint8_t *pu8_Payload, uint32_t u32_Payload_Cnt)
{
  uint8_t au8_Respond_Payload[MAX_SHORT_RES_PAYLOAD_LEN];
  int32_t s32_Vel_Value;
  float flt_Vel_Value;
  
  s32_Vel_Value = (*(pu8_Payload + 1) << 24) & 0x0ff000000;
  s32_Vel_Value += (*(pu8_Payload + 2) << 16) & 0x0ff0000;
  s32_Vel_Value += (*(pu8_Payload + 3) << 8) & 0x0ff00;
  s32_Vel_Value += *(pu8_Payload + 4) & 0x0ff;
  flt_Vel_Value = (float)s32_Vel_Value / POS_VEL_SCALE;
  
  if (*pu8_Payload == 0x01)
  {
    switch (enum_AZ_State)
    {
      case STATE_MANUAL:
        if (flt_Vel_Value < 0) flt_Vel_Value = -flt_Vel_Value;
        v_PID_Set_Max_Set_Point_Step(&stru_PID_AZ_Manual, flt_Vel_Value / 1000.0f);
        break;
      case STATE_TRACKING:
        flt_Body_Rate[YAW] = flt_Vel_Value * DEGREE_TO_MRAD;
        break;
      default:
        break;
    }
  }
  else if (*pu8_Payload == 0x02)
  {
    switch (enum_EL_State)
    {
      case STATE_MANUAL:
        if (flt_Vel_Value < 0) flt_Vel_Value = -flt_Vel_Value;
        v_PID_Set_Max_Set_Point_Step(&stru_PID_EL_Manual, flt_Vel_Value / 1000.0f);
        break;
      case STATE_TRACKING:
        flt_Body_Rate[PITCH] = flt_Vel_Value * DEGREE_TO_MRAD;
      default:
        break;
    }
  }
  
  au8_Respond_Payload[0] = *pu8_Payload;
  au8_Respond_Payload[1] = 0x00; //Ok
  v_Send_Response(u8_Msg_ID, au8_Respond_Payload, 2);
  return true;
}

bool bool_Set_Pos_Vel_Handler(uint8_t u8_Msg_ID, uint8_t *pu8_Payload, uint32_t u32_Payload_Cnt)
{
  uint8_t au8_Respond_Payload[MAX_SHORT_RES_PAYLOAD_LEN];
  
  au8_Respond_Payload[0] = *pu8_Payload;
  au8_Respond_Payload[1] = 0x00; //Ok
  v_Send_Response(u8_Msg_ID, au8_Respond_Payload, 2);
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
  v_Send_Response(u8_Msg_ID, au8_Respond_Payload, 5);
  return true;
}

bool bool_Set_Kp_Handler(uint8_t u8_Msg_ID, uint8_t *pu8_Payload, uint32_t u32_Payload_Cnt)
{
  uint8_t au8_Respond_Payload[MAX_SHORT_RES_PAYLOAD_LEN];
  STRU_PID_T *pstru_Desired_PID;
  uint32_t u32_Desired_Value;
  
  if (*pu8_Payload <= AXIS_INVALID_L) return false;
  if (*pu8_Payload >= AXIS_BOTH) return false;
  if (*(pu8_Payload + 1) <= PID_ID_INVALID_L) return false;
  if (*(pu8_Payload + 1) >= PID_ID_INVALID_H) return false;
  
  pstru_Desired_PID = apstru_PID[*pu8_Payload][*(pu8_Payload + 1)];
  
  if (pstru_Desired_PID == 0) return false;
  
  u32_Desired_Value = (*(pu8_Payload + 2) << 24) & 0x0ff000000;
  u32_Desired_Value += (*(pu8_Payload + 3) << 16) & 0x0ff0000;
  u32_Desired_Value += (*(pu8_Payload + 4) << 8) & 0x0ff00;
  u32_Desired_Value += *(pu8_Payload + 5) & 0x0ff;
  
  v_PID_Set_Kp(pstru_Desired_PID, (float)u32_Desired_Value / PARAMS_SCALE);
  
  au8_Respond_Payload[0] = *pu8_Payload;
  au8_Respond_Payload[1] = 0x00; //Ok
  v_Send_Response(u8_Msg_ID, au8_Respond_Payload, 2);
  return true;
}

bool bool_Set_Ki_Handler(uint8_t u8_Msg_ID, uint8_t *pu8_Payload, uint32_t u32_Payload_Cnt)
{
  uint8_t au8_Respond_Payload[MAX_SHORT_RES_PAYLOAD_LEN];
  STRU_PID_T *pstru_Desired_PID;
  uint32_t u32_Desired_Value;
  
  if (*pu8_Payload <= AXIS_INVALID_L) return false;
  if (*pu8_Payload >= AXIS_BOTH) return false;
  if (*(pu8_Payload + 1) <= PID_ID_INVALID_L) return false;
  if (*(pu8_Payload + 1) >= PID_ID_INVALID_H) return false;
  
  pstru_Desired_PID = apstru_PID[*pu8_Payload][*(pu8_Payload + 1)];
  
  if (pstru_Desired_PID == 0) return false;
  
  u32_Desired_Value = (*(pu8_Payload + 2) << 24) & 0x0ff000000;
  u32_Desired_Value += (*(pu8_Payload + 3) << 16) & 0x0ff0000;
  u32_Desired_Value += (*(pu8_Payload + 4) << 8) & 0x0ff00;
  u32_Desired_Value += *(pu8_Payload + 5) & 0x0ff;
  
  v_PID_Set_Ki(pstru_Desired_PID, (float)u32_Desired_Value / PARAMS_SCALE);
  
  au8_Respond_Payload[0] = *pu8_Payload;
  au8_Respond_Payload[1] = 0x00; //Ok
  v_Send_Response(u8_Msg_ID, au8_Respond_Payload, 2);
  return true;
}

bool bool_Set_Kd_Handler(uint8_t u8_Msg_ID, uint8_t *pu8_Payload, uint32_t u32_Payload_Cnt)
{
  uint8_t au8_Respond_Payload[MAX_SHORT_RES_PAYLOAD_LEN];
  STRU_PID_T *pstru_Desired_PID;
  uint32_t u32_Desired_Value;
  
  if (*pu8_Payload <= AXIS_INVALID_L) return false;
  if (*pu8_Payload >= AXIS_BOTH) return false;
  if (*(pu8_Payload + 1) <= PID_ID_INVALID_L) return false;
  if (*(pu8_Payload + 1) >= PID_ID_INVALID_H) return false;
  
  pstru_Desired_PID = apstru_PID[*pu8_Payload][*(pu8_Payload + 1)];
  
  if (pstru_Desired_PID == 0) return false;
  
  u32_Desired_Value = (*(pu8_Payload + 2) << 24) & 0x0ff000000;
  u32_Desired_Value += (*(pu8_Payload + 3) << 16) & 0x0ff0000;
  u32_Desired_Value += (*(pu8_Payload + 4) << 8) & 0x0ff00;
  u32_Desired_Value += *(pu8_Payload + 5) & 0x0ff;
  
  v_PID_Set_Kd(pstru_Desired_PID, (float)u32_Desired_Value / PARAMS_SCALE);
  
  au8_Respond_Payload[0] = *pu8_Payload;
  au8_Respond_Payload[1] = 0x00; //Ok
  v_Send_Response(u8_Msg_ID, au8_Respond_Payload, 2);
  return true;
}

bool bool_Set_Kff1_Handler(uint8_t u8_Msg_ID, uint8_t *pu8_Payload, uint32_t u32_Payload_Cnt)
{
  uint8_t au8_Respond_Payload[MAX_SHORT_RES_PAYLOAD_LEN];
  STRU_PID_T *pstru_Desired_PID;
  uint32_t u32_Desired_Value;
  
  if (*pu8_Payload <= AXIS_INVALID_L) return false;
  if (*pu8_Payload >= AXIS_BOTH) return false;
  if (*(pu8_Payload + 1) <= PID_ID_INVALID_L) return false;
  if (*(pu8_Payload + 1) >= PID_ID_INVALID_H) return false;
  
  pstru_Desired_PID = apstru_PID[*pu8_Payload][*(pu8_Payload + 1)];
  
  if (pstru_Desired_PID == 0) return false;
  
  u32_Desired_Value = (*(pu8_Payload + 2) << 24) & 0x0ff000000;
  u32_Desired_Value += (*(pu8_Payload + 3) << 16) & 0x0ff0000;
  u32_Desired_Value += (*(pu8_Payload + 4) << 8) & 0x0ff00;
  u32_Desired_Value += *(pu8_Payload + 5) & 0x0ff;
  
  v_PID_Set_Kff1(pstru_Desired_PID, (float)u32_Desired_Value / PARAMS_SCALE);
  
  au8_Respond_Payload[0] = *pu8_Payload;
  au8_Respond_Payload[1] = 0x00; //Ok
  v_Send_Response(u8_Msg_ID, au8_Respond_Payload, 2);
  return true;
}

bool bool_Set_Kff2_Handler(uint8_t u8_Msg_ID, uint8_t *pu8_Payload, uint32_t u32_Payload_Cnt)
{
  uint8_t au8_Respond_Payload[MAX_SHORT_RES_PAYLOAD_LEN];
  STRU_PID_T *pstru_Desired_PID;
  uint32_t u32_Desired_Value;
  
  if (*pu8_Payload <= AXIS_INVALID_L) return false;
  if (*pu8_Payload >= AXIS_BOTH) return false;
  if (*(pu8_Payload + 1) <= PID_ID_INVALID_L) return false;
  if (*(pu8_Payload + 1) >= PID_ID_INVALID_H) return false;
  
  pstru_Desired_PID = apstru_PID[*pu8_Payload][*(pu8_Payload + 1)];
  
  if (pstru_Desired_PID == 0) return false;
  
  u32_Desired_Value = (*(pu8_Payload + 2) << 24) & 0x0ff000000;
  u32_Desired_Value += (*(pu8_Payload + 3) << 16) & 0x0ff0000;
  u32_Desired_Value += (*(pu8_Payload + 4) << 8) & 0x0ff00;
  u32_Desired_Value += *(pu8_Payload + 5) & 0x0ff;
  
  v_PID_Set_Kff2(pstru_Desired_PID, (float)u32_Desired_Value / PARAMS_SCALE);
  
  au8_Respond_Payload[0] = *pu8_Payload;
  au8_Respond_Payload[1] = 0x00; //Ok
  v_Send_Response(u8_Msg_ID, au8_Respond_Payload, 2);
  return true;
}

bool bool_Get_Params_Handler(uint8_t u8_Msg_ID, uint8_t *pu8_Payload, uint32_t u32_Payload_Cnt)
{
  uint8_t au8_Respond_Payload[MAX_LONG_RES_PAYLOAD_LEN];
  uint32_t u32_Params, u32_Cnt;
  STRU_PID_T *pstru_Desired_PID;
  
  if (*pu8_Payload <= AXIS_INVALID_L) return false;
  if (*pu8_Payload >= AXIS_BOTH) return false;
  if (*(pu8_Payload + 1) <= PID_ID_INVALID_L) return false;
  if (*(pu8_Payload + 1) >= PID_ID_INVALID_H) return false;
  
  pstru_Desired_PID = apstru_PID[*pu8_Payload][*(pu8_Payload + 1)];
  
  if (pstru_Desired_PID == 0) return false;
  
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
  
  v_Send_Response(u8_Msg_ID, au8_Respond_Payload, u32_Cnt);
  return true;
}

bool bool_Set_Active_Axis_Handler(uint8_t u8_Msg_ID, uint8_t *pu8_Payload, uint32_t u32_Payload_Cnt)
{
  uint8_t au8_Respond_Payload[MAX_SHORT_RES_PAYLOAD_LEN];
  
  if (*pu8_Payload == 0x01)
  {
    if (*(pu8_Payload + 1) == 0x01)
      bool_Active_AZ = true;
    else
      bool_Active_AZ = false;
  }
  else if (*pu8_Payload == 0x02)
  {
    if (*(pu8_Payload + 1) == 0x01)
      bool_Active_EL = true;
    else
      bool_Active_EL = false;
  }
  
  au8_Respond_Payload[0] = *pu8_Payload;
  au8_Respond_Payload[1] = 0x00; //Ok
  v_Send_Response(u8_Msg_ID, au8_Respond_Payload, 2);
  return true;
}

bool bool_Get_Active_Axis_Handler(uint8_t u8_Msg_ID, uint8_t *pu8_Payload, uint32_t u32_Payload_Cnt)
{
  uint8_t au8_Respond_Payload[MAX_SHORT_RES_PAYLOAD_LEN];
  
  au8_Respond_Payload[0] = *pu8_Payload;
  
  if (*pu8_Payload == 0x01)
  {
    if (bool_Active_AZ == true)
      au8_Respond_Payload[1] = 0x01;
    else
      au8_Respond_Payload[1] = 0x00;
  }
  else if (*pu8_Payload == 0x02)
  {
    if (bool_Active_EL == true)
      au8_Respond_Payload[1] = 0x01;
    else
      au8_Respond_Payload[1] = 0x00;
  }
  
  v_Send_Response(u8_Msg_ID, au8_Respond_Payload, 2);
  return true;
}

static void v_Send_Response(uint8_t u8_Msg_ID, uint8_t *pu8_Payload, uint32_t u32_Payload_Cnt)
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
#ifdef DATA_LOG_GENERAL
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
  
  s32_Temp = (int32_t)(flt_EL_ENC_Get_Angle() * 100);
  v_Int_To_Str_N(s32_Temp, &u8_Tx_Buff[u32_Cnt], 7);
  u32_Cnt += 7;
  u8_Tx_Buff[u32_Cnt++] = ' ';
  
  u8_Tx_Buff[u32_Cnt++] = 0x0d;
  
  bool_DATA_Send(u8_Tx_Buff, u32_Cnt);
#endif

#ifdef DATA_LOG_AZ_VELOCITY_LOOP
  uint8_t u8_Tx_Buff[DATA_TXBUFF_SIZE];
  uint32_t u32_Cnt = 0;
  int32_t s32_Temp = 0;
  
  u8_Tx_Buff[0] = 0x0a;
  u32_Cnt = 1;
  
  /* s16_AZ_PWM_Value */
  s32_Temp = (int32_t)s16_AZ_PWM_Value;
  v_Int_To_Str_N(s32_Temp, &u8_Tx_Buff[u32_Cnt], 5);
  u32_Cnt += 5;
  u8_Tx_Buff[u32_Cnt++] = ' ';
  
  /* flt_Body_Rate[YAW] */
  s32_Temp = (int32_t)(flt_Body_Rate[YAW] * 10);
  v_Int_To_Str_N(s32_Temp, &u8_Tx_Buff[u32_Cnt], 7);
  u32_Cnt += 7;
  u8_Tx_Buff[u32_Cnt++] = ' ';
  
  /* flt_Filtered_Body_Rate[YAW] */
  s32_Temp = (int32_t)(flt_Filtered_Body_Rate[YAW] * 10);
  v_Int_To_Str_N(s32_Temp, &u8_Tx_Buff[u32_Cnt], 7);
  u32_Cnt += 7;
  u8_Tx_Buff[u32_Cnt++] = ' ';
  
  /* stru_Get_IMU_Data().flt_Gyro_z */
  s32_Temp = (int32_t)(stru_Get_IMU_Data().flt_Gyro_z * 10);
  v_Int_To_Str_N(s32_Temp, &u8_Tx_Buff[u32_Cnt], 7);
  u32_Cnt += 7;
  u8_Tx_Buff[u32_Cnt++] = ' ';
  
  /* s16_EL_PWM_Value */
  s32_Temp = (int32_t)s16_EL_PWM_Value;
  v_Int_To_Str_N(s32_Temp, &u8_Tx_Buff[u32_Cnt], 5);
  u32_Cnt += 5;
  u8_Tx_Buff[u32_Cnt++] = ' ';
  
  /* flt_Body_Rate[PITCH] */
  s32_Temp = (int32_t)(flt_Body_Rate[PITCH] * 10);
  v_Int_To_Str_N(s32_Temp, &u8_Tx_Buff[u32_Cnt], 7);
  u32_Cnt += 7;
  u8_Tx_Buff[u32_Cnt++] = ' ';
  
  /* flt_Filtered_Body_Rate[PITCH] */
  s32_Temp = (int32_t)(flt_Filtered_Body_Rate[PITCH] * 10);
  v_Int_To_Str_N(s32_Temp, &u8_Tx_Buff[u32_Cnt], 7);
  u32_Cnt += 7;
  u8_Tx_Buff[u32_Cnt++] = ' ';
  
  /* stru_Get_IMU_Data().flt_Gyro_y */
  s32_Temp = (int32_t)(stru_Get_IMU_Data().flt_Gyro_y * 10);
  v_Int_To_Str_N(s32_Temp, &u8_Tx_Buff[u32_Cnt], 7);
  u32_Cnt += 7;
  u8_Tx_Buff[u32_Cnt++] = ' ';
  
  u8_Tx_Buff[u32_Cnt++] = 0x0d;
  
  bool_DATA_Send(u8_Tx_Buff, u32_Cnt);
#endif
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
  bool_Params_Save(PARAMS_PID_AZ_MANUAL_POS, (uint8_t *)&stru_PID_AZ_Manual);
  bool_Params_Save(PARAMS_PID_EL_MANUAL_POS, (uint8_t *)&stru_PID_EL_Manual);
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
  
  bool_Params_Load(PARAMS_PID_AZ_MANUAL_POS, (uint8_t *)&stru_PID_AZ_Manual);
  bool_Params_Load(PARAMS_PID_EL_MANUAL_POS, (uint8_t *)&stru_PID_EL_Manual);
}
/**
  * @}
  */

/*********************************END OF FILE**********************************/
