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
#include "adc_driver.h"
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
#define DATA_LOG_GENERAL //DATA_LOG_GENERAL DATA_GYRO_FILTER

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static const uint8_t au8_code_version[2] = {7, 7}; //Major.Minor

static volatile ENUM_AXIS_STATE_T enum_az_state = STATE_MANUAL; //STATE_HOME STATE_SINE
static volatile ENUM_AXIS_STATE_T enum_el_state = STATE_MANUAL; //STATE_STOP STATE_MANUAL

static bool bool_active_az = true;
static bool bool_active_el = true;

static int16_t s16_az_speed, s16_el_speed; // BLDC Motor
static int16_t s16_az_speed_raw, s16_el_speed_raw; // BLDC Motor
//static int16_t s16_az_pwm_value = 0; // DC Motor: not use in BLDC version
//static int16_t s16_el_pwm_value = 0; // DC Motor: not use in BLDC version
//static float s16_az_pwm_value_raw = 0;
//static float s16_el_pwm_value_raw = 0;

static volatile bool bool_az_going_home = false;
static volatile bool bool_el_going_home = false;

static uint32_t u32_az_sine_idx = 0, u32_el_sine_idx = 0;
static int32_t s32_az_sine_cnt = 1, s32_el_sine_cnt = 1;

static float flt_euler_angle[3], flt_euler_rate[3], flt_body_rate[3];
static float flt_filtered_body_rate[3];
static float flt_alpha = 0;
static float flt_lamda = 0;

static STRU_PID_T stru_pid_az_manual;
static STRU_PID_T stru_pid_az_pointing;
//static STRU_PID_T stru_pid_az_tracking;
static STRU_PID_T stru_pid_az_velocity;
static STRU_PID_T stru_pid_az_current;

static STRU_PID_T stru_pid_el_manual;
static STRU_PID_T stru_pid_el_pointing;
//static STRU_PID_T stru_pid_el_tracking;
static STRU_PID_T stru_pid_el_velocity;
static STRU_PID_T stru_pid_el_current;

static STRU_PID_T *apstru_pid[3][6] = {
  {0, 0, 0, 0, 0, 0},
  {0, &stru_pid_az_manual, &stru_pid_az_pointing, 0, &stru_pid_az_velocity, &stru_pid_az_current},
  {0, &stru_pid_el_manual, &stru_pid_el_pointing, 0, &stru_pid_el_velocity, &stru_pid_el_current}
};

static STRU_IIR_FILTER_T stru_iir_az_velocity_sp;
static STRU_IIR_FILTER_T stru_iir_az_velocity_pwm;

static STRU_IIR_FILTER_T stru_iir_el_velocity_sp;
static STRU_IIR_FILTER_T stru_iir_el_velocity_pwm;

static STRU_IIR_FILTER_T stru_iir_pitch_gyro;
static STRU_IIR_FILTER_T stru_iir_yaw_gyro;

float flt_filtered_pitch_gyro, flt_filtered_yaw_gyro;
/* Private function prototypes -----------------------------------------------*/
static void v_Control_Change_Mode(ENUM_AXIS_T enum_axis, ENUM_AXIS_STATE_T enum_new_state);
static void v_Home_AZ_Handler(void);
static void v_Home_EL_Handler(void);
static void v_Limit_El_Handler(void);
static void v_Send_Response(uint8_t u8_msg_id, uint8_t *pu8_payload, uint32_t u32_payload_cnt);
static void v_Send_BLDC_Speed(ENUM_AXIS_T enum_axis, int16_t s16_az_speed, int16_t s16_el_speed);

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
  if (u8_DI_Read_Pin(DI_PIN_EL_LIMIT) == 1)
    v_Limit_El_Handler();
  v_EL_Limit_Falling_Register(v_Limit_El_Handler);
  v_EL_Limit_Rising_Register(v_Limit_El_Handler);
  
  /* AZ Manual PID */
  v_PID_Init(&stru_pid_az_manual);
  v_PID_Set_Kp(&stru_pid_az_manual, 100);
  v_PID_Set_Ki(&stru_pid_az_manual, 40);
  v_PID_Set_Kd(&stru_pid_az_manual, 0.5);
  v_PID_Set_Use_Setpoint_Ramp(&stru_pid_az_manual, 1);
  v_PID_Set_Max_Setpoint_Step(&stru_pid_az_manual, 0.05);
  
  /* EL Manual PID */
  v_PID_Init(&stru_pid_el_manual);
  v_PID_Set_Kp(&stru_pid_el_manual, 100);
  v_PID_Set_Ki(&stru_pid_el_manual, 40);
  v_PID_Set_Kd(&stru_pid_el_manual, 0.5);
  v_PID_Set_Use_Setpoint_Ramp(&stru_pid_el_manual, 1);
  v_PID_Set_Max_Setpoint_Step(&stru_pid_el_manual, 0.02);
  v_PID_Set_Max_Response(&stru_pid_el_manual, 500);
  
  /* AZ Velocity PID */
  v_PID_Init(&stru_pid_az_velocity);
  v_PID_Set_Kp(&stru_pid_az_velocity, 0.0001);
  v_PID_Set_Ki(&stru_pid_az_velocity, 0.07);
  v_PID_Set_Kd(&stru_pid_az_velocity, 0.000015);
  v_PID_Set_Max_Response(&stru_pid_az_velocity, 200);
  v_PID_Set_Use_Setpoint_Ramp(&stru_pid_az_velocity, 0);
  v_PID_Set_Setpoint(&stru_pid_az_velocity, 0.0f, 0);
  
  /* EL Velocity PID */
  v_PID_Init(&stru_pid_el_velocity);
  v_PID_Set_Kp(&stru_pid_el_velocity, 0.0005);
  v_PID_Set_Ki(&stru_pid_el_velocity, 0.1);
  v_PID_Set_Kd(&stru_pid_el_velocity, 0.00005);
  v_PID_Set_Max_Response(&stru_pid_az_velocity, 200);
  v_PID_Set_Use_Setpoint_Ramp(&stru_pid_el_velocity, 0);
  v_PID_Set_Setpoint(&stru_pid_el_velocity, 0.0f, 0);
  
  /* AZ, EL Velocity IIR filter */
//  aflt_a[0] = 1.0f; // fs = 500Hz, fc = 40Hz, Butterworth first order
//  aflt_a[1] = -0.591398351399471;
//  aflt_b[0] = 0.204300824300264;
//  aflt_b[1] = 0.204300824300264;
  
//  aflt_a[0] = 1.0f; // fs = 500Hz, fc = 30Hz, Butterworth first order
//  aflt_a[1] = -0.679599298224527;
//  aflt_b[0] = 0.160200350887737;
//  aflt_b[1] = 0.160200350887737;
  
//  aflt_a[0] = 1.0f; // fs = 500Hz, fc = 20Hz, Butterworth first order
//  aflt_a[1] = -0.775679511049613;
//  aflt_b[0] = 0.112160244475193;
//  aflt_b[1] = 0.112160244475193;
  
//  aflt_a[0] = 1.0f; // fs = 500Hz, fc = 15Hz, Butterworth first order
//  aflt_a[1] = -0.827271945972476;
//  aflt_b[0] = 0.086364027013762;
//  aflt_b[1] = 0.086364027013762;
  
  aflt_a[0] = 1.0f; // fs = 500Hz, fc = 10Hz, Butterworth first order
  aflt_a[1] = -0.881618592363189;
  aflt_b[0] = 0.059190703818405;
  aflt_b[1] = 0.059190703818405;
  
  v_IIR_Filter_Init(&stru_iir_az_velocity_sp, 1, aflt_a, aflt_b);
  v_IIR_Filter_Set_Enable(&stru_iir_az_velocity_sp, 0);
  
  v_IIR_Filter_Init(&stru_iir_el_velocity_sp, 1, aflt_a, aflt_b);
  v_IIR_Filter_Set_Enable(&stru_iir_el_velocity_sp, 0);
  
  aflt_a[0] = 1.0f; // fs = 500Hz, fc = 20Hz, Butterworth first order
  aflt_a[1] = -0.775679511049613;
  aflt_b[0] = 0.112160244475193;
  aflt_b[1] = 0.112160244475193;
  
  v_IIR_Filter_Init(&stru_iir_az_velocity_pwm, 1, aflt_a, aflt_b);
  v_IIR_Filter_Set_Enable(&stru_iir_az_velocity_pwm, 0);
  
  v_IIR_Filter_Init(&stru_iir_el_velocity_pwm, 1, aflt_a, aflt_b);
  v_IIR_Filter_Set_Enable(&stru_iir_el_velocity_pwm, 0);
  
  aflt_a[0] = 1.0f; // fs = 500Hz, fc = 10Hz, Butterworth first order
  aflt_a[1] = -0.881618592363189;
  aflt_b[0] = 0.059190703818405;
  aflt_b[1] = 0.059190703818405;
  
  v_IIR_Filter_Init(&stru_iir_pitch_gyro, 1, aflt_a, aflt_b);
  v_IIR_Filter_Set_Enable(&stru_iir_pitch_gyro, 1);
  
  v_IIR_Filter_Init(&stru_iir_yaw_gyro, 1, aflt_a, aflt_b);
  v_IIR_Filter_Set_Enable(&stru_iir_yaw_gyro, 1);
  
  
  /* Need to determine */
  v_PID_Init(&stru_pid_az_pointing);
  //v_PID_Init(&stru_pid_az_tracking);
  v_PID_Init(&stru_pid_az_current);
  
  v_PID_Init(&stru_pid_el_pointing);
  //v_PID_Init(&stru_pid_el_tracking);
  v_PID_Init(&stru_pid_el_current);
  
}

/**
  * @brief  Control Funtion
  * @note   Call it with Ts in loop main (same Ts with PID)
  * @param  none
  * @retval none
  */
void v_Control(void)
{
  static uint16_t u16_adc_value;
  /* gyro filter */
  flt_filtered_pitch_gyro = flt_IIR_Filter_Calc(&stru_iir_pitch_gyro, stru_Get_IMU_Data().flt_gyro_y);
  flt_filtered_yaw_gyro = flt_IIR_Filter_Calc(&stru_iir_yaw_gyro, stru_Get_IMU_Data().flt_gyro_z);
  
  /* Position Loop */
  switch (enum_az_state)
  {
    case STATE_STOP:
    { 
      s16_az_speed = 0;
      break;
    }
    case STATE_HOME:
      if (bool_az_going_home == false)
      {
        bool_az_going_home = true;
        v_AZ_Home_Rising_Register(v_Home_AZ_Handler);
        v_AZ_Home_Falling_Register(v_Home_AZ_Handler);
        
//        if (u8_DI_Read_Pin(DI_PIN_AZ_HOME) == 0)
//          s16_az_pwm_value = 85;
//        else
//          s16_az_pwm_value = -85;
      }
      break;
    case STATE_MANUAL:
      //s16_az_pwm_value = 0.5f + flt_PID_Calc(&stru_pid_az_manual, flt_AZ_ENC_Get_Angle());
      u16_adc_value = u16_ADC_Get_Raw_Value(ADC_ID_0);
      
      if (u16_adc_value > 3500)
        s16_az_speed = 5;
      else if (u16_adc_value < 500)
        s16_az_speed = -5;
      else
        s16_az_speed = 0;
      
      break;
    case STATE_POINTING:
      flt_euler_rate[YAW] = flt_PID_Calc(&stru_pid_az_pointing, stru_Get_IMU_Data().flt_euler_z);
      break;
    case STATE_TRACKING:
      break;
    case STATE_SINE:
      //v_AZ_PWM_Set_Duty(300 * sin(2 * PI * u32_az_sine_idx / 5000));
      //if (++u32_az_sine_idx == 5000) u32_az_sine_idx = 0;
      u32_az_sine_idx += s32_az_sine_cnt;
      if (u32_az_sine_idx == 2000)
        s32_az_sine_cnt = -1;
      else if (u32_az_sine_idx == 0)
        s32_az_sine_cnt = 1;
      s16_az_speed = u32_az_sine_idx;
      break;
    default:
      break;
  }
  
  switch (enum_el_state)
  {
    case STATE_STOP:
      s16_el_speed = 0;
      break;
    case STATE_HOME:
      if (bool_el_going_home == false)
      {
        bool_el_going_home = true;
        v_EL_Home_Rising_Register(v_Home_EL_Handler);
        v_EL_Home_Falling_Register(v_Home_EL_Handler);
        
//        if (u8_DI_Read_Pin(DI_PIN_EL_HOME) == 0)
//          s16_el_pwm_value = -75;
//        else
//          s16_el_pwm_value = 75;
      }
      break;
    case STATE_MANUAL:
      //s16_el_pwm_value = 0.5f + flt_PID_Calc(&stru_pid_el_manual, flt_EL_ENC_Get_Angle());
      
      u16_adc_value = u16_ADC_Get_Raw_Value(ADC_ID_1);
      
      if (u16_adc_value > 3500)
        s16_el_speed = -5;
      else if (u16_adc_value < 500)
        s16_el_speed = 5;
      else
        s16_el_speed = 0;
      
      break;
    case STATE_POINTING:
      flt_euler_rate[PITCH] = flt_PID_Calc(&stru_pid_el_pointing, stru_Get_IMU_Data().flt_euler_y);
      break;
    case STATE_TRACKING:
      break;
    case STATE_SINE:
      //v_EL_PWM_Set_Duty(200 * sin(2 * PI * u32_el_sine_idx / 5000));
      //if (++u32_el_sine_idx == 5000) u32_el_sine_idx = 0;
      u32_el_sine_idx += s32_el_sine_cnt;
      if (u32_el_sine_idx == 2000)
        s32_el_sine_cnt = -1;
      else if (u32_el_sine_idx == 0)
        s32_el_sine_cnt = 1;
      s16_el_speed = u32_el_sine_idx;
      break;
    default:
      break;
  }


  /* Velocity Loop */
  switch (enum_az_state)
  {
    case STATE_POINTING:
      flt_euler_angle[ROLL]   = stru_Get_IMU_Data().flt_gyro_x;
      flt_euler_angle[PITCH]  = stru_Get_IMU_Data().flt_gyro_y;
      flt_euler_angle[YAW]    = stru_Get_IMU_Data().flt_gyro_z;
      v_Euler_To_Body_Rate(flt_euler_angle, flt_euler_rate, flt_body_rate);
    case STATE_TRACKING:
      flt_filtered_body_rate[YAW] = flt_IIR_Filter_Calc(&stru_iir_az_velocity_sp, flt_body_rate[YAW]);
      v_PID_Set_Setpoint(&stru_pid_az_velocity, flt_filtered_body_rate[YAW], 0);
      
      s16_az_speed_raw = flt_PID_Calc(&stru_pid_az_velocity, flt_filtered_yaw_gyro);
      s16_az_speed = 0.5f + flt_IIR_Filter_Calc(&stru_iir_az_velocity_pwm, s16_az_speed_raw);
      break;
    default:
      break;
  }
  
  switch (enum_el_state)
  {
    case STATE_POINTING:
    case STATE_TRACKING:
      flt_filtered_body_rate[PITCH] = flt_IIR_Filter_Calc(&stru_iir_el_velocity_sp, flt_body_rate[PITCH]);
      v_PID_Set_Setpoint(&stru_pid_el_velocity, flt_filtered_body_rate[PITCH], 0);
      
      s16_el_speed_raw = flt_PID_Calc(&stru_pid_el_velocity, flt_filtered_pitch_gyro);
      s16_el_speed = 0.5f + flt_IIR_Filter_Calc(&stru_iir_el_velocity_pwm, s16_el_speed_raw);
      break;
    default:
      break;
  }
  
  /* Write PWM */
  if (bool_active_az == false)
    s16_az_speed = 0;
  
  if (bool_active_el == false)
    s16_el_speed = 0;
  
  v_Send_BLDC_Speed(AXIS_BOTH, s16_az_speed, s16_el_speed);
}

/**
  * @brief  Change State Operation
  * @note   
  * @param  enum_axis: Desired Axis
  * @param  enum_new_state: New State of Axis
  * @retval none
  */
static void v_Control_Change_Mode(ENUM_AXIS_T enum_axis, ENUM_AXIS_STATE_T enum_new_state)
{
  if (enum_new_state != STATE_KEEP)
  {
    if ((enum_axis == AXIS_AZ) || (enum_axis == AXIS_BOTH))
    {
      //v_AZ_PWM_Set_Duty(0);
      v_Send_BLDC_Speed(AXIS_AZ, 0, 0);
      
      if (enum_az_state == STATE_HOME) //current state is home
      {
        v_AZ_Home_Rising_Unregister();
        v_AZ_Home_Falling_Unregister();
      }
      
      switch (enum_new_state)
      {
        case STATE_STOP:
          break;
        case STATE_HOME:
          bool_az_going_home = false;
          break;
        case STATE_MANUAL:
          v_PID_Reset(&stru_pid_az_manual);
          /* Set setpoint_buff */
          v_PID_Set_Setpoint(&stru_pid_az_manual, flt_AZ_ENC_Get_Angle(), 1);
          /* Set setpoint */
          v_PID_Set_Setpoint(&stru_pid_az_manual, flt_AZ_ENC_Get_Angle(), 0);
          break;
        case STATE_POINTING:
          break;
        case STATE_TRACKING:
          v_IIR_Filter_Reset(&stru_iir_az_velocity_sp);
          v_IIR_Filter_Reset(&stru_iir_az_velocity_pwm);
          v_PID_Reset(&stru_pid_az_velocity);
          flt_body_rate[YAW] = 0.0f; //setpoint
          break;
        case STATE_SINE:
          u32_az_sine_idx = 0;
          break;
        default:
          break;
      }
      enum_az_state = enum_new_state;
    }
    
    if ((enum_axis == AXIS_EL) || (enum_axis == AXIS_BOTH))
    {
      //v_EL_PWM_Set_Duty(0);
      v_Send_BLDC_Speed(AXIS_EL, 0, 0);
      
      if (enum_el_state == STATE_HOME) //current state is home
      {
        v_EL_Home_Rising_Unregister();
        v_EL_Home_Falling_Unregister();
      }
      
      switch (enum_new_state)
      {
        case STATE_STOP:
          break;
        case STATE_HOME:
          bool_el_going_home = false;
          break;
        case STATE_MANUAL:
          v_PID_Reset(&stru_pid_el_manual);
          /* Set setpoint_buff */
          v_PID_Set_Setpoint(&stru_pid_el_manual, flt_EL_ENC_Get_Angle(), 1);
          /* Set setpoint */
          v_PID_Set_Setpoint(&stru_pid_el_manual, flt_EL_ENC_Get_Angle(), 0);
          break;
        case STATE_POINTING:
          break;
        case STATE_TRACKING:
          v_IIR_Filter_Reset(&stru_iir_el_velocity_sp);
          v_IIR_Filter_Reset(&stru_iir_el_velocity_pwm);
          v_PID_Reset(&stru_pid_el_velocity);
          flt_body_rate[PITCH] = 0.0f; //setpoint
          break;
        case STATE_SINE:
          u32_el_sine_idx = 0;
          break;
        default:
          break;
      }
      enum_el_state = enum_new_state;
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
  bool_az_going_home = false;
}

static void v_Home_EL_Handler(void)
{
  v_EL_PWM_Set_Duty(0);
  v_EL_ENC_Reset();
  v_EL_Home_Rising_Unregister();
  v_EL_Home_Falling_Unregister();
  v_Control_Change_Mode(AXIS_EL, STATE_MANUAL);
  bool_el_going_home = false;
}

static void v_Limit_El_Handler(void)
{
  //v_Control_Change_Mode(STATE_STOP, STATE_STOP);
  //while (true); //Loop forever
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
  * @param  pu8_data: Pointer to Data
  * @param  u32_Data_Cnt: Number of Data in bytes
  * @retval true if successful and vice versa
  */
bool bool_None_Handler(uint8_t u8_msg_id, uint8_t *pu8_payload, uint32_t u32_payload_cnt)
{
  
  return true;
}

bool bool_Home_Handler(uint8_t u8_msg_id, uint8_t *pu8_payload, uint32_t u32_payload_cnt)
{
  uint8_t au8_respond_payload[MAX_SHORT_RES_PAYLOAD_LEN];
  
  if (*pu8_payload != 0x03) return false; //must be set 2 axis
  
  v_Control_Change_Mode(AXIS_BOTH, STATE_HOME);
  
  au8_respond_payload[0] = *pu8_payload;
  au8_respond_payload[1] = 0x00; //Ok
  v_Send_Response(u8_msg_id, au8_respond_payload, 2);
  return true;
}

bool bool_Stop_Handler(uint8_t u8_msg_id, uint8_t *pu8_payload, uint32_t u32_payload_cnt)
{
  uint8_t au8_respond_payload[MAX_SHORT_RES_PAYLOAD_LEN];
  
  if (*pu8_payload != 0x03) return false; //must be set 2 axis
  
  v_Control_Change_Mode(AXIS_BOTH, STATE_STOP);
  
  au8_respond_payload[0] = *pu8_payload;
  au8_respond_payload[1] = 0x00; //Ok
  v_Send_Response(u8_msg_id, au8_respond_payload, 2);
  return true;
}

bool bool_Emergency_Stop_Handler(uint8_t u8_msg_id, uint8_t *pu8_payload, uint32_t u32_payload_cnt)
{
  uint8_t au8_respond_payload[MAX_SHORT_RES_PAYLOAD_LEN];
  
  v_Limit_El_Handler();
  
  au8_respond_payload[0] = *pu8_payload;
  au8_respond_payload[1] = 0x00; //Ok
  v_Send_Response(u8_msg_id, au8_respond_payload, 2);
  return true;
}

bool bool_Stabilizing_Mode_Handler(uint8_t u8_msg_id, uint8_t *pu8_payload, uint32_t u32_payload_cnt)
{
  uint8_t au8_respond_payload[MAX_SHORT_RES_PAYLOAD_LEN];
  
  if (*pu8_payload != 0x03) return false; //must be set 2 axis
  
  if (*(pu8_payload + 1) == 0x00)
    v_Control_Change_Mode(AXIS_BOTH, STATE_MANUAL);
  else if (*(pu8_payload + 1) == 0x02)
    v_Control_Change_Mode(AXIS_BOTH, STATE_POINTING);
  else if (*(pu8_payload + 1) == 0x01)
    v_Control_Change_Mode(AXIS_BOTH, STATE_TRACKING);
  
  au8_respond_payload[0] = *pu8_payload;
  au8_respond_payload[1] = 0x00; //Ok
  v_Send_Response(u8_msg_id, au8_respond_payload, 2);
  return true;
}

bool bool_Get_Mode_Handler(uint8_t u8_msg_id, uint8_t *pu8_payload, uint32_t u32_payload_cnt)
{
  uint8_t au8_respond_payload[MAX_SHORT_RES_PAYLOAD_LEN];
  
  au8_respond_payload[0] = *pu8_payload;
  
  switch (enum_az_state)
  {
    case STATE_HOME:
      au8_respond_payload[1] = 0x01;
      au8_respond_payload[2] = 0x00;
      break;
    case STATE_STOP:
      au8_respond_payload[1] = 0x02;
      au8_respond_payload[2] = 0x00;
      break;
    case STATE_MANUAL:
      au8_respond_payload[1] = 0x04;
      au8_respond_payload[2] = 0x00;
      break;
    case STATE_POINTING:
      au8_respond_payload[1] = 0x04;
      au8_respond_payload[2] = 0x02;
      break;
    case STATE_TRACKING:
      au8_respond_payload[1] = 0x04;
      au8_respond_payload[2] = 0x01;
      break;
    default:
      break;
  }
  
  v_Send_Response(u8_msg_id, au8_respond_payload, 3);
  
  return true;
}

bool bool_Set_Pos_Handler(uint8_t u8_msg_id, uint8_t *pu8_payload, uint32_t u32_payload_cnt)
{
  uint8_t au8_respond_payload[MAX_SHORT_RES_PAYLOAD_LEN];
  int32_t s32_pos_value;
  
  s32_pos_value = (*(pu8_payload + 1) << 24) & 0x0ff000000;
  s32_pos_value += (*(pu8_payload + 2) << 16) & 0x0ff0000;
  s32_pos_value += (*(pu8_payload + 3) << 8) & 0x0ff00;
  s32_pos_value += *(pu8_payload + 4) & 0x0ff;
  
  if (*pu8_payload == 0x01)
  {
    switch (enum_az_state)
    {
      case STATE_MANUAL:
        v_PID_Set_Setpoint(&stru_pid_az_manual, (float)s32_pos_value / 100.0f, 1);
        break;
      default:
        break;
    }
  }
  else if (*pu8_payload == 0x02)
  {
    switch (enum_el_state)
    {
      case STATE_MANUAL:
        v_PID_Set_Setpoint(&stru_pid_el_manual, (float)s32_pos_value / 100.0f, 1);
        break;
      default:
        break;
    }
  }
  
  au8_respond_payload[0] = *pu8_payload;
  au8_respond_payload[1] = 0x00; //Ok
  v_Send_Response(u8_msg_id, au8_respond_payload, 2);
  return true;
}

bool bool_Set_Vel_Handler(uint8_t u8_msg_id, uint8_t *pu8_payload, uint32_t u32_payload_cnt)
{
  uint8_t au8_respond_payload[MAX_SHORT_RES_PAYLOAD_LEN];
  int32_t s32_vel_value;
  float flt_vel_value;
  
  s32_vel_value = (*(pu8_payload + 1) << 24) & 0x0ff000000;
  s32_vel_value += (*(pu8_payload + 2) << 16) & 0x0ff0000;
  s32_vel_value += (*(pu8_payload + 3) << 8) & 0x0ff00;
  s32_vel_value += *(pu8_payload + 4) & 0x0ff;
  flt_vel_value = (float)s32_vel_value / POS_VEL_SCALE;
  
  if (*pu8_payload == 0x01)
  {
    switch (enum_az_state)
    {
      case STATE_MANUAL:
        if (flt_vel_value < 0) flt_vel_value = -flt_vel_value;
        v_PID_Set_Max_Setpoint_Step(&stru_pid_az_manual, flt_vel_value / 1000.0f);
        break;
      case STATE_TRACKING:
        flt_body_rate[YAW] = flt_vel_value * DEGREE_TO_MRAD;
        break;
      default:
        break;
    }
  }
  else if (*pu8_payload == 0x02)
  {
    switch (enum_el_state)
    {
      case STATE_MANUAL:
        if (flt_vel_value < 0) flt_vel_value = -flt_vel_value;
        v_PID_Set_Max_Setpoint_Step(&stru_pid_el_manual, flt_vel_value / 1000.0f);
        break;
      case STATE_TRACKING:
        flt_body_rate[PITCH] = flt_vel_value * DEGREE_TO_MRAD;
      default:
        break;
    }
  }
  else if (*pu8_payload == 0x03)
  {
    flt_body_rate[YAW] = flt_vel_value * DEGREE_TO_MRAD;
    
    s32_vel_value = (*(pu8_payload + 5) << 24) & 0x0ff000000;
    s32_vel_value += (*(pu8_payload + 6) << 16) & 0x0ff0000;
    s32_vel_value += (*(pu8_payload + 7) << 8) & 0x0ff00;
    s32_vel_value += *(pu8_payload + 8) & 0x0ff;
    flt_vel_value = (float)s32_vel_value / POS_VEL_SCALE;
    flt_body_rate[PITCH] = flt_vel_value * DEGREE_TO_MRAD;
  }
  
  au8_respond_payload[0] = *pu8_payload;
  au8_respond_payload[1] = 0x00; //Ok
  v_Send_Response(u8_msg_id, au8_respond_payload, 2);
  return true;
}

bool bool_Set_Pos_Vel_Handler(uint8_t u8_msg_id, uint8_t *pu8_payload, uint32_t u32_payload_cnt)
{
  uint8_t au8_respond_payload[MAX_SHORT_RES_PAYLOAD_LEN];
  
  au8_respond_payload[0] = *pu8_payload;
  au8_respond_payload[1] = 0x00; //Ok
  v_Send_Response(u8_msg_id, au8_respond_payload, 2);
  return true;
}

bool bool_Get_Pos_Handler(uint8_t u8_msg_id, uint8_t *pu8_payload, uint32_t u32_payload_cnt)
{
  uint8_t au8_respond_payload[MAX_SHORT_RES_PAYLOAD_LEN];
  int32_t i32_enc_angle;
  
  if (*pu8_payload == AXIS_AZ)
    i32_enc_angle = (int32_t)(flt_AZ_ENC_Get_Angle() * POS_VEL_SCALE);
    //i32_enc_angle = (int32_t)(359.999f * 100.0f);
  else if (*pu8_payload == AXIS_EL)
    i32_enc_angle = (int32_t)(flt_EL_ENC_Get_Angle() * POS_VEL_SCALE);
    //i32_enc_angle = (int32_t)(-230.12f * 100.0f);
  else return false;
  
  au8_respond_payload[0] = *pu8_payload;
  au8_respond_payload[1] = (i32_enc_angle >> 24) & 0x0ff;
  au8_respond_payload[2] = (i32_enc_angle >> 16) & 0x0ff;
  au8_respond_payload[3] = (i32_enc_angle >> 8) & 0x0ff;
  au8_respond_payload[4] = i32_enc_angle & 0x0ff;
  v_Send_Response(u8_msg_id, au8_respond_payload, 5);
  return true;
}

bool bool_Set_Kp_Handler(uint8_t u8_msg_id, uint8_t *pu8_payload, uint32_t u32_payload_cnt)
{
  uint8_t au8_respond_payload[MAX_SHORT_RES_PAYLOAD_LEN];
  STRU_PID_T *pstru_desired_pid;
  uint32_t u32_desired_value;
  
  if (*pu8_payload <= AXIS_INVALID_L) return false;
  if (*pu8_payload >= AXIS_BOTH) return false;
  if (*(pu8_payload + 1) <= PID_ID_INVALID_L) return false;
  if (*(pu8_payload + 1) >= PID_ID_INVALID_H) return false;
  
  pstru_desired_pid = apstru_pid[*pu8_payload][*(pu8_payload + 1)];
  
  if (pstru_desired_pid == 0) return false;
  
  u32_desired_value = (*(pu8_payload + 2) << 24) & 0x0ff000000;
  u32_desired_value += (*(pu8_payload + 3) << 16) & 0x0ff0000;
  u32_desired_value += (*(pu8_payload + 4) << 8) & 0x0ff00;
  u32_desired_value += *(pu8_payload + 5) & 0x0ff;
  
  v_PID_Set_Kp(pstru_desired_pid, (float)u32_desired_value / PARAMS_SCALE);
  
  au8_respond_payload[0] = *pu8_payload;
  au8_respond_payload[1] = 0x00; //Ok
  v_Send_Response(u8_msg_id, au8_respond_payload, 2);
  return true;
}

bool bool_Set_Ki_Handler(uint8_t u8_msg_id, uint8_t *pu8_payload, uint32_t u32_payload_cnt)
{
  uint8_t au8_respond_payload[MAX_SHORT_RES_PAYLOAD_LEN];
  STRU_PID_T *pstru_desired_pid;
  uint32_t u32_desired_value;
  
  if (*pu8_payload <= AXIS_INVALID_L) return false;
  if (*pu8_payload >= AXIS_BOTH) return false;
  if (*(pu8_payload + 1) <= PID_ID_INVALID_L) return false;
  if (*(pu8_payload + 1) >= PID_ID_INVALID_H) return false;
  
  pstru_desired_pid = apstru_pid[*pu8_payload][*(pu8_payload + 1)];
  
  if (pstru_desired_pid == 0) return false;
  
  u32_desired_value = (*(pu8_payload + 2) << 24) & 0x0ff000000;
  u32_desired_value += (*(pu8_payload + 3) << 16) & 0x0ff0000;
  u32_desired_value += (*(pu8_payload + 4) << 8) & 0x0ff00;
  u32_desired_value += *(pu8_payload + 5) & 0x0ff;
  
  v_PID_Set_Ki(pstru_desired_pid, (float)u32_desired_value / PARAMS_SCALE);
  
  au8_respond_payload[0] = *pu8_payload;
  au8_respond_payload[1] = 0x00; //Ok
  v_Send_Response(u8_msg_id, au8_respond_payload, 2);
  return true;
}

bool bool_Set_Kd_Handler(uint8_t u8_msg_id, uint8_t *pu8_payload, uint32_t u32_payload_cnt)
{
  uint8_t au8_respond_payload[MAX_SHORT_RES_PAYLOAD_LEN];
  STRU_PID_T *pstru_desired_pid;
  uint32_t u32_desired_value;
  
  if (*pu8_payload <= AXIS_INVALID_L) return false;
  if (*pu8_payload >= AXIS_BOTH) return false;
  if (*(pu8_payload + 1) <= PID_ID_INVALID_L) return false;
  if (*(pu8_payload + 1) >= PID_ID_INVALID_H) return false;
  
  pstru_desired_pid = apstru_pid[*pu8_payload][*(pu8_payload + 1)];
  
  if (pstru_desired_pid == 0) return false;
  
  u32_desired_value = (*(pu8_payload + 2) << 24) & 0x0ff000000;
  u32_desired_value += (*(pu8_payload + 3) << 16) & 0x0ff0000;
  u32_desired_value += (*(pu8_payload + 4) << 8) & 0x0ff00;
  u32_desired_value += *(pu8_payload + 5) & 0x0ff;
  
  v_PID_Set_Kd(pstru_desired_pid, (float)u32_desired_value / PARAMS_SCALE);
  
  au8_respond_payload[0] = *pu8_payload;
  au8_respond_payload[1] = 0x00; //Ok
  v_Send_Response(u8_msg_id, au8_respond_payload, 2);
  return true;
}

bool bool_Set_Kff1_Handler(uint8_t u8_msg_id, uint8_t *pu8_payload, uint32_t u32_payload_cnt)
{
  uint8_t au8_respond_payload[MAX_SHORT_RES_PAYLOAD_LEN];
  STRU_PID_T *pstru_desired_pid;
  uint32_t u32_desired_value;
  
  if (*pu8_payload <= AXIS_INVALID_L) return false;
  if (*pu8_payload >= AXIS_BOTH) return false;
  if (*(pu8_payload + 1) <= PID_ID_INVALID_L) return false;
  if (*(pu8_payload + 1) >= PID_ID_INVALID_H) return false;
  
  pstru_desired_pid = apstru_pid[*pu8_payload][*(pu8_payload + 1)];
  
  if (pstru_desired_pid == 0) return false;
  
  u32_desired_value = (*(pu8_payload + 2) << 24) & 0x0ff000000;
  u32_desired_value += (*(pu8_payload + 3) << 16) & 0x0ff0000;
  u32_desired_value += (*(pu8_payload + 4) << 8) & 0x0ff00;
  u32_desired_value += *(pu8_payload + 5) & 0x0ff;
  
  v_PID_Set_Kff1(pstru_desired_pid, (float)u32_desired_value / PARAMS_SCALE);
  
  au8_respond_payload[0] = *pu8_payload;
  au8_respond_payload[1] = 0x00; //Ok
  v_Send_Response(u8_msg_id, au8_respond_payload, 2);
  return true;
}

bool bool_Set_Kff2_Handler(uint8_t u8_msg_id, uint8_t *pu8_payload, uint32_t u32_payload_cnt)
{
  uint8_t au8_respond_payload[MAX_SHORT_RES_PAYLOAD_LEN];
  STRU_PID_T *pstru_desired_pid;
  uint32_t u32_desired_value;
  
  if (*pu8_payload <= AXIS_INVALID_L) return false;
  if (*pu8_payload >= AXIS_BOTH) return false;
  if (*(pu8_payload + 1) <= PID_ID_INVALID_L) return false;
  if (*(pu8_payload + 1) >= PID_ID_INVALID_H) return false;
  
  pstru_desired_pid = apstru_pid[*pu8_payload][*(pu8_payload + 1)];
  
  if (pstru_desired_pid == 0) return false;
  
  u32_desired_value = (*(pu8_payload + 2) << 24) & 0x0ff000000;
  u32_desired_value += (*(pu8_payload + 3) << 16) & 0x0ff0000;
  u32_desired_value += (*(pu8_payload + 4) << 8) & 0x0ff00;
  u32_desired_value += *(pu8_payload + 5) & 0x0ff;
  
  v_PID_Set_Kff2(pstru_desired_pid, (float)u32_desired_value / PARAMS_SCALE);
  
  au8_respond_payload[0] = *pu8_payload;
  au8_respond_payload[1] = 0x00; //Ok
  v_Send_Response(u8_msg_id, au8_respond_payload, 2);
  return true;
}

bool bool_Get_Params_Handler(uint8_t u8_msg_id, uint8_t *pu8_payload, uint32_t u32_payload_cnt)
{
  uint8_t au8_respond_payload[MAX_LONG_RES_PAYLOAD_LEN];
  uint32_t u32_params, u32_cnt;
  STRU_PID_T *pstru_desired_pid;
  
  if (*pu8_payload <= AXIS_INVALID_L) return false;
  if (*pu8_payload >= AXIS_BOTH) return false;
  if (*(pu8_payload + 1) <= PID_ID_INVALID_L) return false;
  if (*(pu8_payload + 1) >= PID_ID_INVALID_H) return false;
  
  pstru_desired_pid = apstru_pid[*pu8_payload][*(pu8_payload + 1)];
  
  if (pstru_desired_pid == 0) return false;
  
  u32_cnt = 0;
  au8_respond_payload[u32_cnt++] = *pu8_payload;
  
  u32_params = (uint32_t)(flt_PID_Get_Kp(pstru_desired_pid) * PARAMS_SCALE);
  au8_respond_payload[u32_cnt++] = (u32_params >> 24) & 0x0ff;
  au8_respond_payload[u32_cnt++] = (u32_params >> 16) & 0x0ff;
  au8_respond_payload[u32_cnt++] = (u32_params >> 8) & 0x0ff;
  au8_respond_payload[u32_cnt++] = u32_params & 0x0ff;
  
  u32_params = (uint32_t)(flt_PID_Get_Ki(pstru_desired_pid) * PARAMS_SCALE);
  au8_respond_payload[u32_cnt++] = (u32_params >> 24) & 0x0ff;
  au8_respond_payload[u32_cnt++] = (u32_params >> 16) & 0x0ff;
  au8_respond_payload[u32_cnt++] = (u32_params >> 8) & 0x0ff;
  au8_respond_payload[u32_cnt++] = u32_params & 0x0ff;
  
  u32_params = (uint32_t)(flt_PID_Get_Kd(pstru_desired_pid) * PARAMS_SCALE);
  au8_respond_payload[u32_cnt++] = (u32_params >> 24) & 0x0ff;
  au8_respond_payload[u32_cnt++] = (u32_params >> 16) & 0x0ff;
  au8_respond_payload[u32_cnt++] = (u32_params >> 8) & 0x0ff;
  au8_respond_payload[u32_cnt++] = u32_params & 0x0ff;
  
  u32_params = (uint32_t)(flt_PID_Get_Kff1(pstru_desired_pid) * PARAMS_SCALE);
  au8_respond_payload[u32_cnt++] = (u32_params >> 24) & 0x0ff;
  au8_respond_payload[u32_cnt++] = (u32_params >> 16) & 0x0ff;
  au8_respond_payload[u32_cnt++] = (u32_params >> 8) & 0x0ff;
  au8_respond_payload[u32_cnt++] = u32_params & 0x0ff;
  
  u32_params = (uint32_t)(flt_PID_Get_Kff2(pstru_desired_pid) * PARAMS_SCALE);
  au8_respond_payload[u32_cnt++] = (u32_params >> 24) & 0x0ff;
  au8_respond_payload[u32_cnt++] = (u32_params >> 16) & 0x0ff;
  au8_respond_payload[u32_cnt++] = (u32_params >> 8) & 0x0ff;
  au8_respond_payload[u32_cnt++] = u32_params & 0x0ff;
  
  v_Send_Response(u8_msg_id, au8_respond_payload, u32_cnt);
  return true;
}

bool bool_Set_Active_Axis_Handler(uint8_t u8_msg_id, uint8_t *pu8_payload, uint32_t u32_payload_cnt)
{
  uint8_t au8_respond_payload[MAX_SHORT_RES_PAYLOAD_LEN];
  
  if (*pu8_payload == 0x01)
  {
    if (*(pu8_payload + 1) == 0x01)
      bool_active_az = true;
    else
      bool_active_az = false;
  }
  else if (*pu8_payload == 0x02)
  {
    if (*(pu8_payload + 1) == 0x01)
      bool_active_el = true;
    else
      bool_active_el = false;
  }
  
  au8_respond_payload[0] = *pu8_payload;
  au8_respond_payload[1] = 0x00; //Ok
  v_Send_Response(u8_msg_id, au8_respond_payload, 2);
  return true;
}

bool bool_Get_Active_Axis_Handler(uint8_t u8_msg_id, uint8_t *pu8_payload, uint32_t u32_payload_cnt)
{
  uint8_t au8_respond_payload[MAX_SHORT_RES_PAYLOAD_LEN];
  
  au8_respond_payload[0] = *pu8_payload;
  
  if (*pu8_payload == 0x01)
  {
    if (bool_active_az == true)
      au8_respond_payload[1] = 0x01;
    else
      au8_respond_payload[1] = 0x00;
  }
  else if (*pu8_payload == 0x02)
  {
    if (bool_active_el == true)
      au8_respond_payload[1] = 0x01;
    else
      au8_respond_payload[1] = 0x00;
  }
  
  v_Send_Response(u8_msg_id, au8_respond_payload, 2);
  return true;
}

bool bool_Send_Image_Data_Handler(uint8_t u8_msg_id, uint8_t *pu8_payload, uint32_t u32_payload_cnt)
{
  uint8_t au8_respond_payload[MAX_SHORT_RES_PAYLOAD_LEN];
  int16_t s16_x_value, s16_y_value;
  float flt_factor;
  
  s16_x_value += (*(pu8_payload + 1) << 8) & 0x0ff00;
  s16_x_value += *(pu8_payload + 2) & 0x0ff;
  s16_y_value += (*(pu8_payload + 3) << 8) & 0x0ff00;
  s16_y_value += *(pu8_payload + 4) & 0x0ff;
  
  flt_factor = (flt_alpha * flt_lamda) / (flt_lamda * flt_lamda + s16_x_value * s16_x_value + s16_y_value * s16_y_value);
  flt_body_rate[PITCH] = flt_factor * s16_y_value - stru_Get_IMU_Data().flt_euler_x * s16_x_value;
  flt_body_rate[YAW] = flt_factor * s16_x_value - stru_Get_IMU_Data().flt_euler_y * s16_x_value;
  
  au8_respond_payload[0] = *pu8_payload;
  au8_respond_payload[1] = 0x00; //Ok
  
  v_Send_Response(u8_msg_id, au8_respond_payload, 2);
  return true;
}

static void v_Send_Response(uint8_t u8_msg_id, uint8_t *pu8_payload, uint32_t u32_payload_cnt)
{
  uint32_t u32_idx, u32_message_no_checksum_size;
  uint16_t u16_crc_check;
  static uint8_t au8_respond_message[MAX_RES_MESSAGE_LEN] = {0x47, 0x42, 0x01, 0x02, 0x00};
  
  /* Total Length */
  u32_message_no_checksum_size = 7 + u32_payload_cnt;
  
  /* Length */
  au8_respond_message[5] = 1 + u32_payload_cnt + 2;
  
  /* MsgID */
  au8_respond_message[6] = u8_msg_id;
  
  /* Payload */
  for (u32_idx = 0; u32_idx < u32_payload_cnt; u32_idx++)
  {
    au8_respond_message[7 + u32_idx] = *(pu8_payload + u32_idx);
  }
  
  /* Checksum */
  u16_crc_check = 0;
  for (u32_idx = 0; u32_idx < u32_message_no_checksum_size; u32_idx++)
  {
    u16_crc_check += au8_respond_message[u32_idx];
  }
  u16_crc_check = ~u16_crc_check;
  
  au8_respond_message[u32_idx++] = (u16_crc_check >> 8) & 0x0ff;
  au8_respond_message[u32_idx++] = u16_crc_check & 0x0ff;
  
  bool_CMD_Send(au8_respond_message, u32_idx);
}
/**
  * @}
  */

/** @defgroup Send BLDC Speed
 *  @brief    ...
 *
 @verbatim
 ===============================================================================
                            ##### Send BLDC Speed #####
 ===============================================================================  

 @endverbatim
  * @{
  */
static void v_Send_BLDC_Speed(ENUM_AXIS_T enum_axis, int16_t s16_az_speed, int16_t s16_el_speed)
{
  uint32_t u32_idx, u32_message_no_checksum_size;
  uint16_t u16_crc_check;
  static uint8_t au8_respond_message[MAX_RES_MESSAGE_LEN] = {0x47, 0x42, 0x03, 0x02, 0x00};
  
  /* Total Length */
  u32_message_no_checksum_size = 12;
  
  /* Length */
  au8_respond_message[5] = 8;
  
  /* MsgID */
  au8_respond_message[6] = 0x13;
  
  /* Payload */
  au8_respond_message[7] = enum_axis;
  au8_respond_message[8] = (s16_az_speed >> 8) & 0x0ff;
  au8_respond_message[9] = s16_az_speed & 0x0ff;
  au8_respond_message[10] = (s16_el_speed >> 8) & 0x0ff;
  au8_respond_message[11] = s16_el_speed & 0x0ff;
  
  /* Checksum */
  u16_crc_check = 0;
  for (u32_idx = 0; u32_idx < u32_message_no_checksum_size; u32_idx++)
  {
    u16_crc_check += au8_respond_message[u32_idx];
  }
  u16_crc_check = ~u16_crc_check;
  
  au8_respond_message[u32_idx++] = (u16_crc_check >> 8) & 0x0ff;
  au8_respond_message[u32_idx++] = u16_crc_check & 0x0ff;
  
  bool_RESV_Send(au8_respond_message, u32_idx);
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
  static uint8_t au8_tx_buff[DATA_TXBUFF_SIZE];
  uint32_t u32_cnt = 0;
  int32_t s32_temp = 0;
  
  au8_tx_buff[0] = 0x0a;
  u32_cnt = 1;
  
  s32_temp = (int32_t)(flt_body_rate[YAW] * 1000);
  v_Int_To_Str_N(s32_temp, &au8_tx_buff[u32_cnt], 7);
  u32_cnt += 7;
  au8_tx_buff[u32_cnt++] = ' ';
  
  s32_temp = (int32_t)(flt_body_rate[PITCH] * 1000);
  v_Int_To_Str_N(s32_temp, &au8_tx_buff[u32_cnt], 7);
  u32_cnt += 7;
  au8_tx_buff[u32_cnt++] = ' ';
  
  s32_temp = (int32_t)s16_az_speed;
  v_Int_To_Str_N(s32_temp, &au8_tx_buff[u32_cnt], 5);
  u32_cnt += 5;
  au8_tx_buff[u32_cnt++] = ' ';
  
  s32_temp = (int32_t)s16_el_speed;
  v_Int_To_Str_N(s32_temp, &au8_tx_buff[u32_cnt], 5);
  u32_cnt += 5;
  au8_tx_buff[u32_cnt++] = ' ';
  
  s32_temp = (int32_t)(stru_Get_IMU_Data().flt_gyro_x / IMU_SCALE_GYRO_UNIT);
  v_Int_To_Str_N(s32_temp, &au8_tx_buff[u32_cnt], 6);
  u32_cnt += 6;
  au8_tx_buff[u32_cnt++] = ' ';
  
  s32_temp = (int32_t)(stru_Get_IMU_Data().flt_gyro_y / IMU_SCALE_GYRO_UNIT);
  v_Int_To_Str_N(s32_temp, &au8_tx_buff[u32_cnt], 6);
  u32_cnt += 6;
  au8_tx_buff[u32_cnt++] = ' ';
  
  s32_temp = (int32_t)(stru_Get_IMU_Data().flt_gyro_z / IMU_SCALE_GYRO_UNIT);
  v_Int_To_Str_N(s32_temp, &au8_tx_buff[u32_cnt], 6);
  u32_cnt += 6;
  au8_tx_buff[u32_cnt++] = ' ';
  
  s32_temp = (int32_t)(stru_Get_IMU_Data().flt_euler_x / IMU_SCALE_EULER_UNIT);
  v_Int_To_Str_N(s32_temp, &au8_tx_buff[u32_cnt], 7);
  u32_cnt += 7;
  au8_tx_buff[u32_cnt++] = ' ';
  
  s32_temp = (int32_t)(stru_Get_IMU_Data().flt_euler_y / IMU_SCALE_EULER_UNIT);
  v_Int_To_Str_N(s32_temp, &au8_tx_buff[u32_cnt], 7);
  u32_cnt += 7;
  au8_tx_buff[u32_cnt++] = ' ';
  
  s32_temp = (int32_t)(stru_Get_IMU_Data().flt_euler_z / IMU_SCALE_EULER_UNIT);
  v_Int_To_Str_N(s32_temp, &au8_tx_buff[u32_cnt], 7);
  u32_cnt += 7;
  au8_tx_buff[u32_cnt++] = ' ';
  
  au8_tx_buff[u32_cnt++] = 0x0d;
  
  bool_DATA_Send(au8_tx_buff, u32_cnt);
#endif

#ifdef DATA_GYRO_FILTER
  static uint8_t au8_tx_buff[DATA_TXBUFF_SIZE];
  uint32_t u32_cnt = 0;
  int32_t s32_temp = 0;
  
  au8_tx_buff[0] = 0x0a;
  u32_cnt = 1;
  
  s32_temp = (int32_t)(stru_Get_IMU_Data().flt_gyro_x / IMU_SCALE_GYRO_UNIT);
  v_Int_To_Str_N(s32_temp, &au8_tx_buff[u32_cnt], 6);
  u32_cnt += 6;
  au8_tx_buff[u32_cnt++] = ' ';
  
  s32_temp = (int32_t)(stru_Get_IMU_Data().flt_gyro_y / IMU_SCALE_GYRO_UNIT);
  v_Int_To_Str_N(s32_temp, &au8_tx_buff[u32_cnt], 6);
  u32_cnt += 6;
  au8_tx_buff[u32_cnt++] = ' ';
  
  s32_temp = (int32_t)(stru_Get_IMU_Data().flt_gyro_z / IMU_SCALE_GYRO_UNIT);
  v_Int_To_Str_N(s32_temp, &au8_tx_buff[u32_cnt], 6);
  u32_cnt += 6;
  au8_tx_buff[u32_cnt++] = ' ';
  
  s32_temp = (int32_t)(flt_filtered_pitch_gyro / IMU_SCALE_GYRO_UNIT);
  v_Int_To_Str_N(s32_temp, &au8_tx_buff[u32_cnt], 6);
  u32_cnt += 6;
  au8_tx_buff[u32_cnt++] = ' ';
  
  s32_temp = (int32_t)(flt_filtered_yaw_gyro / IMU_SCALE_GYRO_UNIT);
  v_Int_To_Str_N(s32_temp, &au8_tx_buff[u32_cnt], 6);
  u32_cnt += 6;
  au8_tx_buff[u32_cnt++] = ' ';
  
  au8_tx_buff[u32_cnt++] = 0x0d;
  
  bool_DATA_Send(au8_tx_buff, u32_cnt);
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
  bool_Params_Save(PARAMS_CODE_VERSION, au8_code_version);
  bool_Params_Save(PARAMS_PID_AZ_MANUAL_POS, (uint8_t *)&stru_pid_az_manual);
  bool_Params_Save(PARAMS_PID_EL_MANUAL_POS, (uint8_t *)&stru_pid_el_manual);
}

/**
  * @brief  load all params
  * @note   
  * @param  none
  * @retval none
  */
void v_Params_Load_All(void)
{
  uint8_t au8_loaded_version[2] = {0, 0};
  
  bool_Params_Load(PARAMS_CODE_VERSION, au8_loaded_version);
  if ((au8_loaded_version[0] != au8_code_version[0]) || (au8_loaded_version[1] != au8_code_version[1]))
  {
    v_Params_Save_Default();
  }
  
  bool_Params_Load(PARAMS_PID_AZ_MANUAL_POS, (uint8_t *)&stru_pid_az_manual);
  bool_Params_Load(PARAMS_PID_EL_MANUAL_POS, (uint8_t *)&stru_pid_el_manual);
}
/**
  * @}
  */

/*********************************END OF FILE**********************************/
