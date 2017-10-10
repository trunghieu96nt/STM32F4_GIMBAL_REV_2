/**
  ******************************************************************************
  * @file    motor_driver.c
  * @author  Vu Trung Hieu
  * @version V2.0
  * @date    05-September-2017
  * @brief   This file contains functions for PWM and ENC
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
#include "stm32f4xx.h"
#include "motor_driver.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define IS_TIMER_32BIT(x) ((x == TIM2) || (x == TIM5))
#define IS_APB2_TIMER(x) ((x == TIM1) || (x == TIM8) || (x == TIM9) \
   || (x == TIM10) || (x == TIM11))

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static volatile int32_t s32_Enc0_Cur = 0;
static volatile int32_t s32_Enc0_Dp = 0;
static volatile int32_t s32_Enc0_P0 = 0, s32_Enc0_P1 = 0;

static volatile int32_t s32_Enc1_Cur = 0;
static volatile int32_t s32_Enc1_Dp = 0;
static volatile int32_t s32_Enc1_P0 = 0, s32_Enc1_P1 = 0;

static volatile int32_t s32_Enc2_Cur = 0;
static volatile int32_t s32_Enc2_Dp = 0;
static volatile int32_t s32_Enc2_P0 = 0, s32_Enc2_P1 = 0;

/* Private function prototypes -----------------------------------------------*/
static void v_ENC0_Init(void);
static void v_ENC1_Init(void);
static void v_ENC2_Init(void);

static void v_PWM0_Init(void);
static void v_PWM1_Init(void);

static uint32_t u32_Timer_Get_Clock(TIM_TypeDef* TIMx);

/* Private functions ---------------------------------------------------------*/

/** @defgroup Init Function
 *  @brief   ...
 *
 @verbatim
 ===============================================================================
                      ##### Motor Driver Init Function #####
 ===============================================================================  

 @endverbatim
  * @{
  */

/**
  * @brief  Motor Driver Init Function
  * @note   Call function first if using this driver
  * @param  none
  * @retval none
  */
void v_Motor_Init(void)
{
  v_ENC0_Init();
  v_ENC1_Init();
  v_ENC2_Init();
  
  v_PWM0_Init();
  v_PWM1_Init();
}

/**
  * @}
  */

/** @defgroup Encoder Function
 *  @brief   ...
 *
 @verbatim
 ===============================================================================
                           ##### Encoder Function #####
 ===============================================================================  

 @endverbatim
  * @{
  */

/**
  * @brief  Encoder Init Function
  * @note   ...
  * @param  none
  * @retval none
  */
static void v_ENC0_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  
  /* TIM clock enable */
  if (IS_APB2_TIMER(ENC0_TIM_POS))
  {
    RCC_APB2PeriphClockCmd(ENC0_TIM_CLK_POS, ENABLE);
  }
  else
  {
    RCC_APB1PeriphClockCmd(ENC0_TIM_CLK_POS, ENABLE);    
  }
  
  RCC_AHB1PeriphClockCmd(ENC0_PERIPH_PORT_POS, ENABLE);
  /* TIM channel1,2 configuration */
  GPIO_InitStructure.GPIO_Pin   = ENC0_A_POS | ENC0_B_POS;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
  GPIO_Init(ENC0_PORT_POS, &GPIO_InitStructure);

  GPIO_PinAFConfig(ENC0_PORT_POS, ENC0_A_SOURCE_POS, ENC0_AF_POS);
  GPIO_PinAFConfig(ENC0_PORT_POS, ENC0_B_SOURCE_POS, ENC0_AF_POS);

  /* Initialise encoder interface */
  TIM_EncoderInterfaceConfig(ENC0_TIM_POS, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
  
  /* TIM enable counter */
  TIM_Cmd(ENC0_TIM_POS, ENABLE);  
  ENC0_TIM_POS->CNT = 0;
}

static void v_ENC1_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  
  /* TIM clock enable */
  if (IS_APB2_TIMER(ENC1_TIM_POS))
  {
    RCC_APB2PeriphClockCmd(ENC1_TIM_CLK_POS, ENABLE);    
  }
  else
  {
    RCC_APB1PeriphClockCmd(ENC1_TIM_CLK_POS, ENABLE);
  }
  
  RCC_AHB1PeriphClockCmd(ENC1_PERIPH_PORT_POS, ENABLE);
  /* TIM channel1,2 configuration */
  GPIO_InitStructure.GPIO_Pin   = ENC1_A_POS | ENC1_B_POS;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
  GPIO_Init(ENC1_PORT_POS, &GPIO_InitStructure);

  GPIO_PinAFConfig(ENC1_PORT_POS, ENC1_A_SOURCE_POS, ENC1_AF_POS);
  GPIO_PinAFConfig(ENC1_PORT_POS, ENC1_B_SOURCE_POS, ENC1_AF_POS);

  /* Initialise encoder interface */
  TIM_EncoderInterfaceConfig(ENC1_TIM_POS, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
  
  TIM_Cmd(ENC1_TIM_POS, ENABLE);  
  ENC1_TIM_POS->CNT = 0;
}

static void v_ENC2_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  /* TIM clock enable */
  if (IS_APB2_TIMER(ENC2_TIM_POS))
  {
    RCC_APB2PeriphClockCmd(ENC2_TIM_CLK_POS, ENABLE);    
  }
  else
  {
    RCC_APB1PeriphClockCmd(ENC2_TIM_CLK_POS, ENABLE);
  }
  
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
  
  /* TIM channel 1 configuration */
  RCC_AHB1PeriphClockCmd(ENC2_PERIPH_PORT_1_POS, ENABLE);
  GPIO_InitStructure.GPIO_Pin   = ENC2_A_POS;
  GPIO_Init(ENC2_PORT_1_POS, &GPIO_InitStructure);
  GPIO_PinAFConfig(ENC2_PORT_1_POS, ENC2_A_SOURCE_POS, ENC2_AF_POS);
  
  /* TIM channel 2 configuration */
  RCC_AHB1PeriphClockCmd(ENC2_PERIPH_PORT_2_POS, ENABLE);
  GPIO_InitStructure.GPIO_Pin   = ENC2_B_POS;
  GPIO_Init(ENC2_PORT_2_POS, &GPIO_InitStructure);
  GPIO_PinAFConfig(ENC2_PORT_2_POS, ENC2_B_SOURCE_POS, ENC2_AF_POS);

  /* Initialise encoder interface */
  TIM_EncoderInterfaceConfig(ENC2_TIM_POS, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
  
  TIM_Cmd(ENC2_TIM_POS, ENABLE);  
  ENC2_TIM_POS->CNT = 0;
}

/**
  * @brief  Get Encoder Position
  * @note   ...
  * @param  none
  * @retval Current Encoder Position
  */
int32_t s32_ENC0_Get_Pos(void)
{
  s32_Enc0_P0 = (int32_t)ENC0_TIM_POS->CNT;

  if (!IS_TIMER_32BIT(ENC0_TIM_POS))
  {
    s32_Enc0_Dp = s32_Enc0_P0 - s32_Enc0_P1;
    if (s32_Enc0_Dp > 32768)
    {
      s32_Enc0_Dp -= 65536;
    }
    else if (s32_Enc0_Dp < -32768)
    {
      s32_Enc0_Dp += 65536;
    }
    s32_Enc0_P1 = s32_Enc0_P0;
    s32_Enc0_Cur += s32_Enc0_Dp;
  }
  else
  {
    s32_Enc0_Cur = s32_Enc0_P0;
  }
  return s32_Enc0_Cur;
}

int32_t s32_ENC1_Get_Pos(void)
{
  s32_Enc1_P0 = (int32_t)ENC1_TIM_POS->CNT;

  if (!IS_TIMER_32BIT(ENC1_TIM_POS))
  {
    s32_Enc1_Dp = s32_Enc1_P0 - s32_Enc1_P1;
    if (s32_Enc1_Dp > 32768)
    {
      s32_Enc1_Dp -= 65536;
    }
    else if (s32_Enc1_Dp < -32768)
    {
      s32_Enc1_Dp += 65536;
    }
    s32_Enc1_P1 = s32_Enc1_P0;
    s32_Enc1_Cur += s32_Enc1_Dp;
  }
  else
  {
    s32_Enc1_Cur = s32_Enc1_P0;
  }
  return s32_Enc1_Cur;
}

int32_t s32_ENC2_Get_Pos(void)
{
  s32_Enc2_P0 = (int32_t)ENC2_TIM_POS->CNT;

  if (!IS_TIMER_32BIT(ENC2_TIM_POS))
  {
    s32_Enc2_Dp = s32_Enc2_P0 - s32_Enc2_P1;
    if (s32_Enc2_Dp > 32768)
    {
      s32_Enc2_Dp -= 65536;
    }
    else if (s32_Enc2_Dp < -32768)
    {
      s32_Enc2_Dp += 65536;
    }
    s32_Enc2_P1 = s32_Enc2_P0;
    s32_Enc2_Cur += s32_Enc2_Dp;
  }
  else
  {
    s32_Enc2_Cur = s32_Enc2_P0;
  }
  return s32_Enc2_Cur;
}

/**
  * @brief  Get Angle From Encoder
  * @note   ...
  * @param  None
  * @retval Current Encoder Angle
  */
float flt_ENC0_Get_Angle(void)
{
  return (float)s32_ENC0_Get_Pos() * ENC0_ANGLE_SCALE;
}

float flt_ENC1_Get_Angle(void)
{
  return (float)s32_ENC1_Get_Pos() * ENC1_ANGLE_SCALE;
}

float flt_ENC2_Get_Angle(void)
{
  return (float)s32_ENC2_Get_Pos() * ENC2_ANGLE_SCALE;
}

/**
  * @brief  Reset Encoder Position
  * @note   ...
  * @param  none
  * @retval none
  */
void v_ENC0_Reset(void)
{
  s32_ENC0_Get_Pos(); //for save pre variable
  s32_Enc0_Cur = 0;
}

void v_ENC1_Reset(void)
{
  s32_ENC1_Get_Pos(); //for save pre variable
  s32_Enc1_Cur = 0;
}

void v_ENC2_Reset(void)
{
  s32_ENC2_Get_Pos(); //for save pre variable
  s32_Enc2_Cur = 0;
}

/**
  * @}
  */

/**
  * @}
  */

/** @defgroup Encoder Function
 *  @brief   ...
 *
 @verbatim
 ===============================================================================
                              ##### PWM Function #####
 ===============================================================================  

 @endverbatim
  * @{
  */

/**
  * @brief  PWM Init Function
  * @note   ...
  * @param  none
  * @retval none
  */
static void v_PWM0_Init(void)
{
  GPIO_InitTypeDef          GPIO_InitStructure;
  TIM_TimeBaseInitTypeDef   TIM_TimeBaseStructure;
  TIM_OCInitTypeDef         TIM_OCInitStructure;

  /* TIM clock enable */
  if (IS_APB2_TIMER(PWM0_TIM))
  {
    RCC_APB2PeriphClockCmd(PWM0_TIM_CLK, ENABLE);
  }
  else
  {
    RCC_APB1PeriphClockCmd(PWM0_TIM_CLK, ENABLE);
  }
  
  /* GPIO clock enable */
  RCC_AHB1PeriphClockCmd(PWM0_PULSE_PORT_CLK, ENABLE);
  RCC_AHB1PeriphClockCmd(PWM0_DIR_PORT_CLK, ENABLE); 

  /* Pulse pin configuration */
  GPIO_InitStructure.GPIO_Pin = PWM0_PULSE;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN ;
  GPIO_Init(PWM0_PULSE_PORT, &GPIO_InitStructure); 
  GPIO_PinAFConfig(PWM0_PULSE_PORT, PWM0_PULSE_SOURCE, PWM0_PULSE_AF);
  
  /* DIR, EN pins configuration */
  GPIO_InitStructure.GPIO_Pin = PWM0_DIR;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(PWM0_DIR_PORT, &GPIO_InitStructure); 

#if PWM0_USE_EN_PIN
  RCC_AHB1PeriphClockCmd(PWM0_EN_PORT_CLK, ENABLE); 
  GPIO_InitStructure.GPIO_Pin = PWM0_EN;
  GPIO_Init(PWM0_EN_PORT, &GPIO_InitStructure);
#endif

  /* TIM CH1 configuration */
  TIM_TimeBaseStructure.TIM_Period = PWM0_PERIOD - 1;
  TIM_TimeBaseStructure.TIM_Prescaler = PWM0_PRESCALER - 1;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(PWM0_TIM, &TIM_TimeBaseStructure);

  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 0;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  TIM_OC1Init(PWM0_TIM, &TIM_OCInitStructure);
  TIM_OC1PreloadConfig(PWM0_TIM, TIM_OCPreload_Enable);

  TIM_Cmd(PWM0_TIM, ENABLE);
}

static void v_PWM1_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
  TIM_OCInitTypeDef TIM_OCInitStructure;

  /* TIM clock enable */
  if (IS_APB2_TIMER(PWM1_TIM))
  {
    RCC_APB2PeriphClockCmd(PWM1_TIM_CLK, ENABLE);
  }
  else
  {
    RCC_APB1PeriphClockCmd(PWM1_TIM_CLK, ENABLE);
  }
  
  /* GPIO clock enable */
  RCC_AHB1PeriphClockCmd(PWM1_PULSE_PORT_CLK, ENABLE);
  RCC_AHB1PeriphClockCmd(PWM1_DIR_PORT_CLK, ENABLE);
  
  /* Pulse pin configuration */
  GPIO_InitStructure.GPIO_Pin = PWM1_PULSE;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN ;
  GPIO_Init(PWM1_PULSE_PORT, &GPIO_InitStructure); 
  GPIO_PinAFConfig(PWM1_PULSE_PORT, PWM1_PULSE_SOURCE, PWM1_PULSE_AF);
  
  /* DIR, EN pins configuration */
  GPIO_InitStructure.GPIO_Pin = PWM1_DIR;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(PWM1_DIR_PORT, &GPIO_InitStructure); 
  
#if PWM1_USE_EN_PIN
  RCC_AHB1PeriphClockCmd(PWM1_EN_PORT_CLK, ENABLE);
  GPIO_InitStructure.GPIO_Pin = PWM1_EN;
  GPIO_Init(PWM1_EN_PORT, &GPIO_InitStructure);
#endif

  /* TIM CH1 configuration */
  TIM_TimeBaseStructure.TIM_Period = PWM1_PERIOD-1;
  TIM_TimeBaseStructure.TIM_Prescaler = PWM1_PRESCALER - 1;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(PWM1_TIM, &TIM_TimeBaseStructure);

  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = PWM1_DUTY;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  TIM_OC1Init(PWM1_TIM, &TIM_OCInitStructure);
  TIM_OC1PreloadConfig(PWM1_TIM, TIM_OCPreload_Enable);

  TIM_Cmd(PWM1_TIM, ENABLE);
}

/**
  * @brief  Set frequency PWM
  * @note   ...
  * @param  u32_Frequency: Desired frequency
  * @retval none
  */
void v_PWM0_Set_Freq(uint32_t u32_Frequency)
{
  uint32_t u32_Period;
  u32_Period = u32_Timer_Get_Clock(PWM0_TIM) / (u32_Frequency * (PWM0_TIM->PSC + 1)) - 1;
  if ((!IS_TIMER_32BIT(PWM0_TIM)) && (u32_Period > 0xffff))
  {
    u32_Period = 0xffff;
  }
  PWM0_TIM->ARR = u32_Period;
}

void v_PWM1_Set_Freq(uint32_t u32_Frequency)
{
  uint32_t u32_Period;
  u32_Period = u32_Timer_Get_Clock(PWM1_TIM) / (u32_Frequency * (PWM1_TIM->PSC + 1)) - 1;
  if ((!IS_TIMER_32BIT(PWM1_TIM)) && (u32_Period > 0xffff))
  {
    u32_Period = 0xffff;
  }
  PWM1_TIM->ARR = u32_Period;
}

/**
  * @brief  Set Duty PWM
  * @note   ...
  * @param  s16_Duty: signed duty cycle of PWM
  * @retval none
  */
void v_PWM0_Set_Duty(int16_t s16_Duty)
{
  if (s16_Duty<-900)
    s16_Duty = -900;
  else if (s16_Duty>900)
    s16_Duty = 900;
  // DIR = 1: OPEN (FORWARD); DIR = 0: CLOSE (BACKWARD)
  if (s16_Duty == 0)
  {
#if PWM0_USE_EN_PIN
    PWM0_EN_PORT->BSRRH = PWM0_EN;// disable
#endif
  }
  else if (s16_Duty > 0)
  {
#if PWM0_USE_EN_PIN
    PWM0_EN_PORT->BSRRL = PWM0_EN;// enable
#endif
    PWM0_DIR_PORT->BSRRL = PWM0_DIR; //1, forward
  }
  else //(s16_Duty < 0)
  {
#if PWM0_USE_EN_PIN    
    PWM0_EN_PORT->BSRRL = PWM0_EN; // enable
#endif
    PWM0_DIR_PORT->BSRRH = PWM0_DIR; //0,  backward
    s16_Duty = -s16_Duty;
  }
  s16_Duty = ((PWM0_TIM->ARR + 1)* s16_Duty + 500)/1000;
  PWM0_TIM->CCR1 = (uint32_t)s16_Duty;
}

void v_PWM1_Set_Duty(int16_t s16_Duty)
{
  if (s16_Duty<-900)
    s16_Duty = -900;
  else if (s16_Duty>900)
    s16_Duty = 900;
  // DIR = 1: OPEN (FORWARD); DIR = 0: CLOSE (BACKWARD)
  if (s16_Duty == 0)
  {
#if PWM1_USE_EN_PIN
    PWM1_EN_PORT->BSRRH = PWM1_EN; // disable
#endif
  }
  else if (s16_Duty > 0)
  {
#if PWM1_USE_EN_PIN
    PWM1_EN_PORT->BSRRL = PWM1_EN; // enable
#endif
    PWM1_DIR_PORT->BSRRL = PWM1_DIR; //0,  backward
  }
  else // (s16_Duty < 0)
  {
#if PWM1_USE_EN_PIN
    PWM1_EN_PORT->BSRRL = PWM1_EN; // enable
#endif
    PWM1_DIR_PORT->BSRRH = PWM1_DIR; //1, forward
    s16_Duty = -s16_Duty;
  }
  s16_Duty = ((PWM1_TIM->ARR + 1) * s16_Duty + 500) / 1000;
  PWM1_TIM->CCR1 = (uint32_t)s16_Duty;
}

/**
  * @brief  Get timer clock
  * @note   ...
  * @param  Timer TIMx
  * @retval Clock of timer
  */
static uint32_t u32_Timer_Get_Clock(TIM_TypeDef* TIMx)
{
  uint32_t u32_Clock;
  RCC_ClocksTypeDef RCC_Clocks;  
  /* Get system clocks */
  RCC_GetClocksFreq(&RCC_Clocks);
  if(IS_APB2_TIMER(TIMx))
  {
    if (RCC_Clocks.HCLK_Frequency == RCC_Clocks.PCLK2_Frequency)//prescaler 1
      u32_Clock = RCC_Clocks.PCLK2_Frequency;
    else
      u32_Clock = 2 * RCC_Clocks.PCLK2_Frequency;
  }
  else//APB1 timer
  {
    if (RCC_Clocks.HCLK_Frequency == RCC_Clocks.PCLK1_Frequency)//prescaler 1
      u32_Clock = RCC_Clocks.PCLK1_Frequency;
    else
      u32_Clock = 2 * RCC_Clocks.PCLK1_Frequency;
  }
  return u32_Clock;
}
/**
  * @}
  */

/** @defgroup Gimbal Port Functions
 *  @brief   ...
 *
 @verbatim
 ===============================================================================
                        ##### Gimbal Port Functions #####
 ===============================================================================  

 @endverbatim
  * @{
  */
int32_t s32_EL_ENC_Get_Pos(void)
{
  return s32_ENC1_Get_Pos();
}

int32_t s32_AZ_ENC_Get_Pos(void)
{
  return s32_ENC0_Get_Pos();
}

float flt_EL_ENC_Get_Angle(void)
{
  return flt_ENC1_Get_Angle();
}

float flt_AZ_ENC_Get_Angle(void)
{
  return flt_ENC0_Get_Angle();
}

void v_EL_ENC_Reset(void)
{
  v_ENC1_Reset();
}

void v_AZ_ENC_Reset(void)
{
  v_ENC0_Reset();
}

void v_EL_PWM_Set_Freq(uint32_t u32_Frequency)
{
  v_PWM1_Set_Freq(u32_Frequency);
}

void v_AZ_PWM_Set_Freq(uint32_t u32_Frequency)
{
  v_PWM0_Set_Freq(u32_Frequency);
}

void v_EL_PWM_Set_Duty(int16_t s16_Duty)
{
  v_PWM1_Set_Duty(s16_Duty);
}

void v_AZ_PWM_Set_Duty(int16_t s16_Duty)
{
  v_PWM0_Set_Duty(s16_Duty);
}
/**
  * @}
  */

/*********************************END OF FILE**********************************/
