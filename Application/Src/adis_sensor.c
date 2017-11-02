/**
  ******************************************************************************
  * @file    adis_sensor.c
  * @author  Vu Trung Hieu
  * @version V2.0
  * @date    08-September-2017
  * @brief   This file contains functions for adis sensor
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
#include "adis_sensor.h"
#include "string.h"
#include "stdlib.h"
#include "system_timetick.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static uint8_t au8_IMU_rx[IMU_RXBUFF_SIZE]= {0};
static STRU_IMU_DATA_T stru_IMU_data = {false}; //initial bool_available value.

/* Private function prototypes -----------------------------------------------*/
static bool bool_ADIS_Parse(uint8_t *pu8_IMU_frame);

/* Private functions ---------------------------------------------------------*/

/** @defgroup ADIS Initialization
 *  @brief   ...
 *
 @verbatim
 ===============================================================================
                          ##### ADIS Initialization #####
 ===============================================================================  

 @endverbatim
  * @{
  */

/**
  * @brief  Initialize Gimbal Adis
  * @note   DMA UART RX, use IT by define USE_IMU_RX_DMA_IT
  * @param  none
  * @retval none
  */
void v_ADIS_Init(void)
{
  USART_InitTypeDef     USART_InitStructure;
  GPIO_InitTypeDef      GPIO_InitStructure;
  DMA_InitTypeDef       DMA_InitStructure;
#if USE_IMU_RX_DMA_IT
  NVIC_InitTypeDef      NVIC_InitStructure;
#endif
  
  RCC_AHB1PeriphClockCmd(IMU_PORT_CLK, ENABLE);  
  GPIO_InitStructure.GPIO_Pin   = IMU_TX | IMU_RX;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
  GPIO_Init(IMU_PORT, &GPIO_InitStructure);
  
  GPIO_PinAFConfig(IMU_PORT, IMU_TX_SOURCE, IMU_AF);
  GPIO_PinAFConfig(IMU_PORT, IMU_RX_SOURCE, IMU_AF);

  RCC_APB1PeriphClockCmd(IMU_USART_CLK, ENABLE);
  
  USART_InitStructure.USART_BaudRate            = IMU_BAUDRATE;
  USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits            = USART_StopBits_1;
  USART_InitStructure.USART_Parity              = USART_Parity_No;
  USART_InitStructure.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_Init(IMU_USART, &USART_InitStructure);
  USART_Cmd(IMU_USART, ENABLE); 
  USART_ClearFlag(IMU_USART, USART_FLAG_TC);

  RCC_AHB1PeriphClockCmd(IMU_AHB_PERIPH_DMA, ENABLE);
  DMA_DeInit(IMU_RX_DMA_STREAM);  
  DMA_InitStructure.DMA_Channel            = IMU_RX_DMA_CHANNEL;
  DMA_InitStructure.DMA_DIR                = DMA_DIR_PeripheralToMemory;
  DMA_InitStructure.DMA_Memory0BaseAddr    = (uint32_t)&au8_IMU_rx[0];
  DMA_InitStructure.DMA_PeripheralBaseAddr = IMU_DATA_REG;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_Priority           = DMA_Priority_VeryHigh;
  DMA_InitStructure.DMA_FIFOMode           = DMA_FIFOMode_Disable; //direct mode
  DMA_InitStructure.DMA_FIFOThreshold      = DMA_FIFOThreshold_Full;
  DMA_InitStructure.DMA_MemoryInc          = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_Mode               = DMA_Mode_Circular; // avoid buffer overload error
  DMA_InitStructure.DMA_BufferSize         = IMU_RXBUFF_SIZE;
  DMA_InitStructure.DMA_MemoryBurst        = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_PeripheralBurst    = DMA_PeripheralBurst_Single;
  DMA_Init(IMU_RX_DMA_STREAM, &DMA_InitStructure);
  /* Enable  request */
  USART_DMACmd(IMU_USART, USART_DMAReq_Rx, ENABLE);  
  /* Enable DMA RX Channel */
  DMA_Cmd(IMU_RX_DMA_STREAM, ENABLE);
}

/**
  * @}
  */

/** @defgroup ADIS Function
 *  @brief   Read, Read with timeout
 *
 @verbatim
 ===============================================================================
                             ##### ADIS Function #####
 ===============================================================================  

 @endverbatim
  * @{
  */

/**
  * @brief  Read ADIS
  * @note   get raw IMU frame and call Gimbal_ADIS_Parse()
  *         Frame: _123_
  *         u32_idx_pre -> _
  *         u32_idx_cur -> 3
  * @param  none
  * @retval true if get correctly and vice versa
  */
bool bool_ADIS_Read(void)
{
  static uint32_t u32_idx_pre = IMU_RXBUFF_SIZE - 1;
  uint32_t u32_length, u32_idx_cur;
  int32_t s32_idx, s32_cnt;
  uint8_t *pu8_end_chr = NULL; //Can change to bool variable
  uint8_t au8_IMU_frame[IMU_FRAME_LEN + 1];
  
  if (IMU_RX_DMA_STREAM->NDTR == IMU_RXBUFF_SIZE) u32_idx_cur = IMU_RXBUFF_SIZE - 1;
  else u32_idx_cur = IMU_RXBUFF_SIZE - IMU_RX_DMA_STREAM->NDTR - 1;
  
  if (u32_idx_cur >= u32_idx_pre) u32_length = u32_idx_cur - u32_idx_pre;
  else u32_length = IMU_RXBUFF_SIZE - (u32_idx_pre - u32_idx_cur);
  
  /* Check enough lengh */
  if (u32_length < IMU_FRAME_LEN) return false;
  
  /* Search IMU_END_FRAME and Copy backward from au8_IMU_rx[u32_idx_cur] */
  s32_idx = u32_idx_cur; s32_cnt = IMU_FRAME_LEN - 2;
  au8_IMU_frame[IMU_FRAME_LEN] = 0;
  while (true)
  {
    if (pu8_end_chr == NULL)
    {
      if (*(au8_IMU_rx + s32_idx) == IMU_END_FRAME)
      {
        pu8_end_chr = au8_IMU_rx + s32_idx;
        au8_IMU_frame[IMU_FRAME_LEN - 1] = IMU_END_FRAME;
        u32_idx_cur = s32_idx; //Save End index
      }
    }
    else
    {
      au8_IMU_frame[s32_cnt] = *(au8_IMU_rx + s32_idx);
      if (au8_IMU_frame[s32_cnt] == IMU_START_FRAME) break;
      if (--s32_cnt < 0) return false; //Over Length
    }
    if (--s32_idx < 0) s32_idx = IMU_RXBUFF_SIZE - 1;
    if (s32_idx == u32_idx_pre) return false; //Not found
  }
  u32_idx_pre = u32_idx_cur;
  return bool_ADIS_Parse(au8_IMU_frame);
}

/**
  * @brief  parse raw IMU frame
  * @note   ...
  * @param  pu8_IMU_frame: pointer to IMU frame
  * @retval true if parse correctly and vice versa
  */
static bool bool_ADIS_Parse(uint8_t *pu8_IMU_frame)
{
  uint32_t u32_idx = 0;
  uint8_t *pu8_end = NULL, *pu8_start = pu8_IMU_frame + 2;
  
  if(strlen((char *)pu8_IMU_frame) != IMU_FRAME_LEN) return false;
  //get euler
  for(u32_idx = 0; u32_idx < 3; u32_idx++)
  {
    pu8_end = memchr(pu8_start, ' ', IMU_ELEMENT_MAX_LEN);
    if(pu8_end == NULL) return false;
    *pu8_end = 0;
    *(&stru_IMU_data.flt_euler_x + u32_idx) = (float)atoi((char *)pu8_start - 1) * IMU_SCALE_EULER_UNIT;
    pu8_start = pu8_end + 2;
  }
  //get gyro
  for(u32_idx = 0; u32_idx < 3; u32_idx++)
  {
    pu8_end = memchr(pu8_start, ' ', IMU_ELEMENT_MAX_LEN);
    if(pu8_end == NULL) return false;
    *pu8_end = 0;
    *(&stru_IMU_data.flt_gyro_x + u32_idx) = (float)atoi((char *)pu8_start - 1) * IMU_SCALE_GYRO_UNIT;
    pu8_start = pu8_end + 2;
  }
  //get acc
  for(u32_idx = 0; u32_idx < 3; u32_idx++)
  {
    pu8_end = memchr(pu8_start, ' ', IMU_ELEMENT_MAX_LEN);
    if(pu8_end == NULL) return false;
    *pu8_end = 0;
    *(&stru_IMU_data.flt_acc_x + u32_idx) = (float)atoi((char *)pu8_start - 1) * IMU_SCALE_ACC_UNIT;
    pu8_start = pu8_end + 2;
  }
  return true;
}

/**
  * @brief  Read ADIS with timeout
  * @note   call bool_ADIS_Read() with timeout
  * @param  u32_timeout_ms: Desired timeout 
  * @retval true if timeout and vice versa (do not use return if do not understand)
  */
bool bool_ADIS_Read_IsTimeout(uint32_t u32_timeout_ms)
{
  static uint32_t u32_read_done_time = 0;
  if(bool_ADIS_Read() == false)
  {
    if(SysTick_IsTimeout(u32_read_done_time, u32_timeout_ms) == true)
    {
      stru_IMU_data.bool_available = false;
      return true;
    }
  }
  else
  {
    u32_read_done_time = SysTick_GetTick();
    stru_IMU_data.bool_available = true;
  }
  return false;
}

/**
  * @brief  Get struct IMU data
  * @note   ...
  * @param  none
  * @retval stru_IMU_data
  */
STRU_IMU_DATA_T stru_Get_IMU_Data(void)
{
  return stru_IMU_data;
}

/**
  * @}
  */

/*********************************END OF FILE**********************************/
