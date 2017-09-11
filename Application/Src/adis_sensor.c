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
uint8_t au8_IMU_Rx[IMU_RXBUFF_SIZE]= {0};
STRU_IMU_DATA_T stru_IMU_Data = {false}; //initial bool_Available value.

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
void *memrchr(const void *str, int c, size_t n);
bool bool_ADIS_Parse(uint8_t *pu8_IMU_Frame);

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
  DMA_InitStructure.DMA_Memory0BaseAddr    = (uint32_t)&au8_IMU_Rx[0];
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
  * @param  none
  * @retval true if get correctly and vice versa
  */
bool bool_ADIS_Read(void)
{
  static uint8_t *pu8_IMU_Rx_Cur = &au8_IMU_Rx[IMU_RXBUFF_SIZE - 1];
  static uint8_t *pu8_IMU_Rx_Pre = &au8_IMU_Rx[IMU_RXBUFF_SIZE - 1];
  uint8_t *pu8_End_Chr = NULL, *pu8_Start_Chr = NULL;
  uint8_t au8_IMU_Frame[IMU_FRAME_LEN_MAX];
  uint32_t u32_Length = 0;
  
  if (IMU_RX_DMA_STREAM->NDTR == IMU_RXBUFF_SIZE)
    pu8_IMU_Rx_Cur = &au8_IMU_Rx[IMU_RXBUFF_SIZE - 1];
  else
    pu8_IMU_Rx_Cur = &au8_IMU_Rx[IMU_RXBUFF_SIZE - IMU_RX_DMA_STREAM->NDTR - 1];
  
  if (pu8_IMU_Rx_Cur > pu8_IMU_Rx_Pre)
  {
    //check enough lengh
    if ((pu8_IMU_Rx_Cur - pu8_IMU_Rx_Pre + 1) < IMU_FRAME_LEN) return false;
    //search IMU_END_FRAME backward from pu8_IMU_Rx_Cur to pu8_IMU_Rx_Pre
    pu8_End_Chr = memrchr(pu8_IMU_Rx_Cur, IMU_END_FRAME, pu8_IMU_Rx_Cur - pu8_IMU_Rx_Pre + 1);
    if (pu8_End_Chr == NULL) return false;
    //search IMU_START_FRAME backward from pu8_End_Chr to pu8_IMU_Rx_Pre
    pu8_Start_Chr = memrchr(pu8_End_Chr, IMU_START_FRAME, pu8_End_Chr - pu8_IMU_Rx_Pre + 1);
    if (pu8_Start_Chr == NULL) return false;
    u32_Length = pu8_End_Chr - pu8_Start_Chr + 1;
    if (u32_Length > (IMU_FRAME_LEN_MAX - 1)) return false;
    memcpy(au8_IMU_Frame, pu8_Start_Chr, u32_Length);
    au8_IMU_Frame[u32_Length] = 0;
  }
  else if (pu8_IMU_Rx_Cur < pu8_IMU_Rx_Pre)
  {
    //check enough lengh
    if ((pu8_IMU_Rx_Cur + IMU_RXBUFF_SIZE - pu8_IMU_Rx_Pre + 1) < IMU_FRAME_LEN) return false;
    //search IMU_END_FRAME backward from pu8_IMU_Rx_Cur to au8_IMU_Rx
    pu8_End_Chr = memrchr(pu8_IMU_Rx_Cur, IMU_END_FRAME, pu8_IMU_Rx_Cur - au8_IMU_Rx + 1);
    if (pu8_End_Chr != NULL)
    {
      //search IMU_START_FRAME backward from pu8_End_Chr to au8_IMU_Rx
      pu8_Start_Chr = memrchr(pu8_End_Chr, IMU_START_FRAME, pu8_End_Chr - au8_IMU_Rx + 1);
      if (pu8_Start_Chr != NULL)
      {
        u32_Length = pu8_End_Chr - pu8_Start_Chr + 1;
        if (u32_Length > (IMU_FRAME_LEN_MAX - 1)) return false;
        memcpy(au8_IMU_Frame, pu8_Start_Chr, u32_Length);
        au8_IMU_Frame[u32_Length] = 0;
      }
      else
      {
        //search IMU_START_FRAME backward from &au8_IMU_Rx[IMU_RXBUFF_SIZE - 1] to pu8_IMU_Rx_Pre
        pu8_Start_Chr = memrchr(&au8_IMU_Rx[IMU_RXBUFF_SIZE - 1], IMU_START_FRAME, &au8_IMU_Rx[IMU_RXBUFF_SIZE - 1] - pu8_IMU_Rx_Pre + 1);
        if (pu8_Start_Chr == NULL) return false;
        u32_Length = pu8_End_Chr - pu8_Start_Chr + IMU_RXBUFF_SIZE + 1;
        if (u32_Length > (IMU_FRAME_LEN_MAX - 1)) return false;
        memcpy(au8_IMU_Frame, pu8_Start_Chr, &au8_IMU_Rx[IMU_RXBUFF_SIZE - 1] - pu8_Start_Chr + 1);
        memcpy(&au8_IMU_Frame[&au8_IMU_Rx[IMU_RXBUFF_SIZE - 1] - pu8_Start_Chr + 1], au8_IMU_Rx, pu8_End_Chr - au8_IMU_Rx + 1);
        au8_IMU_Frame[u32_Length] = 0;
      }
    }
    else
    {
      //search IMU_END_FRAME backward from &au8_IMU_Rx[IMU_RXBUFF_SIZE - 1] to pu8_IMU_Rx_Pre
      pu8_End_Chr = memrchr(&au8_IMU_Rx[IMU_RXBUFF_SIZE - 1], IMU_END_FRAME, &au8_IMU_Rx[IMU_RXBUFF_SIZE - 1] - pu8_IMU_Rx_Pre + 1);
      if(pu8_End_Chr == NULL) return false;
      //search IMU_START_FRAME backward from pu8_End_Chr to pu8_IMU_Rx_Pre
      pu8_Start_Chr = memrchr(pu8_End_Chr, IMU_START_FRAME, pu8_End_Chr - pu8_IMU_Rx_Pre + 1);
      if(pu8_Start_Chr == NULL) return false;
      u32_Length = pu8_End_Chr - pu8_Start_Chr + 1;
      if (u32_Length > (IMU_FRAME_LEN_MAX - 1)) return false;
      memcpy(au8_IMU_Frame, pu8_Start_Chr, u32_Length);
      au8_IMU_Frame[u32_Length] = 0;
    }
  }
  else //pu8_IMU_Rx_Cur == pu8_IMU_Rx_Pre
  {
    return false;
  }
  pu8_IMU_Rx_Pre = pu8_End_Chr;
  if(bool_ADIS_Parse(au8_IMU_Frame) == false) return false;
  return true;
}

/**
  * @brief  parse raw IMU frame
  * @note   ...
  * @param  pu8_IMU_Frame: pointer to IMU frame
  * @retval true if parse correctly and vice versa
  */
bool bool_ADIS_Parse(uint8_t *pu8_IMU_Frame)
{
  uint32_t u32_Idx = 0;
  uint8_t *pu8_End = NULL, *pu8_Start = pu8_IMU_Frame + 2;
  
  if(strlen((char *)pu8_IMU_Frame) != IMU_FRAME_LEN) return false;
  //get euler
  for(u32_Idx = 0; u32_Idx < 3; u32_Idx++)
  {
    pu8_End = memchr(pu8_Start, ' ', IMU_ELEMENT_MAX_LEN);
    if(pu8_End == NULL) return false;
    *pu8_End = 0;
    *(&stru_IMU_Data.flt_Euler_x + u32_Idx) = (float)atoi((char *)pu8_Start - 1) * IMU_SCALE_EULER_UNIT;
    pu8_Start = pu8_End + 2;
  }
  //get gyro
  for(u32_Idx = 0; u32_Idx < 3; u32_Idx++)
  {
    pu8_End = memchr(pu8_Start, ' ', IMU_ELEMENT_MAX_LEN);
    if(pu8_End == NULL) return false;
    *pu8_End = 0;
    *(&stru_IMU_Data.flt_Gyro_x + u32_Idx) = (float)atoi((char *)pu8_Start - 1) * IMU_SCALE_GYRO_UNIT;
    pu8_Start = pu8_End + 2;
  }
  //get acc
  for(u32_Idx = 0; u32_Idx < 3; u32_Idx++)
  {
    pu8_End = memchr(pu8_Start, ' ', IMU_ELEMENT_MAX_LEN);
    if(pu8_End == NULL) return false;
    *pu8_End = 0;
    *(&stru_IMU_Data.flt_Acc_x + u32_Idx) = (float)atoi((char *)pu8_Start - 1) * IMU_SCALE_ACC_UNIT;
    pu8_Start = pu8_End + 2;
  }
  return true;
}

/**
  * @brief  Read ADIS with timeout
  * @note   call bool_ADIS_Read() with timeout
  * @param  u32_Timeout_ms: Desired timeout 
  * @retval true if timeout and vice versa (do not use return if do not understand)
  */
bool bool_ADIS_Read_IsTimeout(uint32_t u32_Timeout_ms)
{
  static uint32_t u32_Read_Done_Time = 0;
  if(bool_ADIS_Read() == false)
  {
    if(SysTick_IsTimeout(u32_Read_Done_Time, u32_Timeout_ms) == true)
    {
        stru_IMU_Data.bool_Available = false;
        return true;
    }
  }
  else
  {
    u32_Read_Done_Time = SysTick_GetTick();
    stru_IMU_Data.bool_Available = true;
  }
  return false;
}

/**
  * @}
  */

/** @defgroup ADIS Initialization
 *  @brief   ...
 *
 @verbatim
 ===============================================================================
                          ##### ADIS Utilities #####
 ===============================================================================  

 @endverbatim
  * @{
  */

/**
  * @brief  memrchr
  * @note   search from *str backward n
  * @param  str: pointer to string
  * @param  c: desired int (or char)
  * @param  n: length to search
  * @retval pointer of searched char if existed or null
  */
void *memrchr(const void *str, int c, size_t n)
{
  if(n != 0)
  {
    const unsigned char *p = str;
    do {
      if (*p-- == (unsigned char) c)
      {
        return (void *) (p + 1);
      }
    } while (--n);
  }
  return NULL;
}

/**
  * @}
  */

/*********************************END OF FILE**********************************/
