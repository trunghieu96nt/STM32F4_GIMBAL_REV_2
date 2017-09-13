/**
  ******************************************************************************
  * @file    comm_driver.c
  * @author  Vu Trung Hieu
  * @version V2.0
  * @date    11-September-2017
  * @brief   This file contains functions for communication with outer devices
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
#include "comm_driver.h"
#include "string.h"
#include "system_timetick.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static uint8_t au8_CMD_Rx[CMD_RXBUFF_SIZE]= {0};
static uint8_t au8_RESV_Rx[RESV_RXBUFF_SIZE]= {0};

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
void v_CMD_UART_Init(void);
void v_DATA_UART_Init(void);
void v_RESV_UART_Init(void);
uint8_t *pu8_Search_Header(const uint8_t *pu8_Haystack, uint32_t u32_Hlen, 
                           const uint8_t *pu8_Needle, uint32_t u32_Nlen);
bool bool_CMD_Parse(const uint8_t *pu8_Message, uint32_t u32_Message_Size);

/** @defgroup Communication Initialization
 *  @brief   ...
 *
 @verbatim
 ===============================================================================
                    ##### Communication Initialization #####
 ===============================================================================  

 @endverbatim
  * @{
  */

/**
  * @brief  Communication Initialize
  * @note   Including UART (CMD, DATA, RESV), SPI
  * @param  none
  * @retval none
  */
void v_Comm_Init(void)
{
  v_CMD_UART_Init();
  v_DATA_UART_Init();
  v_RESV_UART_Init();
}

/**
  * @}
  */

/** @defgroup CMD - UART
 *  @brief   ...
 *
 @verbatim
 ===============================================================================
                             ##### CMD - UART #####
 ===============================================================================  

 @endverbatim
  * @{
  */
/**
  * @brief  CMD Initialize (Receiver)
  * @note   ...
  * @param  none
  * @retval none
  */
void v_CMD_UART_Init(void)
{
  USART_InitTypeDef     USART_InitStructure;
  GPIO_InitTypeDef      GPIO_InitStructure;
  DMA_InitTypeDef       DMA_InitStructure;
  
  RCC_AHB1PeriphClockCmd(CMD_PORT_CLK, ENABLE);  
  GPIO_InitStructure.GPIO_Pin   = CMD_TX | CMD_RX;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
  GPIO_Init(CMD_PORT, &GPIO_InitStructure);
  
  GPIO_PinAFConfig(CMD_PORT, CMD_TX_SOURCE, CMD_AF);
  GPIO_PinAFConfig(CMD_PORT, CMD_RX_SOURCE, CMD_AF);

  RCC_APB1PeriphClockCmd(CMD_USART_CLK, ENABLE);
  
  USART_InitStructure.USART_BaudRate            = CMD_BAUDRATE;
  USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits            = USART_StopBits_1;
  USART_InitStructure.USART_Parity              = USART_Parity_No;
  USART_InitStructure.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_Init(CMD_USART, &USART_InitStructure);
  USART_Cmd(CMD_USART, ENABLE); 
  USART_ClearFlag(CMD_USART, USART_FLAG_TC);

  /* DMA RX configuration */
  RCC_AHB1PeriphClockCmd(CMD_AHB_PERIPH_DMA, ENABLE);
  DMA_DeInit(CMD_RX_DMA_STREAM);  
  DMA_InitStructure.DMA_Channel            = CMD_RX_DMA_CHANNEL;
  DMA_InitStructure.DMA_DIR                = DMA_DIR_PeripheralToMemory;
  DMA_InitStructure.DMA_Memory0BaseAddr    = (uint32_t)&au8_CMD_Rx[0];
  DMA_InitStructure.DMA_PeripheralBaseAddr = CMD_DATA_REG;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_Priority           = DMA_Priority_VeryHigh;
  DMA_InitStructure.DMA_FIFOMode           = DMA_FIFOMode_Disable; //direct mode
  DMA_InitStructure.DMA_FIFOThreshold      = DMA_FIFOThreshold_Full;
  DMA_InitStructure.DMA_MemoryInc          = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_Mode               = DMA_Mode_Circular; // avoid buffer overload error
  DMA_InitStructure.DMA_BufferSize         = CMD_RXBUFF_SIZE;
  DMA_InitStructure.DMA_MemoryBurst        = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_PeripheralBurst    = DMA_PeripheralBurst_Single;
  DMA_Init(CMD_RX_DMA_STREAM, &DMA_InitStructure);
  /* Enable  request */
  USART_DMACmd(CMD_USART, USART_DMAReq_Rx, ENABLE);  
  /* Enable DMA RX Channel */
  DMA_Cmd(CMD_RX_DMA_STREAM, ENABLE);
  
  /* DMA TX configuration */
  DMA_DeInit(CMD_TX_DMA_STREAM);
  DMA_InitStructure.DMA_Channel            = CMD_TX_DMA_CHANNEL;
  DMA_InitStructure.DMA_DIR                = DMA_DIR_MemoryToPeripheral;
  DMA_InitStructure.DMA_Memory0BaseAddr    = 0;
  DMA_InitStructure.DMA_PeripheralBaseAddr = CMD_DATA_REG;   
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_Priority           = DMA_Priority_Low;
  DMA_InitStructure.DMA_FIFOMode           = DMA_FIFOMode_Disable;
  DMA_InitStructure.DMA_FIFOThreshold      = DMA_FIFOThreshold_Full;
  DMA_InitStructure.DMA_MemoryInc          = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_Mode               = DMA_Mode_Normal;
  DMA_InitStructure.DMA_BufferSize         = 0;
  DMA_InitStructure.DMA_MemoryBurst        = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_PeripheralBurst    = DMA_PeripheralBurst_Single;
  DMA_Init(CMD_TX_DMA_STREAM, &DMA_InitStructure);
  
  // Enable DMA Stream Transfer Complete interrupt
  //DMA_ITConfig(CMD_TX_DMA_STREAM, DMA_IT_TC, ENABLE);
  
  // Enable USART DMA TX request
  USART_DMACmd(CMD_USART, USART_DMAReq_Tx, ENABLE);  
}

/**
  * @brief  Send message through DMA - UART
  * @note   CMD
  * @param  pu8_Message: pointer message to send
  * @param  u32_Message_Size: number of char to send
  * @retval true if success and vice versa
  */
bool bool_CMD_Send(const uint8_t *pu8_Message, uint32_t u32_Message_Size)
{
  if(u32_Message_Size > CMD_TXBUFF_SIZE)
  {
    return false;
  }
  else
  {
    //clear flag
    DMA_ClearFlag(CMD_TX_DMA_STREAM, CMD_TX_DMA_FLAG);
    DMA_MemoryTargetConfig(CMD_TX_DMA_STREAM, (uint32_t)pu8_Message, DMA_Memory_0);
    DMA_SetCurrDataCounter(CMD_TX_DMA_STREAM, u32_Message_Size);
    //STM_TX_DMA_STREAM->NDTR = BUFF_SIZE;
    DMA_Cmd(CMD_TX_DMA_STREAM, ENABLE);
    return true;
  }
}

void v_CMD_Receive(void)
{
  static uint8_t *pu8_CMD_Rx_Cur = &au8_CMD_Rx[CMD_RXBUFF_SIZE - 1];
  static uint8_t *pu8_CMD_Rx_Pre = &au8_CMD_Rx[CMD_RXBUFF_SIZE - 1];
  static uint32_t u32_Start_Receive_Time = 0;
  static bool bool_Receiving = false;
  static uint8_t au8_CMD_Frame[CMD_FRAME_LEN_MAX];
  static uint32_t u32_CMD_Buff_Count = 0;
  //uint32_t u32_Length = 0;
  uint8_t *pu8_CMD_Frame = 0;
  
  if (CMD_RX_DMA_STREAM->NDTR == CMD_RXBUFF_SIZE)
    pu8_CMD_Rx_Cur = &au8_CMD_Rx[CMD_RXBUFF_SIZE - 1];
  else
    pu8_CMD_Rx_Cur = &au8_CMD_Rx[CMD_RXBUFF_SIZE - CMD_RX_DMA_STREAM->NDTR - 1];

  if (pu8_CMD_Rx_Cur == pu8_CMD_Rx_Pre)
  {
    if (bool_Receiving == true)
    {
      if (SysTick_IsTimeout(u32_Start_Receive_Time, CMD_RX_FRAME_TIMEOUT))
      {
        bool_Receiving = false;
        au8_CMD_Frame[u32_CMD_Buff_Count] = 0;
        //Handle Data au8_CMD_Frame Here
        bool_CMD_Parse(au8_CMD_Frame, u32_CMD_Buff_Count);
        u32_CMD_Buff_Count = 0;
      }
    }
  }
  else if (pu8_CMD_Rx_Cur > pu8_CMD_Rx_Pre)
  {
    if (bool_Receiving == false)
    {
      pu8_CMD_Frame = pu8_Search_Header(pu8_CMD_Rx_Pre + 1, pu8_CMD_Rx_Cur - pu8_CMD_Rx_Pre, 
                                        (uint8_t *)STRING_HEADER, strlen(STRING_HEADER));
      if (pu8_CMD_Frame == NULL)
      {
        pu8_CMD_Rx_Pre = pu8_CMD_Rx_Cur;
        return;
      }
      bool_Receiving = true;
      u32_Start_Receive_Time = SysTick_GetTick();
    }
  }
//  {
//    if (bool_Receiving == false)
//    {
//      
//      bool_Receiving = true;
//      u32_Start_Receive_Time = SysTick_GetTick();
//    }
//    
//    pu8_CMD_Frame = au8_CMD_Frame + u32_CMD_Buff_Count; //Calculate the pointer of next copy
//    if (pu8_CMD_Rx_Cur > pu8_CMD_Rx_Pre)
//    {
//      u32_Length = pu8_CMD_Rx_Cur - pu8_CMD_Rx_Pre;
//      u32_CMD_Buff_Count += u32_Length;
//      if (u32_CMD_Buff_Count > (CMD_FRAME_LEN_MAX - 1))
//      {
//        bool_Receiving = false;
//        u32_CMD_Buff_Count = 0;
//        pu8_CMD_Rx_Pre = pu8_CMD_Rx_Cur;
//        return;
//      }
//        memcpy(pu8_CMD_Frame, pu8_CMD_Rx_Pre + 1, u32_Length);
//    }
//    else //(pu8_CMD_Rx_Cur < pu8_CMD_Rx_Pre)
//    {
//      u32_Length = pu8_CMD_Rx_Cur + CMD_RXBUFF_SIZE - pu8_CMD_Rx_Pre;
//      u32_CMD_Buff_Count += u32_Length;
//      if (u32_CMD_Buff_Count > (CMD_FRAME_LEN_MAX - 1))
//      {
//        bool_Receiving = false;
//        u32_CMD_Buff_Count = 0;
//        pu8_CMD_Rx_Pre = pu8_CMD_Rx_Cur;
//        return;
//      }
//      u32_Length = au8_CMD_Rx + CMD_RXBUFF_SIZE - pu8_CMD_Rx_Pre - 1;
//      memcpy(pu8_CMD_Frame, pu8_CMD_Rx_Pre + 1, u32_Length);
//      memcpy(pu8_CMD_Frame + u32_Length, au8_CMD_Rx, pu8_CMD_Rx_Cur - au8_CMD_Rx + 1);
//    }
//    pu8_CMD_Rx_Pre = pu8_CMD_Rx_Cur;
//  }
}

bool bool_CMD_Parse(const uint8_t *pu8_Message, uint32_t u32_Message_Size)
{
  /* Check Header Frame */
  if (memcmp(pu8_Message, STRING_HEADER, 2) != 0) return false;
  
  /* Check DestID */
  if (pu8_Message[BYTE_DEST_ID] != ID_GIMBAL_CONTROLER) return false;
  
  /* Check SrcID */
  if (pu8_Message[BYTE_SRC_ID] != ID_GUI_SOFTWARE) return false;
  
  /* Check Seq */
  //pu8_Message[BYTE_SEQ]
  
  /* Check Length */
  if (pu8_Message[BYTE_LEN] != (u32_Message_Size - LEADING_FRAME_BYTES)) return false;
  
  /* Handle Data */
  
  
  return true;
}

/**
  * @}
  */

/** @defgroup DATA - UART
 *  @brief   ...
 *
 @verbatim
 ===============================================================================
                             ##### DATA - UART #####
 ===============================================================================  

 @endverbatim
  * @{
  */
/**
  * @brief  DATA UART Initialize (Sender)
  * @note   ...
  * @param  none
  * @retval none
  */
void v_DATA_UART_Init(void)
{
  USART_InitTypeDef   USART_InitStructure;
  GPIO_InitTypeDef    GPIO_InitStructure;
  DMA_InitTypeDef     DMA_InitStructure;
  
  RCC_AHB1PeriphClockCmd(DATA_PORT_CLK, ENABLE);
  /* GPIO configuration */
  GPIO_InitStructure.GPIO_Pin   = DATA_TX;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
  GPIO_Init(DATA_PORT, &GPIO_InitStructure);
  GPIO_PinAFConfig(DATA_PORT, DATA_TX_SOURCE, DATA_AF);
  
  /* USART configuration */
  RCC_APB1PeriphClockCmd(DATA_USART_CLK, ENABLE);
  USART_InitStructure.USART_BaudRate            = DATA_BAUDRATE;
  USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits            = USART_StopBits_1;
  USART_InitStructure.USART_Parity              = USART_Parity_No;
  USART_InitStructure.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_Init(DATA_USART, &USART_InitStructure);
  USART_Cmd(DATA_USART, ENABLE);
    
  USART_ClearFlag(DATA_USART, USART_FLAG_TC);
  //USART_ClearFlag(DATA_USART, USART_FLAG_RXNE);

  RCC_AHB1PeriphClockCmd(DATA_AHB_PERIPH_DMA, ENABLE);
  /* DMA TX configuration */
  DMA_DeInit(DATA_TX_DMA_STREAM);
  DMA_InitStructure.DMA_Channel            = DATA_TX_DMA_CHANNEL;
  DMA_InitStructure.DMA_DIR                = DMA_DIR_MemoryToPeripheral;
  DMA_InitStructure.DMA_Memory0BaseAddr    = 0;
  DMA_InitStructure.DMA_PeripheralBaseAddr = DATA_DATA_REG;   
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_Priority           = DMA_Priority_Low;
  DMA_InitStructure.DMA_FIFOMode           = DMA_FIFOMode_Disable;
  DMA_InitStructure.DMA_FIFOThreshold      = DMA_FIFOThreshold_Full;
  DMA_InitStructure.DMA_MemoryInc          = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_Mode               = DMA_Mode_Normal;
  DMA_InitStructure.DMA_BufferSize         = 0;
  DMA_InitStructure.DMA_MemoryBurst        = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_PeripheralBurst    = DMA_PeripheralBurst_Single;
  DMA_Init(DATA_TX_DMA_STREAM, &DMA_InitStructure);
  
  // Enable DMA Stream Transfer Complete interrupt
  //DMA_ITConfig(DATA_TX_DMA_STREAM, DMA_IT_TC, ENABLE);
  
  // Enable USART DMA TX request
  USART_DMACmd(DATA_USART, USART_DMAReq_Tx, ENABLE);  
  // Enable DMA TX Channel
  //DMA_Cmd(DATA_TX_DMA_STREAM, ENABLE);
}

/**
  * @brief  Send message through DMA - UART
  * @note   DATA
  * @param  pu8_Message: pointer message to send
  * @param  u32_Message_Size: number of char to send
  * @retval true if success and vice versa
  */
bool bool_DATA_Send(const uint8_t *pu8_Message, uint32_t u32_Message_Size)
{
  if(u32_Message_Size > DATA_TXBUFF_SIZE)
  {
    return false;
  }
  else
  {
    //clear flag
    DMA_ClearFlag(DATA_TX_DMA_STREAM, DATA_TX_DMA_FLAG);
    DMA_MemoryTargetConfig(DATA_TX_DMA_STREAM, (uint32_t)pu8_Message, DMA_Memory_0);
    DMA_SetCurrDataCounter(DATA_TX_DMA_STREAM, u32_Message_Size);
    //DATA_TX_DMA_STREAM->NDTR = BUFF_SIZE;
    DMA_Cmd(DATA_TX_DMA_STREAM, ENABLE);
    return true;
  }
}
/**
  * @}
  */

/** @defgroup RESV - UART
 *  @brief   ...
 *
 @verbatim
 ===============================================================================
                             ##### RESV - UART #####
 ===============================================================================  

 @endverbatim
  * @{
  */
/**
  * @brief  RESV Initialize (General)
  * @note   ...
  * @param  none
  * @retval none
  */
void v_RESV_UART_Init(void)
{
  USART_InitTypeDef     USART_InitStructure;
  GPIO_InitTypeDef      GPIO_InitStructure;
  DMA_InitTypeDef       DMA_InitStructure;
  
  /* TX Config */
  RCC_AHB1PeriphClockCmd(RESV_PORT_TX_CLK, ENABLE);  
  GPIO_InitStructure.GPIO_Pin   = RESV_TX;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
  GPIO_Init(RESV_PORT_TX, &GPIO_InitStructure);
  GPIO_PinAFConfig(RESV_PORT_TX, RESV_TX_SOURCE, RESV_AF);

  /* RX Config */
  RCC_AHB1PeriphClockCmd(RESV_PORT_RX_CLK, ENABLE);  
  GPIO_InitStructure.GPIO_Pin   = RESV_RX;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
  GPIO_Init(RESV_PORT_RX, &GPIO_InitStructure);
  
  GPIO_PinAFConfig(RESV_PORT_RX, RESV_RX_SOURCE, RESV_AF);
  
  RCC_APB1PeriphClockCmd(RESV_USART_CLK, ENABLE);
  
  USART_InitStructure.USART_BaudRate            = RESV_BAUDRATE;
  USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits            = USART_StopBits_1;
  USART_InitStructure.USART_Parity              = USART_Parity_No;
  USART_InitStructure.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_Init(RESV_USART, &USART_InitStructure);
  USART_Cmd(RESV_USART, ENABLE); 
  USART_ClearFlag(RESV_USART, USART_FLAG_TC);

  /* DMA RX configuration */
  RCC_AHB1PeriphClockCmd(RESV_AHB_PERIPH_DMA, ENABLE);
  DMA_DeInit(RESV_RX_DMA_STREAM);  
  DMA_InitStructure.DMA_Channel            = RESV_RX_DMA_CHANNEL;
  DMA_InitStructure.DMA_DIR                = DMA_DIR_PeripheralToMemory;
  DMA_InitStructure.DMA_Memory0BaseAddr    = (uint32_t)&au8_RESV_Rx[0];
  DMA_InitStructure.DMA_PeripheralBaseAddr = RESV_DATA_REG;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_Priority           = DMA_Priority_VeryHigh;
  DMA_InitStructure.DMA_FIFOMode           = DMA_FIFOMode_Disable; //direct mode
  DMA_InitStructure.DMA_FIFOThreshold      = DMA_FIFOThreshold_Full;
  DMA_InitStructure.DMA_MemoryInc          = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_Mode               = DMA_Mode_Circular; // avoid buffer overload error
  DMA_InitStructure.DMA_BufferSize         = RESV_RXBUFF_SIZE;
  DMA_InitStructure.DMA_MemoryBurst        = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_PeripheralBurst    = DMA_PeripheralBurst_Single;
  DMA_Init(RESV_RX_DMA_STREAM, &DMA_InitStructure);
  /* Enable  request */
  USART_DMACmd(RESV_USART, USART_DMAReq_Rx, ENABLE);  
  /* Enable DMA RX Channel */
  DMA_Cmd(RESV_RX_DMA_STREAM, ENABLE);
  
  /* DMA TX configuration */
  DMA_DeInit(RESV_TX_DMA_STREAM);
  DMA_InitStructure.DMA_Channel            = RESV_TX_DMA_CHANNEL;
  DMA_InitStructure.DMA_DIR                = DMA_DIR_MemoryToPeripheral;
  DMA_InitStructure.DMA_Memory0BaseAddr    = 0;
  DMA_InitStructure.DMA_PeripheralBaseAddr = RESV_DATA_REG;   
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_Priority           = DMA_Priority_Low;
  DMA_InitStructure.DMA_FIFOMode           = DMA_FIFOMode_Disable;
  DMA_InitStructure.DMA_FIFOThreshold      = DMA_FIFOThreshold_Full;
  DMA_InitStructure.DMA_MemoryInc          = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_Mode               = DMA_Mode_Normal;
  DMA_InitStructure.DMA_BufferSize         = 0;
  DMA_InitStructure.DMA_MemoryBurst        = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_PeripheralBurst    = DMA_PeripheralBurst_Single;
  DMA_Init(RESV_TX_DMA_STREAM, &DMA_InitStructure);
  
  // Enable DMA Stream Transfer Complete interrupt
  //DMA_ITConfig(RESV_TX_DMA_STREAM, DMA_IT_TC, ENABLE);
  
  // Enable USART DMA TX request
  USART_DMACmd(RESV_USART, USART_DMAReq_Tx, ENABLE);  
}

/**
  * @brief  Send message through DMA - UART
  * @note   RESV
  * @param  pu8_Message: pointer message to send
  * @param  u32_Message_Size: number of char to send
  * @retval true if success and vice versa
  */
bool bool_RESV_Send(const uint8_t *pu8_Message, uint32_t u32_Message_Size)
{
  if(u32_Message_Size > RESV_TXBUFF_SIZE)
  {
    return false;
  }
  else
  {
    //clear flag
    DMA_ClearFlag(RESV_TX_DMA_STREAM, RESV_TX_DMA_FLAG);
    DMA_MemoryTargetConfig(RESV_TX_DMA_STREAM, (uint32_t)pu8_Message, DMA_Memory_0);
    DMA_SetCurrDataCounter(RESV_TX_DMA_STREAM, u32_Message_Size);
    //STM_TX_DMA_STREAM->NDTR = BUFF_SIZE;
    DMA_Cmd(RESV_TX_DMA_STREAM, ENABLE);
    return true;
  }
}

/**
  * @}
  */

/*
 * The memmem() function finds the start of the first occurrence of the
 * substring 'needle' of length 'nlen' in the memory area 'haystack' of
 * length 'hlen'.
 *
 * The return value is a pointer to the beginning of the sub-string, or
 * NULL if the substring is not found.
 */
uint8_t *pu8_Search_Header(const uint8_t *pu8_Haystack, uint32_t u32_Hlen, 
                           const uint8_t *pu8_Needle, uint32_t u32_Nlen)
{
  uint8_t u8_Needle_First = *pu8_Needle;
  const uint8_t *pu8_Search = pu8_Haystack;
  uint32_t u32_Slen = u32_Hlen;

  if (u32_Nlen == 0) return NULL;

  do {
    pu8_Search = memchr(pu8_Search, u8_Needle_First, u32_Slen - u32_Nlen + 1);
    if (pu8_Search == NULL) return NULL;
    if (!memcmp(pu8_Search, pu8_Needle, u32_Nlen)) return (uint8_t *)pu8_Search;
    pu8_Search++;
    u32_Slen = u32_Hlen - (pu8_Search - pu8_Haystack);
  } while (u32_Slen >= u32_Nlen);
  return NULL;
}

/*********************************END OF FILE**********************************/
