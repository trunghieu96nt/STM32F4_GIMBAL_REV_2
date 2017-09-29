/**
  ******************************************************************************
  * @file    uart_comm.c
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
#include "uart_comm.h"
#include "system_timetick.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static uint8_t au8_CMD_Rx[CMD_RXBUFF_SIZE]= {0};
uint8_t au8_DATA_Rx[DATA_RXBUFF_SIZE]= {0};
uint8_t au8_RESV_Rx[RESV_RXBUFF_SIZE]= {0};

extern bool bool_None_Handler             (uint8_t u8_Msg_ID, uint8_t *pu8_Payload, uint32_t u32_Payload_Cnt);
extern bool bool_Home_Handler             (uint8_t u8_Msg_ID, uint8_t *pu8_Payload, uint32_t u32_Payload_Cnt);
extern bool bool_Stop_Handler             (uint8_t u8_Msg_ID, uint8_t *pu8_Payload, uint32_t u32_Payload_Cnt);
extern bool bool_Emergency_Stop_Handler   (uint8_t u8_Msg_ID, uint8_t *pu8_Payload, uint32_t u32_Payload_Cnt);
extern bool bool_Stabilizing_Mode_Handler (uint8_t u8_Msg_ID, uint8_t *pu8_Payload, uint32_t u32_Payload_Cnt);
extern bool bool_Set_Pos_Handler          (uint8_t u8_Msg_ID, uint8_t *pu8_Payload, uint32_t u32_Payload_Cnt);
extern bool bool_Set_Vel_Handler          (uint8_t u8_Msg_ID, uint8_t *pu8_Payload, uint32_t u32_Payload_Cnt);
extern bool bool_Set_Pos_Vel_Handler      (uint8_t u8_Msg_ID, uint8_t *pu8_Payload, uint32_t u32_Payload_Cnt);
extern bool bool_Get_Pos_Handler          (uint8_t u8_Msg_ID, uint8_t *pu8_Payload, uint32_t u32_Payload_Cnt);
extern bool bool_Set_Kp_Handler           (uint8_t u8_Msg_ID, uint8_t *pu8_Payload, uint32_t u32_Payload_Cnt);
extern bool bool_Set_Ki_Handler           (uint8_t u8_Msg_ID, uint8_t *pu8_Payload, uint32_t u32_Payload_Cnt);
extern bool bool_Set_Kd_Handler           (uint8_t u8_Msg_ID, uint8_t *pu8_Payload, uint32_t u32_Payload_Cnt);
extern bool bool_Set_Kff1_Handler         (uint8_t u8_Msg_ID, uint8_t *pu8_Payload, uint32_t u32_Payload_Cnt);
extern bool bool_Set_Kff2_Handler         (uint8_t u8_Msg_ID, uint8_t *pu8_Payload, uint32_t u32_Payload_Cnt);
extern bool bool_Get_Params_Handler       (uint8_t u8_Msg_ID, uint8_t *pu8_Payload, uint32_t u32_Payload_Cnt);

const STRU_CMD_HANDLER_T astru_CMD_Handler[CMD_NUM_MSG_ID_MAX] =
{
  {MSG_NONE,                0,    bool_None_Handler},
  {MSG_HOME,                1,    bool_Home_Handler},
  {MSG_STOP,                1,    bool_Stop_Handler},
  {MSG_EMERGENCY_STOP,      1,    bool_Emergency_Stop_Handler},
  {MSG_STABILIZING_MODE,    2,    bool_Stabilizing_Mode_Handler},
  {MSG_SET_POS,             5,    bool_Set_Pos_Handler},
  {MSG_SET_VEL,             5,    bool_Set_Vel_Handler},
  {MSG_SET_POS_VEL,         9,    bool_Set_Pos_Vel_Handler},
  {MSG_GET_POS,             1,    bool_Get_Pos_Handler},
  {MSG_SET_KP,              6,    bool_Set_Kp_Handler},
  {MSG_SET_KI,              6,    bool_Set_Ki_Handler},
  {MSG_SET_KD,              6,    bool_Set_Kd_Handler},
  {MSG_SET_KFF1,            6,    bool_Set_Kff1_Handler},
  {MSG_SET_KFF2,            6,    bool_Set_Kff2_Handler},
  {MSG_GET_PARAMS,          2,    bool_Get_Params_Handler}
};

/* Private function prototypes -----------------------------------------------*/
static void v_CMD_UART_Init(void);
static void v_DATA_UART_Init(void);
static void v_RESV_UART_Init(void);

/* Private functions ---------------------------------------------------------*/

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
void v_UART_Comm_Init(void)
{
  v_CMD_UART_Init();
  v_DATA_UART_Init();
  v_RESV_UART_Init();
}

/**
  * @}
  */

/** @defgroup CMD - UART
 *  @brief    ...
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
static void v_CMD_UART_Init(void)
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
  DMA_InitStructure.DMA_Priority           = DMA_Priority_High;
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

/**
  * @brief  Receive and Parse message through DMA - UART
  * @note   CMD, Call in while loop (main.c) if used
  * @param  None
  * @retval None
  */
void v_CMD_Receive(void)
{
  static uint32_t u32_Idx_Pre = 0, u32_Idx = 0, u32_Time_Tick = 0;
  static bool bool_Header_Detected = false, bool_Length_Detected = false;
  static uint8_t au8_CMD_Frame[CMD_FRAME_LEN_MAX] = {'G', 'B', 0x02, 0x01};
  uint32_t u32_Length, u32_Idx_Cur, u32_Cnt;
  uint16_t u16_CRC_Check;
  
  u32_Idx_Cur = CMD_RXBUFF_SIZE - CMD_RX_DMA_STREAM->NDTR;
  
  if (u32_Idx_Cur == u32_Idx_Pre)
  {
    if (bool_Header_Detected == true)
    {
      if (SysTick_IsTimeout(u32_Time_Tick, 50))
      {
        u32_Idx = 0;
        bool_Length_Detected = false;
        bool_Header_Detected = false;
      }
    }
    return;
  }
  
  u32_Time_Tick = SysTick_GetTick();
  
  /* Search Header "GB" */
  if (bool_Header_Detected == false)
  {
    while (true)
    {
      if (u32_Idx_Cur >= u32_Idx_Pre) u32_Length = u32_Idx_Cur - u32_Idx_Pre;
      else u32_Length = CMD_RXBUFF_SIZE - (u32_Idx_Pre - u32_Idx_Cur);
      
      if (u32_Length < 2) return;
      
      if (*(au8_CMD_Rx + u32_Idx_Pre) == 'G')
      {
        if (++u32_Idx_Pre >= CMD_RXBUFF_SIZE) u32_Idx_Pre = 0;
        if (*(au8_CMD_Rx + u32_Idx_Pre) == 'B')
        {
          if (++u32_Idx_Pre >= CMD_RXBUFF_SIZE) u32_Idx_Pre = 0;
          bool_Header_Detected = true;
          break;
        }
      }
      else
      {
        if (++u32_Idx_Pre >= CMD_RXBUFF_SIZE) u32_Idx_Pre = 0;
      }
    }
  }
  
  /* Search Length of Frame*/
  if (bool_Length_Detected == false)
  {
    if (u32_Idx_Cur >= u32_Idx_Pre) u32_Length = u32_Idx_Cur - u32_Idx_Pre;
    else u32_Length = CMD_RXBUFF_SIZE - (u32_Idx_Pre - u32_Idx_Cur);
    
    if (u32_Length < 5) return;
    
    /* Check DEST_ID - ID_GIMBAL_CONTROLER */
    if (au8_CMD_Rx[u32_Idx_Pre] != 0x02) 
    {
      bool_Header_Detected = false;
      return;
    }
    if (++u32_Idx_Pre == CMD_RXBUFF_SIZE) u32_Idx_Pre = 0;
    
    /* Check SRC_ID - ID_GUI_SOFTWARE */
    if (au8_CMD_Rx[u32_Idx_Pre] != 0x01) 
    {
      bool_Header_Detected = false;
      return;
    }
    if (++u32_Idx_Pre == CMD_RXBUFF_SIZE) u32_Idx_Pre = 0;
    
    /* Get Seq */
    au8_CMD_Frame[4] = au8_CMD_Rx[u32_Idx_Pre];
    if (++u32_Idx_Pre == CMD_RXBUFF_SIZE) u32_Idx_Pre = 0;
    
    /* Get Length */
    bool_Length_Detected = true;
    au8_CMD_Frame[5] = au8_CMD_Rx[u32_Idx_Pre];
    if (++u32_Idx_Pre == CMD_RXBUFF_SIZE) u32_Idx_Pre = 0;
  }
  
  /* Getting Frame */
  while (true)
  {
    au8_CMD_Frame[u32_Idx + 6] = *(au8_CMD_Rx + u32_Idx_Pre);
    if (++u32_Idx_Pre >= CMD_RXBUFF_SIZE) u32_Idx_Pre = 0;
    if (++u32_Idx == au8_CMD_Frame[5])
    {
      u32_Idx = 0;
      bool_Length_Detected = false;
      bool_Header_Detected = false;
      break;
    }
    if (u32_Idx_Pre == u32_Idx_Cur) return;
  }
  
  /* Check CRC */
  u16_CRC_Check = 0;
  u32_Length = au8_CMD_Frame[5] + 6 - 2; //Total Length except 2 byte CRC
  for (u32_Cnt = 0; u32_Cnt < u32_Length; u32_Cnt++)
  {
    u16_CRC_Check += au8_CMD_Frame[u32_Cnt];
  }
  u16_CRC_Check = ~u16_CRC_Check;
  if (((u16_CRC_Check >> 8) & 0x0FF) != au8_CMD_Frame[u32_Length]) return;
  if ((u16_CRC_Check & 0x0FF) != au8_CMD_Frame[u32_Length + 1]) return;
  
  /* Check enough length */
  u32_Length = au8_CMD_Frame[5] - 3; //Length Payload
  if (astru_CMD_Handler[au8_CMD_Frame[6]].u32_Data_Num_Bytes != u32_Length) return;
  
  /* Handle Data */
  astru_CMD_Handler[au8_CMD_Frame[6]].bool_Msg_Handler(astru_CMD_Handler[au8_CMD_Frame[6]].enum_Msg_ID, 
                                                       &au8_CMD_Frame[7], u32_Length);
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
static void v_DATA_UART_Init(void)
{
  USART_InitTypeDef   USART_InitStructure;
  GPIO_InitTypeDef    GPIO_InitStructure;
  DMA_InitTypeDef     DMA_InitStructure;
  
  /* GPIO configuration */
  RCC_AHB1PeriphClockCmd(DATA_PORT_CLK, ENABLE);
  GPIO_InitStructure.GPIO_Pin   = DATA_TX | DATA_RX;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
  GPIO_Init(DATA_PORT, &GPIO_InitStructure);
  
  GPIO_PinAFConfig(DATA_PORT, DATA_TX_SOURCE, DATA_AF);
  GPIO_PinAFConfig(DATA_PORT, DATA_RX_SOURCE, DATA_AF);
  
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
  
  /* DMA RX configuration */
  RCC_AHB1PeriphClockCmd(DATA_AHB_PERIPH_DMA, ENABLE);
  DMA_DeInit(DATA_RX_DMA_STREAM);  
  DMA_InitStructure.DMA_Channel            = DATA_RX_DMA_CHANNEL;
  DMA_InitStructure.DMA_DIR                = DMA_DIR_PeripheralToMemory;
  DMA_InitStructure.DMA_Memory0BaseAddr    = (uint32_t)&au8_DATA_Rx[0];
  DMA_InitStructure.DMA_PeripheralBaseAddr = DATA_DATA_REG;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_Priority           = DMA_Priority_Low;
  DMA_InitStructure.DMA_FIFOMode           = DMA_FIFOMode_Disable; //direct mode
  DMA_InitStructure.DMA_FIFOThreshold      = DMA_FIFOThreshold_Full;
  DMA_InitStructure.DMA_MemoryInc          = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_Mode               = DMA_Mode_Circular; // avoid buffer overload error
  DMA_InitStructure.DMA_BufferSize         = DATA_RXBUFF_SIZE;
  DMA_InitStructure.DMA_MemoryBurst        = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_PeripheralBurst    = DMA_PeripheralBurst_Single;
  DMA_Init(DATA_RX_DMA_STREAM, &DMA_InitStructure);
  /* Enable  request */
  USART_DMACmd(DATA_USART, USART_DMAReq_Rx, ENABLE);  
  /* Enable DMA RX Channel */
  DMA_Cmd(DATA_RX_DMA_STREAM, ENABLE);
  
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
static void v_RESV_UART_Init(void)
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
  DMA_InitStructure.DMA_Priority           = DMA_Priority_Low;
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

/*********************************END OF FILE**********************************/
