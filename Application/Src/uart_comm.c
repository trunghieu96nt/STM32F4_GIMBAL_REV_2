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
static uint8_t au8_CMD_rx[CMD_RXBUFF_SIZE]= {0};
static uint8_t au8_DATA_rx[DATA_RXBUFF_SIZE]= {0};
static uint8_t au8_DATA_tx[DATA_TXBUFF_SIZE]= {0};
static uint8_t au8_RESV_rx[RESV_RXBUFF_SIZE]= {0};

extern bool bool_None_Handler             (uint8_t u8_msg_id, uint8_t *pu8_payload, uint32_t u32_payload_cnt);
extern bool bool_Home_Handler             (uint8_t u8_msg_id, uint8_t *pu8_payload, uint32_t u32_payload_cnt);
extern bool bool_Stop_Handler             (uint8_t u8_msg_id, uint8_t *pu8_payload, uint32_t u32_payload_cnt);
extern bool bool_Emergency_Stop_Handler   (uint8_t u8_msg_id, uint8_t *pu8_payload, uint32_t u32_payload_cnt);
extern bool bool_Stabilizing_Mode_Handler (uint8_t u8_msg_id, uint8_t *pu8_payload, uint32_t u32_payload_cnt);
extern bool bool_Get_Mode_Handler         (uint8_t u8_msg_id, uint8_t *pu8_payload, uint32_t u32_payload_cnt);
extern bool bool_Set_Pos_Handler          (uint8_t u8_msg_id, uint8_t *pu8_payload, uint32_t u32_payload_cnt);
extern bool bool_Set_Vel_Handler          (uint8_t u8_msg_id, uint8_t *pu8_payload, uint32_t u32_payload_cnt);
extern bool bool_Set_Pos_Vel_Handler      (uint8_t u8_msg_id, uint8_t *pu8_payload, uint32_t u32_payload_cnt);
extern bool bool_Get_Pos_Handler          (uint8_t u8_msg_id, uint8_t *pu8_payload, uint32_t u32_payload_cnt);
extern bool bool_Set_Kp_Handler           (uint8_t u8_msg_id, uint8_t *pu8_payload, uint32_t u32_payload_cnt);
extern bool bool_Set_Ki_Handler           (uint8_t u8_msg_id, uint8_t *pu8_payload, uint32_t u32_payload_cnt);
extern bool bool_Set_Kd_Handler           (uint8_t u8_msg_id, uint8_t *pu8_payload, uint32_t u32_payload_cnt);
extern bool bool_Set_Kff1_Handler         (uint8_t u8_msg_id, uint8_t *pu8_payload, uint32_t u32_payload_cnt);
extern bool bool_Set_Kff2_Handler         (uint8_t u8_msg_id, uint8_t *pu8_payload, uint32_t u32_payload_cnt);
extern bool bool_Get_Params_Handler       (uint8_t u8_msg_id, uint8_t *pu8_payload, uint32_t u32_payload_cnt);
extern bool bool_Set_Active_Axis_Handler  (uint8_t u8_msg_id, uint8_t *pu8_payload, uint32_t u32_payload_cnt);
extern bool bool_Get_Active_Axis_Handler  (uint8_t u8_msg_id, uint8_t *pu8_payload, uint32_t u32_payload_cnt);
extern bool bool_Send_Image_Data_Handler  (uint8_t u8_msg_id, uint8_t *pu8_payload, uint32_t u32_payload_cnt);

const STRU_CMD_HANDLER_T astru_CMD_handler[CMD_NUM_MSG_ID_MAX] =
{
  {MSG_NONE,                0,    bool_None_Handler},
  {MSG_HOME,                1,    bool_Home_Handler},
  {MSG_STOP,                1,    bool_Stop_Handler},
  {MSG_EMERGENCY_STOP,      1,    bool_Emergency_Stop_Handler},
  {MSG_STABILIZING_MODE,    2,    bool_Stabilizing_Mode_Handler},
  {MSG_GET_MODE,            1,    bool_Get_Mode_Handler},
  {MSG_SET_POS,             5,    bool_Set_Pos_Handler},
  {MSG_SET_VEL,             5,    bool_Set_Vel_Handler},
  {MSG_SET_POS_VEL,         9,    bool_Set_Pos_Vel_Handler},
  {MSG_GET_POS,             1,    bool_Get_Pos_Handler},
  {MSG_SET_KP,              6,    bool_Set_Kp_Handler},
  {MSG_SET_KI,              6,    bool_Set_Ki_Handler},
  {MSG_SET_KD,              6,    bool_Set_Kd_Handler},
  {MSG_SET_KFF1,            6,    bool_Set_Kff1_Handler},
  {MSG_SET_KFF2,            6,    bool_Set_Kff2_Handler},
  {MSG_GET_PARAMS,          2,    bool_Get_Params_Handler},
  {MSG_SET_ACTIVE_AXIS,     2,    bool_Set_Active_Axis_Handler},
  {MSG_GET_ACTIVE_AXIS,     1,    bool_Get_Active_Axis_Handler},
  {MSG_SEND_IMAGE_DATA,     5,    bool_Send_Image_Data_Handler}
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
  DMA_InitStructure.DMA_Memory0BaseAddr    = (uint32_t)&au8_CMD_rx[0];
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
  * @param  pu8_message: pointer message to send (should be static)
  * @param  u32_message_size: number of char to send
  * @retval true if success and vice versa
  */
bool bool_CMD_Send(const uint8_t *pu8_message, uint32_t u32_message_size)
{
  if(u32_message_size > CMD_TXBUFF_SIZE)
  {
    return false;
  }
  else
  {
    //clear flag
    DMA_ClearFlag(CMD_TX_DMA_STREAM, CMD_TX_DMA_FLAG);
    DMA_MemoryTargetConfig(CMD_TX_DMA_STREAM, (uint32_t)pu8_message, DMA_Memory_0);
    DMA_SetCurrDataCounter(CMD_TX_DMA_STREAM, u32_message_size);
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
  static uint32_t u32_idx_pre = 0, u32_idx = 0, u32_time_tick = 0;
  static bool bool_header_detected = false, bool_length_detected = false;
  static uint8_t au8_CMD_frame[CMD_FRAME_LEN_MAX] = {'G', 'B', 0x02, 0x01};
  uint32_t u32_length, u32_idx_cur, u32_cnt;
  uint16_t u16_crc_check;
  
  u32_idx_cur = CMD_RXBUFF_SIZE - CMD_RX_DMA_STREAM->NDTR;
  
  if (u32_idx_cur == u32_idx_pre)
  {
    if (bool_header_detected == true)
    {
      if (SysTick_IsTimeout(u32_time_tick, CMD_RX_FRAME_TIMEOUT))
      {
        u32_idx = 0;
        bool_length_detected = false;
        bool_header_detected = false;
      }
    }
    return;
  }
  
  u32_time_tick = SysTick_GetTick();
  
  /* Search Header "GB" */
  if (bool_header_detected == false)
  {
    while (true)
    {
      if (u32_idx_cur >= u32_idx_pre) u32_length = u32_idx_cur - u32_idx_pre;
      else u32_length = CMD_RXBUFF_SIZE - (u32_idx_pre - u32_idx_cur);
      
      if (u32_length < 2) return;
      
      if (*(au8_CMD_rx + u32_idx_pre) == 'G')
      {
        if (++u32_idx_pre >= CMD_RXBUFF_SIZE) u32_idx_pre = 0;
        if (*(au8_CMD_rx + u32_idx_pre) == 'B')
        {
          if (++u32_idx_pre >= CMD_RXBUFF_SIZE) u32_idx_pre = 0;
          bool_header_detected = true;
          break;
        }
      }
      else
      {
        if (++u32_idx_pre >= CMD_RXBUFF_SIZE) u32_idx_pre = 0;
      }
    }
  }
  
  /* Search Length of Frame*/
  if (bool_length_detected == false)
  {
    if (u32_idx_cur >= u32_idx_pre) u32_length = u32_idx_cur - u32_idx_pre;
    else u32_length = CMD_RXBUFF_SIZE - (u32_idx_pre - u32_idx_cur);
    
    if (u32_length < 5) return;
    
    /* Check DEST_ID - ID_GIMBAL_CONTROLER */
    if (au8_CMD_rx[u32_idx_pre] != 0x02) 
    {
      bool_header_detected = false;
      return;
    }
    if (++u32_idx_pre == CMD_RXBUFF_SIZE) u32_idx_pre = 0;
    
    /* Check SRC_ID - ID_GUI_SOFTWARE */
    if (au8_CMD_rx[u32_idx_pre] != 0x01) 
    {
      bool_header_detected = false;
      return;
    }
    if (++u32_idx_pre == CMD_RXBUFF_SIZE) u32_idx_pre = 0;
    
    /* Get Seq */
    au8_CMD_frame[4] = au8_CMD_rx[u32_idx_pre];
    if (++u32_idx_pre == CMD_RXBUFF_SIZE) u32_idx_pre = 0;
    
    /* Get Length */
    bool_length_detected = true;
    au8_CMD_frame[5] = au8_CMD_rx[u32_idx_pre];
    if (++u32_idx_pre == CMD_RXBUFF_SIZE) u32_idx_pre = 0;
  }
  
  /* Getting Frame */
  while (true)
  {
    au8_CMD_frame[u32_idx + 6] = *(au8_CMD_rx + u32_idx_pre);
    if (++u32_idx_pre >= CMD_RXBUFF_SIZE) u32_idx_pre = 0;
    if (++u32_idx == au8_CMD_frame[5])
    {
      u32_idx = 0;
      bool_length_detected = false;
      bool_header_detected = false;
      break;
    }
    if (u32_idx_pre == u32_idx_cur) return;
  }
  
  /* Check CRC */
  u16_crc_check = 0;
  u32_length = au8_CMD_frame[5] + 6 - 2; //Total Length except 2 byte CRC
  for (u32_cnt = 0; u32_cnt < u32_length; u32_cnt++)
  {
    u16_crc_check += au8_CMD_frame[u32_cnt];
  }
  u16_crc_check = ~u16_crc_check;
  if (((u16_crc_check >> 8) & 0x0FF) != au8_CMD_frame[u32_length]) return;
  if ((u16_crc_check & 0x0FF) != au8_CMD_frame[u32_length + 1]) return;
  
  /* Check enough length */
  u32_length = au8_CMD_frame[5] - 3; //Length Payload
  if (astru_CMD_handler[au8_CMD_frame[6]].u32_data_num_bytes > u32_length) return;
  
  /* Handle Data */
  astru_CMD_handler[au8_CMD_frame[6]].bool_msg_handler(astru_CMD_handler[au8_CMD_frame[6]].enum_msg_id, 
                                                       &au8_CMD_frame[7], u32_length);
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
  DMA_InitStructure.DMA_Memory0BaseAddr    = (uint32_t)&au8_DATA_rx[0];
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
  DMA_InitStructure.DMA_Memory0BaseAddr    = (uint32_t)&au8_DATA_tx[0];
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
  * @param  pu8_message: pointer message to send (should be static)
  * @param  u32_message_size: number of char to send
  * @retval true if success and vice versa
  */
bool bool_DATA_Send(const uint8_t *pu8_message, uint32_t u32_message_size)
{
  uint32_t u32_idx;
  
  if(u32_message_size > DATA_TXBUFF_SIZE)
  {
    return false;
  }
  else
  {
    //copy buff
    for (u32_idx = 0; u32_idx < u32_message_size; u32_idx++)
    {
      au8_DATA_tx[u32_idx] = *(pu8_message + u32_idx);
    }
    
    //clear flag
    DMA_ClearFlag(DATA_TX_DMA_STREAM, DATA_TX_DMA_FLAG);
    //DMA_MemoryTargetConfig(DATA_TX_DMA_STREAM, (uint32_t)au8_DATA_tx, DMA_Memory_0);
    DMA_SetCurrDataCounter(DATA_TX_DMA_STREAM, u32_message_size);
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
  DMA_InitStructure.DMA_Memory0BaseAddr    = (uint32_t)&au8_RESV_rx[0];
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
  * @param  pu8_message: pointer message to send (should be static)
  * @param  u32_message_size: number of char to send
  * @retval true if success and vice versa
  */
bool bool_RESV_Send(const uint8_t *pu8_message, uint32_t u32_message_size)
{
  if(u32_message_size > RESV_TXBUFF_SIZE)
  {
    return false;
  }
  else
  {
    //clear flag
    DMA_ClearFlag(RESV_TX_DMA_STREAM, RESV_TX_DMA_FLAG);
    DMA_MemoryTargetConfig(RESV_TX_DMA_STREAM, (uint32_t)pu8_message, DMA_Memory_0);
    DMA_SetCurrDataCounter(RESV_TX_DMA_STREAM, u32_message_size);
    //STM_TX_DMA_STREAM->NDTR = BUFF_SIZE;
    DMA_Cmd(RESV_TX_DMA_STREAM, ENABLE);
    return true;
  }
}

/**
  * @}
  */

/*********************************END OF FILE**********************************/
