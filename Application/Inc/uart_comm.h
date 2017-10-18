/**
  ******************************************************************************
  * @file    uart_comm.h
  * @author  Vu Trung Hieu
  * @version V2.0
  * @date    11-September-2017
  * @brief   This file contains all the functions prototypes for comm_driver.c
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __UART_COMM_H
#define __UART_COMM_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stdbool.h"
#include "stdint.h"

/* Exported types ------------------------------------------------------------*/
typedef enum{
  MSG_NONE                = 0x00,
  MSG_HOME                = 0x01,
  MSG_STOP                = 0x02,
  MSG_EMERGENCY_STOP      = 0x03,
  MSG_STABILIZING_MODE    = 0x04,
  MSG_GET_MODE            = 0x05,
  MSG_SET_POS             = 0x06,
  MSG_SET_VEL             = 0x07,
  MSG_SET_POS_VEL         = 0x08,
  MSG_GET_POS             = 0x09,
  MSG_SET_KP              = 0x0A,
  MSG_SET_KI              = 0x0B,
  MSG_SET_KD              = 0x0C,
  MSG_SET_KFF1            = 0x0D,
  MSG_SET_KFF2            = 0x0E,
  MSG_GET_PARAMS          = 0x0F,
  MSG_SET_ACTIVE_AXIS     = 0x10,
  MSG_GET_ACTIVE_AXIS     = 0x11,
} ENUM_MSG_ID_T;

typedef bool (*CMD_HANDLER_FUNC)(uint8_t, uint8_t *, uint32_t);

typedef struct{
  ENUM_MSG_ID_T enum_Msg_ID;
  uint32_t u32_Data_Num_Bytes;
  CMD_HANDLER_FUNC bool_Msg_Handler;
} STRU_CMD_HANDLER_T;

/* Exported constants --------------------------------------------------------*/
/** @defgroup CMD UART (Receiver)
  * @{
  */
#define CMD_RX_FRAME_TIMEOUT      50 //ms
#define CMD_NUM_MSG_ID_MAX        20

#define CMD_TXBUFF_SIZE           64
#define CMD_RXBUFF_SIZE           512
#define CMD_FRAME_LEN_MAX         64

#define CMD_USART                 USART2
#define CMD_USART_CLK             RCC_APB1Periph_USART2
#define CMD_PORT                  GPIOA
#define CMD_PORT_CLK              RCC_AHB1Periph_GPIOA
#define CMD_TX                    GPIO_Pin_2
#define CMD_TX_SOURCE             GPIO_PinSource2
#define CMD_RX                    GPIO_Pin_3
#define CMD_RX_SOURCE             GPIO_PinSource3
#define CMD_AF                    GPIO_AF_USART2
#define CMD_BAUDRATE              (uint32_t)115200 //921600 //115200

#define CMD_AHB_PERIPH_DMA        RCC_AHB1Periph_DMA1
#define CMD_DATA_REG              (uint32_t)CMD_USART + 0x04
#define CMD_TX_DMA_STREAM         DMA1_Stream6
#define CMD_TX_DMA_CHANNEL        DMA_Channel_4
#define CMD_TX_STREAM_IRQ         DMA1_Stream6_IRQn
#define CMD_TX_DMA_FLAG           DMA_FLAG_TCIF6
#define CMD_RX_DMA_STREAM         DMA1_Stream5
#define CMD_RX_DMA_CHANNEL        DMA_Channel_4
#define CMD_RX_STREAM_IRQ         DMA1_Stream5_IRQn
#define CMD_RX_TC_IT_FLAG         DMA_IT_TCIF5
#define CMD_RX_HT_IT_FLAG         DMA_IT_HTIF5
#define CMD_RX_Interrupt          DMA1_Stream5_IRQHandler
/**
  * @}
  */

/** @defgroup DATA UART Define (Sender)
  * @{
  */
#define DATA_TXBUFF_SIZE          512
#define DATA_RXBUFF_SIZE          64

#define DATA_USART                USART3
#define DATA_USART_CLK            RCC_APB1Periph_USART3
#define DATA_PORT                 GPIOB
#define DATA_PORT_CLK             RCC_AHB1Periph_GPIOB
#define DATA_TX                   GPIO_Pin_10
#define DATA_TX_SOURCE            GPIO_PinSource10
#define DATA_RX                   GPIO_Pin_11
#define DATA_RX_SOURCE            GPIO_PinSource11
#define DATA_AF                   GPIO_AF_USART3
#define DATA_BAUDRATE             (uint32_t)115200 //921600 //115200

#define DATA_AHB_PERIPH_DMA       RCC_AHB1Periph_DMA1
#define DATA_DATA_REG             (uint32_t)DATA_USART + 0x04
#define DATA_TX_DMA_STREAM        DMA1_Stream3
#define DATA_TX_DMA_CHANNEL       DMA_Channel_4
#define DATA_TX_STREAM_IRQ        DMA1_Stream3_IRQn
#define DATA_TX_DMA_FLAG          DMA_FLAG_TCIF3
#define DATA_RX_DMA_STREAM        DMA1_Stream1
#define DATA_RX_DMA_CHANNEL       DMA_Channel_4
#define DATA_RX_STREAM_IRQ        DMA1_Stream1_IRQn
#define DATA_RX_TC_IT_FLAG        DMA_IT_TCIF1
#define DATA_RX_HT_IT_FLAG        DMA_IT_HTIF1
#define DATA_RX_Interrupt         DMA1_Stream1_IRQHandler
/**
  * @}
  */

/** @defgroup RESV UART (General)
  * @{
  */
#define RESV_TXBUFF_SIZE          64
#define RESV_RXBUFF_SIZE          1024

#define RESV_USART                UART5
#define RESV_USART_CLK            RCC_APB1Periph_UART5
#define RESV_PORT_TX              GPIOC
#define RESV_PORT_TX_CLK          RCC_AHB1Periph_GPIOC
#define RESV_PORT_RX              GPIOD
#define RESV_PORT_RX_CLK          RCC_AHB1Periph_GPIOD
#define RESV_TX                   GPIO_Pin_12
#define RESV_TX_SOURCE            GPIO_PinSource12
#define RESV_RX                   GPIO_Pin_2
#define RESV_RX_SOURCE            GPIO_PinSource2
#define RESV_AF                   GPIO_AF_UART5
#define RESV_BAUDRATE             (uint32_t)115200 //921600 //115200

#define RESV_AHB_PERIPH_DMA       RCC_AHB1Periph_DMA1
#define RESV_DATA_REG             (uint32_t)RESV_USART + 0x04
#define RESV_TX_DMA_STREAM        DMA1_Stream7
#define RESV_TX_DMA_CHANNEL       DMA_Channel_4
#define RESV_TX_STREAM_IRQ        DMA1_Stream7_IRQn
#define RESV_TX_DMA_FLAG          DMA_FLAG_TCIF7
#define RESV_RX_DMA_STREAM        DMA1_Stream0
#define RESV_RX_DMA_CHANNEL       DMA_Channel_4
#define RESV_RX_STREAM_IRQ        DMA1_Stream0_IRQn
#define RESV_RX_TC_IT_FLAG        DMA_IT_TCIF0
#define RESV_RX_HT_IT_FLAG        DMA_IT_HTIF0
#define RESV_RX_Interrupt         DMA1_Stream0_IRQHandler
/**
  * @}
  */

/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
/* Initialization and Configuration functions *********************************/
void v_UART_Comm_Init(void);

/* Communication functions ****************************************************/
bool bool_DATA_Send(const uint8_t *pu8_Message, uint32_t u32_Message_Size);
bool bool_CMD_Send(const uint8_t *pu8_Message, uint32_t u32_Message_Size);
bool bool_RESV_Send(const uint8_t *pu8_Message, uint32_t u32_Message_Size);

void v_CMD_Receive(void);

#ifdef __cplusplus
}
#endif

#endif /* __UART_COMM_H */

/*********************************END OF FILE**********************************/
