/**
  ******************************************************************************
  * @file    adis_sensor.h
  * @author  Vu Trung Hieu
  * @version V2.0
  * @date    08-September-2017
  * @brief   This file contains all the functions prototypes for adis_sensor.c
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ADIS_SENSOR_H
#define __ADIS_SENSOR_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stdbool.h"
#include "stdint.h"

/* Exported types ------------------------------------------------------------*/
typedef struct
{
  bool bool_available;
  float flt_euler_x;
  float flt_euler_y;
  float flt_euler_z;
  float flt_gyro_x;
  float flt_gyro_y;
  float flt_gyro_z;
  float flt_acc_x;
  float flt_acc_y;
  float flt_acc_z;
} STRU_IMU_DATA_T;

/* Exported constants --------------------------------------------------------*/
/** @defgroup ADIS
  * @{
  */
#define USE_IMU_RX_DMA_IT     0

#if USE_IMU_RX_DMA_IT
#define IMU_TXBUFF_SIZE       87
#define IMU_RXBUFF_SIZE       87
#else
#define IMU_TXBUFF_SIZE       1024
#define IMU_RXBUFF_SIZE       1024
#endif

#define IMU_FRAME_LEN         65
#define IMU_FRAME_LEN_MAX     (IMU_FRAME_LEN + 1)
#define IMU_ELEMENT_MAX_LEN   15
#define IMU_START_FRAME       0x0a
#define IMU_END_FRAME         0x0d

#define IMU_SCALE_EULER_UNIT  0.0001f // deg
#define IMU_SCALE_GYRO_UNIT   0.1f // mrad/s
#define IMU_SCALE_MAG_UNIT    0.1f // mgauss
#define IMU_SCALE_ACC_UNIT    0.1f // mg
#define IMU_SCALE_FOG_UNIT    0.01f // mdeg/s

#define IMU_USART             UART4
#define IMU_USART_CLK         RCC_APB1Periph_UART4
#define IMU_PORT              GPIOA
#define IMU_PORT_CLK          RCC_AHB1Periph_GPIOA
#define IMU_TX                GPIO_Pin_0
#define IMU_TX_SOURCE         GPIO_PinSource0
#define IMU_RX                GPIO_Pin_1
#define IMU_RX_SOURCE         GPIO_PinSource1
#define IMU_AF                GPIO_AF_UART4
#define IMU_BAUDRATE          (uint32_t)921600 //921600//115200

#define IMU_AHB_PERIPH_DMA    RCC_AHB1Periph_DMA1
#define IMU_DATA_REG          (uint32_t)IMU_USART + 0x04
#define IMU_TX_DMA_STREAM     DMA1_Stream4
#define IMU_TX_DMA_CHANNEL    DMA_Channel_4
#define IMU_TX_STREAM_IRQ     DMA1_Stream4_IRQn
#define IMU_TX_DMA_FLAG       DMA_FLAG_TCIF4
#define IMU_RX_DMA_STREAM     DMA1_Stream2
#define IMU_RX_DMA_CHANNEL    DMA_Channel_4
#define IMU_RX_STREAM_IRQ     DMA1_Stream2_IRQn
#define IMU_RX_TC_IT_FLAG     DMA_IT_TCIF2
#define IMU_RX_HT_IT_FLAG     DMA_IT_HTIF2
#define IMU_RX_Interrupt      DMA1_Stream2_IRQHandler
/**
  * @}
  */

/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
/* Initialization and Configuration functions *********************************/
void v_ADIS_Init(void);

/* ADIS Read functions ********************************************************/
bool bool_ADIS_Read(void);
bool bool_ADIS_Read_IsTimeout(uint32_t u32_timeout_ms);
STRU_IMU_DATA_T stru_Get_IMU_Data(void);

#ifdef __cplusplus
}
#endif

#endif /* __ADIS_SENSOR_H */

/*********************************END OF FILE**********************************/
