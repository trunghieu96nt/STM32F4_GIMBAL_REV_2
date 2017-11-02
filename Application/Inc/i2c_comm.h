/**
  ******************************************************************************
  * @file    i2c_comm.h
  * @author  Vu Trung Hieu
  * @version V2.0
  * @date    16-September-2017
  * @brief   This file contains all the functions prototypes for i2c_comm.c
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __I2C_COMM_H
#define __I2C_COMM_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stdbool.h"
#include "stdint.h"

/* Exported types ------------------------------------------------------------*/
typedef enum{
  PARAMS_CODE_VERSION = 0,
  PARAMS_PID_AZ_MANUAL_POS,
  PARAMS_PID_EL_MANUAL_POS,
} ENUM_PARAMS_T;

/* Exported constants --------------------------------------------------------*/
/** @defgroup EEP - RESV Init
  * @{
  */
#define EEP_RESV_I2C               I2C1
#define EEP_RESV_I2C_CLK           RCC_APB1Periph_I2C1
#define EEP_RESV_PORT              GPIOB
#define EEP_RESV_SCL               GPIO_Pin_8
#define EEP_RESV_SCL_SOURCE        GPIO_PinSource8
#define EEP_RESV_SDA               GPIO_Pin_9
#define EEP_RESV_SDA_SOURCE        GPIO_PinSource9
#define EEP_RESV_AF                GPIO_AF_I2C1
#define EEP_RESV_BAUDRATE          400000  // 400kHz
#define EEP_RESV_DATA_REG          (uint32_t)EEP_I2C + 0x10
#define I2C_TIMEOUT                100000
/**
  * @}
  */

/** @defgroup EEPROM AT24C04
  * @{
  */
/* Note: EEPROM AT24C04 address is 8 bits:|1010|A2|A1|Page|R/W| */
#define EEP_ADD                    0xA0
#define EEP_PAGE_0                 0x00
#define EEP_PAGE_1                 0x02
/**
  * @}
  */

/** @defgroup Gimbal Params
  * @{
  */
#define NUM_PARAMS_MAX             20
/**
  * @}
  */

/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
/* Initialization and Configuration functions *********************************/
void v_I2C_Comm_Init(void);

/* EPP Read and Write functions ***********************************************/
bool bool_EEP_ReadBytes(uint8_t* pu8_buff, uint16_t u16_reg_add, uint16_t u16_length);
bool bool_EEP_WriteBytes(const uint8_t* pu8_buff, uint16_t u16_reg_add, uint16_t u16_length);

/* Params Save and Load Functions *********************************************/
bool bool_Params_Save(ENUM_PARAMS_T enum_params, const uint8_t *pu8_data);
bool bool_Params_Load(ENUM_PARAMS_T enum_params, uint8_t *pu8_data);

#ifdef __cplusplus
}
#endif

#endif /* __I2C_COMM_H */

/*********************************END OF FILE**********************************/
