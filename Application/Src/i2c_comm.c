/**
  ******************************************************************************
  * @file    i2c_comm.c
  * @author  Vu Trung Hieu
  * @version V2.0
  * @date    16-September-2017
  * @brief   This file contains functions for i2c (EEPROM, RESV)
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
#include "i2c_comm.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static const uint8_t au8_params_length[NUM_PARAMS_MAX] =
{
  /* Length. */     /* PARAMS_ID. */
  2,                //CODE_VERSION = 0,
  12,               //PARAMS_PID_AZ_MANUAL_POS,
  12,               //PARAMS_PID_EL_MANUAL_POS,
  12,               //PARAMS_PID_AZ_VELOCITY,
  12,               //PARAMS_PID_EL_VELOCITY,
  1,                //PARAMS_PID_AZ_STARTUP_MODE,
  1,                //PARAMS_PID_EL_STARTUP_MODE,
  1,                //PARAMS_PID_AZ_ACTIVE,
  1,                //PARAMS_PID_EL_ACTIVE,
};

/* Private function prototypes -----------------------------------------------*/
static bool bool_I2C_ReadBytes(I2C_TypeDef* I2Cx, uint8_t *pu8_buff,
                          uint8_t u8_slave_add, uint8_t u8_reg_add, uint8_t u8_length);
static bool bool_I2C_WriteBytes(I2C_TypeDef * I2Cx, const uint8_t *pu8_buff,
                           uint8_t u8_slave_add, uint8_t u8_reg_add, uint8_t u8_length);
static uint32_t u32_Params_Get_Pos(ENUM_PARAMS_T enum_params);
static void delay_us(uint32_t micros);

/* Private functions ---------------------------------------------------------*/

/** @defgroup Eeprom and Reserve i2c initialization
 *  @brief   ...
 *
 @verbatim
 ===============================================================================
                ##### EEPROM & Reserve I2C Initialization #####
 ===============================================================================  

 @endverbatim
  * @{
  */

/**
  * @brief  EEP - RESV Init
  * @note   ...
  * @param  none
  * @retval none
  */
void v_I2C_Comm_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  I2C_InitTypeDef  I2C_InitStructure;

  /* GPIO configuration */
  GPIO_InitStructure.GPIO_Pin   = EEP_RESV_SCL | EEP_RESV_SDA;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  GPIO_PinAFConfig(GPIOB, EEP_RESV_SDA_SOURCE, EEP_RESV_AF);
  GPIO_PinAFConfig(GPIOB, EEP_RESV_SCL_SOURCE, EEP_RESV_AF);

  /* I2C configuration */
  RCC_APB1PeriphClockCmd(EEP_RESV_I2C_CLK, ENABLE);
  I2C_DeInit(EEP_RESV_I2C);
  I2C_InitStructure.I2C_ClockSpeed          = EEP_RESV_BAUDRATE;
  I2C_InitStructure.I2C_Mode                = I2C_Mode_I2C;
  I2C_InitStructure.I2C_DutyCycle           = I2C_DutyCycle_2;
  I2C_InitStructure.I2C_OwnAddress1         = 0;
  I2C_InitStructure.I2C_Ack                 = I2C_Ack_Enable;
  I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
  I2C_Init(EEP_RESV_I2C, &I2C_InitStructure);
  I2C_Cmd(EEP_RESV_I2C, ENABLE);
}

/**
  * @}
  */

/** @defgroup I2C Read Write
 *  @brief   ...
 *
 @verbatim
 ===============================================================================
                            ##### I2C Read Write #####
 ===============================================================================

 @endverbatim
  * @{
  */
/**
  * @brief  bool_I2C_ReadBytes
  * @note   ...
  * @param  pu8_buff: pointer to array that will store bytes from slave
  * @param  u8_slave_add: slave's address
  * @param  u8_reg_add: start address of the register of the slave
  * @param  u8_length: length of the array
  * @retval true if succeed and vice versa
  */
static bool bool_I2C_ReadBytes(I2C_TypeDef* I2Cx, uint8_t *pu8_buff,
                          uint8_t u8_slave_add, uint8_t u8_reg_add, uint8_t u8_length)
{
  uint32_t u32_time;

  if (u8_length == 0)   return false;

  /* While the bus is busy */
  u32_time = I2C_TIMEOUT;
  while (I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY))
  {
    if ((u32_time--) == 0) return false;
  }

  /* Send START condition */
  I2C_GenerateSTART(I2Cx, ENABLE);

  /* Test on EV5 and clear it (cleared by reading SR1 then writing to DR) */
  u32_time = I2C_TIMEOUT;
  while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT))
  {
    if ((u32_time--) == 0) return false;
  }

  /* Send Slave address for write */
  I2C_Send7bitAddress(I2Cx, u8_slave_add, I2C_Direction_Transmitter);
      
  /* Test on EV6 and clear it */
  u32_time = I2C_TIMEOUT;
  while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
  {
    if ((u32_time--) == 0) return false;
  }

  /* Send the Register address to read from: Only one byte address */
  I2C_SendData(I2Cx, u8_reg_add);  

  /* Test on EV8 and clear it */
  u32_time = I2C_TIMEOUT;
  while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) 
  {
    if ((u32_time--) == 0) return false;
  }

  /* Send START condition a second time to RESTART bus */
  I2C_GenerateSTART(I2Cx, ENABLE);

  /* Test on EV5 and clear it (cleared by reading SR1 then writing to DR) */
  u32_time = I2C_TIMEOUT;
  while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT))
  {
    if ((u32_time--) == 0) return false;
  } 

  /* Send Slave address for read */
  I2C_Send7bitAddress(I2Cx, u8_slave_add, I2C_Direction_Receiver);  

  /* SHOULD DISABLE INTERRUPT HERE, IF NOT IT MAY BE CORRUPT I2C TRANSACTION */
  __disable_irq();

  /* Test on EV6 and clear it */
  u32_time = I2C_TIMEOUT;
  while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED))
  {
      if ((u32_time--) == 0) return false;
  }

  while (u8_length)
  {
    if (u8_length == 1)
    {
        /* This configuration should be in the last second transfer byte */
        /* Disable Acknowledgement */
        I2C_AcknowledgeConfig(I2Cx, DISABLE);
        
        /* Send STOP Condition */
        I2C_GenerateSTOP(I2Cx, ENABLE);
    }

    /* Test on EV7 and clear it */
    u32_time = I2C_TIMEOUT;
    while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED))
    {
      if ((u32_time--) == 0) return false;
    }

    /* Read the byte received from the Sensor */
    /* Point to the next location where the byte read will be saved */
    *pu8_buff = I2C_ReceiveData(I2Cx);
    pu8_buff++;
    
    /* Decrease the read bytes counter */
    u8_length--;
  }

  /* Wait to make sure that STOP control bit has been cleared */
  u32_time = I2C_TIMEOUT;
  while (I2Cx->CR1 & I2C_CR1_STOP)
  {
      if ((u32_time--) == 0) return false;
  }

  /* Re-Enable Acknowledgement to be ready for another reception */
  I2C_AcknowledgeConfig(I2Cx, ENABLE);  
    
  /* ENABLE ALL INTERRUPT */
  __enable_irq();     
  return true; 
}

/**
  * @brief  bool_I2C_WriteBytes
  * @note   ...
  * @param  pu8_buff: pointer to array that will store bytes from slave
  * @param  u8_slave_add: slave's address
  * @param  u8_reg_add: start address of the register of the slave
  * @param  u8_length: length of the array
  * @retval true if succeed and vice versa
  */
static bool bool_I2C_WriteBytes(I2C_TypeDef * I2Cx, const uint8_t *pu8_buff,
                           uint8_t u8_slave_add, uint8_t u8_reg_add, uint8_t u8_length)
{
  uint32_t u32_time;
  I2C_AcknowledgeConfig(I2Cx, ENABLE); 
  /* While the bus is busy */
  u32_time = I2C_TIMEOUT;
  while (I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY))
  {
    if ((u32_time--) == 0) return false;
  }

  /* Send START condition */
  I2C_GenerateSTART(I2Cx, ENABLE);

  /* Test on EV5 and clear it (cleared by reading SR1 then writing to DR) */
  u32_time = I2C_TIMEOUT;
  while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT))
  {
    if ((u32_time--) == 0) return false;
  }

  /* Send Slave address for write */
  I2C_Send7bitAddress(I2Cx, u8_slave_add, I2C_Direction_Transmitter);

  /* Test on EV6 and clear it */
  u32_time = I2C_TIMEOUT;
  while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
  {
    if ((u32_time--) == 0) return false;
  }

  /* Send the Register address to read from: Only one byte address */
  I2C_SendData(I2Cx, u8_reg_add);  

  /* Test on EV8 and clear it */
  u32_time = I2C_TIMEOUT;
  while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
  {
      if (u32_time-- == 0) return false;
  }
    
  /* SHOULD DISABLE INTERRUPT HERE, IF NOT IT MAY BE CORRUPT I2C TRANSACTION */
  __disable_irq();
  while (u8_length)
  {
    /* Send the data & increase the pointer of write buffer */
    I2C_SendData(I2Cx, *pu8_buff); 
    pu8_buff++;
    u8_length--;  
    /* Test on EV8_2 to ensure data is transmitted, can used EV_8 for faster transmission*/
    u32_time = I2C_TIMEOUT;
    while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
    {
      if ((u32_time--) == 0) return false;
    }
  }

  /* Send STOP Condition */
  I2C_GenerateSTOP(I2Cx, ENABLE);
  /* ENABLE ALL INTERRUPT */
  __enable_irq();
  return true;
}
/**
  * @}
  */

/** @defgroup EPP Read Write
 *  @brief   ...
 *
 @verbatim
 ===============================================================================
                            ##### EPP Read Write #####
 ===============================================================================  

 @endverbatim
  * @{
  */
/**
  * @brief  bool_EEP_ReadBytes
  * @note   ...
  * @param  pu8_buff: pointer to array that will store bytes from EEP
  * @param  u16_reg_add: start regiter address from 0 to 511 
  *         (EEPROM 24C04: 512 bytes/address)
  *         And it is divided into 2 pages (256 bytes/page)
  * @param  u16_length: length of data to read
  * @retval true if succeed and vice versa
  */
bool bool_EEP_ReadBytes(uint8_t* pu8_buff, uint16_t u16_reg_add, uint16_t u16_length)
{
  uint16_t u16_reg_add_buff;
  uint8_t u8_eep_add_buff;
  uint16_t u16_idx;
    
  for (u16_idx = 0; u16_idx < u16_length; u16_idx++)
  {
    u8_eep_add_buff = EEP_ADD;
    u16_reg_add_buff = u16_reg_add + u16_idx;
    
    if (u16_reg_add_buff > 511) return false; // Out of range
    else if (u16_reg_add_buff > 255)
    {
      /* Page 1: insert bit page 1 in eeprom address */
      u16_reg_add_buff -= 256;
      u8_eep_add_buff |= EEP_PAGE_1;
    }
    /*
    else if (u16_reg_add_buff < 255) Page 0: do nothing
    */
      
    /* Random read 1 byte */
    if (bool_I2C_ReadBytes(EEP_RESV_I2C, pu8_buff, u8_eep_add_buff, u16_reg_add_buff, 1) == false) 
    {
      return false;
    }
    pu8_buff++;
  }
  return true;
}

/**
  * @brief  bool_EEP_WriteBytes
  * @note   ...
  * @param  pu8_buff: pointer to write array data
  * @param  u16_reg_add: start regiter address from 0 to 511 
  *         (EEPROM 24C04: 512 bytes/address)
  *         And it is divided into 2 pages (256 bytes/page)
  * @param  u16_length: length of data to write
  * @retval true if succeed and vice versa
  */
bool bool_EEP_WriteBytes(const uint8_t* pu8_buff, uint16_t u16_reg_add, uint16_t u16_length)
{
  uint16_t u16_reg_add_buff;
  uint8_t u8_eep_add_buff;
  uint16_t u16_idx;
    
  for (u16_idx = 0; u16_idx < u16_length; u16_idx++)
  {
    u8_eep_add_buff = EEP_ADD;
    u16_reg_add_buff = u16_reg_add + u16_idx;

    if (u16_reg_add_buff > 511) return false; // Out of range
    else if (u16_reg_add_buff > 255)
    {
      /* Page 1: insert bit page 1 in eeprom address */
      u16_reg_add_buff -= 256;
      u8_eep_add_buff |= EEP_PAGE_1;
    }
    /*
    else if (reg_add < 255) page 0: do nothing
    */

    // Random write 1 byte into eeprom
    if (bool_I2C_WriteBytes(EEP_RESV_I2C, pu8_buff, u8_eep_add_buff, u16_reg_add_buff, 1) == false)
    {
      return false;
    }
    pu8_buff++;
    delay_us(5000); //Important: Maximum write cycle time = 5ms
  }
  return true;
}
/**
  * @}
  */

/** @defgroup Gimbal Params
 *  @brief   ...
 *
 @verbatim
 ===============================================================================
                             ##### Gimbal Params #####
 ===============================================================================  

 @endverbatim
  * @{
  */

/**
  * @brief  Get Pos of params in real eeprom
  * @note   ...
  * @param  enum_params: Desired Param
  * @retval u32_pos: Position of Params
  */
static uint32_t u32_Params_Get_Pos(ENUM_PARAMS_T enum_params)
{
  uint32_t u32_idx = 0, u32_pos = 0;
  for (u32_idx = 0; u32_idx < enum_params; u32_idx++)
  {
    u32_pos += au8_params_length[u32_idx];
  }
  return u32_pos;
}

/**
  * @brief  save params
  * @note   ...
  * @param  enum_params: Desired Param
  * @param  pu8Data: pointer of data need to save
  * @retval true if succeed and vice versa
  */
bool bool_Params_Save(ENUM_PARAMS_T enum_params, const uint8_t *pu8_data)
{
  return bool_EEP_WriteBytes(pu8_data, u32_Params_Get_Pos(enum_params), au8_params_length[enum_params]);
}

/**
  * @brief  load params
  * @note   ...
  * @param  enum_params: Desired Param
  * @param  pu8Data: pointer of data store data loaded from eeprom
  * @retval true if succeed and vice versa
  */
bool bool_Params_Load(ENUM_PARAMS_T enum_params, uint8_t *pu8_data)
{
  return bool_EEP_ReadBytes(pu8_data, u32_Params_Get_Pos(enum_params), au8_params_length[enum_params]);
}

/**
  * @brief  delay in us
  * @note   ...
  * @param  micros: Desired delay in us
  * @retval none
  */
static void delay_us(uint32_t micros)
{
  RCC_ClocksTypeDef RCC_Clocks;
  /* Get system clocks */
  RCC_GetClocksFreq(&RCC_Clocks);
  micros = micros * (RCC_Clocks.HCLK_Frequency / 4000000) - 10;
  /* 4 cycles for one loop */
  while (micros--);
}
/**
  * @}
  */

/*********************************END OF FILE**********************************/
