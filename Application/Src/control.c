/**
  ******************************************************************************
  * @file    control.c
  * @author  Vu Trung Hieu
  * @version V2.0
  * @date    15-September-2017
  * @brief   This file contains functions for controling gimbal
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
#include "control.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/** @defgroup CMD Handler
 *  @brief   ...
 *
 @verbatim
 ===============================================================================
                           ##### CMD Handler #####
 ===============================================================================  

 @endverbatim
  * @{
  */

/**
  * @brief  CMD Handler (CMD received from UART)
  * @note   comm_driver.c need these funtions
  * @param  pu8_Data: Pointer to Data
  * @param  u32_Data_Cnt: Number of Data in bytes
  * @retval true if successful and vice versa
  */
bool bool_None_Handler(uint8_t *pu8_Data, uint32_t u32_Data_Cnt)
{
  
  return true;
}

bool bool_Home_Handler(uint8_t *pu8_Data, uint32_t u32_Data_Cnt)
{
  
  return true;
}

bool bool_Stop_Handler(uint8_t *pu8_Data, uint32_t u32_Data_Cnt)
{
  
  return true;
}

bool bool_Emergency_Stop_Handler(uint8_t *pu8_Data, uint32_t u32_Data_Cnt)
{
  
  return true;
}

bool bool_Stabilizing_Mode_Handler(uint8_t *pu8_Data, uint32_t u32_Data_Cnt)
{
  
  return true;
}

bool bool_Set_Pos_Handler(uint8_t *pu8_Data, uint32_t u32_Data_Cnt)
{
  
  return true;
}

bool bool_Set_Vel_Handler(uint8_t *pu8_Data, uint32_t u32_Data_Cnt)
{
  
  return true;
}

bool bool_Set_Pos_Vel_Handler(uint8_t *pu8_Data, uint32_t u32_Data_Cnt)
{
  
  return true;
}

bool bool_Get_Pos_Handler(uint8_t *pu8_Data, uint32_t u32_Data_Cnt)
{
  
  return true;
}

bool bool_Set_Kp_Handler(uint8_t *pu8_Data, uint32_t u32_Data_Cnt)
{
  
  return true;
}

bool bool_Set_Ki_Handler(uint8_t *pu8_Data, uint32_t u32_Data_Cnt)
{
  
  return true;
}

bool bool_Set_Kd_Handler(uint8_t *pu8_Data, uint32_t u32_Data_Cnt)
{
  
  return true;
}

bool bool_Set_Kff1_Handler(uint8_t *pu8_Data, uint32_t u32_Data_Cnt)
{
  
  return true;
}

bool bool_Set_Kff2_Handler(uint8_t *pu8_Data, uint32_t u32_Data_Cnt)
{
  
  return true;
}

bool bool_Get_Params_Handler(uint8_t *pu8_Data, uint32_t u32_Data_Cnt)
{
  
  return true;
}
/**
  * @}
  */

/*********************************END OF FILE**********************************/
