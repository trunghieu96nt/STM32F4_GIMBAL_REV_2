#include "include.h"

void v_Board_Init()
{
  /* Check System Clock*/
  RCC_ClocksTypeDef RCC_ClocksStructure;
  uint8_t u8_clock_source;
  u8_clock_source = RCC_GetSYSCLKSource();
  if (u8_clock_source != 0x08) // 0x08: PLL used as system clock
  {
    //while (true);
  }
  RCC_GetClocksFreq(&RCC_ClocksStructure);
  if (RCC_ClocksStructure.SYSCLK_Frequency != 168000000)
  {
    //while (true);
  }
  
  
  /* Enable SysTick at 1ms interrupt */
  SysTick_Config(SystemCoreClock / F_CTRL);
  
  v_GPIO_Init();
  v_Motor_Init();
  v_ADIS_Init();
  v_UART_Comm_Init();
  v_I2C_Comm_Init();
  v_Control_Init();
  v_ADC_Init();
  
  //v_Params_Load_All();
  
  /* waiting for IMU Data is available */
//  while (stru_Get_IMU_Data().bool_available == false)
//  {
//    bool_ADIS_Read_IsTimeout(10);
//    
//    if (u32_system_tick_count > 1000)
//    {
//      u32_system_tick_count = 0;
//      //v_Red_Toggle();
//      v_Blue_Toggle();
//      //v_Green_Toggle();
//    }
//  }
//  v_Blue_Off();
  
  //bool_CMD_Send((uint8_t *)"CMD Ok\r\n", strlen("CMD Ok\r\n"));
  //bool_DATA_Send((uint8_t *)"DATA Ok\r\n", strlen("DATA Ok\r\n"));
  //bool_RESV_Send((uint8_t *)"RESV Ok\r\n", strlen("RESV Ok\r\n"));
}

int main(void)
{
  v_Board_Init();
  
  while (true)
  {
    if (u32_system_tick_count >= 1000)
    {
      u32_system_tick_count = 0;
      //v_Red_Toggle();
      //v_Blue_Toggle();
      v_Green_Toggle();
    }
    
    v_CMD_Receive();
    bool_ADIS_Read_IsTimeout(10);
    
    //Controller
    if (tick_flag == true)
    {
      tick_flag = false;
      v_Control();
      v_Send_Data();
      if (DMA_GetCmdStatus(DMA1_Stream3) == DISABLE)
      {
        v_Red_On();
      }
      else
        v_Red_Off();
    }
  }
}


