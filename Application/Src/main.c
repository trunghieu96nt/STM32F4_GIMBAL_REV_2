#include "include.h"

void Board_Init()
{
  /* Enable SysTick at 1ms interrupt */
  SysTick_Config(SystemCoreClock / F_CTRL);
  
  v_GPIO_Init();
  v_Motor_Init();
  v_ADIS_Init();
  v_UART_Comm_Init();
  v_I2C_Comm_Init();
  v_Control_Init();
  
  //v_Params_Load_All();
  
  /* waiting for IMU Data is available */
  while(stru_Get_IMU_Data().bool_Available == false)
  {
    bool_ADIS_Read_IsTimeout(10);
    
    if(u32_System_Tick_Count > 1000)
    {
      u32_System_Tick_Count = 0;
      //v_Red_Toggle();
      v_Blue_Toggle();
      //v_Green_Toggle();
    }
  }
  v_Blue_Off();
  
  //bool_CMD_Send((uint8_t *)"CMD Ok\r\n", strlen("CMD Ok\r\n"));
  //bool_DATA_Send((uint8_t *)"DATA Ok\r\n", strlen("DATA Ok\r\n"));
  //bool_RESV_Send((uint8_t *)"RESV Ok\r\n", strlen("RESV Ok\r\n"));
}

int main(void)
{
  Board_Init();
  
  while(true)
  {
    if(u32_System_Tick_Count > 1000)
    {
      u32_System_Tick_Count = 0;
      //v_Red_Toggle();
      //v_Blue_Toggle();
      v_Green_Toggle();
    }
    
    v_CMD_Receive();
    bool_ADIS_Read_IsTimeout(10);
    
    //Controller
    if(tick_flag == true)
    {
      tick_flag = false;
      v_Control();
      v_Send_Data();
    }
  }
}


