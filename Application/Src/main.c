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
  
  //for testing
  v_PWM0_Set_Duty(400);
  v_PWM1_Set_Duty(400);
  
  v_DO_Reset(DO0_PIN);
  v_DO_Reset(DO1_PIN);
  
  bool_CMD_Send((uint8_t *)"CMD Ok\r\n", strlen("CMD Ok\r\n"));
  bool_DATA_Send((uint8_t *)"DATA Ok\r\n", strlen("DATA Ok\r\n"));
  bool_RESV_Send((uint8_t *)"RESV Ok\r\n", strlen("RESV Ok\r\n"));
}

int main(void)
{
  Board_Init();
  
  
  while(true)
  {
    if(u32_System_Tick_Count > 500)
    {
      u32_System_Tick_Count = 0;
      v_Red_Toggle();
      //v_Blue_Toggle();
      //v_Green_Toggle();
    }
    
    v_CMD_Receive();
    bool_ADIS_Read_IsTimeout(500);
    
    //Controller
    if(tick_flag == true)
    {
      tick_flag = false;
    }
  }
}


