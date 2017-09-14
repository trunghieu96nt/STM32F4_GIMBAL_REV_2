#include "include.h"

void Board_Init()
{
  /* Enable SysTick at 1ms interrupt */
  SysTick_Config(SystemCoreClock / F_CTRL);
  
  v_GPIO_Init();
  v_Motor_Init();
  v_ADIS_Init();
  v_Comm_Init();
  
  //for testing
  v_PWM0_Set_Duty(400);
  v_PWM1_Set_Duty(400);
  
  v_DO_Reset(DO0_PIN);
  v_DO_Reset(DO1_PIN);
  
  bool_CMD_Send((uint8_t *)"Done", strlen("Done"));
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
    bool_ADIS_Read_IsTimeout(100);
    
    //Controller
    if(tick_flag == true)
    {
      tick_flag = false;
    }
  }
}


