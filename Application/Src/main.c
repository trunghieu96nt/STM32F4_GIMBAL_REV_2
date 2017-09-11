#include "include.h"

void Board_Init()
{
  /* Enable SysTick at 1ms interrupt */
  SysTick_Config(SystemCoreClock / F_CTRL);
  
  v_GPIO_Init();
  v_Motor_Init();
  v_ADIS_Init();
  v_Comm_Init();
  
  bool_CMD_Send((uint8_t *)"Done", strlen("Done"));
}

int main(void)
{
  Board_Init();
  
  while(true)
  {
    v_CMD_Receive();
    //Controller
    if(tick_flag == true)
    {
      tick_flag = false;
    }
  }
}


