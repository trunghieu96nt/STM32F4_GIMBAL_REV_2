#include "include.h"

void Board_Init()
{
  /* Enable SysTick at 1ms interrupt */
  SysTick_Config(SystemCoreClock / F_CTRL);

}

int main(void)
{
  Board_Init();
    
  while(true)
  {
    //Controller
    if(tick_flag == true)
    {
      tick_flag = false;
    }
  }
}


