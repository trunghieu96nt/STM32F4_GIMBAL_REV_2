/* Includes ------------------------------------------------------------------*/
#include "system_timetick.h"

volatile uint32_t tick_count = 0; // for system
volatile uint32_t u32_system_tick_count = 0; // for user
volatile uint32_t tick_flag = 0;


void NMI_Handler(void)
{
}

void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

void SVC_Handler(void)
{
}

void DebugMon_Handler(void)
{
}
  
void PendSV_Handler(void)
{
}

void SysTick_Handler(void)
{
  tick_flag = 1;
  tick_count++;
  u32_system_tick_count++;
}

uint32_t SysTick_GetTick(void)
{
  return tick_count;
}

bool SysTick_IsTimeout(uint32_t u32_start_time_ms, uint32_t u32_time_out_ms)
{
  return (tick_count - u32_start_time_ms > u32_time_out_ms * F_CTRL / 1000);
}
