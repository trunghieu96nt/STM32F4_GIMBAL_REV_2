/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F4xx_IT_H
#define __STM32F4xx_IT_H

#ifdef __cplusplus
 extern "C" {
#endif 

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "stdbool.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

/*  These function configure scheduler timer
    Sampling time is set by changing value PERIOD
    in the file "system_timetick.c"
*/

#define F_CTRL (float)1000

extern volatile uint32_t tick_count;
extern volatile uint32_t sysTickCount;
extern volatile uint32_t tick_flag;


void NMI_Handler(void);
void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);
void SVC_Handler(void);
void DebugMon_Handler(void);
void PendSV_Handler(void);
void SysTick_Handler(void);
uint32_t SysTick_GetTick(void);
bool SysTick_IsTimeout(uint32_t ui32StartTime_ms, uint32_t ui32TimeOut_ms);

#ifdef __cplusplus
}
#endif

#endif /* __STM32F4xx_IT_H */


