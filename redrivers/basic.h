#ifndef __BASIC_H__
#define __BASIC_H__ 
#include "MKL25Z4.h"

#define DEFAULT_BUS_CLOCK (DEFAULT_SYSTEM_CLOCK / 2)

#ifndef NULL
    #define NULL ((void *)0U)
#endif

// #define Global_IRQ_Enbale() __ASM volatile ("cpsie i")
// #define Global_IRQ_Disable() __ASM volatile ("cpsid i")
// #define Wait_Interrupt() __ASM volatile ("wfi")
// #define Wait_Event() __ASM volatile ("wfe")

#define NVIC_Init(IRQn, priority) NVIC_SetPriority(IRQn, priority); NVIC_ClearPendingIRQ(IRQn); NVIC_EnableIRQ(IRQn)

#endif


