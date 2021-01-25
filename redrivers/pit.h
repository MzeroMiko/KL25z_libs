#ifndef __PIT_H__
#define __PIT_H__

#include "MKL25Z4.h"

// diff between RefMan and header, header is right
// #define _PIT_MCR_MDIS_MASK  (0x40000000U)
// #define _PIT_MCR_FRZ_MASK   (0x80000000U)
// #define _PIT_MCR_FRZ_SHIFT  (0U)
// #define _PIT_TCTRL_TEN_MASK (0x80000000U)
// #define _PIT_TCTRL_TIE_MASK (0x40000000U)
// #define _PIT_TCTRL_CHN_MASK (0x20000000U)
// #define _PIT_TFLG_TIF_MASK  (0x80000000U)
// #define _PIT_TFLG_TIF_SHIFT (31U)

#define _PIT_MCR_MDIS_MASK  PIT_MCR_MDIS_MASK
#define _PIT_MCR_FRZ_MASK   PIT_MCR_FRZ_MASK
#define _PIT_MCR_FRZ_SHIFT  PIT_MCR_FRZ_SHIFT
#define _PIT_TCTRL_TEN_MASK PIT_TCTRL_TEN_MASK
#define _PIT_TCTRL_TIE_MASK PIT_TCTRL_TIE_MASK
#define _PIT_TCTRL_CHN_MASK PIT_TCTRL_CHN_MASK
#define _PIT_TFLG_TIF_MASK  PIT_TFLG_TIF_MASK
#define _PIT_TFLG_TIF_SHIFT PIT_TFLG_TIF_SHIFT

// 207 / 807 in KL25RefMan.pdf --------------------------------------------------------------------- //
#define PIT_Enable_Clock() SIM->SCGC6 |= SIM_SCGC6_PIT_MASK
#define PIT_Disable_Clock() SIM->SCGC6 &= ~SIM_SCGC6_PIT_MASK

// 575 / 807 in KL25RefMan.pdf  Periodic Trigger mode -> Module Control Register MDIS 0 means enable;
#define PIT_Disable() PIT->MCR |= _PIT_MCR_MDIS_MASK
#define PIT_Enable(enable_run_in_debug) PIT->MCR = (PIT->MCR & ~_PIT_MCR_MDIS_MASK & ~_PIT_MCR_FRZ_MASK) | ((enable_run_in_debug & 1U) << _PIT_MCR_FRZ_SHIFT )

// 577 / 807 in KL25RefMan.pdf Timer Load Value Register, x can be 0,1, Timer Control Register 
#define PIT_Get_Lifetime_Count  (((uint64_t)(PIT->LTMR64H) << 32U) + (uint64_t)(PIT->LTMR64L)) // LTMR64H should be read first
#define PIT_Channel_Set_Time_Period(x, cnt) PIT->CHANNEL[x].LDVAL = ((uint32_t) cnt)
#define PIT_Channel_Get_Current_Count(x) (PIT->CHANNEL[x].CVAL)
#define PIT_Channel_Enable(x) PIT->CHANNEL[x].TCTRL |= _PIT_TCTRL_TEN_MASK 
#define PIT_Channel_Disable(x) PIT->CHANNEL[x].TCTRL &= ~_PIT_TCTRL_TEN_MASK
#define PIT_Channel_IT_Enable(x) PIT->CHANNEL[x].TCTRL |= _PIT_TCTRL_TIE_MASK
#define PIT_Channel_IT_Disable(x) PIT->CHANNEL[x].TCTRL &= ~_PIT_TCTRL_TIE_MASK
#define PIT_Channel_Chain_Enable(x) PIT->CHANNEL[x].TCTRL |= _PIT_TCTRL_CHN_MASK
#define PIT_Channel_Chain_Disable(x) PIT->CHANNEL[x].TCTRL &= ~_PIT_TCTRL_CHN_MASK

#define PIT_Channel_Get_IT_Flag(x) (PIT->CHANNEL[x].TFLG >> _PIT_TFLG_TIF_SHIFT)
#define PIT_Channel_Clear_IT_Flag(x) PIT->CHANNEL[x].TFLG |= _PIT_TFLG_TIF_MASK // Writing 1 to this flag clears it

// x=0 means channel0, x=1 means channel1
#define PIT_Init(x, period, priority, enable_run_in_debug) PIT_Enable_Clock(); PIT_Enable(enable_run_in_debug); PIT_Channel_Set_Time_Period(x, period); PIT_Channel_Chain_Disable(x); PIT_Channel_IT_Enable(x); NVIC_Init(PIT_IRQn, priority)

#endif

#ifdef DEMO

void pit_isr(void) {
    NVIC_ClearPendingIRQ(PIT_IRQn);
    if (PIT_Channel_Get_IT_Flag(0)) {
        PIT_Channel_Clear_IT_Flag(0);
        GPIO_ToggleBit(PORT_B, 18);
    } 
    // else if (PIT_Channel_Get_IT_Flag(1)) {
        // PIT_Channel_Clear_IT_Flag(1);
    // }
}

int PIT_test(void *args[]) {
    int freq_Hz = intarg(args[0]);
    ADC_Init();
    if (freq_Hz == 0) {
        PIT_Disable();
        PIT_Disable_Clock();
        PIT_Channel_Disable(0);
        PIT_ISR = &__void_function;
        return 0;
    }
    PORT_Enable_Clock(PORT_B);
    GPIO_Init(PORT_B, 18, GPIO_Output_1);
    PIT_Init(0, DEFAULT_SYSTEM_CLOCK / freq_Hz , 128, 0);
    PIT_Channel_Enable(0);
    PIT_ISR = &pit_isr;
    return 0;
}


#endif



