#ifndef __TPM_H__
#define __TPM_H__

#include "MKL25Z4.h"

// Page 208
#define TPM(x) ((TPM_Type *)(TPM0_BASE + (((uint32_t) x) << 12)))

typedef enum _tpm_clk_source {
    TPM_Clock_Disable = 0U,      // 0000
    TPM_Clock_MCGFLLCLK = 5U,    // 0101
    TPM_Clock_OSCERCLK = 6U,     // 0110
    TPM_Clock_MCGIRCLK = 7U,     // 0111
    TPM_Clock_External = 8U,     // 1000
} tpm_clk_source_t;

typedef enum _tpm_prescale {
    TPM_Prescale_Divide_1   = 0U, 
    TPM_Prescale_Divide_2   = 1U,
    TPM_Prescale_Divide_4   = 2U,
    TPM_Prescale_Divide_8   = 3U,
    TPM_Prescale_Divide_16  = 4U,
    TPM_Prescale_Divide_32  = 5U,
    TPM_Prescale_Divide_64  = 6U,
    TPM_Prescale_Divide_128 = 7U,
} tpm_prescale_t;

typedef enum _tpm_count_mode {
    TPM_EdgeAligned = 0U,   // count Up
    TPM_CenterAligned = 1U, // count UpDown
} tpm_count_mode_t;

typedef enum _tpm_channel_mode {
    TPM_Capture_RisingEdge = 1U,
    TPM_Capture_FallingEdge = 2U,
    TPM_Capture_EitherEdge = 3U,
    TPM_Compare_NoOutputSignal = 4U,
    TPM_Compare_ToggleOnMatch = 5U,
    TPM_COmpare_ClearOnMatch = 6U,
    TPM_Compare_SetOnMatch = 7U,
    TPM_PWM_NoSignal = 8U,
    TPM_PWM_LowLevel = 9U,
    TPM_PWM_HighLevel = 10U,
    TPM_Compare_HighPulseOutput = 13U,
    TPM_Compare_LowPulseOutput = 14U,
} tpm_channel_mode_t;

typedef enum _tpm_trigger {
    TPM_Trigger_External = 0U,
    TPM_Trigger_CMP0 = 1U,
    TPM_Trigger_PIT0 = 4U,
    TPM_Trigger_PIT1 = 5U,
    TPM_Trigger_TPM0 = 8U,
    TPM_Trigger_TPM1 = 9U,
    TPM_Trigger_TPM2 = 10U,
    TPM_Trigger_RTC_Alarm = 12U,
    TPM_Trigger_RTC_Second = 13U,
    TPM_Trigger_LPTMR = 14U,
} tpm_trigger_t;

// ALL INTERUPT BITS ARE CLEARED BY WRITING 1.

#define TPM_Enable_Clock(x, clock_source) SIM->SOPT2 = (SIM->SOPT2 & ~SIM_SOPT2_TPMSRC_MASK) | SIM_SOPT2_TPMSRC(clock_source & 0x3); SIM->SCGC6 |= (0x1000000 << (x))      // Set TPMx as 0b1, ser TPMSRC
#define TPM_Disable_Clock(x) SIM->SCGC6 &= ~(0x1000000 << (x))              // Set TPMx as 0b0

#define TPM_Start(x, use_external_clock) TPM(x)->SC = (TPM(x)->SC & ~TPM_SC_CMOD_MASK) | TPM_SC_CMOD((use_external_clock) ? 2 : 1 );  TPM(x)->CNT = 0x0000FFFF // see tpm_clock_source_t
#define TPM_Stop(x) TPM(x)->SC &= ~TPM_SC_CMOD_MASK; while(TPM(x)->SC & TPM_SC_CMOD_MASK); TPM(x)->CNT = 0x0000FFFF // wait to check Timer Stop
#define TPM_Set_Count_Mode(x, count_mode) TPM(x)->SC = (TPM(x)->SC & ~TPM_SC_CPWMS_MASK) | TPM_SC_CPWMS(count_mode) // see tpm_count_mode_t
#define TPM_Set_Prescale(x, prescale) TPM(x)->SC = (TPM(x)->SC & ~TPM_SC_PS_MASK) | TPM_SC_PS(prescale) // see tpm_prescale_t
#define TPM_Set_Period(x, count) TPM(x)->CNT = 0x0000FFFF; TPM(x)->MOD = ((TPM(x)->SC & TPM_SC_CPWMS_MASK) ? ((count) / 2) : ((count) - 1) ) & 0x0000FFFF // 554 / 807 in KL25RefMan.pdf it is recommended to initialize the LPTPM counter (write to CNT) before writing to the MOD register to avoid confusion about when the first counter overflow will occur, Writing any value to COUNT also clears the counter.
#define TPM_Get_Period(x) ((TPM(x)->SC & TPM_SC_CPWMS_MASK) ? (2 * TPM(x)->MOD) : (TPM(x)->MOD + 1))
// srcClock / prescale / freqHz == count ==  (2 * MOD) if CenterAligned else (MOD + 1)
#define TPM_Set_Freq(x, freq_Hz, srcClock_Hz)  TPM_Set_Period(x, srcClock_Hz / freq_Hz / (1U << ((TPM(x)->SC & TPM_SC_PS_MASK) >> TPM_SC_PS_SHIFT)) )
#define TPM_Get_Current_Count(x) (TPM(x)->CNT)

#define TPM_DMA_Enable(x) TPM(x)->SC |= TPM_SC_DMA_MASK
#define TPM_DMA_Disable(x) TPM(x)->SC &= ~TPM_SC_DMA_MASK
#define TPM_Timer_IT_Enable(x) TPM(x)->SC |= TPM_SC_TOIE_MASK
#define TPM_Timer_IT_Disable(x) TPM(x)->SC &= ~TPM_SC_TOIE_MASK
#define TPM_Timer_Get_IT_Flag(x) ((TPM(x)->SC >> TPM_SC_TOF_SHIFT) & 1UL)
#define TPM_Timer_Clear_IT_Flag(x) TPM(x)->SC |= TPM_SC_TOF_MASK

#define TPM_Doze_Enable(x) TPM(x)->CONF |= TPM_CONF_DOZEEN_MASK
#define TPM_Doze_Disable(x) TPM(x)->CONF &= ~TPM_CONF_DOZEEN_MASK
#define TPM_GlobalTime_Enable(x) TPM(x)->CONF |= TPM_CONF_GTBEEN_MASK
#define TPM_GlobalTime_Disable(x) TPM(x)->CONF &= ~TPM_CONF_GTBEEN_MASK
#define TPM_Reload_On_Trigger_Enable(x) TPM(x)->CONF |= TPM_CONF_CROT_MASK
#define TPM_Reload_On_Trigger_Disable(x) TPM(x)->CONF &= ~TPM_CONF_CROT_MASK
#define TPM_Start_On_Trigger_Enable(x) TPM(x)->CONF |= TPM_CONF_CSOT_MASK
#define TPM_Start_On_Trigger_Disable(x) TPM(x)->CONF &= ~TPM_CONF_CSOT_MASK
#define TPM_Stop_On_Overflow_Enable(x) TPM(x)->CONF |= TPM_CONF_CSOO_MASK
#define TPM_Stop_On_Overflow_Disable(x) TPM(x)->CONF &= ~TPM_CONF_CSOO_MASK
#define TPM_Debug_Mode_Enable(x) TPM(x)->CONF |= TPM_CONF_DBGMODE_MASK
#define TPM_Debug_Mode_Disable(x) TPM(x)->CONF &= ~TPM_CONF_DBGMODE_MASK
#define TPM_Trigger_Select(x, trigger) TPM(x)->CONF = (TPM(x)->CONF & ~TPM_CONF_TRGSEL_MASK) | TPM_CONF_TRGSEL(trigger) // TRGSEL should only be changed when the LPTPM counter is disabled

// another way to get channel IT
// #define TPM_Channel_Get_IT_Flag(x, c) ((TPM(x)->STATUS >> (c)) & 1UL)  // Get CHnF
// #define TPM_Channel_Clear_IT_Flag(x, c) TPM(x)->STATUS |= (1UL << (c)) // Set CHnF as 0b1 to clear
// #define TPM_Timer_Get_IT_Flag(x) ((TPM(x)->STATUS >> TPM_STATUS_TOF_SHIFT) & 1UL)
// #define TPM_Timer_Clear_IT_Flag(x) TPM(x)->STATUS |= TPM_STATUS_TOF_MASK 

// When switching from one channel mode to a different channel mode, the channel must first be disabled
#define TPM_Channel_DMA_Enable(x, c) TPM(x)->CONTROLS[c].CnSC |= TPM_CnSC_DMA_MASK
#define TPM_Channel_DMA_Disable(x, c) TPM(x)->CONTROLS[c].CnSC &= ~TPM_CnSC_DMA_MASK
#define TPM_Channel_IT_Enable(x, c) TPM(x)->CONTROLS[c].CnSC |= TPM_CnSC_CHIE_MASK  
#define TPM_Channel_IT_Disable(x, c) TPM(x)->CONTROLS[c].CnSC &= ~TPM_CnSC_CHIE_MASK
#define TPM_Channel_Get_IT_Flag(x, c) ((TPM(x)->CONTROLS[c].CnSC >> TPM_CnSC_CHF_SHIFT) & 1UL)
#define TPM_Channel_Clear_IT_Flag(x, c) TPM(x)->CONTROLS[c].CnSC |= TPM_CnSC_CHF_MASK
#define TPM_Channel_Set_Mode(x, c, channel_mode)\
    TPM(x)->CONTROLS[c].CnSC &= ~(0xF << TPM_CnSC_ELSA_SHIFT);\
    while (TPM(x)->CONTROLS[c].CnSC & (0xF << TPM_CnSC_ELSA_SHIFT));\
    TPM(x)->CONTROLS[c].CnSC |= (((channel_mode) & 0xF) << TPM_CnSC_ELSA_SHIFT);\
    while (!(TPM(x)->CONTROLS[c].CnSC & (0xF << TPM_CnSC_ELSA_SHIFT))) 
    // see tpm_channel_mode_t, wait to check value(which can not be zero) is set

#define TPM_PWM_Set_DutyPercent(x, c, dutyPercent) TPM(x)->CONTROLS[c].CnV = ((dutyPercent) >= 100) ? (TPM(x)->MOD + 1) : ((TPM(x)->MOD) * (dutyPercent) / 100) // If (CMOD[1:0] ≠ 0:0), then CnV register is updated according to the selected mode, that is:  If the selected mode is EPWM then CnV register is updated after CnV register was written and the TPM counter changes from MOD to zero. If the selected mode is CPWM then CnV register is updated after CnV register was written and the TPM counter changes from MOD to (MOD – 1).

#define TPM_Capture_Mode(x, c, channel_mode) TPM_Set_Count_Mode(x, TPM_EdgeAligned); TPM_Channel_Set_Mode(x, c, channel_mode)
#define TPM_Capture_Get_Current_Count(x, c) TPM(x)->CONTROLS[c].CnV

#define TPM_Compare_Mode(x, c, channel_mode, compare_value) TPM_Set_Count_Mode(x, TPM_EdgeAligned); TPM_Channel_Set_Mode(x, c, channel_mode); TPM(x)->CONTROLS[c].CnV = (compareValue & 0x0000FFFF)

#endif

#ifdef DEMO

void TPM_ISR_(void) {
    if (TPM_Channel_Get_IT_Flag(2, 0)) {
        TPM_Channel_Clear_IT_Flag(2, 0);
        GPIO_ToggleBit(PORT_B, 18);
    }
    if (TPM_Timer_Get_IT_Flag(2)) {
        TPM_Timer_Clear_IT_Flag(2);
        GPIO_ToggleBit(PORT_B, 18);
    }
}

void TPM_Timer() {
    PORT_Enable_Clock(PORT_B);
    PORT_Set_Mux(PORT_B, 18, 1);
    PORT_Set_Mux(PORT_B, 18, 1);
    GPIO_Init(PORT_B, 18, GPIO_Output_1);

    TPM_Enable_Clock(2);
    TPM_Set_Prescale(2, TPM_Prescale_Divide_4);

    TPM_Set_Count_Mode(2, TPM_CenterAligned);
    TPM_Set_Freq(2, 1000, DEFAULT_SYSTEM_CLOCK / 4); // clock not exactly right, should get PLLFLL clock
    TPM_Timer_IT_Enable(2);
    TPM2_ISR = &TPM_ISR_;
    NVIC_Init(TPM2_IRQn, 128);
    TPM_Start(2, TPM_Clock_MCGFLLCLK);
}

void TPM_Channel() {
    PORT_Enable_Clock(PORT_B);
    PORT_Set_Mux(PORT_B, 18, 1);
    PORT_Set_Mux(PORT_B, 18, 1);
    GPIO_Init(PORT_B, 18, GPIO_Output_1);

    TPM_Enable_Clock(2);
    TPM_Set_Prescale(2, TPM_Prescale_Divide_4);
    TPM_Set_Count_Mode(2, TPM_CenterAligned);
    TPM_Set_Freq(2, 1000, DEFAULT_SYSTEM_CLOCK / 4); // clock not exactly right, should get PLLFLL clock
    TPM_Channel_IT_Enable(2, 0);
    TPM_Channel_Set_Mode(2, 0, TPM_PWM_LowLevel);
    TPM_Channel_Set_Mode(2, 1, TPM_PWM_LowLevel); 
    TPM2_ISR = &TPM_ISR_;
    NVIC_Init(TPM2_IRQn, 128);
    TPM_Start(2, TPM_Clock_MCGFLLCLK);
}

// attention !!!!!!! if you are using eclipse, this way of using PWM is not working
// but in keil, it works

void TPM_PWM_Test(void) {
    // PTA12, 13 as TPM0_CH0, CH1, PTB 18 19 as TPM2_CH0, CH1 in ALT3
    PORT_Enable_Clock(PORT_B);
    PORT_Set_Mux(PORT_B, 18, 3);
    PORT_Set_Mux(PORT_B, 19, 3);

    TPM_Enable_Clock(2);
    TPM_Set_Prescale(2, TPM_Prescale_Divide_4);
    TPM_Set_Count_Mode(2, TPM_CenterAligned);    
    TPM_Set_Freq(2, 1000, DEFAULT_SYSTEM_CLOCK / 4); // clock not exactly right, should get PLLFLL clock
    TPM_Channel_Set_Mode(2, 0, TPM_PWM_LowLevel);
    TPM_Channel_Set_Mode(2, 1, TPM_PWM_LowLevel);    
    TPM_PWM_Set_DutyPercent(2, 0, 30);
    TPM_PWM_Set_DutyPercent(2, 1, 60);
    TPM_Start(2, TPM_Clock_MCGFLLCLK);
}

#endif


