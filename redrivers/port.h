#ifndef __PORT_H__
#define __PORT_H__

#include "MKL25Z4.h"

typedef enum _PORT_x {
    PORT_A = 0x0U, PORT_B = 0x1U, PORT_C = 0x2U, PORT_D = 0x3U, PORT_E = 0x4U, 
} PORT_x;
#define PORT(x) ((PORT_Type *)(PORTA_BASE + (((uint32_t) (x)) << 12)))   // Add x * 0x1000

typedef enum _port_output_mode {
    PORT_OpenDrain = 0U,
    PORT_PushPull = 1U,
} port_output_mode_t;

typedef enum _port_input_mode {
    PORT_Floating = 0U,
    PORT_PullDown = 2U,
    PORT_PullUp = 3U,
} port_input_mode_t;

typedef enum _port_irqc_config {
    PORT_IT_DMA_Disable = 0x0U,
    PORT_DMA_RisingEdge = 0x1U,
    PORT_DMA_FallingEdge = 0x2U,
    PORT_DMA_EitherEdge = 0x3U,
    PORT_IT_LogicZero = 0x8U,
    PORT_IT_RisingEdge = 0x9U,
    PORT_IT_FallingEdge = 0xAU,
    PORT_IT_EitherEdge = 0xBU,
    PORT_IT_LogicOne = 0xCU,
} port_irqc_config_t; // 184

#define PORT_Enable_Clock(x) SIM->SCGC5 |= (0x200 << (x))     // x=0 for PORTA,..., x=4 for PORTE
#define PORT_Disable_Clock(x) SIM->SCGC5 &= ~(0x200 << (x))   // x=0 for PORTA,..., x=4 for PORTE
#define PORT_Set_Mux(x, bitx, mux) PORT(x)->PCR[bitx] = (PORT(x)->PCR[bitx] & ~PORT_PCR_MUX_MASK) | PORT_PCR_MUX(mux)
#define PORT_Set_IRQC(x, bitx, irqc_config) PORT(x)->PCR[bitx] = (PORT(x)->PCR[bitx] & ~PORT_PCR_IRQC_MASK) | PORT_PCR_IRQC(irqc_config)
#define PORT_Set_Output(x, bitx, output_mode, enable_slow_slew) PORT(x)->PCR[bitx] = (PORT(x)->PCR[bitx] & ~PORT_PCR_DSE_MASK & ~PORT_PCR_SRE_MASK) | PORT_PCR_DSE(output_mode) | PORT_PCR_SRE(enable_slow_slew) // see port_out_mode_t
#define PORT_Set_Input(x, bitx, input_mode, enable_input_filter) PORT(x)->PCR[bitx] = (PORT(x)->PCR[bitx] & ~PORT_PCR_PS_MASK & ~PORT_PCR_PE_MASK & ~PORT_PCR_PFE_MASK) | PORT_PCR_PS(input_mode & 0x1) | PORT_PCR_PE(input_mode & 0x2) | PORT_PCR_PFE(enable_input_filter) // see port_input_mode_t

#define PORT_Get_IT_Flag(x, bitx) ((PORT(x)->ISFR >> (bitx)) & 1U)
#define PORT_Clear_IT_Flag(x, bitx) PORT(x)->ISFR |= (1U << (bitx)) // write 1 to clear
#define PORT_Get_IT_Flags(x) (PORT(x)->ISFR)
#define PORT_Clear_IT_Flags(x) PORT(x)->ISFR = 0xFFFFFFFFU 

// #define PORT_Init(x, bitx, irqc_config, input_mode, output_mode, enable_slow_slew, enable_input_filter) PORT_Enable_Clock(x); PORT_Set_IRQC(x, bitx, irqc_config); PORT_Set_Output(x, bitx, output_mode, enable_slow_slew); PORT_Set_Input(x, bitx, input_mode, enable_input_filter)



// BE CAREFUL USING PORT BELLOW:
// PTA0 PTA1 PTA2 PTA3 is RESERVED for SWD and UART0
// PTA4 is RESERVED for NMI_b
// PTB18, PTB19, PTD1 is RESERVED for LED 
// Mux_PORTx_Pinx, Mux = ((val & 0xF000)>> 12), PORTx = ((val & 0x0F00)>> 8), Pinx = (val & 0x00FF)

typedef enum _port_mux_tpm_channel {
    PORT_TPM0_CH2_PTA5_Mux3 = ((3 << 12) + (0 << 8) + 5),
    PORT_TPM1_CH0_PTA12_Mux3 = ((3 << 12) + (0 << 8) + 12),
    PORT_TPM1_CH1_PTA13_Mux3 = ((3 << 12) + (0 << 8) + 13),

    PORT_TPM1_CH0_PTB0_Mux3 = ((3 << 12) + (1 << 8) + 0),
    PORT_TPM1_CH1_PTB1_Mux3 = ((3 << 12) + (1 << 8) + 1),
    PORT_TPM2_CH0_PTB2_Mux3 = ((3 << 12) + (1 << 8) + 2),
    PORT_TPM2_CH1_PTB3_Mux3 = ((3 << 12) + (1 << 8) + 3),
    PORT_TPM2_CH0_PTB18_Mux3 = ((3 << 12) + (1 << 8) + 18),
    PORT_TPM2_CH1_PTB19_Mux3 = ((3 << 12) + (1 << 8) + 19),

    PORT_TPM0_CH0_PTC1_Mux4 = ((4 << 12) + (2 << 8) + 1),
    PORT_TPM0_CH1_PTC2_Mux4 = ((4 << 12) + (2 << 8) + 2),
    PORT_TPM0_CH2_PTC3_Mux4 = ((4 << 12) + (2 << 8) + 3),
    PORT_TPM0_CH3_PTC4_Mux4 = ((4 << 12) + (2 << 8) + 4),
    PORT_TPM0_CH4_PTC8_Mux3 = ((3 << 12) + (2 << 8) + 8),
    PORT_TPM0_CH5_PTC9_Mux3 = ((3 << 12) + (2 << 8) + 9),

    PORT_TPM0_CH0_PTD0_Mux3 = ((3 << 12) + (3 << 8) + 0),
    PORT_TPM0_CH1_PTD1_Mux3 = ((3 << 12) + (3 << 8) + 1),
    PORT_TPM0_CH2_PTD2_Mux3 = ((3 << 12) + (3 << 8) + 2),
    PORT_TPM0_CH3_PTD3_Mux3 = ((3 << 12) + (3 << 8) + 3),
    PORT_TPM0_CH4_PTD4_Mux3 = ((3 << 12) + (3 << 8) + 4),
    PORT_TPM0_CH5_PTD5_Mux3 = ((3 << 12) + (3 << 8) + 5),

    PORT_TPM1_CH0_PTE20_Mux3 = ((3 << 12) + (4 << 8) + 20),
    PORT_TPM1_CH1_PTE21_Mux3 = ((3 << 12) + (4 << 8) + 21),
    PORT_TPM2_CH0_PTE22_Mux3 = ((3 << 12) + (4 << 8) + 22),
    PORT_TPM2_CH1_PTE23_Mux3 = ((3 << 12) + (4 << 8) + 23),
    PORT_TPM0_CH0_PTE24_Mux3 = ((3 << 12) + (4 << 8) + 24),
    PORT_TPM0_CH1_PTE25_Mux3 = ((3 << 12) + (4 << 8) + 25),
    PORT_TPM0_CH2_PTE29_Mux3 = ((3 << 12) + (4 << 8) + 26),
    PORT_TPM0_CH3_PTE30_Mux3 = ((3 << 12) + (4 << 8) + 27),
    PORT_TPM0_CH4_PTE31_Mux3 = ((3 << 12) + (4 << 8) + 28),

} port_mux_tpm_channel_t;

typedef enum _port_mux_dac_adc {
    PORT_ADC0_SE8_PTB0_Mux0 = ((0 << 12) + (1 << 8) + 0),
    PORT_ADC0_SE9_PTB1_Mux0 = ((0 << 12) + (1 << 8) + 1),
    PORT_ADC0_SE12_PTB2_Mux0 = ((0 << 12) + (1 << 8) + 2),
    PORT_ADC0_SE13_PTB3_Mux0 = ((0 << 12) + (1 << 8) + 3),
    
    PORT_ADC0_SE14_PTC0_Mux0 = ((0 << 12) + (2 << 8) + 0),
    PORT_ADC0_SE15_PTC1_Mux0 = ((0 << 12) + (2 << 8) + 1),
    PORT_ADC0_SE11_PTC2_Mux0 = ((0 << 12) + (2 << 8) + 2),
    
    PORT_ADC0_SE6b_PTD5_Mux0 = ((0 << 12) + (3 << 8) + 5),
    PORT_ADC0_SE7b_PTD6_Mux0 = ((0 << 12) + (3 << 8) + 6),

    PORT_ADC0_DP0_PTE20_Mux0 = ((0 << 12) + (4 << 8) + 20),
    PORT_ADC0_DM0_PTE21_Mux0 = ((0 << 12) + (4 << 8) + 21),
    PORT_ADC0_DP3_PTE22_Mux0 = ((0 << 12) + (4 << 8) + 22),
    PORT_ADC0_DM3_PTE23_Mux0 = ((0 << 12) + (4 << 8) + 23),

    PORT_ADC0_SE0_PTE20_Mux0 = ((0 << 12) + (4 << 8) + 20),
    PORT_ADC0_SE4a_PTE21_Mux0 = ((0 << 12) + (4 << 8) + 21),
    PORT_ADC0_SE3_PTE22_Mux0 = ((0 << 12) + (4 << 8) + 22),
    PORT_ADC0_SE7a_PTE23_Mux0 = ((0 << 12) + (4 << 8) + 23),
    PORT_ADC0_SE4b_PTE29_Mux0 = ((0 << 12) + (4 << 8) + 29),
    PORT_ADC0_SE23_PTE30_Mux0 = ((0 << 12) + (4 << 8) + 30),
    
    PORT_DAC0_OUT_PTE30_Mux0 = ((0 << 12) + (4 << 8) + 30),

} port_mux_dac_adc_t;

#define PORT_Set_GPIO(x, bitx) PORT_Enable_Clock(x); PORT_Set_Mux(x, bitx, 1)  
#define PORT_Set_Func(func) PORT_Enable_Clock(((func >> 8) & 0xF)); PORT_Set_Mux(((func >> 8) & 0xF), (func & 0xFF), ((func >> 12) & 0xF))

#endif

#ifdef DEMO

void PORT_Init(PORT_x x, uint8_t bitx, port_irqc_config_t irqc_config, port_input_mode_t input_mode, port_output_mode_t output_mode, uint8_t enable_slow_slew, uint8_t enable_input_filter) {
    PORT_Enable_Clock(x);
    PORT_Set_IRQC(x, bitx, irqc_config);
    PORT_Set_Output(x, bitx, output_mode, enable_slow_slew);
    PORT_Set_Input(x, bitx, input_mode, enable_input_filter);
}

#endif

