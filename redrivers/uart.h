#ifndef __UART_H__
#define __UART_H__

#include "MKL25Z4.h"
#include "port.h"
#include "basic.h"

#define UART(x) ((UART_Type *)(0x4006A000U + ((uint32_t)x << 12))) 

// 724 / 807 in KL25RefMan.pdf UART0 is differ with UART1/2 ------------------- //

typedef enum _uart_oversample_rate {
    UART0_OSR_4 = 4U,
    UART0_OSR_8 = 8U,
    UART0_OSR_16 = 16U,
    UART0_OSR_32 = 32U,
    UART_OSR = 16U, // can not be set, that bit is not modifiable
} uart_oversample_rate_t;

typedef enum _uart_parity_mode {
    UART_No_Parity = 0U,
    UART_Even_Parity = 2U,
    UART_Odd_Parity = 3U,
} uart_parity_mode_t;

typedef enum _uart0_clk_source {
    UART0_Clock_Disable = 0U,
    UART0_Clock_MCGFLLCLK = 1U,
    UART0_Clock_OSCERCLK = 2U,
    UART0_Clock_MCGIRCLK = 3U,
} uart0_clk_source_t;

#define UART_Enable_Clock(x) SIM->SCGC4 |= (0x400 << x)
#define UART_Disable_Clock(x) SIM->SCGC4 &= ~(0x400 << x)

#define UART_Read_Data(x) (UART(x)->D) 
#define UART_Push_Data(x, data) UART(x)->D = (data & 0xFF)

#define UART_Tx_Enable(x) UART(x)->C2 |= UART_C2_TE_MASK
#define UART_Tx_Disable(x) UART(x)->C2 &= ~UART_C2_TE_MASK
#define UART_Rx_Enable(x) UART(x)->C2 |= UART_C2_RE_MASK
#define UART_Rx_Disable(x) UART(x)->C2 &= ~UART_C2_RE_MASK

#define UART_Tx_Empty_IT_Enable(x)  UART(x)->C2 |= UART_C2_TIE_MASK 
#define UART_Tx_Empty_IT_Disable(x)  UART(x)->C2 &= ~UART_C2_TIE_MASK
#define UART_Tx_Complete_IT_Enable(x) UART(x)->C2 |= UART_C2_TCIE_MASK
#define UART_Tx_Complete_IT_Disable(x) UART(x)->C2 &= ~UART_C2_TCIE_MASK
#define UART_Rx_Full_IT_Enable(x)  UART(x)->C2 |= UART_C2_RIE_MASK
#define UART_Rx_Full_IT_Disable(x)  UART(x)->C2 &= ~UART_C2_RIE_MASK
#define UART_Line_Idle_IT_Enable(x) UART(x)->C2 |= UART_C2_ILIE_MASK
#define UART_Line_Idle_IT_Disable(x) UART(x)->C2 &= ~UART_C2_ILIE_MASK
#define UART_Rx_Overrun_IT_Enable(x) UART(x)->C3 |= UART_C3_ORIE_MASK
#define UART_Rx_Overrun_IT_Disable(x) UART(x)->C3 &= ~UART_C3_ORIE_MASK
#define UART_Noise_Error_IT_Enable(x) UART(x)->C3 |= UART_C3_NEIE_MASK
#define UART_Noise_Error_IT_Disable(x) UART(x)->C3 &= ~UART_C3_NEIE_MASK
#define UART_Frame_Error_IT_Enable(x) UART(x)->C3 |= UART_C3_FEIE_MASK
#define UART_Frame_Error_IT_Disable(x) UART(x)->C3 &= ~UART_C3_FEIE_MASK
#define UART_Parity_Error_IT_Enable(x) UART(x)->C3 |= UART_C3_PEIE_MASK
#define UART_Parity_Error_IT_Disable(x) UART(x)->C3 &= ~UART_C3_PEIE_MASK
#define UART_Lin_Break_IT_Enable(x) UART(x)->BHD |= UART_BDH_LBKDIE_MASK
#define UART_Lin_Break_IT_Disable(x) UART(x)->BHD &= ~UART_BDH_LBKDIE_MASK
#define UART_Rx_ActiveEdge_IT_Enable(x) UART(x)->BHD |= UART_BDH_RXEDGIE_MASK
#define UART_Rx_ActiveEdge_IT_Disable(x) UART(x)->BHD &= ~UART_BDH_RXEDGIE_MASK

#define UART_Get_Tx_Empty_IT_Flag(x) ((UART(x)->S1 & UART_S1_TDRE_MASK) >> UART_S1_TDRE_SHIFT)
#define UART_Get_Tx_Complete_IT_Flag(x) ((UART(x)->S1 & UART_S1_TC_MASK) >> UART_S1_TC_SHIFT)
#define UART_Get_Rx_Full_IT_Flag(x)  ((UART(x)->S1 & UART_S1_RDRF_MASK) >> UART_S1_RDRF_SHIFT)
#define UART_Get_Line_Idle_IT_Flag(x) ((UART(x)->S1 & UART_S1_IDLE_MASK) >> UART_S1_IDLE_SHIFT)
#define UART_Get_Rx_Overrun_IT_Flag(x) ((UART(x)->S1 & UART_S1_OR_MASK) >> UART_S1_OR_SHIFT)
#define UART_Get_Noise_Error_IT_Flag(x) ((UART(x)->S1 & UART_S1_NF_MASK) >> UART_S1_NF_SHIFT)
#define UART_Get_Frame_Error_IT_Flag(x) ((UART(x)->S1 & UART_S1_FE_MASK) >> UART_S1_FE_SHIFT)
#define UART_Get_Parity_Error_IT_Flag(x) ((UART(x)->S1 & UART_S1_PF_MASK) >> UART_S1_PF_SHIFT)
#define UART_Get_Lin_Break_IT_Flag(x) ((UART(x)->S2 & UART_S2_LBKDIF_MASK) >> UART_S2_LBKDIF_SHIFT)
#define UART_Get_Rx_ActiveEdge_IT_Flag(x) ((UART(x)->S2 & UART_S2_RXEDGIF_MASK) >> UART_S2_RXEDGIF_SHIFT)
#define UART_Get_Rx_Active_IT_Flag(x) ((UART(x)->S2 & UART_S2_RAF_MASK) >> UART_S2_RAF_SHIFT)

#define UART_Set_Divisor(x, divisor) UART(x)->C2 &= ~(UART_C2_TE_MASK | UART_C2_RE_MASK); UART(x)->BDH = (UART(x)->BDH & ~UART_BDH_SBR_MASK) | (((divisor) >> 8) & UART_BDH_SBR_MASK); UART(x)->BDL = ((divisor) & 0xFF)
#define UART_Set_OverSampleRate(x, OSR) UART(x)->C4 &= ~UART0_C4_OSR_MASK; UART(x)->C4 |= UART0_C4_OSR(OSR-1) // no use setting OSR of UARTx(x!=0), as that bit is reserved, and OSR always be 16
#define UART_Set_Baud(x, baud, srcClock_Hz, OSR) UART_Set_OverSampleRate(x, OSR); UART_Set_Divisor(x,  (((uint32_t)(2 * srcClock_Hz / (baud * OSR))) - 2 * ((uint32_t)(srcClock_Hz / (baud * OSR)))) ? (srcClock_Hz / (baud * OSR) + 1) : srcClock_Hz / (baud * OSR) ) // src/baud/osr = m + n;  n > 0.5 <=> 2*src/baud/osr - 2*m > 1 <=> divider should be m+1
#define UART_Set_Parity(x, pa) UART(x)->C1 = (UART(x)->C1 & ~(UART_C1_PE_MASK | UART_C1_PT_MASK | UART_C1_M_MASK)) | ((pa & 0x3) << UART_C1_PT_SHIFT) | UART_C1_M(pa >> 1)
#define UART_Set_StopBit(x, bit) UART(x)->BDH = (UART(x)->BDH & ~UART_BDH_SBNS_MASK) | UART_BDH_SBNS((bit-1) & 0x1);
#define UART_DMA_Tx_Enable(x) (x>0)? (UART(x)->C4 |= UART_C4_TDMAS_MASK) : (UART(0)->C5 |= UART_C5_TDMAE_MASK) // must have TIE set
#define UART_DMA_Tx_Disable(x) (x>0)? (UART(x)->C4 &= ~UART_C4_TDMAS_MASK) : (UART(0)->C5 &= ~UART_C5_TDMAE_MASK)
#define UART_DMA_Rx_Enable(x) (x>0)? (UART(x)->C4 |= UART_C4_RDMAS_MASK) : (UART(0)->C5 |= UART_C5_RDMAE_MASK) // must have TIE set
#define UART_DMA_Rx_Disable(x) (x>0)? (UART(x)->C4 &= ~UART_C4_RDMAS_MASK) : (UART(0)->C5 &= ~UART_C5_RDMAE_MASK)
#define UART_DMA_Get_DataAddr(x) (UART(x)->D)

#define UART0_PORT_Enable() PORT_Enable_Clock(PORT_A); (PORT(PORT_A)->PCR)[1] =  ((PORT(PORT_A)->PCR)[1] & ~PORT_PCR_MUX_MASK) | PORT_PCR_MUX(2); (PORT(PORT_A)->PCR)[2] =  ((PORT(PORT_A)->PCR)[2] & ~PORT_PCR_MUX_MASK) | PORT_PCR_MUX(2)

#define UART0_Set_Clock_Source(src) SIM->SOPT2 = (SIM->SOPT2 & ~SIM_SOPT2_UART0SRC_MASK) | SIM_SOPT2_UART0SRC(src)

#define MAX_UART0_Buffer_Size 100

void UART0_SendByte(uint8_t data);
uint8_t UART0_RecvByte(void);
void UART0_Init(int baud);
void UART0_SendData(uint8_t *data, int length);
void UART0_RecvData(int length, void callback(uint8_t *buffer, int length));

#endif

#ifdef DEMO // copy it to your code

void UART0_RecvCallBack2(uint8_t *buffer, int length) {
    UART0_SendData((buffer+2), length);
}

void UART0_RecvCallBack1(uint8_t *buffer, int length) {
    UART0_SendData((buffer+1), length);
}

void UART0_RecvCallBack(uint8_t *buffer, int length) {
    UART0_SendData(buffer, length);
    UART0_RecvData(13, UART0_RecvCallBack1);
}

void UART0_test() {
    UART0_Init(9600);
    UART0_SendData((uint8_t *)("hello, world!\n"), 15);
    UART0_RecvData(14, UART0_RecvCallBack);
}

#endif






