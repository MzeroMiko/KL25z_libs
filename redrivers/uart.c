#include "uart.h"

void UART0_SendByte(uint8_t data) { UART_Push_Data(0, (uint8_t) data); while(!UART_Get_Tx_Complete_IT_Flag(0)); }
uint8_t UART0_RecvByte(void) { while(!UART_Get_Rx_Full_IT_Flag(0)); return UART_Read_Data(0); }

uint8_t uart0_recv_buffer[MAX_UART0_Buffer_Size];
uint32_t uart0_recv_buffer_length, uart0_recv_buffer_current;
void (*UART0_Recv_CallBack)(uint8_t *buffer, int length);

void (*UART0_ISR)(void);
void UART0_IRQHandler(void) { UART0_ISR(); } 
void __void_function(void) {} // used to switch ISR to a void NULL

void UART0_Init(int baud) {
    UART_Enable_Clock(0);
    UART_Tx_Disable(0);
    UART_Rx_Disable(0);
    UART0_Set_Clock_Source(UART0_Clock_MCGFLLCLK);
    UART0_PORT_Enable();
    UART_Set_Baud(0, baud, DEFAULT_SYSTEM_CLOCK, 16);
    UART_Set_Parity(0, UART_No_Parity);
    UART_Set_StopBit(0, 1);
    UART_Tx_Enable(0);
    UART_Rx_Enable(0);
    NVIC_Init(UART0_IRQn, 128);
}

void UART0_Recv_ISR(void) {
    uint8_t recv_data;
    if (UART_Get_Rx_Full_IT_Flag(0)) {
        recv_data = UART_Read_Data(0);
        if (uart0_recv_buffer_length > uart0_recv_buffer_current) {
            uart0_recv_buffer[(uart0_recv_buffer_current)++] = recv_data;
        } else {
            UART_Rx_Full_IT_Disable(0);
            UART0_ISR = &__void_function;
            uart0_recv_buffer_current = uart0_recv_buffer_length;
            UART0_Recv_CallBack(uart0_recv_buffer, uart0_recv_buffer_length); // do exec by end
        }
    }
}

void UART0_SendData(uint8_t *data, int length) {
    while(length--) {
        UART_Push_Data(0, *(data++));
        while(!UART_Get_Tx_Complete_IT_Flag(0));
    }
}

void UART0_RecvData(int length, void callback(uint8_t *buffer, int length)) {
    uart0_recv_buffer_length = length;
    uart0_recv_buffer_current = 0;
    UART0_ISR = &UART0_Recv_ISR;
    UART0_Recv_CallBack = callback;
    UART_Rx_Full_IT_Enable(0);
}
