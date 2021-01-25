#ifndef __GPIO_H__
#define __GPIO_H__

#include "port.h"
#include "MKL25Z4.h"

typedef enum _gpio_direction {
    GPIO_Input = 0U,
    GPIO_Output_0 = 1U,
    GPIO_Output_1 = 3U,
} gpio_direction_t;

#define GPIO(x) ((GPIO_Type *)(GPIOA_BASE + (((uint32_t) x) << 6)))    // Add x * 0x40
#define FGPIO(x) ((GPIO_Type *)(FGPIOA_BASE + (((uint32_t) x) << 6)))  // Add x * 0x40

// 773/ 807 in KL25RefMan.pdf General Purpose Input Output ---------------------------------------------- //
// #define GPIO_OutputBits(x, bits) GPIO(x)->PDDR = bits
// #define GPIO_InputBits(x, bits) GPIO(x)->PDDR = ~bits
// #define GPIO_OutputBit(x, bitx) GPIO(x)->PDDR |= (1UL << (bitx))
// #define GPIO_InputBit(x, bitx) GPIO(x)->PDDR &= ~(1UL << (bitx))

#define GPIO_IO_Bits(x, bits) GPIO(x)->PDDR = bits // bits[i] = 0: input, 1 output
#define GPIO_IO_Bit(x, bitx, IO) GPIO(x)->PDDR =  (GPIO(x)->PDDR & ~(1U << (bitx))) | (((IO) & 1U) << (bitx)) // IO 0: input, 1 output

#define FGPIO_IO_Bits(x, bits) FGPIO(x)->PDDR = bits // bits[i] = 0: input, 1 output
#define FGPIO_IO_Bit(x, bitx, IO) FGPIO(x)->PDDR =  (FGPIO(x)->PDDR & ~(1U << (bitx))) | (((IO) & 1U) << (bitx)) // IO 0: input, 1 output

#define GPIO_ReadBits(x) (GPIO(x)->PDIR)
#define GPIO_ReadOutBits(x) (GPIO(x)->PDOR)
#define GPIO_WriteBits(x, bits) GPIO(x)->PDOR = bits // bits should be like 0x00000000

#define GPIO_SetBit(x, bitx) GPIO(x)->PDOR |= (1UL << (bitx)) // bitx is a number like 18
#define GPIO_ResetBit(x, bitx) GPIO(x)->PDOR &= ~(1UL << (bitx))
#define GPIO_ToggleBit(x, bitx) GPIO(x)->PDOR ^= (1UL << (bitx))
#define GPIO_ReadBit(x, bitx) ((GPIO(x)->PDIR >> bitx) & (1UL))
#define GPIO_ReadOutBit(x, bitx) ((GPIO(x)->PDOR >> bitx) & (1UL))
#define GPIO_WriteBit(x, bitx, data) GPIO(x)->PDOR = (GPIO(x)->PDOR & ~(1U << (bitx))) | (((data) & 1U) << (bitx))
// #define GPIO_SetBit(x, bitx) GPIO(x)->PSOR |= (1UL << (bitx)) // Same Function as using PDOR
// #define GPIO_ResetBit(x, bitx) GPIO(x)->PCOR |= (1UL << (bitx))
// #define GPIO_ToggleBit(x, bitx) GPIO(x)->PTOR |= (1UL << (bitx))

#define FGPIO_ReadBits(x) (FGPIO(x)->PDIR)
#define FGPIO_ReadOutBits(x) (FGPIO(x)->PDOR)
#define FGPIO_WriteBits(x, bits) FGPIO(x)->PDOR = bits // bits should be like 0x00000000

#define FGPIO_SetBit(x, bitx) FGPIO(x)->PDOR |= (1UL << (bitx)) // bitx is a number like 18
#define FGPIO_ResetBit(x, bitx) FGPIO(x)->PDOR &= ~(1UL << (bitx))
#define FGPIO_ToggleBit(x, bitx) FGPIO(x)->PDOR ^= (1UL << (bitx))
#define FGPIO_ReadBit(x, bitx) ((FGPIO(x)->PDIR >> bitx) & (1UL))
#define FGPIO_ReadOutBit(x, bitx) ((FGPIO(x)->PDOR >> bitx) & (1UL))
#define FGPIO_WriteBit(x, bitx, data) FGPIO(x)->PDOR = (FGPIO(x)->PDOR & ~(1U << (bitx))) | (((data) & 1U) << (bitx))
// #define FGPIO_SetBit(x, bitx) FGPIO(x)->PSOR |= (1UL << (bitx)) // Same Function as using PDOR
// #define FGPIO_ResetBit(x, bitx) FGPIO(x)->PCOR |= (1UL << (bitx))
// #define FGPIO_ToggleBit(x, bitx) FGPIO(x)->PTOR |= (1UL << (bitx))


#define GPIO_Init(x, bitx, IO) PORT_Enable_Clock(x); PORT_Set_Mux(x, bitx, 1); GPIO_IO_Bit(x, bitx, (IO & 0x1)); GPIO_WriteBit(x, bitx, IO >> 1)
#define FGPIO_Init(x, bitx, IO) PORT_Enable_Clock(x); PORT_Set_Mux(x, bitx, 1); FGPIO_IO_Bit(x, bitx, (IO & 0x1)); FGPIO_WriteBit(x, bitx, IO >> 1)


#endif

#ifdef DEMO
void displayLED(int R_time, int G_time, int B_time, int flash) {
    int i = 0, j = 0, k = 0, multi = 10, inner_iter = (flash)? 32:1;
    int iter = flash? flash : (20000 / multi); 
    // int iter = 20000 / multi / inner_iter;
    PORT_Enable_Clock(PORT_B); 
    PORT_Enable_Clock(PORT_D);
    GPIO_Init(PORT_B, 18, GPIO_Output_1); 
    GPIO_Init(PORT_B, 19, GPIO_Output_1); 
    GPIO_Init(PORT_D, 1, GPIO_Output_1);
    for (i=0; i<iter; i++) {
        for (j=0; j<inner_iter; j++) {
            GPIO_ResetBit(1, 18); for (k = 0; k < multi * (R_time); k++);
            GPIO_SetBit(1, 18); for (k = 0; k < multi * (255 - R_time); k++);
            GPIO_ResetBit(1, 19); for (k = 0; k < multi * (G_time); k++);
            GPIO_SetBit(1, 19); for (k = 0; k < multi * (255 - G_time); k++);
            GPIO_ResetBit(3, 1); for (k = 0; k < multi * (B_time); k++);
            GPIO_SetBit(3, 1); for (k = 0; k < multi * (255 - B_time); k++);
        }
        for (j=0; j<inner_iter-1; j++) {
            for (k = 0; k < multi * 255; k++);
            for (k = 0; k < multi * 255; k++);
            for (k = 0; k < multi * 255; k++);
        }
    }
}
#endif


