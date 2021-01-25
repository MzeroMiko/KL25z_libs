This is for the course "Microcontroller and Embedded System" written by Mzero. Code is fit for ARMCC and GNU/arm_none_eabi. 

The final race uses code file "race.c", which can detect open loop system identity (use with race.m)，detect close loop system identity (use with race.m) race a car.

"sh.c & sh.h" provided basic functions dealing with strings and gets/printf,  and a serial (front statge application) shell.

"os/*" provides a simple design of operating system in KL25Z, use os_asm_gcc.s when using compiler arm_none_eabi instead of os_asm_keil.s, which uses armcc as its compiler. Use os_test.c to test the functions.

"redrivers/*" provides a simple library of some external devices in KL25Z, like DAC, ADC, GPIO, PORT, PIT, TPM, UART.

All code above is not supposed to be 100% right, but makes sense  in normal cases. You can do what ever you want with these code files.
