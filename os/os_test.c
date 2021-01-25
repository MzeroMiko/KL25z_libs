// #define testOS

#if defined(testOS)

#include "../sh.h"
#include "os.h"

int arr[1000];
int cnt = 0;

mutex_t lock;

void task1_handler(void) {
	int i = 0;
    while (1) {
        syscall(sys_mutex_acquire, &lock);
        GPIO_ToggleBit(PORT_B, 18);
        syscall(sys_delay, 10000);
        GPIO_ToggleBit(PORT_B, 18);
        syscall(sys_delay, 10000);
        shell_printf("task1 is running.\n");
        syscall(sys_yield);
        for (i=0;i<1000;i++);
        syscall(sys_mutex_release, &lock);
	}
}

void task2_handler(void) {
	int i = 0;
	while (1) {
        syscall(sys_mutex_acquire, &lock);
        GPIO_ToggleBit(PORT_B, 19);
        syscall(sys_delay, 10000);
        GPIO_ToggleBit(PORT_B, 19);
        syscall(sys_delay, 10000);
        shell_printf("task2 is running.\n");
        syscall(sys_yield);
        for (i=0;i<1000;i++);
        syscall(sys_mutex_release, &lock);
	}
}

void task3_handler(void) {
	int i = 0;
	while (1) {
        syscall(sys_mutex_acquire, &lock);
        GPIO_ToggleBit(PORT_D, 1);
        syscall(sys_delay, 10000);
        GPIO_ToggleBit(PORT_D, 1);
        syscall(sys_delay, 10000);
        shell_printf("task3 is running.\n");
        syscall(sys_yield);
        for (i=0;i<1000;i++);
        syscall(sys_mutex_release, &lock);
    }
}

uint32_t stack1[128];
uint32_t stack2[128];
uint32_t stack3[128];

int OS_test(void) {
    UART0_Init(9600);
    PORT_Enable_Clock(PORT_B); 
    PORT_Enable_Clock(PORT_D);
    GPIO_Init(PORT_B, 18, GPIO_Output_1); 
    GPIO_Init(PORT_B, 19, GPIO_Output_1); 
    GPIO_Init(PORT_D, 1, GPIO_Output_1);


	task_create(&task1_handler, (void *)stack1, sizeof(uint32_t) * 128, NULL, 0);
	task_create(&task2_handler, (void *)stack2, sizeof(uint32_t) * 128, NULL, 0);
	task_create(&task3_handler, (void *)stack3, sizeof(uint32_t) * 128, NULL, 0);

    mutex_init(&lock); // since we are here at main

	os_start(SystemCoreClock / 10000);

	while (1);
}

int main() {

    OS_test();    
    // shell_registercmd("led", led_test, 4, "led red green blue flash");
    // shell_registercmd("pit", PIT_test, 1, "pit freqency");
    // shell_registercmd("dac", DAC_test, 2, "dac start_num count_down_step");
    // shell_main();
    // while(1) Wait_Interrupt();
}


#endif

