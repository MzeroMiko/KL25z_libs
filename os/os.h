#ifndef __OS_H__
#define __OS_H__

#include "stdint.h"
#include "stdarg.h"

#ifndef NULL
	#define NULL ((void *)0)
#endif

// #define SCB_BASE                     (0xE000ED00UL)
#define SCB_IT_CTRL                     ((volatile uint32_t *)0xE000ED04UL)
#define SCB_IT_CTRL_PENDSVSET_MASK      (0x10000000UL)

// #define SysTick_BASE                 (0xE000E010UL)
#define SysTick_CTRL                    ((volatile uint32_t *)0xE000E010UL)
#define SysTick_LOAD                    ((volatile uint32_t *)0xE000E014UL)
#define SysTick_VAL                     ((volatile uint32_t *)0xE000E018UL)
#define SysTick_CTRL_CLKSET_MASK        (0x00000007UL)
#define SysTick_CTRL_COUNTFLAG_MASK     (0x00010000UL)
#define SysTick_CTRL_CLKSOURCE_MASK     (0x00000004UL)
#define SysTick_CTRL_TICKINT_MASK       (0x00000002UL)
#define SysTick_CTRL_ENABLE_MASK        (0x00000001UL)

// #define NVIC_BASE                    (0xE000E100UL)
#define NVIC_CORE_PRI2				    ((volatile uint32_t *)0xE000ED20UL) // SCB->SHP[1]
#define NVIC_MIN_IT_PRIORITY			(0xFFUL)
#define NVIC_PENDSV_PRI_MASK			(0x00FF0000UL)
#define NVIC_SYSTICK_PRI_MASK			(0xFF000000UL)
#define NVIC_PENDSV_PRI_SHIFT			(16UL)
#define NVIC_SYSTICK_PRI_SHIFT			(24UL)

#define switch_context() *SCB_IT_CTRL |= SCB_IT_CTRL_PENDSVSET_MASK // trigger PendSV_Handler
#define PendSV_Init(priority) *NVIC_CORE_PRI2 = (*NVIC_CORE_PRI2 & ~NVIC_PENDSV_PRI_MASK) | ((priority & 0xFFUL) << NVIC_PENDSV_PRI_SHIFT)
#define SysTick_Init(ticks, priority) *SysTick_LOAD = (uint32_t)(ticks - 1UL), *SysTick_VAL = 0UL, *SysTick_CTRL = SysTick_CTRL_CLKSET_MASK, *NVIC_CORE_PRI2 = (*NVIC_CORE_PRI2 & ~NVIC_SYSTICK_PRI_MASK) | ((priority & 0xFFUL) << NVIC_SYSTICK_PRI_SHIFT) 

typedef struct ctx {
    // xPSR -> R0 is autosaved when interrupt, 
    // in PendSV_Handler, we saved R7 -> R8
    uint32_t R8;
    uint32_t R9;
    uint32_t R10;
    uint32_t R11;
    uint32_t R4;
    uint32_t R5;
    uint32_t R6;
    uint32_t R7;
    uint32_t R0;
    uint32_t R1;
    uint32_t R2;
    uint32_t R3;
    uint32_t R12;
    uint32_t LR;
    uint32_t PC;
    uint32_t xPSR;
} ctx_t;

typedef struct tcb {
    uint32_t pid;
    uint32_t realtime;             
    uint32_t stacksize;
    uint32_t sleepcnt;
    ctx_t *context;
    void *chan;
    void *stackbase;
    struct tcb *next;             
    struct tcb *prev;
} tcb_t;

typedef struct {
    int32_t cnt;
    tcb_t *waitqueue;
} sem_t;

typedef struct {
    int32_t lock;
    tcb_t *owner;
    tcb_t *waitqueue;
} mutex_t;

typedef struct {
    mutex_t lock;
    uint8_t *buffer;
    uint32_t nwrite;
    uint32_t nread; 
    uint32_t buffersize;
    void *write_chan;
    void *read_chan;
} pipe_t;

typedef void *(*syscall_func)(va_list ap);

#define NTASK 16
extern volatile uint32_t intdepth;
extern tcb_t *curtask; 
extern tcb_t *runqueue;
extern tcb_t *rtrunqueue;
extern uint32_t task_num;
extern tcb_t task_table[NTASK];

#define task_queue_append(p, queue) {\
    if (*queue == (tcb_t *)(NULL)) {\
        p->next = p;\
        p->prev = p;\
        *queue = p;\
    } else {\
        p->next = *queue;\
        p->prev = (*queue)->prev;\
        p->next->prev = p;\
        p->prev->next = p;\
    }\
} // tcb_t *p, tcb_t **queue;

#define task_queue_delete(p, queue) {\
    if (p->next == p) *queue = (tcb_t *)(NULL);\
    else {\
        if (*queue == p) *queue = p->next;\
        p->prev->next = p->next;\
        p->next->prev = p->prev;\
    }\
} // tcb_t *p, tcb_t **queue;

// defined in os_asm.s
void task_start(void);
void *__syscall(syscall_func f, va_list *ap);
void SysTick_Handler(void);
void PendSV_Handler(void);
void SVC_Handler(void);
void HardFault_Handler(void);

// defined in os_core.c
void task_create(void (*func)(void), void *stackbase, uint32_t stacksize, void *args, uint8_t realtime);
void os_start(uint32_t systick_ticks);
void task_finish(void);
void scheduler(void);

void yield(void);
void delay(uint32_t cnt);
void delay_until(uint32_t cnt);
void timer_wakeup(tcb_t **wqueue);

void mutex_init(mutex_t *lock);
void mutex_acquire(mutex_t *lock);
void mutex_release(mutex_t *lock);

void sem_init(sem_t *sem, int32_t value);
void sem_down(sem_t *sem);
void sem_post(sem_t *sem);

void chan_sleep(void *chan, mutex_t *lock);
void chan_wakeup(void *chan, tcb_t *target);

void *syscall(syscall_func f,...);
void *sys_yield(va_list ap);
void *sys_delay(va_list ap);
void *sys_delay_until(va_list ap);
void *sys_mutex_init(va_list ap);
void *sys_mutex_acquire(va_list ap);
void *sys_mutex_release(va_list ap);
void *sys_sem_init(va_list ap);
void *sys_sem_down(va_list ap);
void *sys_sem_post(va_list ap);
void *sys_chan_sleep(va_list ap);
void *sys_chan_wakeup(va_list ap);
void *sys_pipe_init(va_list ap);
void *sys_pipe_write(va_list ap);
void *sys_pipe_read(va_list ap);

#endif
