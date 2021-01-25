#include "os.h"

#define DEFAULTXPSR (0x01000000U)

static uint64_t os_time = 0;
tcb_t *timer_waitqueue = (tcb_t *)(NULL);       // wait task
tcb_t *chan_waitqueue = (tcb_t *)(NULL);        // wait task

volatile uint32_t intdepth = 0;                 // interrupt depth
volatile ctx_t **curctx = (volatile ctx_t **)(NULL);    // current context, used by context switch and start
volatile ctx_t **hfctx = (volatile ctx_t **)(NULL);     // context to save where occurs hardfault
tcb_t *curtask = (tcb_t *)(NULL);       // current task, state == RUNNING
tcb_t *runqueue = (tcb_t *)(NULL);      // runnable task, would be a backup for curtask, state == RUNNABLE
tcb_t *rtrunqueue = (tcb_t *)(NULL);    // runnable realtimetask, would be a backup for curtask, state == RUNNABLE. when rtrunqueue was activated, it would make a context_switch 

#define _wait_for_interrupt __asm volatile ("wfi")
#define _disable_interrupt __asm volatile ("cpsid i")
#define _enable_interrupt __asm volatile ("cpsie i")
#define pushcli() { _disable_interrupt; intdepth++; }
#define popcli() { intdepth--; if(intdepth == 0) _enable_interrupt; }

tcb_t *task_void;
#define task_void_stack_size (0x100) // 16*4*4
uint8_t task_void_stack[task_void_stack_size];
void task_void_func(void) { while(1) _wait_for_interrupt; }

void task_create_void(void) {
    task_void = (tcb_t *)((char *)(task_void_stack) + task_void_stack_size - sizeof(tcb_t));
    task_void->pid = 0;
    task_void->realtime = 0;
    task_void->stackbase = (void *)(task_void_stack);
    task_void->stacksize = task_void_stack_size - sizeof(tcb_t);
    task_void->chan = NULL;
    task_void->sleepcnt = 0;
    task_void->context = (ctx_t *)((char *)task_void - sizeof(ctx_t));
    task_void->context->xPSR = DEFAULTXPSR;
    task_void->context->PC = (uint32_t)(&task_void_func);
    task_void->context->LR = (uint32_t)(&task_finish);
}

void task_runqueue_delete(tcb_t *p) { 
    tcb_t **queue = (p->realtime)? &rtrunqueue : &runqueue;
    if (p->next == p) *queue = (tcb_t *)(NULL);
    else {
        if (*queue == p) *queue = p->next;
        p->prev->next = p->next;
        p->next->prev = p->prev;
    }
}

void task_runqueue_append(tcb_t *p) { 
    tcb_t **queue = (p->realtime)? &rtrunqueue : &runqueue;
    if (*queue == (tcb_t *)(NULL)) {
        p->next = p;
        p->prev = p;
        *queue = p;
    } else {
        p->next = *queue;
        p->prev = (*queue)->prev;
        p->next->prev = p;
        p->prev->next = p;
    }
}

void task_create(void (*func)(void), void *stackbase, uint32_t stacksize, void *args, uint8_t realtime) {
    // NULL to ...->p0->p1->...->pN->p0->..., runnable first
    tcb_t *p;
    pushcli();
    p = (tcb_t *)((char *)(stackbase) + stacksize - sizeof(tcb_t));

    p->realtime = realtime;
    p->stackbase = stackbase;
    p->stacksize = stacksize;
    p->chan = (void *)NULL;
    p->sleepcnt = 0;

    p->context = (ctx_t *)((char *)p - sizeof(ctx_t));
    p->context->xPSR = DEFAULTXPSR;
    p->context->PC = (uint32_t)(func);
    p->context->LR = (uint32_t)(&task_finish);
    p->context->R0 = (uint32_t)(args);
    p->context->R1 = (uint32_t)(0); // reserved
    p->context->R2 = (uint32_t)(0); // reserved
    p->context->R3 = (uint32_t)(0); // reserved

    task_runqueue_append(p);
    popcli();
}

void task_finish(void) {
    pushcli();
    task_runqueue_delete(curtask);
    switch_context();
    popcli();
}

void os_start(uint32_t systick_ticks) {
    _disable_interrupt;
    task_create_void();
    SysTick_Init(systick_ticks, 0xFF);
    PendSV_Init(0xFF);
    curtask = (rtrunqueue != (tcb_t *)(NULL))? rtrunqueue : ((runqueue != (tcb_t *)(NULL))? runqueue : task_void); 
    curctx = (volatile ctx_t **)(&(curtask->context));
    task_start(); // would enable int in this func
}

void scheduler(void) {
    // sometimes curtask can be neither rtrunqueue or runqueue as it has been deleted
    if (curtask == rtrunqueue) rtrunqueue = rtrunqueue;
    else if (curtask == runqueue) runqueue = runqueue->next;
    curtask = (rtrunqueue != (tcb_t *)(NULL))? rtrunqueue : ((runqueue != (tcb_t *)(NULL))? runqueue : task_void); 
    curctx = (volatile ctx_t **)(&(curtask->context));
}

void SysTick_Handler(void) { 
    pushcli();
    os_time = os_time + 1;
    timer_wakeup(&timer_waitqueue);
	switch_context();
    popcli();
}

void yield(void) {
	switch_context();
}

// syscall function ----------------------
void *syscall(syscall_func f,...) { va_list ap; va_start(ap, f); return __syscall(f, &ap);  }

void syscall_handler(uint32_t *para) {
    // used inside SVC_Handler
    // now we have pushed(args), pushed(R0-R3, R12, LR, PC, xPSR) in R0, which is the para
    // parameter has been saved to r0, r1 when calling __syscall
    syscall_func f = (syscall_func)(para[0]);
    va_list *ap = (va_list *)(para[1]); // para0 is the valist
    void *res = f(*ap);
    para[0] = (uint32_t)res;
}

// time based functions -------------------------
void delay(uint32_t cnt) {
    pushcli();
    curtask->sleepcnt = cnt;
    task_runqueue_delete(curtask);
    task_queue_append(curtask, &timer_waitqueue);
	switch_context();
    popcli();
}

void delay_until(uint32_t cnt) {
    pushcli();
    if(cnt - os_time > 0) {
        curtask->sleepcnt = cnt - os_time;
        task_runqueue_delete(curtask);
        task_queue_append(curtask, &timer_waitqueue);
	    switch_context();
    }
    popcli();
}

void timer_wakeup(tcb_t **wqueue) {
    tcb_t *p = *wqueue, *next;
    // check wait queue
    if (*wqueue != NULL) while(1) {
        next = p->next;
        if (p->sleepcnt == 0) {
            task_queue_delete(p, wqueue);
            task_runqueue_append(p);
        } else p->sleepcnt--;
        p = next;
        if (p == *wqueue || *wqueue == (tcb_t *)(NULL)) break;
    }
}

// mutex based functions -------------------------
void mutex_init(mutex_t *lock) {
    lock->lock = 0;
    lock->owner = (tcb_t *)NULL;
    lock->waitqueue = (tcb_t *)NULL;
}

void mutex_acquire(mutex_t *lock) {
    pushcli();
    if (lock->lock && curtask != lock->owner) {
        task_runqueue_delete(curtask);
        task_queue_append(curtask, &(lock->waitqueue));
        switch_context();
    } else {
        lock->lock = 1;
        lock->owner = curtask;
    }
    popcli();
}

void mutex_release(mutex_t *lock) {
    tcb_t *p;
    pushcli();
    if (curtask == lock->owner) {    
        if(lock->waitqueue != (tcb_t *)NULL) {
            p = lock->waitqueue;
            task_queue_delete(p, &(lock->waitqueue));
            task_runqueue_append(p);
            lock->lock = 1;
            lock->owner = p;    // redo mutex_acquire
            switch_context();   // try to avoid curtask get the lock again
        } else {
            lock->lock = 0;
            lock->owner = (tcb_t *)NULL;
        }
    }
    popcli();
}

// semaphor based functions ----------------------
void sem_init(sem_t *sem, int32_t value) {
    sem->cnt = value;
    sem->waitqueue = (tcb_t *)NULL;
}

void sem_down(sem_t *sem) {
    pushcli();
    if (sem->cnt == 0) {
        task_runqueue_delete(curtask);
        task_queue_append(curtask, &(sem->waitqueue));
        switch_context();
    } else {
        (sem->cnt)--;
    }
    popcli();
}

void sem_post(sem_t *sem) {
    tcb_t *p;
    pushcli();
    (sem->cnt)++;
    if (sem->cnt == 1) {
        if(sem->waitqueue != (tcb_t *)NULL) {
            p = sem->waitqueue;
            task_queue_delete(p, &(sem->waitqueue));
            task_runqueue_append(p);
            (sem->cnt)--;       // redo sem_down
            switch_context();   // try to avoid curtask get the sem_down again
        }
    } 
    popcli();
}

// chan based functions --------------------------
void chan_sleep(void *chan, mutex_t *lock) {
    pushcli();
    mutex_release(lock);
    curtask->chan = chan;
    task_runqueue_delete(curtask);
    task_queue_append(curtask, &chan_waitqueue);
	switch_context();
    popcli();
}

void chan_wakeup(void *chan, tcb_t *target) {
    // if target == NULL, wake up all
    tcb_t *p = chan_waitqueue, *next;
    pushcli();
    if (chan_waitqueue != NULL) while(1) {
        next = p->next;
        if (p->chan == chan && (target == (tcb_t *)NULL || p == target)) {
            p->chan = (void *)NULL;
            task_queue_delete(p, &chan_waitqueue);
            task_runqueue_append(p);
        }
        p = next;
        if (p == chan_waitqueue || chan_waitqueue == (tcb_t *)(NULL)) break;
    }
    popcli();
}

// pipe based functions -------------------------
void pipe_init(pipe_t *pipe, uint8_t *buffer, uint32_t buffersize) {
    pipe->buffer = buffer;
    pipe->buffersize = buffersize;
    pipe->nwrite = 0;
    pipe->nread = 0;
    mutex_init(&(pipe->lock));
}

void pipe_write(pipe_t *pipe, uint8_t *data, uint32_t length) {
    int i;
    mutex_acquire(&(pipe->lock));
    for(i = 0; i< length; i++) {
        // check if write is full
        while ((pipe->nwrite == pipe->nread + pipe->buffersize) || (pipe->nwrite + 1 == pipe->nread)) {
            chan_sleep(&(pipe->write_chan), &(pipe->lock));
            mutex_acquire(&(pipe->lock));
        }
        pipe->buffer[pipe->nwrite] = data[i];
        pipe->nwrite = (pipe->nwrite == pipe->buffersize - 1) ? 0 : (pipe->nwrite + 1);
        chan_wakeup(&(pipe->read_chan), NULL);
    }
    mutex_release(&(pipe->lock));
}

void pipe_read(pipe_t *pipe, uint8_t *data, uint32_t length) {
    int i;
    mutex_acquire(&(pipe->lock));
    for(i = 0; i< length; i++) {
        // check if read empty
        while (pipe->nread == pipe->nwrite) {
            chan_sleep(&(pipe->read_chan), &(pipe->lock));
            mutex_acquire(&(pipe->lock));
        }
        data[i] = pipe->buffer[pipe->nread];
        pipe->nread = (pipe->nread == pipe->buffersize - 1) ? 0 : (pipe->nread + 1);
        chan_wakeup(&(pipe->write_chan), NULL);
    }
    mutex_release(&(pipe->lock));
}

// syscall based functions ----------------------
void *sys_yield(va_list ap) { 
    yield(); 
    return NULL; 
}
void *sys_delay(va_list ap) { 
    delay((uint32_t)va_arg(ap, uint32_t));
    return NULL; 
}
void *sys_delay_until(va_list ap) { 
    delay_until((uint32_t)va_arg(ap, uint32_t));
    return NULL; 
}
void *sys_mutex_init(va_list ap) { 
    mutex_init((mutex_t *)va_arg(ap, mutex_t *));
    return NULL; 
}
void *sys_mutex_acquire(va_list ap) { 
    mutex_acquire((mutex_t *)va_arg(ap, mutex_t *)); 
    return NULL; 
}
void *sys_mutex_release(va_list ap) { 
    mutex_release((mutex_t *)va_arg(ap, mutex_t *)); 
    return NULL; 
}
void *sys_sem_init(va_list ap) { 
    sem_init((sem_t *)va_arg(ap, sem_t *), (int32_t)va_arg(ap, uint32_t));
    return NULL; 
}
void *sys_sem_down(va_list ap) { 
    sem_down((sem_t *)va_arg(ap, sem_t *)); 
    return NULL; 
}
void *sys_sem_post(va_list ap) { 
    sem_post((sem_t *)va_arg(ap, sem_t *)); 
    return NULL; 
}
void *sys_chan_sleep(va_list ap) { 
    chan_sleep((void *)va_arg(ap, void *), (mutex_t *)va_arg(ap, mutex_t *)); 
    return NULL; 
}
void *sys_chan_wakeup(va_list ap) { 
    chan_wakeup((void *)va_arg(ap, void *), (tcb_t *)va_arg(ap, tcb_t *)); 
    return NULL; 
}
void *sys_pipe_init(va_list ap) { 
    pipe_init((pipe_t *)va_arg(ap, pipe_t *), (uint8_t *)va_arg(ap, uint8_t *), (uint32_t)va_arg(ap, uint32_t)); 
    return NULL; 
}
void *sys_pipe_write(va_list ap) { 
    pipe_write((pipe_t *)va_arg(ap, pipe_t *), (uint8_t *)va_arg(ap, uint8_t *), (uint32_t)va_arg(ap, uint32_t)); 
    return NULL; 
}
void *sys_pipe_read(va_list ap) { 
    pipe_read((pipe_t *)va_arg(ap, pipe_t *), (uint8_t *)va_arg(ap, uint8_t *), (uint32_t)va_arg(ap, uint32_t)); 
    return NULL; 
}


