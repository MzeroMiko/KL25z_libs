/* os_asm_keil.c is for armcc, and in source_bak/keil/ */

.extern hfctx
.extern curctx
.extern scheduler
.extern syscall_handler
.global task_start
.global __syscall
.global SVC_Handler
.global PendSV_Handler
.global HardFault_Handler

.align 4
.thumb
.syntax unified

/* 
    when using inline assembly, this func would swallow 8bits ( compiler would auto add "push r7, lr" but can not reach to pop in the very end of the function (after asm), as bx lr has been written in asm ) 
*/
/*
    // MRS / MSR Dest, Src: loading/saving a special registers
    // SUBS Rd, Rn, Rm: Rd = Rn - Rm;
    // LDR Dest, Src; STR Src, Dest; MOV Dest, Src;
    // cpsid i: disable interrupt; cpsie i: enable interrupt; 
    // STMIA R0! {Ri}: while(i++) { store Ri to [R0]; R0+=4 }
    // STMDB R0! {Ri}: while(i++) { store Ri to [R0]; R0-=4 }
    // LDMIA R0! {Ri}: while(i++) { load Ri from [R0]; R0+=4 }

    // The Hardware would push old registers {R12,LR,PC,xPSR} into old psp (but you dont know what time it would store), and pop new registers {R12,LR,PC,xPSR} from new psp (when return). but you can not set the psp manually (cause it was used by tasks, so you have to move the context.) 

    // EXC_RETURN_Value    Mode_to_Return_To	                Stack_to_use
    // 0xFFFFFFF1	        Handler_Mode	                    MSP
    // 0xFFFFFFF9	        Thread_Mode	                        MSP
    // 0xFFFFFFFD	        Thread_Mode	                        PSP
    // 0xFFFFFFE1	        Handler_Mode_(FPU Extended Frame)	MSP
    // 0xFFFFFFE9	        Thread_Mode_(FPU Extended Frame)	MSP
    // 0xFFFFFFED	        Thread_Mode_(FPU Extended Frame)	PSP

*/
.thumb_func
PendSV_Handler:
    mrs     r0, primask 
    push    {r0}        
    cpsid	i           
    mrs	    r0, psp     
    subs	r0, r0, #16 
    stmia	r0!,{r4-r7} 
    subs	r0, r0, #16   /* same as stmdb	r0!,{r4-r7} , but stmdb is not supported */
    mov	    r4, r8      
    mov	    r5, r9      
    mov	    r6, r10     
    mov	    r7, r11     
    subs	r0, r0, #16 
    stmia	r0!,{r4-r7} 
    subs	r0, r0, #16 
    ldr	    r2, =curctx         /* get addr of curctx */    
    ldr	    r1, [r2]    
    str	    r0, [r1]    
    push    {r2, lr}    
    /* cpsid	i */      
    bl      scheduler        /* scheduler would decide witch current_context to switch to */
    /* cpsie	i */     
    pop     {r2, r3}    
    ldr	    r1, [r2]    
    ldr	    r0, [r1]    
    ldmia	r0!,{r4-r7} 
    mov	    r8, r4      
    mov	    r9, r5      
    mov	    r10, r6     
    mov	    r11, r7     
    ldmia	r0!,{r4-r7} 
    msr	    psp, r0     
    pop     {r0}        
    msr     primask, r0    
    bx	    r3              


.thumb_func
task_start:
    ldr	    r2, =curctx	
	ldr     r1, [r2]    
    ldr     r0, [r1]    
    adds    r0, r0, #32	/* skip r8-r11,r4-r7 (use adds and movs to use #imm or compiler would get error)*/
    ldmia   r0!,{r1-r6}  /* pop r0-r3, r12, lr (r0, r0-r3 may have parameters) */
    mov     lr, r6  	
    ldmia   r0!,{r5-r6}  /* pop pc, xPSR, skip xPSR */
    msr     psp, r0	    
    mov     r0, r1       /* re get r0-r3 from r1-r4 */
    mov     r1, r2      
    mov     r2, r3      
    mov     r3, r4      
    movs    r4, #3
    cpsie   i
    msr     CONTROL, r4     /* switch to psp stack, user mode */
    dsb				    /* data barrier */
    isb				    /* instruction barrier */
    bx      r5	

.thumb_func
__syscall:
    svc     #0
    bx      lr

.thumb_func
SVC_Handler:
    mov     r2, lr
    push    {r2}
    mrs     r0, psp
    bl      syscall_handler
    pop     {r2}
    bx      r2

.thumb_func
HardFault_Handler:
    mrs     r0, primask 
    push    {r0}        
    cpsid	i
    
    movs    r1, #4
    mov     r2, lr
    tst     r2, r1             /*check if lr is 0xFFFFFFF9 / 0xFFFFFFF1 or 0xFFFFFFFD, if D, get 1 */
    beq     _HF_MRS_MSP
    mrs     r0, psp
    b       _HF_STORE
_HF_MRS_MSP:
    mrs     r0, msp
_HF_STORE:
    subs	r0, r0, #16 
    stmia	r0!,{r4-r7} 
    subs	r0, r0, #16
    mov	    r4, r8      
    mov	    r5, r9      
    mov	    r6, r10     
    mov	    r7, r11     
    subs	r0, r0, #16 
    stmia	r0!,{r4-r7} 
    subs	r0, r0, #16
    ldr	    r2, =hfctx         /* get addr of hard fault ctx */    
    ldr	    r1, [r2]    
    str	    r0, [r1]

    movs    r1, #4
    mov     r2, lr
    tst     r2, r1
    beq     _HF_MSR_MSP
    msr	    psp, r0
    b       _HF_PROC
_HF_MSR_MSP:
    msr     msp, r0
_HF_PROC:
    /* did not have any idea*/    
    pop     {r0}        
    msr     primask, r0    
    b	    .               /* loop */





