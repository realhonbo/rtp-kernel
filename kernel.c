/*
 * RTP Multi-Task Real-Time Kernel for ARMv7-M Chips
 *
 * Copyright(C) Honbo He
 * 2022-12-16
 */
#include <rtp-kernel.h>
#include <stdint.h>
#include <cmsis_gcc.h>

#ifndef __always_inline
#define __always_inline inline __attribute__((always_inline))
#endif

#define	BIT(n)		(1UL << n)
#define STACK_SIZE	(RTP_STACK_SIZE >> 2)

#define SCB_ICSR            (*(volatile uint32_t *)0xE000ED04)
#define ICSR_PENDSVSET      BIT(28)
#define RTP_STACK_MAGIC     0xDEADBEEF
#define RTP_TIME_SLICE      10

enum stat {
	RTP_STAT_UNUSED = 0,
	RTP_STAT_RUNNING,
	RTP_STAT_BLOCKED
};

struct task {
	rtp_tid_t       tid;
	uint32_t       *stack;
	uint32_t        tick;	/* wake up timestamp */
	uint32_t        slice;	/* time slice */
	enum stat       stat;
};

/* task list */
static struct task task_list[MAX_TASKS];
static uint32_t task_stacks[MAX_TASKS][STACK_SIZE] __attribute__((aligned(8)));


static void rtp_kernel_panic(const char *msg)
{
	(void) msg;
	while(1) __NOP();
}

/* bitmap for O(1) scheduling lookup
 *
 * rtp_rdy_grp: task ready: RUNNING stat and has slice
 * rtp_exp_grp: task expired: RUNNING stat but slice = 0
 */
typedef volatile uint32_t rtp_bm_t;

static rtp_bm_t rtp_rdy_grp;
static rtp_bm_t rtp_exp_grp;

static void rtp_bitmap_init(rtp_bm_t *bitmap)
{
	*bitmap = 0;
}

static __always_inline void rtp_bitmap_set(rtp_bm_t *bitmap, uint32_t bit)
{
    *bitmap |= (1UL << bit);
}

static __always_inline void rtp_bitmap_clear(rtp_bm_t *bitmap, uint32_t bit)
{
    *bitmap &= ~(1UL << bit);
}

/* scheduler reference */
static uint32_t current_task;
static uint32_t next_task;

/* global os tick count
 * 0x80000000 = 2^31, the highest bit is 1 which indicate
 * a negative number in signed int
 */
static volatile uint32_t rtp_os_tick;
#define RTP_TICK_AFTER(tick) ((int)(rtp_os_tick - tick) >= 0)

rtp_tid_t rtp_alloc_tid(void)
{
	int i, tid = -1;

	for (i = 0; i < MAX_TASKS; i++)
		if (task_list[i].stat == RTP_STAT_UNUSED) {
			tid = i;
			break;
		}
	return tid;
}

int rtp_current_get(void)
{
	return current_task;
}

/* idle task: tid should be 0 and won't exit
 * if no ready or running task, idle will be schedule
 */
void rtp_idle_task(void)
{
	while (1)
		__WFI();
}

void rtp_pendsv_call(void)
{
	SCB_ICSR |= ICSR_PENDSVSET;

	__DSB();
	__ISB();
}

int rtp_stack_init(int tid, taskloop_t task)
{
	uint32_t *stk;

	/* guard for stack overflow */
	task_stacks[tid][0] = RTP_STACK_MAGIC;

	stk = &task_stacks[tid][STACK_SIZE];

	/* fill magic number for a new task at start
	 * xPSR: set the BIT(24) to indicate a Thumb2 code
	 */
	*(--stk) = BIT(24);					// xPSR
	*(--stk) = (uint32_t) task;			// PC
    *(--stk) = (uint32_t) rtp_task_exit;// LR
    *(--stk) = 0x12121212;				// R12
    *(--stk) = 0x03030303;				// R3
    *(--stk) = 0x02020202;				// R2
    *(--stk) = 0x01010101;				// R1
    *(--stk) = 0x00000000;				// R0

	*(--stk) = 0x11111111;				// R11
    *(--stk) = 0x10101010;				// R10
    *(--stk) = 0x09090909;				// R9
    *(--stk) = 0x08080808;				// R8
    *(--stk) = 0x07070707;				// R7
    *(--stk) = 0x06060606;				// R6
    *(--stk) = 0x05050505;				// R5
    *(--stk) = 0x04040404;				// R4

	task_list[tid].stack = stk;
	return 0;
}

int rtp_create_task(taskloop_t task)
{
	int id;

	RTP_CLI();
	id = rtp_alloc_tid();
	if (id < 0)
		goto out;

	rtp_stack_init(id, task);
	task_list[id].tid   = id;
	task_list[id].tick  = 0;
	task_list[id].stat  = RTP_STAT_RUNNING;
	task_list[id].slice = RTP_TIME_SLICE;

	rtp_bitmap_set(&rtp_rdy_grp, id);
out:
	RTP_STI();
	return id;
}

/*
 * rtp scheduler: RR with O(1) lookup
 * task_list[0] always keep for idle
 */
void rtp_os_schedule(void)
{
	rtp_bm_t mask_upper;

	/* epoch switch
	 * if no task whose stat == running & slice > 0, exchange
	 * the ready bitmap and expired bitmap
	 */
	if ((rtp_rdy_grp & ~1UL) == 0 && (rtp_exp_grp & ~1UL) != 0) {
		rtp_rdy_grp |= rtp_exp_grp;
		rtp_exp_grp = 0;
	}

	/* ignore the prev tasks, get bitmap of tasks
	 * which are the next
	 * if no next tasks, round and jump over idle
	 */
	mask_upper = rtp_rdy_grp & ~((1UL << (current_task + 1)) - 1);
	if (! mask_upper)
		mask_upper = rtp_rdy_grp & ~1UL;

    if (! mask_upper)
        next_task = 0;
    else {
		/* get the lowest set bit, which is the tid of next task
		 */
        next_task = __builtin_ctz(mask_upper);

		/* lazy refill
		 * only if task was chosen by. in this case slice == 0 indicates
		 * the task has just returned from the expired group, recharge it
		 * to avoids the O(N) traversal assignment overhead
		 */
        if (task_list[next_task].slice == 0) {
             task_list[next_task].slice = RTP_TIME_SLICE;
        }
    }
}

void rtp_yield(void)
{
	RTP_CLI();

	/* give up the remaining time slice
	 * move from ready group to expires
	 */
	task_list[current_task].slice = 0;
	rtp_bitmap_clear(&rtp_rdy_grp, current_task);
	rtp_bitmap_set(&rtp_exp_grp, current_task);

	rtp_os_schedule();
	/*
	 * if there is only 1 task, next_task will be
	 * set to itself after schedule
	 * in this case should not trigger task_switch
	 */
	if (next_task != current_task)
		rtp_pendsv_call();

	RTP_STI();
}

void rtp_delete_task(int tid)
{
	if (tid < 1 || tid > MAX_TASKS - 1)
		return;

	RTP_CLI();
	task_list[tid].stat = RTP_STAT_UNUSED;

	rtp_bitmap_clear(&rtp_rdy_grp, tid);
	rtp_bitmap_clear(&rtp_exp_grp, tid);

	if (tid == current_task) {
		rtp_os_schedule();
		rtp_pendsv_call();
	}
	RTP_STI();
}

void rtp_task_exit(void)
{
	rtp_delete_task(current_task);

	/* If PendSV was preempted by a high-prio interrupt,
	 * after the interrupt ends, pop {lr} returns to a
	 * task which has already exit
	 * it will cause os panic, so the nop loop is nescessry
	 */
	while (1) __NOP();
}

/* rtp_msleep(0) will try schedule to other tasks
 * if no another, it equals to rtp_msleep(1 tick)
 */
void rtp_msleep(uint32_t delay)
{
	RTP_CLI();

	task_list[current_task].tick = rtp_os_tick + delay;
	task_list[current_task].stat = RTP_STAT_BLOCKED;
	rtp_bitmap_clear(&rtp_rdy_grp, current_task);

	rtp_os_schedule();
	rtp_pendsv_call();

	RTP_STI();
}

void rtp_tick_handler(void)
{
	int i;
	rtp_os_tick ++;

	/* check and wake sleeping tasks */
	for (i = 1; i < MAX_TASKS; i++) {
		if (task_list[i].stat != RTP_STAT_BLOCKED)
			continue;

		if (RTP_TICK_AFTER(task_list[i].tick)) {
			task_list[i].stat = RTP_STAT_RUNNING;
			rtp_bitmap_set(&rtp_rdy_grp, i);
		}
	}

	/* modify the time slice. if time slice exhausted,
	 * move the task to expires group and trigger schedule
	 */
	if (current_task != 0) {
		if (task_list[current_task].slice > 0)
			task_list[current_task].slice --;
		if (task_list[current_task].slice > 0)
			return;
		rtp_bitmap_clear(&rtp_rdy_grp, current_task);
		rtp_bitmap_set(&rtp_exp_grp, current_task);
	}

	rtp_os_schedule();
	rtp_pendsv_call();
}

int rtp_os_init(void)
{
	/* clear task bitmap */
	rtp_bitmap_init(&rtp_rdy_grp);
	rtp_bitmap_init(&rtp_exp_grp);

	/* init idle task */
	rtp_create_task(rtp_idle_task);
	task_list[0].slice = 1;

	return 0;
}

void rtp_os_start(void)
{
	uint32_t *cstack;

	current_task = 1;
	rtp_bitmap_set(&rtp_rdy_grp, 0);
	if ((rtp_rdy_grp & ~1UL) == 0)
		current_task = 0;
	next_task = current_task;

	cstack = task_list[current_task].stack;

	__asm volatile (
	/* set the core to `Privileged Thread Mode: control[0] = 0
	 * is for `rtp_pendsv_call works in scheduler
	 * by the way, disabling user access to registers in RTOS
	 * is an inappropriate operation
	 */
	"msr psp, %0            \n"
	"mov r0, #2             \n"
	"msr control, r0        \n"
	"isb                    \n"

	"ldmia sp!, {r4-r11}    \n"

	/* caller register won't pop automatically */
	"ldmia sp!, {r0-r3}     \n"
	"ldmia sp!, {r12}       \n"
	"ldmia sp!, {lr}        \n"

	"ldr r12, [sp], #8      \n"
	"cpsie i                \n"

	"bx r12                 \n"
	".align 4               \n"
	:: "r" (cstack)
	: "r0", "memory"
	);
}

__attribute__((unused))
static void rtp_check_stack(rtp_tid_t tid)
{
	if (task_stacks[tid][0] != RTP_STACK_MAGIC)
		rtp_kernel_panic("Stack Overflow!");
}

uint32_t rtp_stack_switch(uint32_t cstack)
{
	task_list[current_task].stack = (uint32_t *)cstack;
#ifdef RTP_DEBUG
	rtp_check_stack(current_task);
#endif
	current_task = next_task;
	return (uint32_t)task_list[current_task].stack;
}

__attribute__((naked))
void PendSV_Handler(void)
{
	__asm volatile (
	/* store current context */
	"mrs r0, psp            \n"
	"isb                    \n"
	"stmdb r0!, {r4-r11}    \n"

	"push {lr}				\n"
	"bl rtp_stack_switch    \n"
	"pop {lr}               \n"

	"ldmia r0!, {r4-r11}    \n"
	"msr psp, r0            \n"
	"isb                    \n"

	"bx lr                  \n"
	".align 4               \n"
	);
}
