/*
 * Author: Honbo He
 * RTP Multi-Task Kernel
 *
 * 2022-12-16
 */
#include <rtp-kernel.h>
#include <stdint.h>
#include <cmsis_gcc.h>

#define	BIT(n)		(1UL << n)
#define svc(n) __asm ("svc %0" :: "i"(n))

#define SCB_ICSR        (*(volatile uint32_t *)0xE000ED04)
#define ICSR_PENDSVSET  BIT(28)

#define RTP_TIME_SLICE	10

enum stat {
	RTP_STAT_UNUSED,
	RTP_STAT_RUNNING,
	RTP_STAT_BLOCKED
};

struct task {
	rtp_tid_t       tid;
	uint32_t       *stack;
	uint32_t        tick;
	uint32_t        slice;
	enum stat       stat;
};

/* task list */
static struct task task_list[MAX_TASKS];
static uint32_t task_stacks[MAX_TASKS][STACK_SIZE];

/* scheduler reference */
static uint32_t current_task;
static uint32_t next_task;

/* global os tick count
 * 0x80000000 = 2^31, the highest bit is 1 which indicate
 * a negative number in signed int
 */
static uint32_t rtp_os_tick;
#define RTP_TICK_AFTER(tick) (rtp_os_tick - tick < 0x80000000)


rtp_tid_t rtp_alloc_tid(void)
{
	int i, tid = -1;

	for (i = 0; i < MAX_TASKS; i++) {
		if (task_list[i].stat == RTP_STAT_UNUSED) {
			tid = i;
			break;
		}
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

	id = rtp_alloc_tid();
	if (id < 0)
		return -1;

	rtp_stack_init(id, task);
	task_list[id].tid	= id;
	task_list[id].tick	= 0;
	task_list[id].stat	= RTP_STAT_RUNNING;
	task_list[id].slice = RTP_TIME_SLICE;

	return 0;
}

/* rtp scheduler: simpl RR policy
 * task_list[0] always keep for idle
 */
void rtp_os_schedule(void)
{
	int i;
	uint32_t idx = current_task;

	for (i = 1; i < MAX_TASKS; i++) {
		idx = (current_task + i) & (MAX_TASKS - 1);
		if (idx == 0) idx = 1;

		if (task_list[idx].stat == RTP_STAT_RUNNING &&
			task_list[idx].slice > 0) {

			next_task = idx;
			return;
		}
	}

	for (i = 1; i < MAX_TASKS; i++) {
		if (task_list[i].stat == RTP_STAT_RUNNING)
			task_list[i].slice = RTP_TIME_SLICE;
	}

	next_task = 0;
}

void rtp_yield(void)
{
	rtp_os_schedule();
	rtp_pendsv_call();
}

void rtp_delete_task(int tid)
{
	if (tid < 1 || tid > MAX_TASKS - 1)
		return;

	task_list[tid].stat = RTP_STAT_UNUSED;

	if (tid == current_task) {
		rtp_yield();
	}
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

void rtp_msleep(uint32_t delay)
{
	if (delay == 0)
		return;

	task_list[current_task].tick = rtp_os_tick + delay;
	task_list[current_task].stat = RTP_STAT_BLOCKED;
	rtp_yield();
}

void rtp_tick_handler(void)
{
	int i;
	rtp_os_tick ++;

	/* check and wave sleeping tasks */
	for (i = 1; i < MAX_TASKS; i++) {
		if (task_list[i].stat != RTP_STAT_BLOCKED)
			continue;
		if (RTP_TICK_AFTER(task_list[i].tick)) {
			task_list[i].stat = RTP_STAT_RUNNING;
		}
	}

	/* modify time slice */
	if (current_task != 0) {
		if (task_list[current_task].slice > 0) {
			task_list[current_task].slice --;
			return;
		}
	}

	rtp_yield();
}

int rtp_os_init(void)
{
	rtp_create_task(rtp_idle_task);
	task_list[0].slice = 1;
	return 0;
}

void rtp_os_start(void)
{
	uint32_t *cstack;

	current_task = 1;
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

uint32_t rtp_stack_switch(uint32_t cstack)
{
	task_list[current_task].stack = (uint32_t *)cstack;
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
