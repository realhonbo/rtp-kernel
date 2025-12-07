/*
 * Author: Honbo He
 * RTP Kernel
 * 2022-12-16
 */
#include <stdint.h>
#include <cmsis_gcc.h>

#define	BIT(n)		(1UL << n)

#define sti()  __asm ("cpsie i" ::: "memory")
#define cli()  __asm ("cpsid i" ::: "memory")
#define svc(n) __asm ("svc %0" :: "i"(n))

#define SCB_ICSR        (*(volatile uint32_t *)0xE000ED04)
#define ICSR_PENDSVSET  BIT(28)

#define MAX_TASKS		5
#define STACK_SIZE		256

enum stat {
	RTP_STAT_UNUSED,
	RTP_STAT_READY,
	RTP_STAT_RUNNING,
};

struct task {
	uint32_t       *stack;
	uint32_t        tick;
	enum stat       stat;
};

typedef void (*taskloop_t)(void);

static struct task task_list[MAX_TASKS];
static uint32_t task_stacks[MAX_TASKS][STACK_SIZE];
static uint32_t current_task;
static uint32_t next_task;

void rtp_task_exit(void);


void rtp_idle_task(void)
{
	while (1)
		__WFI();
}

void rtp_pendsv_call(void)
{
	SCB_ICSR = ICSR_PENDSVSET;

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

int rtp_os_init(void)
{
	//TODO: Get the sysclk and adjust mdelay
	return 0;
}

int rtp_create_task(taskloop_t task)
{
	int free_slot = -1;
	int i;

	for (i = 0; i < MAX_TASKS; i++) {
		if (task_list[i].stat != RTP_STAT_UNUSED)
			continue;
		free_slot = i;
		break;
	}

	if (free_slot < 0)
		return -1;

	rtp_stack_init(free_slot, task);
	task_list[free_slot].tick = 0;
	task_list[free_slot].stat = RTP_STAT_READY;

	return 0;
}

void rtp_os_schedule(void)
{
	int i;
	uint32_t next = current_task;

	// simpl RR
	for (i = 0; i < MAX_TASKS; i++) {
		next ++;
		if (next > MAX_TASKS - 1)
			next = 0;

		if (task_list[next].stat != RTP_STAT_UNUSED &&
			task_list[next].tick == 0) {
			next_task = next;
			return;
		}
	}

	/* none running, wake idle */
	next_task = 0;
}

void rtp_delete_task(int tid)
{
	if (tid < 0 || tid > MAX_TASKS - 1)
		return;

	task_list[tid].stat = RTP_STAT_UNUSED;

	if (tid == current_task) {
		rtp_os_schedule();
		rtp_pendsv_call();
	}
}

void rtp_task_exit(void)
{
	rtp_delete_task(current_task);

	/* If PendSV was preempted by a high-prio interrupt,
	 * after the interrupt ends, pop {lr} returns to a
	 * task which has already exist
	 * it will cause os panic, so the nop loop is nescessry
	 */
	while (1) __NOP();
}

void rtp_mdelay(uint32_t delay)
{
	if (delay == 0)
		return;

	task_list[current_task].tick = delay;
	rtp_os_schedule();
	rtp_pendsv_call();
}

void rtp_tick_handler(void)
{
	int i;

	for (i = 0; i < MAX_TASKS; i++) {
		if (task_list[i].stat != RTP_STAT_UNUSED &&
			task_list[i].tick > 0) {

			task_list[i].tick --;
		}
	}
	rtp_os_schedule();
	rtp_pendsv_call();
}

void rtp_os_start(void)
{
	uint32_t *cstack;

	current_task = 0;
	next_task = 0;

	rtp_create_task(rtp_idle_task);
	cstack = task_list[current_task].stack;

	__asm volatile (
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
		:
		: "r" (cstack)
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
