#ifndef RTP_KERNEL
#define RTP_KERNEL

#include <stdint.h>

//#define RTP_DEBUG
#define RTP_TASKS       15

typedef int rtp_tid_t;

int rtp_os_init(void);
int rtp_create_task(void *entry,
					void *stack_addr, uint32_t stack_size,
					uint32_t slice);
void rtp_os_start(void);

void rtp_delete_task(int tid);
void rtp_task_exit(void);

int rtp_current_get(void);
void rtp_yield(void);
void rtp_msleep(uint32_t delay);

#define __CLI()		__asm volatile ("cpsid i" ::: "memory")
#define __STI()		__asm volatile ("cpsie i" ::: "memory")
#define __WFI()		__asm volatile ("wfi" ::: "memory")
#define __NOP()		__asm volatile ("nop")
#define __DMB()		__asm volatile ("dmb 0xf" ::: "memory")
#define __DSB()		__asm volatile ("dsb 0xf" ::: "memory")
#define __ISB()		__asm volatile ("isb 0xf" ::: "memory")

typedef struct {
union {
        struct {
            uint16_t owner;
            uint16_t next;
        } tickets;
        uint32_t lock;
};
} spinlock_t;

static inline void rtp_spin_init(spinlock_t *spin)
{
	__asm ("":::"memory");
	spin->lock = 0;
	spin->tickets.next = 1;
	__asm ("":::"memory");
}


static inline void rtp_spin_lock(spinlock_t *spin)
{
#define REREAD(x) (* (volatile __typeof__(x) *)&(x))
	extern void rtp_os_schedule(void);
	extern void rtp_pendsv_call(void);

	uint16_t ticket, ret;

	/* fetch owner and add it atomic */
	__asm volatile (
	"1: ldrexh	%0, [%2]	\n"
	"	add		%0, %0, #1	\n"
	"	strexh	%1, %0, [%2]\n"
	"	teq		%1, #0		\n"
	"	bne		1b			\n"
	: "=&r"(ticket), "=&r"(ret)
	: "r"(&spin->tickets.owner)
	: "cc", "memory"
	);

	while (ticket != spin->tickets.next) {
		__CLI();
		rtp_os_schedule();
		rtp_pendsv_call();
		__STI();

		__WFI();
		spin->tickets.next = REREAD(spin->tickets.next);
	}
	__DMB();
}

static inline void rtp_spin_unlock(spinlock_t *spin)
{
	__DMB();
	spin->tickets.next ++;
	__DSB();
}

static inline int spin_is_locked(const spinlock_t *spin)
{
	spinlock_t _spin = REREAD(*spin);
	return _spin.tickets.owner == _spin.tickets.next;
#undef REREAD
}


#endif
