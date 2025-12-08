#ifndef RTP_KERNEL
#define RTP_KERNEL

#include <stdint.h>
#include <cmsis_gcc.h>

//#define RTP_DEBUG
/* MAX_TASKS can only be the power of 2
 * because it will be used in scheduler as mask code
 * for extremely preformance
 *
 * it include idle task
 */
#define RTP_MAXTASK_SHIFT	3
#define MAX_TASKS			(1 << RTP_MAXTASK_SHIFT)
#define STACK_SIZE			256

typedef void (*taskloop_t)(void);
typedef int rtp_tid_t;

int rtp_os_init(void);
int rtp_create_task(taskloop_t task);
void rtp_os_start(void);

void rtp_delete_task(int tid);
void rtp_task_exit(void);

int rtp_current_get(void);
void rtp_yield(void);
void rtp_msleep(uint32_t delay);


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
#define REREAD(x) (* (volatile typeof(x) *)&(x))
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
		rtp_os_schedule();
		rtp_pendsv_call();
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
