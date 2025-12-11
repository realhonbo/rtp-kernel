/*
 * libopencm3 testing examples
 * I like this library further more than fucking HAL
 *
 * Copyright(C) Honbo
 * 2025-12
 */
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/cm3/dwt.h>
#include <libopencm3/cm3/scs.h>
#include <stdio.h>
#include <rtp-kernel.h>
#include <malloc.h>

extern void rtp_tick_handler(void);

int _write(int fd, char *ptr, int len)
{
	int i = 0;
	(void) fd;

	for (i = 0; i < len; i++) {
		usart_send_blocking(USART1, ptr[i]);
		if (ptr[i] == '\n') {
			usart_send_blocking(USART1, '\r');		
		}
	}
	return len;
}

void sys_tick_handler(void)
{
	rtp_tick_handler();
}

void hw_setup(void)
{
	rcc_clock_setup_pll(&rcc_hse_configs[RCC_CLOCK_HSE8_72MHZ]);

	/* systick */
	systick_set_clocksource(STK_CSR_CLKSOURCE_AHB_DIV8);
	systick_set_reload(8999);
	systick_clear();
	systick_interrupt_enable();
	systick_counter_enable();

    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_USART1);

	/* heart-beat led */
    gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_2_MHZ,
		GPIO_CNF_OUTPUT_PUSHPULL, GPIO2);

	/* console */
    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
		GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART1_TX);
    gpio_set_mode(GPIOA, GPIO_MODE_INPUT,
		GPIO_CNF_INPUT_FLOAT, GPIO_USART1_RX);
    usart_set_baudrate(USART1, 115200);
    usart_set_databits(USART1, 8);
    usart_set_stopbits(USART1, USART_STOPBITS_1);
    usart_set_parity(USART1, USART_PARITY_NONE);
    usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);
    usart_set_mode(USART1, USART_MODE_TX_RX);
    usart_enable(USART1);
}

uint32_t t_start, t_end;

void dwt_init(void)
{
    SCS_DEMCR |= SCS_DEMCR_TRCENA;
    DWT_CYCCNT = 0;
    DWT_CTRL |= DWT_CTRL_CYCCNTENA;
}

void heart_beat_task(void)
{
	printf("INFO: Heart-beat led started\n");

	while (1) {
		/* heart beat rhythm */
        gpio_set(GPIOB, GPIO2);
        rtp_msleep(100);
        gpio_clear(GPIOB, GPIO2);
        rtp_msleep(100);
        gpio_set(GPIOB, GPIO2);
        rtp_msleep(100);
        gpio_clear(GPIOB, GPIO2);

        /* diastole */
        rtp_msleep(700);
	}
}

static unsigned long sync_num;
static spinlock_t *uspin;

void task_loop(void)
{
	printf("INFO: Task loop has been scheduled\n");

	while(1) {
		rtp_spin_lock(uspin);
		printf("Task(tid: %d) => %lu\n", rtp_current_get(), sync_num ++);
		rtp_spin_unlock(uspin);
		rtp_msleep(100);
	}
}

void task_loop1(void)
{
	while(1) {
		rtp_spin_lock(uspin);
		printf("Task(tid: %d) => %lu\n", rtp_current_get(), sync_num ++);
		rtp_spin_unlock(uspin);
		t_start = DWT_CYCCNT;
		rtp_msleep(100);
	}
}

void task_loop2(void)
{
	t_end = DWT_CYCCNT;
	printf("sched period: %ld\n", t_end - t_start);

	while(1) {
		rtp_spin_lock(uspin);
		printf("Task(tid: %d) => %lu\n", rtp_current_get(), sync_num ++);
		rtp_spin_unlock(uspin);
		rtp_msleep(100);
	}
}

void task_loop3(void)
{
	while(1) {
		rtp_spin_lock(uspin);
		printf("Task(tid: %d) => %lu\n", rtp_current_get(), sync_num ++);
		rtp_spin_unlock(uspin);
		rtp_msleep(100);
	}
}

void task_loop4(void)
{
	while(1) {
		rtp_spin_lock(uspin);
		printf("Task(tid: %d) => %lu\n", rtp_current_get(), sync_num ++);
		rtp_spin_unlock(uspin);
		rtp_msleep(100);
	}
}

void task_loop5(void)
{
	while(1) {
		rtp_spin_lock(uspin);
		printf("Task(tid: %d) => %lu\n", rtp_current_get(), sync_num ++);
		rtp_spin_unlock(uspin);
		rtp_msleep(50);
	}
}

char stack_heart_beat[256];
char stack_loop[512];
char stack_loop1[512];
char stack_loop2[512];
char stack_loop3[512];
char stack_loop4[512];
char stack_loop5[512];

int main(void)
{
	hw_setup();
	dwt_init();
    rtp_os_init();

	uspin = malloc(sizeof(spinlock_t));
	if (!uspin) {
		printf("Error, no memory enough\n");
		return -1;
	}
	rtp_spin_init(uspin);

	rtp_create_task(heart_beat_task, stack_heart_beat, 256, 1);
	rtp_create_task(task_loop,  stack_loop,  512, 50);
	rtp_create_task(task_loop1, stack_loop1, 512, 0);
	rtp_create_task(task_loop2, stack_loop2, 512, 0);
	rtp_create_task(task_loop3, stack_loop3, 512, 0);
	rtp_create_task(task_loop4, stack_loop4, 512, 0);
	rtp_create_task(task_loop5, stack_loop5, 512, 0);

	rtp_os_start();

	printf("Notify: RTP terminated unexpectly\n");
	return 0;
}
