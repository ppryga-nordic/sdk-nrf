/*
 * Copyright (c) Nordic Semiconductor ASA. All Rights Reserved.
 *
 * The information contained herein is confidential property of Nordic Semiconductor ASA.
 * The use, copying, transfer or disclosure of such information is prohibited except by
 * express written agreement with Nordic Semiconductor ASA.
 */

#include <zephyr/types.h>
#include <stddef.h>
#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#include <nrf.h>
#include <hal/nrf_clock.h>
#include <hal/nrf_timer.h>
#include <hal/nrf_rtc.h>
#include <hal/nrf_gpio.h>
#include <hal/nrf_gpiote.h>
#include <hal/nrf_egu.h>
#include <helpers/nrfx_gppi.h>
#include <assert.h>

#define NRF_PLATFORM_LUMOS

#pragma GCC push_options
#pragma GCC optimize("-O0")

#if !defined(NRF5340_XXAA_APPLICATION) && !defined(CONFIG_SOC_SERIES_BSIM_NRFXX)
#include <nrfx_temp.h>
#endif

#if defined(NRF_PLATFORM_LUMOS)
#define CLOCK NRF_CLOCK
#define RTC NRF_RTC10

#define REF_TIMER NRF_TIMER22
#define GPIOTE NRF_GPIOTE20
#define GPIO NRF_P1
#define EGU NRF_EGU10

#define CLOCK_IRQ CLOCK_POWER_IRQn
#define EGU_IRQ EGU10_IRQn
#elif defined(NRF5340_XXAA_NETWORK)
#define CLOCK NRF_CLOCK_NS
#define RTC NRF_RTC0_NS

#define REF_TIMER NRF_TIMER2_NS

#define GPIOTE NRF_GPIOTE_NS
#define GPIO NRF_P0_NS
#define EGU NRF_EGU0_NS

#define CLOCK_IRQ CLOCK_POWER_IRQn
#define EGU_IRQ EGU0_IRQn
#else
#define CLOCK NRF_CLOCK
#define RTC NRF_RTC0

#define REF_TIMER NRF_TIMER2

#define GPIOTE NRF_GPIOTE
#define GPIO NRF_P0
#define EGU NRF_EGU0

#define CLOCK_IRQ POWER_CLOCK_IRQn
#define EGU_IRQ SWI0_EGU0_IRQn
#endif

#if defined(NRF_PLATFORM_LUMOS)
#define TIMER_IRQ TIMER22_IRQn
#define RTC_IRQ RTC10_IRQn
#else
#define CAL_TIMER_IRQ TIMER1_IRQn
#define RTC_IRQ RTC0_IRQn
#endif

#define RTC_CC 1
#define RTC_EVENT_COMPARE NRFX_CONCAT_2(NRF_RTC_EVENT_COMPARE_, RTC_CC)
#define RTC_EVENTSET_COMPARE NRFX_CONCAT_3(RTC_EVTENSET_COMPARE, RTC_CC, _Msk)
#define RTC_INTENSET_COMPARE NRFX_CONCAT_3(RTC_INTENSET_COMPARE, RTC_CC, _Msk)

#define REF_TIMER_CC 0
#define REF_TIMER_EVENT_COMPARE NRFX_CONCAT_2(NRF_TIMER_EVENT_COMPARE, REF_TIMER_CC)
#define REF_TIMER_TASK_CAPTURE NRFX_CONCAT_2(NRF_TIMER_TASK_CAPTURE, REF_TIMER_CC)

#define RTC_EVENT_TICK_PIN 7
#define RTC_CC_READ_TOGGLE_PIN 8
#define RTC_ISR_TICK_TOGGLE_PIN 9
#define RTC_ISR_EVENT_CC_TOGGLE_PIN 10
#define RTC_EVENT_CC_PIN 11
#define RTC_EVENT_CLEAR_PIN 1


#define RTC_RELOAD_VALUE 32768

#define RTC_EVT_INTERVAL_US 1000000

#if defined(NRF_PLATFORM_LUMOS)
#define GPIO_PIN_CNF_DEFAULT \
  (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos) | (GPIO_PIN_CNF_DRIVE0_S0 << GPIO_PIN_CNF_DRIVE0_Pos) | (GPIO_PIN_CNF_DRIVE1_S1 << GPIO_PIN_CNF_DRIVE1_Pos) | (GPIO_PIN_CNF_PULL_Pulldown << GPIO_PIN_CNF_PULL_Pos) | (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos)
#else
#define GPIO_PIN_CNF_DEFAULT \
  (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos) | (GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos) | (GPIO_PIN_CNF_PULL_Pulldown << GPIO_PIN_CNF_PULL_Pos) | (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos)
#endif /* NRF_PLATFORM_LUMOS */

static uint8_t rtc_cc_ppi_ch;
static uint8_t rtc_clear_ppi_ch;

static uint32_t rtc_time;
static uint32_t ref_time_cc0;
static uint32_t ref_time_cc1;

static inline void debug_pin_set(uint32_t pin)
{
  GPIO->OUTSET = (1 << pin);
}

static inline void debug_pin_clear(uint32_t pin)
{
  GPIO->OUTCLR = (1 << pin);
}

ISR_DIRECT_DECLARE(rtc_isr)
{
	if (nrf_rtc_event_check(RTC, RTC_EVENT_COMPARE)) {
		debug_pin_set(RTC_ISR_EVENT_CC_TOGGLE_PIN);

		nrf_rtc_event_clear(RTC, RTC_EVENT_COMPARE);

		REF_TIMER->TASKS_CAPTURE[1] = 1;
		RTC->TASKS_CAPTURE[1] = 1;

		ref_time_cc0 = REF_TIMER->CC[0];
		ref_time_cc1 = REF_TIMER->CC[1];
		rtc_time = RTC->CC[1];

		int32_t drift;
		if (ref_time_cc0 > RTC_EVT_INTERVAL_US) {
			drift = ref_time_cc0 - RTC_EVT_INTERVAL_US;
		} else {
			drift = RTC_EVT_INTERVAL_US - ref_time_cc0;
		}

		printk("ref[0]: %d, rtc: %d diff: %d, ref[1] %d\n", ref_time_cc0, rtc_time, drift, ref_time_cc1);

		//EGU->TASKS_TRIGGER[1]=1;
		debug_pin_clear(RTC_ISR_EVENT_CC_TOGGLE_PIN);
	} else {
		//debug_pin_set(RTC_ISR_TICK_TOGGLE_PIN);
		nrf_rtc_event_clear(RTC, NRF_RTC_EVENT_TICK);
		//debug_pin_clear(RTC_ISR_TICK_TOGGLE_PIN);
	}
	return 1;
}

ISR_DIRECT_DECLARE(egu_isr)
{
	if (EGU->EVENTS_TRIGGERED[0]) {
		nrf_egu_event_clear(EGU, NRF_EGU_EVENT_TRIGGERED0);
	}
	return 1;
}

// ISR_DIRECT_DECLARE(timer_isr)
// {
// 	printk("timer1\n");
// 	if (REF_TIMER->EVENTS_COMPARE[1]) {
// 		nrf_timer_event_clear(REF_TIMER, NRF_TIMER_EVENT_COMPARE1);
// 		REF_TIMER->TASKS_CLEAR =1;
// 		printk("timer\n");
// 	}
// 	return 1;
// }

void reference_timer_init(void)
{
	nrf_timer_task_trigger(REF_TIMER, NRF_TIMER_TASK_STOP);
	nrf_timer_task_trigger(REF_TIMER, NRF_TIMER_TASK_CLEAR);
	nrf_timer_mode_set(REF_TIMER, NRF_TIMER_MODE_TIMER);
	nrf_timer_bit_width_set(REF_TIMER, NRF_TIMER_BIT_WIDTH_32);
	nrf_timer_prescaler_set(
		REF_TIMER,
		NRF_TIMER_PRESCALER_CALCULATE(NRF_TIMER_BASE_FREQUENCY_GET(REF_TIMER), 1000000));
	// nrf_timer_shorts_enable(REF_TIMER, NRF_TIMER_SHORT_COMPARE1_CLEAR_MASK);

	// nrf_timer_cc_set(REF_TIMER, NRF_TIMER_CC_CHANNEL1, 1000000);

	// /* Enable IRQ for compare event */
	// nrf_timer_int_enable(REF_TIMER,  nrf_timer_compare_int_get(1));

	// IRQ_DIRECT_CONNECT(TIMER_IRQ, 0, timer_isr, 0);
	// irq_enable(TIMER_IRQ);
}

void clock_init(void)
{
	/* Initialize HFCLK to use in reference timer (REF_TIMER).*/
	nrf_clock_task_trigger(CLOCK, NRF_CLOCK_TASK_HFCLKSTART);
	while (!nrf_clock_hf_is_running(CLOCK, NRF_CLOCK_HFCLK_HIGH_ACCURACY)) {
		void arch_busy_wait(uint32_t);
		arch_busy_wait(10);
	}

	/* RTC uses LFCLK and its source is set as RC osc.*/
	nrf_clock_task_trigger(CLOCK, NRF_CLOCK_TASK_LFCLKSTOP);
	if (IS_ENABLED(CONFIG_CLOCK_CONTROL_NRF_K32SRC_RC)) {
		nrf_clock_lf_src_set(CLOCK, NRF_CLOCK_LFCLK_RC);
	} else {
		nrf_clock_lf_src_set(CLOCK, NRF_CLOCK_LFCLK_XTAL);
	}
	nrf_clock_event_clear(CLOCK, NRF_CLOCK_EVENT_LFCLKSTARTED);
	nrf_clock_task_trigger(CLOCK, NRF_CLOCK_TASK_LFCLKSTART);
	while (!nrf_clock_lf_is_running(CLOCK)) {
		void arch_busy_wait(uint32_t);
		arch_busy_wait(10);
	}
}

void rtc_init(void)
{
	nrf_rtc_task_trigger(RTC, NRF_RTC_TASK_STOP);
	nrf_rtc_task_trigger(RTC, NRF_RTC_TASK_CLEAR);
	/* Set to max freq 32768Hz*/
	nrf_rtc_prescaler_set(RTC, 0);

	nrf_rtc_cc_set(RTC, RTC_CC, RTC_RELOAD_VALUE);

	/* Enable IRQ for compare event */
	nrf_rtc_event_enable(RTC, RTC_EVENTSET_COMPARE);
	nrf_rtc_int_enable(RTC, RTC_INTENSET_COMPARE);

	/* Enable IRQ for tick event */
	nrf_rtc_event_enable(RTC, RTC_EVTENSET_TICK_Msk);
	nrf_rtc_int_enable(RTC, RTC_INTENSET_TICK_Msk);

	nrfx_err_t err = nrfx_gppi_channel_alloc(&rtc_clear_ppi_ch);
	__ASSERT_NO_MSG(err == NRFX_SUCCESS);

	nrfx_gppi_channel_endpoints_setup(rtc_clear_ppi_ch,
					  nrf_egu_event_address_get(EGU, NRF_EGU_EVENT_TRIGGERED1),
					  nrf_timer_task_address_get(REF_TIMER, NRF_TIMER_TASK_CLEAR));

	nrfx_gppi_fork_endpoint_setup(rtc_clear_ppi_ch, nrf_rtc_task_address_get(RTC, NRF_RTC_TASK_CLEAR));
	nrfx_gppi_channels_enable(BIT(rtc_clear_ppi_ch));

	err = nrfx_gppi_channel_alloc(&rtc_cc_ppi_ch);
	__ASSERT_NO_MSG(err == NRFX_SUCCESS);

	nrfx_gppi_channel_endpoints_setup(
		rtc_cc_ppi_ch, nrf_rtc_event_address_get(RTC, RTC_EVENT_COMPARE),
		nrf_timer_task_address_get(REF_TIMER, REF_TIMER_TASK_CAPTURE));
	nrfx_gppi_channels_enable(BIT(rtc_cc_ppi_ch));

	/* Enable IRQ handler */
	IRQ_DIRECT_CONNECT(RTC_IRQ, 0, rtc_isr, 0);
	irq_enable(RTC_IRQ);
}

void conf_gpio(uint16_t pin) {
	GPIO->PIN_CNF[pin] = GPIO_PIN_CNF_DEFAULT | (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos);

	GPIO->DIRCLR = (1 << pin);
	GPIO->DIRSET = (1 << pin);
	GPIO->OUTCLR = (1 << pin);

	/* Test toggle of the pin */
	GPIO->OUTSET = (1 << pin);
	k_msleep(1);
	GPIO->OUTCLR = (1 << pin);
}

void pin_debug_init(void)
{
	conf_gpio(RTC_ISR_TICK_TOGGLE_PIN);
	conf_gpio(RTC_EVENT_TICK_PIN);
	conf_gpio(RTC_CC_READ_TOGGLE_PIN);
	conf_gpio(RTC_ISR_EVENT_CC_TOGGLE_PIN);
	conf_gpio(RTC_EVENT_CC_PIN);
	conf_gpio(RTC_EVENT_CLEAR_PIN);

	nrf_gpiote_task_configure(GPIOTE, 0, NRF_GPIO_PIN_MAP(1, RTC_EVENT_TICK_PIN), NRF_GPIOTE_POLARITY_TOGGLE,
				  NRF_GPIOTE_INITIAL_VALUE_LOW);
	nrf_gpiote_task_enable(GPIOTE, 0);

	nrf_gpiote_task_configure(GPIOTE, 1, NRF_GPIO_PIN_MAP(1, RTC_EVENT_CC_PIN), NRF_GPIOTE_POLARITY_TOGGLE,
				  NRF_GPIOTE_INITIAL_VALUE_LOW);
	nrf_gpiote_task_enable(GPIOTE, 1);

	nrf_gpiote_task_configure(GPIOTE, 2, NRF_GPIO_PIN_MAP(1, RTC_ISR_TICK_TOGGLE_PIN), NRF_GPIOTE_POLARITY_TOGGLE,
				  NRF_GPIOTE_INITIAL_VALUE_LOW);
	nrf_gpiote_task_enable(GPIOTE, 2);

	uint8_t rtc_ppi;
	nrfx_err_t err = nrfx_gppi_channel_alloc(&rtc_ppi);
	__ASSERT_NO_MSG(err == NRFX_SUCCESS);

	/* Connect RTC->TASKS_TICK to GPIOTE->TASKS_OUT[0] to toggle configured pin */
	nrfx_gppi_channel_endpoints_setup(rtc_ppi,
					  nrf_rtc_event_address_get(RTC, NRF_RTC_EVENT_TICK),
					  nrf_gpiote_task_address_get(GPIOTE, NRF_GPIOTE_TASK_OUT_0));

	nrfx_gppi_channel_endpoints_setup(rtc_ppi,
					  nrf_timer_event_address_get(REF_TIMER, NRF_TIMER_EVENT_COMPARE1),
					  nrf_gpiote_task_address_get(GPIOTE, NRF_GPIOTE_TASK_OUT_0));

	nrfx_gppi_channels_enable(BIT(rtc_ppi));

	nrf_gpiote_subscribe_set(GPIOTE, NRF_GPIOTE_TASK_OUT_1, rtc_cc_ppi_ch);
	nrf_gpiote_subscribe_set(GPIOTE, NRF_GPIOTE_TASK_OUT_2, rtc_clear_ppi_ch);
}

void test_rtc()
{
	/* Initialize HFCLK and LFCLK.*/
	clock_init();

	/* Initialize RTC to create compare event on each seconds.*/
	rtc_init();

	/* Initialize a reference timer(REF_TIMER) to compare with RTC timing.*/
	reference_timer_init();

	/* Initialize debug pin toggle to check timings for RTC compare events.*/
	pin_debug_init();

	debug_pin_set(RTC_CC_READ_TOGGLE_PIN);
	k_msleep(2);
	debug_pin_clear(RTC_CC_READ_TOGGLE_PIN);

	/* Start RTC and timers.*/
	debug_pin_set(RTC_CC_READ_TOGGLE_PIN);
	nrf_rtc_task_trigger(RTC, NRF_RTC_TASK_START);
	debug_pin_clear(RTC_CC_READ_TOGGLE_PIN);
	nrf_timer_task_trigger(REF_TIMER, NRF_TIMER_TASK_START);

	uint32_t counter;
	uint32_t ref_time_local;

	for(int idx = 0; idx < 10; idx++) {
		debug_pin_set(RTC_CC_READ_TOGGLE_PIN);
		REF_TIMER->TASKS_CAPTURE[1] = 1;
		ref_time_local = REF_TIMER->CC[1];
		RTC->TASKS_CAPTURE[2] = 1;
		//counter = RTC->COUNTER;
		rtc_time = RTC->CC[2];
		debug_pin_clear(RTC_CC_READ_TOGGLE_PIN);

		printk("ref time aft. start[%d]: %d, rtc: %d, coutner %d\n", idx, ref_time_local, rtc_time, counter);
	}

	debug_pin_set(RTC_CC_READ_TOGGLE_PIN);
	k_msleep(1);
	debug_pin_clear(RTC_CC_READ_TOGGLE_PIN);
	EGU->TASKS_TRIGGER[1]=1;

	for(int idx = 0; idx < 10; idx++) {
		debug_pin_set(RTC_CC_READ_TOGGLE_PIN);
		REF_TIMER->TASKS_CAPTURE[1] = 1;
		ref_time_local = REF_TIMER->CC[1];
		RTC->TASKS_CAPTURE[2] = 1;
		//counter = RTC->COUNTER;
		rtc_time = RTC->CC[2];
		debug_pin_clear(RTC_CC_READ_TOGGLE_PIN);

		printk("ref time aft. clear[%d]: %d, rtc: %d, coutner %d\n", idx, ref_time_local, rtc_time, counter);
	}

	REF_TIMER->TASKS_CAPTURE[1] = 1;
	ref_time_local = REF_TIMER->CC[1];
	printk("end ref time: %d\n", ref_time_local);
}

int main(void)
{
	test_rtc();

	return 0;
}

#pragma GCC pop_options
