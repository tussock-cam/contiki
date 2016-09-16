#define OSLMIC_FROM_CONTIKI

#include "lmic/hal.h"
#include "rfm9x-arch.h"
#include "lmic/radio.h"
#include "oslmic.h"
#include "lmic/lmic.h"
#include "dev/spi.h"
#include "contiki.h"
#include "lib/random.h"
#include <stdint.h>
#include <stdio.h>

DECLARE_LMIC;

#define PRINTF(...) \
	do { \
		printf("rfm9x: "); \
		printf(__VA_ARGS__); \
		printf("\n"); \
	} while (0)

#define INFO(...) PRINTF(__VA_ARGS__)

#define RFM_ASSERT(cond) do { \
	if(!(cond)) { \
		PRINTF("Assertion failed at line: %d", __LINE__); \
		hal_failed(); \
	} \
} while(0)

static volatile uint8_t g_irq_disable_counter;

PROCESS(lora_rfm9x_process, "LoRa stack");

/* -------------------------------------------------- */

static void reset_job(osjob_t* job)
{
	job->rtimer_deadline = 0;
	job->func = NULL;
	job->pending = false;
	job->run_immediately = false;
}

static void finish_job(osjob_t* job)
{
	INFO("Finishing job...");

	hal_disableIRQs();

	osjobcb_t callback = NULL;

	/*
	 * We can't cancel rtimer jobs, so if one is cancelled then
	 * this callback is still called.. So we have to check if the job
	 * is still pending, if it has a timer, and its expiration time.
	 */
	if (job->pending) {
		 if (!job->run_immediately) {
			rtimer_clock_t now = RTIMER_NOW();
			bool expired = RTIMER_CLOCK_LT(now, job->rtimer_deadline) || job->rtimer_deadline == RTIMER_NOW();
			if (expired) {
				callback = job->func;
			}
		} else {
			callback = job->func;
		}

		if (callback) {
			/*
			 * Reset job before the callback because the callback doesn't
			 * care (LMIC.osjob is global) and this way we don't lose
			 * jobs scheduled in a callback..
			 */
			reset_job(job);
		}
	}

	hal_enableIRQs();

	if (callback) {
		INFO("Notified callback!");
		callback(job);
	}
}

static void pollhandler(void)
{
	finish_job(&LMIC.osjob);
}

PROCESS_THREAD(lora_rfm9x_process, ev, data)
{
  PROCESS_POLLHANDLER(pollhandler());
  PROCESS_BEGIN();

  INFO("Initialize rfm9x...");

  reset_job(&LMIC.osjob);

  hal_init();
  lmic_radio_init();
  LMIC_init();
  LMIC_reset();

  INFO("LoRa rfm9x starting...");

  LMIC_setLinkCheckMode(0);
  LMIC_setDrTxpow(DR_SF12, 20);

  INFO("Wait for polls...");

  PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_EXIT);
  PROCESS_END();
}


ostime_t os_getTime ()
{
    return hal_ticks();
}

u2_t os_getRndU2()
{
	return random_rand();
}

void onEvent(ev_t e)
{
	INFO("Got event %d", e);
}

static void job_timer_callback(struct rtimer* t, void* ptr)
{
	finish_job((osjob_t*)ptr);
}

static void schedule_job(osjob_t* job, rtimer_clock_t deadline, bool run_immediately, osjobcb_t cb)
{
	hal_disableIRQs();

	/* Simplifying assumption is that lmic/radio use a single job (which they do) */
	if (job != &LMIC.osjob) {
		RFM_ASSERT("Expected only a single job!");
	}

	if (job->pending) {
		RFM_ASSERT("New event scheduled before old one was finished!");
	}

	job->run_immediately = run_immediately;
	job->rtimer_deadline = deadline;
	job->func = cb;
	job->pending = true;

	if (!run_immediately) {
		rtimer_set(&job->timer, deadline, 0, job_timer_callback, (void*)job);
	} else {
		process_poll(&lora_rfm9x_process);
	}

	hal_enableIRQs();
}

void os_setCallback (osjob_t* job, osjobcb_t cb)
{
	INFO("Schedule immediate callback!!");
	schedule_job(job, 0, true, cb);
}

void os_setTimedCallback (osjob_t* job, ostime_t time, osjobcb_t cb)
{
	INFO("Schedule timed callback!!");
	rtimer_clock_t real_time = time * OSTICKS_PER_SEC_DIVIDER;
	schedule_job(job, real_time, false, cb);
}

void os_clearCallback (osjob_t* job)
{
    hal_disableIRQs();
    job->pending = false;
    hal_enableIRQs();
}

void rfm9x_irq_handler(uint8_t dio)
{
	radio_irq_handler(dio);
}

void rfm9x_init(void)
{
	g_irq_disable_counter = 0;
	process_start(&lora_rfm9x_process, NULL);
}

/*
 * initialize hardware (IO, SPI, TIMER, IRQ).
 */
void hal_init (void)
{
	rfm9x_io_init();
	rfm9x_spi_init();
	rfm9x_interrupt_init();
}

/*
 * drive radio NSS pin (0=low, 1=high).
 */
void hal_pin_nss (u1_t val)
{
	rfm9x_set_nss(val);
}

/*
 * drive radio RX/TX pins (0=rx, 1=tx).
 */
void hal_pin_rxtx (u1_t val)
{
	u1_t rx = val == 0;
	u1_t tx = val == 1;

	rfm9x_set_rxtx(rx, tx);
}

/*
 * control radio RST pin (0=low, 1=high, 2=floating)
 */
void hal_pin_rst (u1_t val)
{
	if (val != 2) {
		rfm9x_set_rst(val);
	} else {
		rfm9x_disable_rst();
	}
}

/*
 * perform 8-bit SPI transaction with radio.
 *   - write given byte 'outval'
 *   - read byte and return value
 */
u1_t hal_spi (u1_t outval)
{
	return rfm9x_spi_read_write(outval);
}

/*
 * disable all CPU interrupts.
 *   - might be invoked nested
 *   - will be followed by matching call to hal_enableIRQs()
 */
void hal_disableIRQs (void)
{
	rfm9x_disable_all_irq();
	++g_irq_disable_counter;
}

/*
 * enable CPU interrupts.
 */
void hal_enableIRQs (void)
{
	// At this point interrupts are either enabled and thus
	// g_irq_disable_counter is zero (and we can therefore be
	// preempted) or it is greater than 1 and interrupts are
	// disabled (and we can't be preempted).
	rfm9x_disable_all_irq();

	// Now we can check if we actually want to enable them
	if (g_irq_disable_counter == 0 || --g_irq_disable_counter == 0) {
		rfm9x_enable_all_irq();
	}
}

/*
 * put system and CPU in low-power mode, sleep until interrupt.
 */
void hal_sleep (void)
{

}

inline uint32_t deltaticks(u4_t time)
{
	int32_t diff = time - hal_ticks();
	if (diff <= 0) {
		return 0;
	} else if (diff > 0xffff) {
		return 0xffff;
	} else {
		return (uint16_t)diff;
	}
}

#define STATIC_ASSERT(COND,MSG) typedef char static_assertion_##MSG[(COND)?1:-1]

/*
 * Will need to improve timer logic if this isn't the case
 */
STATIC_ASSERT(sizeof(rtimer_clock_t) == 4, LMIC_only_tested_with_32bit_rtimers);

/*
 * return 32-bit system time in ticks.
 */
u4_t hal_ticks (void)
{
	return RTIMER_NOW() / OSTICKS_PER_SEC_DIVIDER;
}

/*
 * busy-wait until specified timestamp (in ticks) is reached.
 */
void hal_waitUntil (u4_t time)
{
	while (deltaticks(time) != 0) ;
}

/*
 * check and rewind timer for target time.
 *   - return 1 if target time is close
 *   - otherwise rewind timer for target time or full period and return 0
 * TODO: implement if this ever starts being used (leave undefined for linker error)
 */
u1_t hal_checkTimer (u4_t targettime);

/*
 * perform fatal failure action.
 *   - called by assertions
 *   - action could be HALT or reboot
 */
void hal_failed (void)
{
	printf("LMIC: FATAL ERROR!\n");
	while (1) ;

}

