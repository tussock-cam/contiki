#include "contiki.h"
#include "ti-lib.h"
#include "ti-lib-rom.h"
#include "dev/aux-ctrl.h"
#include "driverlib/aux_tdc.h"
#include "mb7369-sensor.h"
#include "inc/hw_aux_aiodio.h"

#define PRINTF(...) \
	do { \
		printf("mb7369: "); \
		printf(__VA_ARGS__); \
		printf("\n"); \
	} while (0)

#define INFO(...) PRINTF(__VA_ARGS__)
#define WARN(...) PRINTF(__VA_ARGS__)


#define MB_CONTROL_IOID					(IOID_22)

/*
 * Note: Mapping from IOID to AUXIO is hardcoded, each IOID goes
 * to a specific AUXIO. Refer to Table 11-2 in cc1310 user guide.
 *
 * IF YOU CHANGE THE IOID THEN YOU MUST CHANGE ALL SUBSEQUENT DEFINES!
 */
#define MB_DATA_IOID					(IOID_23)
#define MB_DATA_AUXTDC_START			(AUXTDC_START_AUXIO7)
#define MB_DATA_AUXTDC_STOP				(AUXTDC_STOP_AUXIO7)
#define MB_AUX_TDC_BASE					(AUX_TDC_BASE)


#define BUSYWAIT_UNTIL(cond, max_time) \
  do { \
    rtimer_clock_t t0; \
    t0 = RTIMER_NOW(); \
    while(!(cond) && RTIMER_CLOCK_LT(RTIMER_NOW(), t0 + (max_time))) {} \
  } while(0)


static aux_consumer_module_t g_aux_consumer;
static int enabled = 0;

/*---------------------------------------------------------------------------*/
static void hw_init(void)
{
	INFO("Initializing...");


	INFO("Configure output...");

	ti_lib_ioc_pin_type_gpio_output(MB_CONTROL_IOID);
	ti_lib_ioc_io_port_pull_set(MB_CONTROL_IOID, IOC_IOPULL_DOWN);
	ti_lib_gpio_clear_dio(MB_CONTROL_IOID);

	// todo: check pulldowns work with AUX
	INFO("Set IOIC -> AUX");
	//ti_lib_ioc_pin_type_gpio_input(MB_DATA_IOID);
	//ti_lib_ioc_io_port_pull_set(MB_DATA_IOID, IOC_IOPULL_DOWN);
	//ti_lib_ioc_pin_type_aux(MB_DATA_IOID);

	INFO("hw_init done");
}

static void print_tdc_state(void)
{
	uint32_t state = AUXTDCStatusGet(MB_AUX_TDC_BASE);

	switch (state) {
	case AUXTDC_WAIT_START:
		INFO("TDC_STATE: AUXTDC_WAIT_START");
		break;
	case AUXTDC_WAIT_START_CNTEN:
		INFO("TDC_STATE: AUXTDC_WAIT_START_CNTEN");
		break;
	case AUXTDC_IDLE:
		INFO("TDC_STATE: AUXTDC_IDLE");
		break;
	case AUXTDC_CLRCNT:
		INFO("TDC_STATE: AUXTDC_CLRCNT");
		break;
	case AUXTDC_WAIT_STOP:
		INFO("TDC_STATE: AUXTDC_WAIT_STOP");
		break;
	case AUXTDC_WAIT_STOP_CNTDOWN:
		INFO("TDC_STATE: AUXTDC_WAIT_STOP_CNTDOWN");
		break;
	case AUXTDC_GETRESULTS:
		INFO("TDC_STATE: AUXTDC_GETRESULTS");
		break;
	case AUXTDC_POR:
		INFO("TDC_STATE: AUXTDC_POR");
		break;
	case AUXTDC_WAIT_CLRCNT_DONE:
		INFO("TDC_STATE: AUXTDC_WAIT_CLRCNT_DONE");
		break;
	case AUXTDC_START_FALL:
		INFO("TDC_STATE: AUXTDC_START_FALL");
		break;
	case AUXTDC_FORCE_STOP:
		INFO("TDC_STATE: AUXTDC_FORCE_STOP");
		break;
	default:
		INFO("TDC_STATE: %08X -> UNKNOWN", state);
		break;
	}

}

static uint16_t measure_distance_with_aux(void)
{
	int interrupts_disabled = 0;

	INFO("Check current TDC state...");
	print_tdc_state();

	if (!AUXTDCIdle(MB_AUX_TDC_BASE)) {
		WARN("TDC not idle, aborting");
		print_tdc_state();
		return 0;
	}

//	INFO("initial wait");
//	ti_lib_gpio_clear_dio(MB_CONTROL_IOID);
//
//	/*
//	 * Takes ~160 ms to boot and outputs serial data during this time. We don't
//	 * care, so just ingore it..
//	 */
//	/* todo: this probably isn't necessary at the moment because we don't ever power it off. */
//	BUSYWAIT_UNTIL(0, RTIMER_SECOND / 5);
//
//	/*
//	 * Toggle enable pin to start reading; needs to be high for at least
//	 * 20 microseconds (we use 1 ms). Go low again to stay in realtime mode.
//	 *
//	 * Disable interrupts to ensure correct timing between when
//	 * we start the sequence and when we start the TDC.
//	 */
//	interrupts_disabled = ti_lib_int_master_disable();
//
//	ti_lib_gpio_set_dio(MB_CONTROL_IOID);
//	BUSYWAIT_UNTIL(0, RTIMER_SECOND / 1000);
//	ti_lib_gpio_clear_dio(MB_CONTROL_IOID);

	INFO("Configure TDC...");
	print_tdc_state();

	AUXTDCConfigSet(MB_AUX_TDC_BASE,
				MB_DATA_AUXTDC_START | AUXTDC_STARTPOL_RIS,
				MB_DATA_AUXTDC_STOP | AUXTDC_STOPPOL_FALL);

	print_tdc_state();
	INFO("Configured!");

	INFO("Enable...");

	/* Run async (we start well before the sonar is ready) */
	AUXTDCEnable(MB_AUX_TDC_BASE, AUX_TDC_RUNSYNC);

	print_tdc_state();
	INFO("Enabled!");

//	if (!interrupts_disabled) {
//		INFO("Reenable interrupts");
//		ti_lib_int_master_enable();
//	}

	INFO("Wait for measurement...");
	print_tdc_state();

	/*
	 * Reading should be available ~118ms after enable
	 * goes high. Maximum pulse width is 5000 microseconds,
	 * so ~123 ms at max. We wait 200 ms.
	 */
	watchdog_periodic();
	BUSYWAIT_UNTIL(AUXTDCMeasurementDone(MB_AUX_TDC_BASE) != AUX_TDC_BUSY, (RTIMER_SECOND)); // /5

	INFO("Measuring done...");
	print_tdc_state();
	uint32_t measurement_status = AUXTDCMeasurementDone(MB_AUX_TDC_BASE);

	INFO("Measurement status: %08X", measurement_status);

	if (measurement_status != AUX_TDC_DONE) {
		INFO("TDC measurement failed: %08X", measurement_status);
		//AUXTDCIdleForce(MB_AUX_TDC_BASE);
		return 0;
	}

	uint32_t result = (uint32_t)AUXTDCMeasurementGet(MB_AUX_TDC_BASE);

	INFO("Read %08X", result);

	//AUXTDCIdleForce(MB_AUX_TDC_BASE);

	return result;
}

uint16_t
ti_lib_ddi_16_bit_read1(uint32_t ui32Base, uint32_t ui32Reg, uint32_t ui32Mask)
{
    return HWREG(ui32Base + ui32Reg) & ui32Mask;
}

void
ti_lib_ddi_16_bit_write1(uint32_t ui32Base, uint32_t ui32Reg, uint32_t ui32Mask,
		uint32_t value)
{
     HWREG(ui32Base + ui32Reg) = value & ui32Mask;
}


static void debug_write_16_bit_field(uint32_t base, uint32_t reg, uint32_t mask,
		uint32_t value, const char* desc)
{
	uint16_t old_value, new_value;

	old_value = ti_lib_ddi_16_bit_read1(base, reg, mask);
	ti_lib_ddi_16_bit_write1(base, reg, mask, value);
	new_value = ti_lib_ddi_16_bit_read1(base, reg, mask);

	INFO("%-25s %04X -> %04X", desc, old_value, new_value);
}

static uint16_t measure_distance(void)
{
	INFO("Configure AUX module");

	//oscillators_switch_to_hf_rc();

	g_aux_consumer.next = NULL;
	g_aux_consumer.clocks = AUX_WUC_TDCIF_CLOCK | AUX_WUC_TDC_CLOCK |  AUX_WUC_OSCCTRL_CLOCK
			| AUX_WUC_SMPH_CLOCK | AUX_WUC_AIODIO0_CLOCK | AUX_WUC_AIODIO1_CLOCK | AUX_WUC_ADI_CLOCK;
	aux_ctrl_register_consumer(&g_aux_consumer);

//	INFO("Write DDI_CTL0");
//	ti_lib_ddi_16_bit_field_write(AUX_DDI0_OSC_BASE, DDI_0_OSC_O_CTL0,
//	                                  DDI_0_OSC_CTL0_ACLK_TDC_SRC_SEL_M,
//									  DDI_0_OSC_CTL0_ACLK_TDC_SRC_SEL_S,
//	                                  0x00);
//
//	INFO("Read DDI_CTL0");
//	uint16_t val = ti_lib_ddi_16_bit_read(AUX_DDI0_OSC_BASE, DDI_0_OSC_O_CTL0, DDI_0_OSC_CTL0_ACLK_TDC_SRC_SEL_M);
//
//	INFO("DDI_CTL0: %02X", val >> DDI_0_OSC_CTL0_ACLK_TDC_SRC_SEL_S);

	/*
	 * There are up to 16 signals (AUXIO0 to AUXIO15) in the sensor controller domain (AUX). These signals
can be routed to specific pins given in Table 11-2. AUXIO0 to AUXIO7 have analog capability, but can
also be used as digital I/Os, while AUXIO8 to AUXIO15 are digital only. The signals routed from the
sensor controller domain (AUX) are configured differently than GPIO and other peripheral functions. This
section does not cover the use of all the capabilities of the sensor controller (see Chapter 17, Sensor
Controller chapter, for more details).
In this example, AUXIO1 is mapped to DIO29 on the 7 × 7 package type and set up as a digital input. The
pin number and DIO number differs for different package types. The module must be powered, and the
clock to the specific module within the AUX domain must be enabled (AIODIO1 for AUXIO0 to AUXIO7).
1. Set the IOC:IOCFG29 PORTID bit field to 0x08 (AUX_I/O) to route AUXIO1 to DIO29.
2. The I/O signals in the AUX domain have their own open-source or open-drain configuration, which
must be set in the AUX_AIODIO:IOMODE register in the AUX domain. Set AUX_AIODIO:IOMODE.IO1
to 0x01 to enable AUXIO1 as a digital input.
3. Enable the digital input buffer for AUXIO1 by setting the IO7_0 bit field to 0x02 in the
AUX_AIODIO0:GPIODIE register.
4. The AUX latch is set to static configuration by default (values from AUXIOs are latched). Release the
latch and set in transparent mode by writing 0x01 to the AUX_WUC:AUXIOLATCH register.

pg 978 11.3.2
	 */

	/*
	 * I'm not sure about the ordering of this. Section 11.3.2 of the user guide
	 * says we should set latching to transparent in step 4, but a code comment
	 * for AUX_WUC_AUXIOLATCH_EN says we should do it before configuring AUX_AIODIO.
	 *
	 * Neither way seems to work, anyway..
	 */
	debug_write_16_bit_field(AUX_WUC_BASE, AUX_WUC_O_AUXIOLATCH,
			AUX_WUC_AUXIOLATCH_EN_M,
			AUX_WUC_AUXIOLATCH_EN_TRANSP, "AUX_WUC:AUXIOLATCH");

	/* Route IOID to the AUX domain */
	ti_lib_ioc_pin_type_aux(MB_DATA_IOID);

	/*
		 * Enable input buffer for AUXIO7
		 */
		debug_write_16_bit_field(AUX_AIODIO0_BASE, AUX_AIODIO_O_GPIODIE,
					AUX_AIODIO_GPIODIE_IO7_0_M,
					1 << 7, "AUX_AIODIO0:GPIODIE");


	/*
	 * Set AUXIO7 to an input
	 */
	debug_write_16_bit_field(AUX_AIODIO0_BASE, AUX_AIODIO_O_IOMODE,
			AUX_AIODIO_IOMODE_IO7_M,
			AUX_AIODIO_IOMODE_IO7_IN, "AUX_AIODIO0:IOMODE");




	// todo: TDC gets in bad state after a failed read (stuck in AUXTDC_WAIT_CLRCNT_DONE)
	uint16_t result = measure_distance_with_aux();

	//aux_ctrl_unregister_consumer(&g_aux_consumer);

	return result;
}

/*---------------------------------------------------------------------------*/
static int
value(int type)
{
  return (int)measure_distance();
}
/*---------------------------------------------------------------------------*/
static int
configure(int type, int enable)
{
  switch(type) {
  case SENSORS_HW_INIT:
	if (!enabled) {
		hw_init();
	}
    break;
  case SENSORS_ACTIVE:
    /* Must be initialised first */
    if(!enabled) {
      return 0;
    }
    break;
  default:
    break;
  }
  return enabled;
}
/*---------------------------------------------------------------------------*/
static int
status(int type)
{
  switch(type) {
  case SENSORS_ACTIVE:
  case SENSORS_READY:
    return enabled;
    break;
  default:
    break;
  }
  return 0;
}

SENSORS_SENSOR(mb7369_sensor, "MB7639", value, configure, status);



