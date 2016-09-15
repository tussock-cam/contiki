#include "dev/rfm9x/rfm9x-arch.h"
#include "contiki.h"
#include "ti-lib.h"
#include "dev/gpio-interrupt.h"
#include "board-spi.h"

#define RFM_DIO0_IOID			(IOID_1)
#define RFM_DIO1_IOID			(IOID_4)
#define RFM_DIO2_IOID			(IOID_5)
#define RFM_SPI_CLK_IOID		(IOID_10)
#define RFM_SPI_NSS_IOID		(IOID_29)
#define RFM_RST_IOID			(IOID_30)
//#define RFM_RX_IOID				()
//#define RFM_TX_IOID				()

static void gpio_isr(uint8_t ioid);

static void set_pin(int ioid, uint8_t val)
{
	if (val) {
		ti_lib_gpio_set_dio(ioid);
	} else {
		ti_lib_gpio_clear_dio(ioid);
	}
}

static inline void configure_input(int ioid, void(*isr_handler)(uint8_t))
{
	ti_lib_ioc_pin_type_gpio_input(ioid);
	ti_lib_ioc_io_port_pull_set(ioid, IOC_IOPULL_UP);
	gpio_interrupt_register_handler(ioid, isr_handler);
}

static inline void configure_output(int ioid)
{
	ti_lib_ioc_pin_type_gpio_output(ioid);
	ti_lib_ioc_io_port_pull_set(ioid, IOC_IOPULL_UP);
}

void rfm9x_io_init(void)
{
	configure_input(RFM_DIO0_IOID, gpio_isr);
}

void rfm9x_spi_init(void)
{
	ti_lib_ioc_pin_type_gpio_output(RFM_SPI_NSS_IOID);
	ti_lib_ioc_io_port_pull_set(RFM_SPI_NSS_IOID, IOC_IOPULL_UP);

	rfm9x_set_nss(1); /* Deasserted */
	board_spi_open(4000000, RFM_SPI_CLK_IOID);
}

void rfm9x_interrupt_init(void)
{

}

void rfm9x_disable_all_irq(void)
{
	ti_lib_int_master_disable();
}

void rfm9x_enable_all_irq(void)
{
	ti_lib_int_master_enable();
}

void rfm9x_set_nss(uint8_t val)
{
	set_pin(RFM_SPI_NSS_IOID, val);
}

void rfm9x_set_rxtx(uint8_t rx, uint8_t tx)
{
	/*
	 * These are not used on our board.
	 */
}

void rfm9x_set_rst(uint8_t val)
{
	configure_output(RFM_RST_IOID);
	set_pin(RFM_RST_IOID, val);
}

void rfm9x_disable_rst(void)
{
	ti_lib_ioc_pin_type_gpio_input(RFM_RST_IOID);
}

uint8_t rfm9x_spi_read_write(uint8_t out)
{
	uint32_t ul;
	ti_lib_ssi_data_put(SSI0_BASE, out);
	ti_lib_rom_ssi_data_get(SSI0_BASE, &ul);
	return (uint8_t)ul;
}

static void gpio_isr(uint8_t ioid)
{
	uint8_t dio;
	switch (ioid) {
	case RFM_DIO0_IOID:
		dio = 0;
		break;
	case RFM_DIO1_IOID:
		dio = 1;
		break;
	case RFM_DIO2_IOID:
		dio = 2;
		break;

	default:
		return;
	}

	rfm9x_irq_handler(dio);
}



