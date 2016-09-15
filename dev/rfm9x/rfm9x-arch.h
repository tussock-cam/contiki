#ifndef RFM9X_ARCH_H
#define RFM9X_ARCH_H

#include <stdint.h>

void rfm9x_init(void);

void rfm9x_io_init(void);
void rfm9x_spi_init(void);
void rfm9x_interrupt_init(void);

void rfm9x_disable_all_irq(void);
void rfm9x_enable_all_irq(void);

void rfm9x_set_nss(uint8_t val);
void rfm9x_set_rxtx(uint8_t rx, uint8_t tx);
void rfm9x_set_rst(uint8_t val);
void rfm9x_disable_rst(void);
uint8_t rfm9x_spi_read_write(uint8_t out);

void rfm9x_irq_handler(uint8_t dio);

#endif
