#ifndef ADS1256_DMA_H
#define ADS1256_DMA_H

#include <stdint.h>
#include "stm32f4xx.h"
#include <stdbool.h>    // ← ADD THIS

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*ads1256_dma_sample_cb_t)(int32_t v);

/* Call once after SPI1 + GPIO are set, before enabling EXTI */
void ads1256_dma_init(ads1256_dma_sample_cb_t cb);

/* Call from DRDY ISR (very fast): kicks a 3-byte DMA read */
void ads1256_dma_kick(void);

/* RX TC IRQ handler (wire in vector table via the .c file’s wrapper) */
void ads1256_dma_irq_rx(void);

bool ads1256_dma_is_busy(void);


#ifdef __cplusplus
}
#endif
#endif
