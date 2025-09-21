#ifndef INC_BSP_CLOCK_H_
#define INC_BSP_CLOCK_H_

#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Configure F407 to 168 MHz SYSCLK, APB1=42 MHz, APB2=84 MHz.
// Also provides 48 MHz on PLLQ for USB FS (harmless if unused).
void bsp_clock_init_168mhz(void);

#ifdef __cplusplus
}
#endif






#endif /* INC_BSP_CLOCK_H_ */
