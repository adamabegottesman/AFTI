// bsp_clock.h — core and audio clock helpers for STM32F4 (DISC1), CMSIS-only.
// - Brings SYSCLK to 168 MHz from external HSE = 8 MHz
// - Programs the I2S clock domain (PLLI2S) for common audio sample rates
// - Declares a 1 kHz SysTick init used by the timing utilities

#ifndef BSP_CLOCK_H
#define BSP_CLOCK_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Configure system clocks for 168 MHz using HSE = 8 MHz.
 *
 * Effects:
 *  - SYSCLK = 168 MHz (AHB/APB prescalers set to 168/84/42 MHz)
 *  - Flash wait states updated for Scale 1 @ 168 MHz
 *  - Seeds PLLI2S (will be retuned later by i2s_pll_set_fs)
 *
 * Call before any peripheral init that depends on final clocks.
 *
 * @retval 0  Success (SystemCoreClock == 168000000)
 * @retval <0 Failure (e.g., HSE/PLL lock timeout)
 */
int system_clock_init_168mhz(void);

/**
 * @brief Program PLLI2S for the requested audio sample rate.
 *
 * Configures N/R using a small preset table for typical Fs values:
 * { 8000, 11025, 16000, 22050, 32000, 44100, 48000, 96000 } Hz.
 *
 * Notes:
 *  - Must be called after system_clock_init_168mhz().
 *  - Does not touch I2S peripherals or DMA; clocks only.
 *  - If Fs is not in the table, implementation currently falls back to 96 kHz.
 *
 * @param audio_fs_hz Target sample rate in Hz.
 */
void i2s_pll_set_fs(uint32_t audio_fs_hz);

/**
 * @brief Initialize a 1 kHz SysTick timebase.
 *
 * Effects:
 *  - Configures SysTick to tick every 1 ms, enables IRQ
 *  - Intended for use by timing utilities (delay_ms, timeouts, etc.)
 *
 * Call after system_clock_init_168mhz() so the tick derives from final HCLK.
 */
void timing_init_1khz(void);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* BSP_CLOCK_H */
