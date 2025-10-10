// timing.h — 1 kHz timebase + small timing utilities for STM32F4 (CMSIS-only)
//
// Provides:
//  - timing_init_1khz(): sets up SysTick for a 1 ms tick (uses SystemCoreClock)
//  - timing_now_ms():    monotonic millisecond counter (wraps at 2^32)
//  - delay_ms():         blocking delay based on the 1 ms tick
//  - timing_start_ms()/timing_expired(): simple one-shot timeout helper
//  - wait_flag_with_timeout(): poll a bit with a millisecond timeout
//
// Notes:
//  - Call timing_init_1khz() AFTER final clocks are configured
//    (e.g., after system_clock_init_168mhz()) so SystemCoreClock is correct.
//  - timing.c provides a weak SysTick_Handler that calls timing_tick();
//    if you override SysTick_Handler in your code, call timing_tick() inside it.
//  - APIs are foreground-friendly; avoid calling from high-rate ISRs.

#ifndef TIMING_H
#define TIMING_H

#include <stdint.h>
#include "stm32f407xx.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Initialize SysTick to generate a 1 kHz timebase.
 *  Must be called once after SystemCoreClock is final (post-PLL). */
void timing_init_1khz(void);

/** Return the current millisecond tick (wraps at 2^32). */
uint32_t timing_now_ms(void);

/** Busy-wait delay for the requested number of milliseconds. */
void delay_ms(uint32_t ms);

/** Start (or restart) a one-shot timeout that expires in `ms` milliseconds. */
void timing_start_ms(uint32_t ms);

/** Check whether the active timeout has expired.
 *  @return 1 if expired or not armed; 0 if still pending. */
uint32_t timing_expired(void);

/** Poll a memory-mapped bit until it matches the requested value or a timeout.
 *  @param addr       Pointer to 32-bit register to poll (volatile).
 *  @param bit_pos    Bit position [0..31] to test.
 *  @param value01    Expected bit value (0 or 1).
 *  @param timeout_ms Milliseconds to wait before giving up.
 *  @return 0 on success (bit reached value); 1 on timeout. */
uint32_t wait_flag_with_timeout(volatile uint32_t *addr,
                                uint32_t bit_pos,
                                uint32_t value01,
                                uint32_t timeout_ms);

int wait_flag_with_timeout_us(volatile const uint32_t *reg,
                              uint32_t bit_index,
                              uint32_t want_value,
                              uint32_t timeout_us);

uint32_t timing_now_us(void);

void timing_init_dwt_us(void);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* TIMING_H */
