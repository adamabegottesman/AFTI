#ifndef TIMING_H
#define TIMING_H

#include <stdint.h>
#include "stm32f407xx.h"

/* Public API (same names you already use) */
void     initAudioTimer(void);                    // config SysTick for 1 kHz
void     audioDelay(uint32_t ms);                 // blocking ms delay
void     setAudioTimer(uint32_t ms);              // arm one-shot
uint32_t hasAudioTimerFinished(void);             // 1 when elapsed
uint32_t waitForFlagWithTimeout(volatile uint32_t *addr,
                                uint32_t bit,
                                uint32_t value01,
                                uint32_t timeout_ms);

/* If your project already has a SysTick_Handler, call timing_tick() from it. */
void timing_tick(void);

#endif /* TIMING_H */
