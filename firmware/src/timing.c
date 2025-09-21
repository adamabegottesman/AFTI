#include "timing.h"

/* free-running ms counter + one-shot deadline */
static volatile uint32_t g_ms_ticks = 0;
static volatile uint32_t g_deadline_ms = 0; // 0 => idle

void timing_tick(void) { g_ms_ticks++; }

/* Weak ISR: if you provide your own SysTick_Handler elsewhere,
   it will override this; but call timing_tick() from yours. */
__attribute__((weak)) void SysTick_Handler(void)
{
  timing_tick();
}

void initAudioTimer(void)
{
  /* call AFTER SystemCoreClock is final (i.e., after PLL setup) */
  uint32_t reload = (SystemCoreClock / 1000U) - 1U;
  SysTick->LOAD  = reload;
  SysTick->VAL   = 0;
  SysTick->CTRL  = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;
}

void setAudioTimer(uint32_t ms)
{
  uint32_t cap = (ms < 65000U) ? ms : 65000U;
  g_deadline_ms = g_ms_ticks + cap;
}

uint32_t hasAudioTimerFinished(void)
{
  if (g_deadline_ms == 0U) return 1U;                      // treat as elapsed if not armed
  int32_t diff = (int32_t)(g_ms_ticks - g_deadline_ms);
  return (diff >= 0) ? 1U : 0U;
}

void audioDelay(uint32_t ms)
{
  setAudioTimer(ms);
  while (!hasAudioTimerFinished()) { /* spin */ }
  g_deadline_ms = 0U; // disarm
}

uint32_t waitForFlagWithTimeout(volatile uint32_t *addr,
                                uint32_t bit,
                                uint32_t value01,
                                uint32_t timeout_ms)
{
  uint32_t cur = ((*addr) >> bit) & 0x01U;
  if (cur == value01) return 0;

  setAudioTimer(timeout_ms);
  while (!hasAudioTimerFinished())
  {
    cur = ((*addr) >> bit) & 0x01U;
    if (cur == value01) return 0;
  }
  return 1; // timeout
}
