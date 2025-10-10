// timing.c — minimal 1 kHz timebase + tiny timeout helpers (STM32F4, CMSIS)

#include "timing.h"
#include "stm32f4xx.h"
#include "core_cm4.h"   // __NOP(), __get_PRIMASK()

/* ============================== Internal state ============================= */
static volatile uint32_t g_ms_ticks    = 0;   /* free-running ms counter */
static volatile uint32_t g_deadline_ms = 0;   /* 0 => not armed */
static uint8_t           g_inited      = 0;

#define TIMING_MAX_TIMEOUT_MS  65000u  /* cap to avoid huge wrap windows */

/* ================================ ISR glue ================================= */
void timing_tick(void) { g_ms_ticks++; }

/* Weak ISR so user code can override SysTick_Handler; call timing_tick() if you do. */
__attribute__((weak)) void SysTick_Handler(void) { timing_tick(); }

/* ================================ Public API =============================== */
void timing_init_1khz(void)
{
  if (g_inited) return;

  /* Call AFTER SystemCoreClock is final (post-PLL). */
  uint32_t reload = SystemCoreClock / 1000u;
  if (reload == 0u) reload = 1u;
  reload -= 1u;
  if (reload > SysTick_LOAD_RELOAD_Msk) reload = SysTick_LOAD_RELOAD_Msk;

  SysTick->LOAD = reload;
  SysTick->VAL  = 0u;
  SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk   /* core clock */
                | SysTick_CTRL_TICKINT_Msk     /* enable IRQ */
                | SysTick_CTRL_ENABLE_Msk;     /* start */

  g_inited = 1u;
}

/* Arm one-shot timeout (ms). 0 => immediately expired. */
void timing_start_ms(uint32_t ms)
{
  if (ms == 0u) {
    g_deadline_ms = g_ms_ticks;   /* already expired */
    return;
  }
  const uint32_t cap = (ms < TIMING_MAX_TIMEOUT_MS) ? ms : TIMING_MAX_TIMEOUT_MS;
  g_deadline_ms = g_ms_ticks + cap;   /* wrap-safe checked in timing_expired() */
}

/* True if timeout expired (wrap-safe). If not armed, returns true. */
uint32_t timing_expired(void)
{
  const uint32_t dl = g_deadline_ms;
  if (dl == 0u) return 1u;                       /* not armed => expired */
  return ((int32_t)(g_ms_ticks - dl) >= 0) ? 1u : 0u;  /* wrap-safe diff */
}

/* Busy-wait delay (ms). Uses the one-shot helper.
   Includes a very small fallback if IRQs are masked (SysTick not ticking). */
void delay_ms(uint32_t ms)
{
  timing_start_ms(ms);

  for (;;) {
    if (timing_expired()) break;

    /* If IRQs are globally masked, SysTick cannot advance. Do a bounded spin. */
    if ((__get_PRIMASK() & 1u) != 0u) {
      /* Roughly ~1 ms worth of cycles per loop chunk; tune divisor if needed. */
      volatile uint32_t spins = (SystemCoreClock / 8000u) * (ms ? ms : 1u);
      while (spins--) { __NOP(); }
      break;  /* consider delay satisfied; avoid infinite block */
    }

    __NOP();  /* keep loop well-formed without hammering the bus */
  }

  g_deadline_ms = 0u;  /* disarm */
}

/* Poll bit `bit_index` in *reg until it equals expect_val or timeout (ms).
   Returns 0 on success, 1 on timeout. */
uint32_t wait_flag_with_timeout(volatile uint32_t *reg,
                                uint32_t bit_index,
                                uint32_t expect_val,
                                uint32_t timeout_ms)
{
  const uint32_t mask   = (1u << bit_index);
  const uint32_t expect = (expect_val ? 1u : 0u);

  timing_start_ms(timeout_ms);

  for (;;) {
    if ((((*reg) & mask) ? 1u : 0u) == expect)
      return 0u;  /* matched */

    /* If IRQs are masked, SysTick won’t advance: do a bounded, crude spin. */
    if ((__get_PRIMASK() & 1u) != 0u) {
      volatile uint32_t spins = (SystemCoreClock / 8000u) * (timeout_ms ? timeout_ms : 1u);
      while (spins--) {
        if ((((*reg) & mask) ? 1u : 0u) == expect) return 0u;
        __NOP();
      }
      return 1u;  /* timed out under masked IRQs */
    }

    if (timing_expired())
      return 1u;  /* timeout */
  }
}

/* Current ms tick (wraps at 2^32). */
uint32_t timing_now_ms(void) { return g_ms_ticks; }
