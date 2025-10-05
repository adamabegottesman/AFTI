// timing.c — 1 kHz timebase + small timing utilities for STM32F4 (CMSIS-only)
//
// Provides:
//  - A free-running millisecond counter (incremented by SysTick)
//  - A simple one-shot timeout primitive
//  - Busy-wait delay and a bit-poll-with-timeout helper
//
// Notes:
//  - You may override SysTick_Handler in your own file; if you do, call
//    timing_tick() in your handler so the ms counter advances.

#include "timing.h"

/* ============================== Internal state ============================= */
/* Free-running ms counter + one-shot deadline.
   g_deadline_ms == 0 → not armed (treated as "already expired"). */
static volatile uint32_t g_ms_ticks    = 0;
static volatile uint32_t g_deadline_ms = 0;  // 0 => idle/not armed
static uint8_t           g_inited      = 0;

#define TIMING_MAX_TIMEOUT_MS 65000u  /* cap avoids large wrap windows */

/* ================================ ISR glue ================================= */

void timing_tick(void) { g_ms_ticks++; }

/* Weak ISR: user code may override. If you provide your own SysTick_Handler,
   call timing_tick() inside it so utilities here continue to work. */
__attribute__((weak)) void SysTick_Handler(void)
{
  timing_tick();
}

/* ================================ Public API =============================== */

void timing_init_1khz(void)
{
  if (g_inited) return;  // idempotent; safe to call multiple times

  /* Call AFTER SystemCoreClock is final (post-PLL). */
  uint32_t reload = (SystemCoreClock / 1000u);
  if (reload == 0u) reload = 1u;
  reload -= 1u;

  /* Clamp to SysTick LOAD width (24 bits). */
  if (reload > SysTick_LOAD_RELOAD_Msk) reload = SysTick_LOAD_RELOAD_Msk;

  SysTick->LOAD = reload;
  SysTick->VAL  = 0u;
  SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk   /* core clock */
                | SysTick_CTRL_TICKINT_Msk     /* enable IRQ */
                | SysTick_CTRL_ENABLE_Msk;     /* start */

  g_inited = 1u;
}

/* Start/restart a one-shot timeout (ms). 0 ms => immediately expired.
   Value is capped to keep arithmetic well-behaved. */
void timing_start_ms(uint32_t ms)
{
  if (ms == 0u) {
    g_deadline_ms = g_ms_ticks;   // immediately expired
    return;
  }

  const uint32_t cap = (ms < TIMING_MAX_TIMEOUT_MS) ? ms : TIMING_MAX_TIMEOUT_MS;
  g_deadline_ms = g_ms_ticks + cap;  // wrap-safe comparison used in timing_expired()
}

/* True if timeout expired (wrap-safe). If not armed, returns true. */
uint32_t timing_expired(void)
{
  if (g_deadline_ms == 0u) return 1u;  // treat unarmed as expired
  const int32_t diff = (int32_t)(g_ms_ticks - g_deadline_ms);
  return (diff >= 0) ? 1u : 0u;
}

/* Busy-wait delay (ms). Uses the one-shot helper. */
void delay_ms(uint32_t ms)
{
  timing_start_ms(ms);
  while (!timing_expired()) {
    __NOP();  // small hint to the core; keeps the loop well-formed
  }
  g_deadline_ms = 0u; // disarm
}

/* Poll a bit at position bit_pos in *addr until it equals value01 or times out.
   Returns 0 on success (bit matched), 1 on timeout or invalid args. */
uint32_t wait_flag_with_timeout(volatile uint32_t *addr,
                                uint32_t bit_pos,
                                uint32_t value01,
                                uint32_t timeout_ms)
{
  if (addr == 0 || bit_pos >= 32u) {
    return 1u; // invalid arguments -> treat as failure/timeout
  }

  const uint32_t expect = (value01 & 0x1u);
  uint32_t cur = ((*addr) >> bit_pos) & 0x01u;
  if (cur == expect) return 0u;

  timing_start_ms(timeout_ms);
  while (!timing_expired())
  {
    cur = ((*addr) >> bit_pos) & 0x01u;
    if (cur == expect) {
      return 0u;
    }
  }
  return 1u; // timeout
}

/* Current ms tick (wraps at 2^32). Single 32-bit read is atomic on Cortex-M4. */
uint32_t timing_now_ms(void)
{
  return g_ms_ticks;
}
