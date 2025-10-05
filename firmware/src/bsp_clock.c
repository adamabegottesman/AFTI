// bsp_clock.c — core and audio clock helpers for STM32F4 (DISC1), CMSIS-only.
// - Brings SYSCLK to 168 MHz from HSE = 8 MHz
// - Seeds/retunes the I2S domain (PLLI2S) for audio sample rates
// - Pairs with timing.c for 1 kHz SysTick helpers (wait_flag_with_timeout, etc.)

#include "bsp_clock.h"
#include "timing.h"             // wait_flag_with_timeout(), delay helpers
#include "stm32f407xx.h"
#include "system_stm32f4xx.h"   // AHBPrescTable, SystemCoreClock

/* --------------------------------------------------------------------------
   Internal helpers (return 0 on success, -1 on failure)
   -------------------------------------------------------------------------- */

static int i2s_pll_config(uint32_t new_n, uint32_t new_r)
{
  /* Disable PLLI2S and wait until it stops */
  RCC->CR &= ~RCC_CR_PLLI2SON;
  if (wait_flag_with_timeout(&RCC->CR, RCC_CR_PLLI2SON_Pos, 0, 100)) return -1;

  /* Program new factors */
  RCC->PLLI2SCFGR =
      (new_n << RCC_PLLI2SCFGR_PLLI2SN_Pos) |
      (new_r << RCC_PLLI2SCFGR_PLLI2SR_Pos);

  /* Enable PLLI2S and wait for lock */
  RCC->CR |= RCC_CR_PLLI2SON;
  if (wait_flag_with_timeout(&RCC->CR, RCC_CR_PLLI2SRDY_Pos, 1, 100)) return -1;

  return 0;
}

static int pll_main_config(void)
{
  /* Turn on HSE and wait ready */
  RCC->CR |= RCC_CR_HSEON;
  if (wait_flag_with_timeout(&RCC->CR, RCC_CR_HSERDY_Pos, 1, 100)) return -1;

  /* Disable main PLL if running and wait it off */
  RCC->CR &= ~RCC_CR_PLLON;
  if (wait_flag_with_timeout(&RCC->CR, RCC_CR_PLLRDY_Pos, 0, 100)) return -1;

  /* Configure main PLL (HSE=8 MHz → SYSCLK=168 MHz) */
  uint32_t cfg = 0;
  cfg |= (8U   << RCC_PLLCFGR_PLLM_Pos);  /* M = 8   */
  cfg |= (336U << RCC_PLLCFGR_PLLN_Pos);  /* N = 336 */
  cfg |= (((2U >> 1U) - 1U) << RCC_PLLCFGR_PLLP_Pos); /* P = 2 (encoding 0) */
  cfg |= (7U   << RCC_PLLCFGR_PLLQ_Pos);  /* Q = 7 (USB etc.) */
  cfg |= RCC_PLLCFGR_PLLSRC_HSE;          /* source = HSE */
  RCC->PLLCFGR = cfg;

  /* Enable PLL and wait lock */
  RCC->CR |= RCC_CR_PLLON;
  if (wait_flag_with_timeout(&RCC->CR, RCC_CR_PLLRDY_Pos, 1, 100)) return -1;

  return 0;
}

static int flash_ws_busclocks_config(void)
{
  /* Flash latency for 168 MHz (Scale 1 / 5 WS) — set before raising HCLK */
  if (FLASH_ACR_LATENCY_5WS > (FLASH->ACR & FLASH_ACR_LATENCY)) {
    FLASH->ACR = (FLASH->ACR & ~FLASH_ACR_LATENCY) | FLASH_ACR_LATENCY_5WS;
    if ((FLASH->ACR & FLASH_ACR_LATENCY) != FLASH_ACR_LATENCY_5WS) return -1;
  }

  /* Prescalers to safe settings before SYSCLK switch */
  RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_PPRE1) | RCC_CFGR_PPRE1_DIV16;
  RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_PPRE2) | RCC_CFGR_PPRE2_DIV16;
  RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_HPRE)  | RCC_CFGR_HPRE_DIV1;

  if ((RCC->CR & RCC_CR_PLLRDY) == 0) return -1;

  /* Switch SYSCLK <- PLL and confirm switch */
  RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_PLL;
  if (wait_flag_with_timeout(&RCC->CFGR, 3U /*SWS[1:0]*/, 1U /*PLL*/, 100)) return -1;

  /* Final prescalers: APB1=42 MHz, APB2=84 MHz (AHB remains 168 MHz) */
  RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_PPRE1) | RCC_CFGR_PPRE1_DIV4;
  RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_PPRE2) | RCC_CFGR_PPRE2_DIV2;

  /* Recompute SystemCoreClock (mirror CMSIS logic; extract PLL factors correctly) */
  const uint32_t pllm   = (RCC->PLLCFGR & RCC_PLLCFGR_PLLM) >> RCC_PLLCFGR_PLLM_Pos;
  const uint32_t pllp   = ((((RCC->PLLCFGR & RCC_PLLCFGR_PLLP) >> RCC_PLLCFGR_PLLP_Pos) + 1U) * 2U);
  const uint32_t pllsrc = (RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC) ? 8000000U : 16000000U;
  const uint32_t plln   = (RCC->PLLCFGR & RCC_PLLCFGR_PLLN) >> RCC_PLLCFGR_PLLN_Pos;

  const uint32_t pllvco = (uint32_t)(((uint64_t)pllsrc * (uint64_t)plln) / (uint64_t)pllm);
  const uint32_t sysclk = pllvco / pllp;

  SystemCoreClock = sysclk >> AHBPrescTable[(RCC->CFGR & RCC_CFGR_HPRE) >> RCC_CFGR_HPRE_Pos];

  return 0;
}

/* --------------------------------------------------------------------------
   Public API
   -------------------------------------------------------------------------- */

int system_clock_init_168mhz(void)
{
  /* Power interface clock + Voltage scaling (Scale 1) for 168 MHz */
  RCC->APB1ENR |= RCC_APB1ENR_PWREN;
  (void)RCC->APB1ENR;  /* ensure write takes effect */
  PWR->CR = (PWR->CR & ~PWR_CR_VOS) | PWR_CR_VOS;  /* Scale 1 mode */

  if (pll_main_config()           != 0) return -1;
  if (flash_ws_busclocks_config() != 0) return -1;

  /* Seed PLLI2S; actual Fs chosen later in i2s_pll_set_fs() */
  if (i2s_pll_config(192, 2)      != 0) return -1;

  return (SystemCoreClock == 168000000U) ? 0 : -1;
}

/* PLLI2S presets for common sample rates (96 kHz default index = 7) */
static const uint32_t i2s_fs_table[8]     = {  8000, 11025, 16000, 22050, 32000, 44100, 48000, 96000 };
static const uint32_t i2s_pll_n_table[8]  = {   256,   429,   213,   429,   426,   271,   258,   344 };
static const uint32_t i2s_pll_r_table[8]  = {     5,     4,     4,     4,     4,     6,     3,     1 };

void i2s_pll_set_fs(uint32_t audio_fs_hz)
{
  /* Choose exact match; fall back to 96 kHz if not found */
  uint8_t idx = 7U; /* default 96 kHz */
  for (uint8_t i = 0; i < 8U; ++i) {
    if (i2s_fs_table[i] == audio_fs_hz) { idx = i; break; }
  }

  /* Disable, program, and re-enable PLLI2S with timeouts (consistent with helpers) */
  RCC->CR &= ~RCC_CR_PLLI2SON;
  if (wait_flag_with_timeout(&RCC->CR, RCC_CR_PLLI2SRDY_Pos, 0, 100)) return;

  RCC->PLLI2SCFGR =
      ((i2s_pll_n_table[idx] << RCC_PLLI2SCFGR_PLLI2SN_Pos) |
       (i2s_pll_r_table[idx] << RCC_PLLI2SCFGR_PLLI2SR_Pos));

  RCC->CR |= RCC_CR_PLLI2SON;
  (void)wait_flag_with_timeout(&RCC->CR, RCC_CR_PLLI2SRDY_Pos, 1, 100);
}
