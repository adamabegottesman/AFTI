#include "stm32f4xx.h"
#include "bsp_clock.h"

// Helper: wait for a bit set in a register (simple, timeout-free on purpose)
static inline void _wait_set(volatile uint32_t* reg, uint32_t mask){
  while(((*reg) & mask) != mask) {}
}

void bsp_clock_init_168mhz(void){
  // 1) Enable HSE and set Flash latency first (5 WS for 168 MHz)
  FLASH->ACR = FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_PRFTEN | FLASH_ACR_LATENCY_5WS;

  RCC->CR |= RCC_CR_HSEON;
  _wait_set(&RCC->CR, RCC_CR_HSERDY);

  // 2) Prescalers: AHB=1, APB1=4, APB2=2
  RCC->CFGR =
      (RCC->CFGR & ~(RCC_CFGR_HPRE | RCC_CFGR_PPRE1 | RCC_CFGR_PPRE2)) |
      (RCC_CFGR_HPRE_DIV1 | RCC_CFGR_PPRE1_DIV4 | RCC_CFGR_PPRE2_DIV2);

  // 3) Main PLL: HSE(8) -> VCO 336, SYSCLK 168 (PLLP=2), USB 48 (PLLQ=7)
  // PLLM=8, PLLN=336, PLLP=2 (00b), PLLQ=7
  RCC->PLLCFGR =
      RCC_PLLCFGR_PLLSRC_HSE |
      (8u   << RCC_PLLCFGR_PLLM_Pos) |
      (336u << RCC_PLLCFGR_PLLN_Pos) |
      (0u   << RCC_PLLCFGR_PLLP_Pos) |
      (7u   << RCC_PLLCFGR_PLLQ_Pos);

  RCC->CR |= RCC_CR_PLLON;
  _wait_set(&RCC->CR, RCC_CR_PLLRDY);

  // 4) Switch SYSCLK to PLL
  RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_PLL;
  while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL) {}

  // 5) Let CMSIS compute SystemCoreClock from registers
  SystemCoreClockUpdate();
}
