#include "debug_print.h"
#include "stm32f4xx.h"   // CMSIS

// ---- low-level helpers (private) ----
static inline uint32_t _itm_ready(void){
  return (ITM->TCR & ITM_TCR_ITMENA_Msk) && (ITM->TER & 1U) && ITM->PORT[0].u32;
}

// Route PB3 as TRACESWO (AF0) explicitly (optional but safe)
static void _enable_swo_pin(void){
  // Enable trace pins, async SWO
  DBGMCU->CR |= DBGMCU_CR_TRACE_IOEN;
  DBGMCU->CR &= ~DBGMCU_CR_TRACE_MODE;   // 00 = async

  // PB3 to AF0
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
  GPIOB->MODER  &= ~(3u << (3*2));
  GPIOB->MODER  |=  (2u << (3*2));       // AF
  GPIOB->AFR[0] &= ~(0xFu << (3*4));     // AF0
}

void DebugPrint_Init(uint32_t swo_baud){
  _enable_swo_pin();

  // Enable trace subsystem
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

  // Unlock ITM + enable stimulus port 0; SWO on
  ITM->LAR = 0xC5ACCE55;
  ITM->TER = 1U;
  ITM->TCR = ITM_TCR_ITMENA_Msk | ITM_TCR_SWOENA_Msk;

  // SWO speed: SystemCoreClock / (ACPR+1)
  uint32_t acpr = (SystemCoreClock / swo_baud);
  if (acpr) acpr -= 1U;
  TPI->ACPR = acpr;

  // Async NRZ on SWO
  TPI->SPPR = 2U;
  TPI->FFCR = 0U;
}



// Non-blocking: give it a short spin then drop the byte if host isn't ready
void DebugPrint_PutChar(uint8_t ch){
  if ((ITM->TCR & ITM_TCR_ITMENA_Msk) && (ITM->TER & 1U)){
    for (int i = 0; i < 1024; ++i){
      if (_itm_ready()){ ITM->PORT[0].u8 = ch; return; }
      __NOP();
    }
  }
}

void DebugPrint_Write(const void* buf, uint32_t len){
  const uint8_t* p = (const uint8_t*)buf;
  while (len--) DebugPrint_PutChar(*p++);
}

// printf backend
int __io_putchar(int ch){
  if (ch == '\n') DebugPrint_PutChar('\r');
  DebugPrint_PutChar((uint8_t)ch);
  return ch;
}

// for newlib/newlib-nano
int _write(int fd, const char* buf, int len){
  (void)fd;
  for (int i = 0; i < len; ++i) __io_putchar((unsigned char)buf[i]);
  return len;
}
