// i2s3_dma_pcm.c — minimal I2S3 (SPI3) + DMA1 Stream7 driver for PCM5102 #2
// Pins (AF6): PA4=WS (LRCK), PC10=CK (BCLK), PC12=SD (DAC DIN), optional PC7=MCLK
// Format: Philips I2S, Master TX, 24-in-32, stereo interleaved (L,R,...)

#include "i2s3_dma_pcm.h"
#include "stm32f407xx.h"
#include "bsp_clock.h"   // i2s_pll_set_fs()
#include "audio3_app.h"  // audio3_on_* ISR callbacks
#include <stddef.h>

/* ---------------- Build-time options -------------------------------------- */
#ifndef PCM5102B_USE_MCLK
#define PCM5102B_USE_MCLK 1   // Set to 0 if MCLK is not wired to the second PCM
#endif

#ifndef AUDIO3_DMA_IRQ_PRIO
#define AUDIO3_DMA_IRQ_PRIO 14
#endif

/* ---------------- Local helpers ------------------------------------------- */

static void dma1_stream7_disable_and_clear(void)
{
  DMA_Stream_TypeDef *s = DMA1_Stream7;

  /* Disable if running, then wait for EN to clear (per RM0090) */
  if (s->CR & DMA_SxCR_EN) {
    s->CR &= ~DMA_SxCR_EN;
    while (s->CR & DMA_SxCR_EN) { /* wait */ }
  }

  /* Clear all Stream7 flags */
  DMA1->HIFCR = DMA_HIFCR_CFEIF7 | DMA_HIFCR_CDMEIF7 | DMA_HIFCR_CTEIF7
              | DMA_HIFCR_CHTIF7 | DMA_HIFCR_CTCIF7;
}

/* ---------------- Public API ---------------------------------------------- */

void dma_i2s3_init_pcm(void)
{
  /* Clocks: SPI3 + GPIOA + GPIOC + DMA1 */
  RCC->APB1ENR |= RCC_APB1ENR_SPI3EN;
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOCEN | RCC_AHB1ENR_DMA1EN;
  (void)RCC->AHB1ENR;

  /* GPIOA: PA4=WS (AF6) */
  GPIOA->MODER   &= ~GPIO_MODER_MODER4;   GPIOA->MODER   |=  GPIO_MODER_MODER4_1;
  GPIOA->OTYPER  &= ~GPIO_OTYPER_OT4;
  GPIOA->OSPEEDR |=  GPIO_OSPEEDER_OSPEEDR4_0;
  GPIOA->PUPDR   &= ~GPIO_PUPDR_PUPDR4;
  GPIOA->AFR[0]   = (GPIOA->AFR[0] & ~(0xFu << (4u*4u))) | (6u << (4u*4u));

  /* GPIOC: PC10=CK, PC12=SD (AF6) */
  // PC10 CK
  GPIOC->MODER   &= ~GPIO_MODER_MODER10;  GPIOC->MODER   |=  GPIO_MODER_MODER10_1;
  GPIOC->OTYPER  &= ~GPIO_OTYPER_OT10;
  GPIOC->OSPEEDR |=  GPIO_OSPEEDER_OSPEEDR10_0;
  GPIOC->PUPDR   &= ~GPIO_PUPDR_PUPDR10;
  GPIOC->AFR[1]   = (GPIOC->AFR[1] & ~(0xFu << ((10u-8u)*4u))) | (6u << ((10u-8u)*4u));

  // PC12 SD
  GPIOC->MODER   &= ~GPIO_MODER_MODER12;  GPIOC->MODER   |=  GPIO_MODER_MODER12_1;
  GPIOC->OTYPER  &= ~GPIO_OTYPER_OT12;
  GPIOC->OSPEEDR |=  GPIO_OSPEEDER_OSPEEDR12_0;
  GPIOC->PUPDR   &= ~GPIO_PUPDR_PUPDR12;
  GPIOC->AFR[1]   = (GPIOC->AFR[1] & ~(0xFu << ((12u-8u)*4u))) | (6u << ((12u-8u)*4u));

#if PCM5102B_USE_MCLK
  /* GPIOC: PC7=MCLK (AF6) */
  GPIOC->MODER   &= ~GPIO_MODER_MODER7;   GPIOC->MODER   |=  GPIO_MODER_MODER7_1;
  GPIOC->OTYPER  &= ~GPIO_OTYPER_OT7;
  GPIOC->OSPEEDR |=  GPIO_OSPEEDER_OSPEEDR7_0;
  GPIOC->PUPDR   &= ~GPIO_PUPDR_PUPDR7;
  GPIOC->AFR[0]   = (GPIOC->AFR[0] & ~(0xFu << (7u*4u))) | (6u << (7u*4u));
#endif

  /* NVIC: DMA1 Stream7 IRQ (HT/TC only) */
  uint32_t pg = NVIC_GetPriorityGrouping();
  NVIC_SetPriority(DMA1_Stream7_IRQn, NVIC_EncodePriority(pg, AUDIO3_DMA_IRQ_PRIO, 0));
  NVIC_EnableIRQ(DMA1_Stream7_IRQn);

  /* Reset I2S regs (disable I2S mode + prescaler) */
  SPI3->I2SCFGR = 0u;
  SPI3->I2SPR   = 0u;
}

void i2s3_init_pcm(uint32_t audio_fs_hz)
{
  /* Ensure I2S kernel clock is PLLI2S (not external I2S_CKIN) */
  RCC->CFGR &= ~RCC_CFGR_I2SSRC;

  /* Program PLLI2S for requested Fs (bsp_clock chooses N/R from a table) */
  i2s_pll_set_fs(audio_fs_hz);

  /* Philips I2S, Master TX, 24-in-32 */
  uint32_t cfg = 0u;
  cfg |=  SPI_I2SCFGR_I2SMOD;
  cfg |=  SPI_I2SCFGR_I2SCFG_1;     // Master TX
  cfg &= ~SPI_I2SCFGR_I2SSTD;       // Philips
  cfg &= ~SPI_I2SCFGR_CKPOL;        // normal CK polarity
  cfg |=  SPI_I2SCFGR_DATLEN_1;     // 24-bit samples
  cfg |=  SPI_I2SCFGR_CHLEN;        // 32-bit channel length
  SPI3->I2SCFGR = cfg;

  /* Prescaler:
     - with MCLK: target denom ≈ Fs * 256 (MCLK = 256*Fs)
     - without MCLK: denom ≈ Fs * 64 (BCK = 64*Fs for 32b × 2ch) */
  const uint32_t pllsrc = (RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC) ? 8000000u : 16000000u;
  const uint32_t pllm   = (RCC->PLLCFGR & RCC_PLLCFGR_PLLM) >> RCC_PLLCFGR_PLLM_Pos;
  const uint32_t n      = (RCC->PLLI2SCFGR & RCC_PLLI2SCFGR_PLLI2SN) >> RCC_PLLI2SCFGR_PLLI2SN_Pos;
  const uint32_t r      = (RCC->PLLI2SCFGR & RCC_PLLI2SCFGR_PLLI2SR) >> RCC_PLLI2SCFGR_PLLI2SR_Pos;
  const uint32_t i2sclk = (pllsrc / pllm) * n / r;

#if PCM5102B_USE_MCLK
  const uint64_t denom_target = (uint64_t)audio_fs_hz * 256ull;
#else
  const uint64_t denom_target = (uint64_t)audio_fs_hz * 64ull;
#endif
  uint64_t denom = (i2sclk + (denom_target/2ull)) / denom_target; // rounded

  uint16_t odd = (uint16_t)(denom & 1u);
  uint16_t div = (uint16_t)((denom - odd) / 2u);
  odd <<= 8u;

  /* Guard div range (RM0090: 2..255); clamp odd to 0 if clamped */
  if (div < 2u)         { div = 2u;     odd = 0u; }
  else if (div > 0xFFu) { div = 0xFFu;  odd = 0u; }

#if PCM5102B_USE_MCLK
  SPI3->I2SPR = (div & SPI_I2SPR_I2SDIV) | (odd ? SPI_I2SPR_ODD : 0u) | SPI_I2SPR_MCKOE;
#else
  SPI3->I2SPR = (div & SPI_I2SPR_I2SDIV) | (odd ? SPI_I2SPR_ODD : 0u); // no MCLK
#endif
}

void audio3_dma_start(int32_t *buf, uint32_t bytes_total)
{
  /* Validate arguments: 32-bit stereo frames => multiple of 8 bytes */
  if ((buf == NULL) || ((bytes_total & 0x7u) != 0u) || (bytes_total == 0u)) return;

  DMA_Stream_TypeDef *s = DMA1_Stream7;

  /* Disable/clear before reconfig */
  dma1_stream7_disable_and_clear();

  /* Fixed peripheral address: SPI3->DR, then length in 32b words */
  s->PAR  = (uint32_t)&(SPI3->DR);
  s->M0AR = (uint32_t)buf;
  s->NDTR = (bytes_total >> 2);

  /* mem->periph, 32/32-bit, MINC, circular, HT/TC IRQs */
  s->CR  = 0u;
  s->CR |= (0u << 25);                 // CHSEL=0 (SPI3_TX on F407)
  s->CR |= DMA_SxCR_DIR_0;             // memory-to-peripheral
  s->CR |= DMA_SxCR_MINC | DMA_SxCR_CIRC | DMA_SxCR_HTIE | DMA_SxCR_TCIE;
  s->CR |= DMA_SxCR_MSIZE_1 | DMA_SxCR_PSIZE_1; // 32/32-bit
  s->CR |= DMA_SxCR_PL_1;                        // high priority

  /* FIFO half threshold, direct mode disabled */
  s->FCR = DMA_SxFCR_DMDIS | DMA_SxFCR_FTH_0;

  /* Enable SPI3 TX DMA and start the stream */
  SPI3->CR2 |= SPI_CR2_TXDMAEN;
  s->CR     |= DMA_SxCR_EN;

  /* Ensure I2S engine is enabled (I2SE must be set after prescaler) */
  if ((SPI3->I2SCFGR & SPI_I2SCFGR_I2SE) == 0u) {
    SPI3->I2SCFGR |= SPI_I2SCFGR_I2SE;
  }
}

/* ---------------- IRQ handler --------------------------------------------- */
/* DMA1 Stream7 ISR: acknowledge errors, then fire half/complete callbacks. */
void DMA1_Stream7_IRQHandler(void)
{
  const uint32_t hisr = DMA1->HISR;

  if (hisr & DMA_HISR_TEIF7)  DMA1->HIFCR = DMA_HIFCR_CTEIF7;
  if (hisr & DMA_HISR_DMEIF7) DMA1->HIFCR = DMA_HIFCR_CDMEIF7;
  if (hisr & DMA_HISR_FEIF7)  DMA1->HIFCR = DMA_HIFCR_CFEIF7;

  if (hisr & DMA_HISR_HTIF7) { DMA1->HIFCR = DMA_HIFCR_CHTIF7; audio3_on_half_transfer(); }
  if (hisr & DMA_HISR_TCIF7) { DMA1->HIFCR = DMA_HIFCR_CTCIF7; audio3_on_transfer_complete(); }
}
