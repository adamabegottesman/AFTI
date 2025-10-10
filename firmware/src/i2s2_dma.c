// i2s2_dma.c — minimal I2S2 (SPI2) + DMA1 Stream4 driver for PCM5102
// Pins (AF5): PB12=WS (LRCK), PB13=CK (BCLK), PB15=SD (DAC DIN), optional PC6=MCLK
// Format: Philips I2S, Master TX, 24-in-32, stereo interleaved (L,R,...)

#include <dds_tone_i2s2.h>      // audio2_on_* ISR callbacks (default DDS fillers)
#include "i2s2_dma.h"
#include "stm32f407xx.h"
#include "bsp_clock.h"          // i2s_pll_set_fs()
#include <stdint.h>
#include <stddef.h>

/* ---------------- Build-time options -------------------------------------- */
#ifndef PCM5102_USE_MCLK
#define PCM5102_USE_MCLK 1   // Set to 0 if MCLK is not wired to the PCM5102
#endif

#ifndef AUDIO2_DMA_IRQ_PRIO
#define AUDIO2_DMA_IRQ_PRIO 14
#endif

/* ---------------- Optional dynamic filler selection (tiny hook) ----------- */
/* If set, ISR will call these instead of the default audio2_on_* DDS fillers. */
typedef void (*i2s_cb_t)(void);
static i2s_cb_t s_i2s2_cb_half = 0;
static i2s_cb_t s_i2s2_cb_full = 0;

/* Exposed so main/app can switch to ADC monitor (or back to DDS with NULLs). */
void audio2_set_callbacks(i2s_cb_t on_half, i2s_cb_t on_full)
{
  s_i2s2_cb_half = on_half;
  s_i2s2_cb_full = on_full;
}

/* ---------------- Local helpers ------------------------------------------- */

static void dma1_stream4_disable_and_clear(void)
{
  DMA_Stream_TypeDef *s = DMA1_Stream4;

  /* Disable if running, then wait for EN to clear (RM0090-recommended) */
  if (s->CR & DMA_SxCR_EN) {
    s->CR &= ~DMA_SxCR_EN;
    while (s->CR & DMA_SxCR_EN) { /* wait */ }
  }

  /* Clear all Stream4 flags */
  DMA1->HIFCR = DMA_HIFCR_CFEIF4 | DMA_HIFCR_CDMEIF4 | DMA_HIFCR_CTEIF4
              | DMA_HIFCR_CHTIF4 | DMA_HIFCR_CTCIF4;
}

/* ---------------- Public API ---------------------------------------------- */

void dma_i2s2_init(void)
{
  /* Clocks: SPI2 + GPIOB (+GPIOC if MCLK) + DMA1 */
  RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_DMA1EN;
#if PCM5102_USE_MCLK
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
#endif
  (void)RCC->AHB1ENR;

  /* GPIOB: PB12=WS, PB13=CK, PB15=SD (AF5) */
  // PB12 WS
  GPIOB->MODER   &= ~GPIO_MODER_MODER12; GPIOB->MODER   |=  GPIO_MODER_MODER12_1;
  GPIOB->OTYPER  &= ~GPIO_OTYPER_OT12;
  GPIOB->OSPEEDR |=  GPIO_OSPEEDER_OSPEEDR12_0;  /* medium/high speed */
  GPIOB->PUPDR   &= ~GPIO_PUPDR_PUPDR12;
  GPIOB->AFR[1]   = (GPIOB->AFR[1] & ~(0xFu << ((12u-8u)*4u))) | (5u << ((12u-8u)*4u));

  // PB13 CK
  GPIOB->MODER   &= ~GPIO_MODER_MODER13; GPIOB->MODER   |=  GPIO_MODER_MODER13_1;
  GPIOB->OTYPER  &= ~GPIO_OTYPER_OT13;
  GPIOB->OSPEEDR |=  GPIO_OSPEEDER_OSPEEDR13_0;
  GPIOB->PUPDR   &= ~GPIO_PUPDR_PUPDR13;
  GPIOB->AFR[1]   = (GPIOB->AFR[1] & ~(0xFu << ((13u-8u)*4u))) | (5u << ((13u-8u)*4u));

  // PB15 SD
  GPIOB->MODER   &= ~GPIO_MODER_MODER15; GPIOB->MODER   |=  GPIO_MODER_MODER15_1;
  GPIOB->OTYPER  &= ~GPIO_OTYPER_OT15;
  GPIOB->OSPEEDR |=  GPIO_OSPEEDER_OSPEEDR15_0;
  GPIOB->PUPDR   &= ~GPIO_PUPDR_PUPDR15;
  GPIOB->AFR[1]   = (GPIOB->AFR[1] & ~(0xFu << ((15u-8u)*4u))) | (5u << ((15u-8u)*4u));

#if PCM5102_USE_MCLK
  /* GPIOC: PC6=MCLK (AF5) */
  GPIOC->MODER   &= ~GPIO_MODER_MODER6;  GPIOC->MODER   |=  GPIO_MODER_MODER6_1;
  GPIOC->OTYPER  &= ~GPIO_OTYPER_OT6;
  GPIOC->OSPEEDR |=  GPIO_OSPEEDER_OSPEEDR6_0;
  GPIOC->PUPDR   &= ~GPIO_PUPDR_PUPDR6;
  GPIOC->AFR[0]   = (GPIOC->AFR[0] & ~(0xFu << (6u*4u))) | (5u << (6u*4u));
#endif

  /* NVIC: DMA1 Stream4 IRQ (HT/TC only) */
  uint32_t pg = NVIC_GetPriorityGrouping();
  NVIC_SetPriority(DMA1_Stream4_IRQn, NVIC_EncodePriority(pg, AUDIO2_DMA_IRQ_PRIO, 0));
  NVIC_EnableIRQ(DMA1_Stream4_IRQn);

  /* Reset I2S regs (disable I2S mode + prescaler) */
  SPI2->I2SCFGR = 0u;
  SPI2->I2SPR   = 0u;
}

void i2s2_init(uint32_t audio_fs_hz)
{
  /* Ensure I2S clock source is PLLI2S (not external I2S_CKIN) */
  RCC->CFGR &= ~RCC_CFGR_I2SSRC;

  /* Program PLLI2S for requested Fs (bsp_clock chooses N/R from a table) */
  i2s_pll_set_fs(audio_fs_hz);

  /* Philips I2S, Master TX, 24-bit data, 32-bit channel length */
  uint32_t cfg = 0u;
  cfg |=  SPI_I2SCFGR_I2SMOD;          // I2S mode
  cfg |=  SPI_I2SCFGR_I2SCFG_1;        // Master TX
  cfg &= ~SPI_I2SCFGR_I2SSTD;          // Philips
  cfg &= ~SPI_I2SCFGR_CKPOL;           // normal polarity
  cfg |=  SPI_I2SCFGR_DATLEN_1;        // 24-bit data
  cfg |=  SPI_I2SCFGR_CHLEN;           // 32-bit channel length
  SPI2->I2SCFGR = cfg;

  /* Prescaler:
     - with MCLK: target denom ≈ Fs * 256 (MCLK = 256*Fs)
     - without MCLK: denom ≈ Fs * 64 (BCK = 64*Fs for 32b × 2ch) */
  const uint32_t pllsrc = (RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC) ? 8000000u : 16000000u;
  const uint32_t pllm   = (RCC->PLLCFGR & RCC_PLLCFGR_PLLM) >> RCC_PLLCFGR_PLLM_Pos;
  const uint32_t n      = (RCC->PLLI2SCFGR & RCC_PLLI2SCFGR_PLLI2SN) >> RCC_PLLI2SCFGR_PLLI2SN_Pos;
  const uint32_t r      = (RCC->PLLI2SCFGR & RCC_PLLI2SCFGR_PLLI2SR) >> RCC_PLLI2SCFGR_PLLI2SR_Pos;
  const uint32_t i2sclk = (pllsrc / pllm) * n / r;

#if PCM5102_USE_MCLK
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

#if PCM5102_USE_MCLK
  SPI2->I2SPR = (div & SPI_I2SPR_I2SDIV) | (odd ? SPI_I2SPR_ODD : 0u) | SPI_I2SPR_MCKOE;
#else
  SPI2->I2SPR = (div & SPI_I2SPR_I2SDIV) | (odd ? SPI_I2SPR_ODD : 0u); // MCKOE off
#endif
}

void audio2_dma_start(int32_t *buf, uint32_t bytes_total)
{
  /* Validate arguments: 32-bit stereo frames => multiple of 8 bytes */
  if ((buf == NULL) || ((bytes_total & 0x7u) != 0u)) return;

  DMA_Stream_TypeDef *s = DMA1_Stream4;

  /* Disable/clear before reconfig */
  dma1_stream4_disable_and_clear();

  /* Fixed peripheral address: SPI2->DR */
  s->PAR  = (uint32_t)&(SPI2->DR);
  s->M0AR = (uint32_t)buf;
  s->NDTR = (bytes_total >> 2);            // number of 32-bit words

  /* mem->periph, 32/32-bit, MINC, circular, HT/TC IRQs */
  s->CR  = 0u;
  s->CR |= (0u << 25);                     // CHSEL=0 (SPI2_TX on F407)
  s->CR |= DMA_SxCR_DIR_0;                 // memory-to-peripheral
  s->CR |= DMA_SxCR_MINC | DMA_SxCR_CIRC | DMA_SxCR_HTIE | DMA_SxCR_TCIE;
  s->CR |= DMA_SxCR_MSIZE_1 | DMA_SxCR_PSIZE_1;   // 32-bit source & dest
  s->CR |= DMA_SxCR_PL_1;                          // high priority
  s->CR &= ~DMA_SxCR_DBM;                          // single buffer (no DBM)

  /* FIFO: half threshold, direct mode disabled (better bus utilization) */
  s->FCR = DMA_SxFCR_DMDIS | DMA_SxFCR_FTH_0;

  /* Enable SPI2 TX DMA, then start the stream */
  SPI2->CR2 |= SPI_CR2_TXDMAEN;
  s->CR     |= DMA_SxCR_EN;

  /* Ensure I2S engine is enabled (I2SE must be set after prescaler) */
  if ((SPI2->I2SCFGR & SPI_I2SCFGR_I2SE) == 0u) {
    SPI2->I2SCFGR |= SPI_I2SCFGR_I2SE;
  }
}

/* ---------------- IRQ handler --------------------------------------------- */
/* DMA1 Stream4 ISR: acknowledge errors, then fire half/complete callbacks. */
void DMA1_Stream4_IRQHandler(void)
{
  const uint32_t hisr = DMA1->HISR;

  if (hisr & DMA_HISR_TEIF4)  DMA1->HIFCR = DMA_HIFCR_CTEIF4;
  if (hisr & DMA_HISR_DMEIF4) DMA1->HIFCR = DMA_HIFCR_CDMEIF4;
  if (hisr & DMA_HISR_FEIF4)  DMA1->HIFCR = DMA_HIFCR_CFEIF4;

  if (hisr & DMA_HISR_HTIF4) {
    DMA1->HIFCR = DMA_HIFCR_CHTIF4;
    if (s_i2s2_cb_half) s_i2s2_cb_half();   // ADC monitor (if installed)
    else                audio2_on_half_transfer();  // default DDS filler
  }

  if (hisr & DMA_HISR_TCIF4) {
    DMA1->HIFCR = DMA_HIFCR_CTCIF4;
    if (s_i2s2_cb_full) s_i2s2_cb_full();   // ADC monitor (if installed)
    else                audio2_on_transfer_complete();
  }
}
