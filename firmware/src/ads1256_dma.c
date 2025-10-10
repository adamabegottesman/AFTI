#include "ads1256_dma.h"
#include "ads1256.h"      // pin macros/ports, SPI1, EXTI
#include "core_cm4.h"
#include <stdbool.h>

/* Some STM32F4 headers don't define FRXTH; it's CR2 bit 12 */
#ifndef SPI_CR2_FRXTH
#define SPI_CR2_FRXTH (1U << 12)
#endif

#ifndef CS_PAD_NOP
#define CS_PAD_NOP 24
#endif

/* ===== micro-timing knobs (NOPs @168 MHz; tune 0,8,16,24,32,...) ===== */
#ifndef DRDY2CS_NOP
#define DRDY2CS_NOP        16    /* DRDY edge -> CS low */
#endif
#ifndef CS2SCK_NOP
#define CS2SCK_NOP         24    /* CS low  -> first SCK */
#endif
#ifndef LASTSCK2CSH_NOP
#define LASTSCK2CSH_NOP    8     /* last SCK -> CS high */
#endif

static inline void spin_nops(unsigned n){ while(n--) __NOP(); }

/* Called by EXTI ISR to pad between DRDY edge and CS low */
void ads1256_dma_drdy2cs_pad(void) { spin_nops(DRDY2CS_NOP); }

/* ---- Static state ---- */
static volatile uint8_t s_busy = 0;
static ads1256_dma_sample_cb_t s_cb = 0;

/* Make RX buffer volatile so compiler never caches DMA-written bytes */
static volatile uint8_t s_rx3[3];          /* 3-byte RX buffer */
static const uint8_t    s_tx_dummy[3] = {0,0,0};  /* 3 dummy bytes for TX clocks */

bool ads1256_dma_is_busy(void) { return s_busy != 0; }

/* ---- Small helpers ---- */
static inline void dma_stream_disable_and_wait(DMA_Stream_TypeDef *s){
  if (s->CR & DMA_SxCR_EN){ s->CR &= ~DMA_SxCR_EN; while (s->CR & DMA_SxCR_EN) { __NOP(); } }
}
static inline void clear_rx_flags(void){
  /* RX Stream0 -> LOW bank */
  DMA2->LIFCR = DMA_LIFCR_CFEIF0 | DMA_LIFCR_CDMEIF0 |
                DMA_LIFCR_CTEIF0 | DMA_LIFCR_CHTIF0 | DMA_LIFCR_CTCIF0;
}
static inline void clear_tx_flags(void){
  /* TX Stream5 -> HIGH bank */
  DMA2->HIFCR = DMA_HIFCR_CFEIF5 | DMA_HIFCR_CDMEIF5 |
                DMA_HIFCR_CTEIF5 | DMA_HIFCR_CHTIF5 | DMA_HIFCR_CTCIF5;
}

/* ---- Public API ---- */
void ads1256_dma_init(ads1256_dma_sample_cb_t cb)
{
  s_cb = cb;

  RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;

  dma_stream_disable_and_wait(DMA2_Stream0);  /* RX */
  dma_stream_disable_and_wait(DMA2_Stream5);  /* TX */
  clear_rx_flags();
  clear_tx_flags();

  /* RX: SPI1->DR -> s_rx3 (P->M, MINC, TCIE), Channel 3, HIGH prio */
  DMA2_Stream0->CR   = 0;
  DMA2_Stream0->NDTR = 0;
  DMA2_Stream0->PAR  = (uint32_t)&SPI1->DR;
  DMA2_Stream0->M0AR = (uint32_t)s_rx3;
  DMA2_Stream0->CR = (3u << DMA_SxCR_CHSEL_Pos)  /* Channel 3 = SPI1 */
                   | DMA_SxCR_MINC
                   | (1u << DMA_SxCR_PL_Pos)     /* PL=10b HIGH */
                   | DMA_SxCR_TCIE;              /* RX TC IRQ */
  DMA2_Stream0->FCR = 0;                         /* direct mode */

  /* TX: s_tx_dummy[3] -> SPI1->DR (M->P, MINC), Channel 3, HIGH prio */
  DMA2_Stream5->CR   = 0;
  DMA2_Stream5->NDTR = 0;
  DMA2_Stream5->PAR  = (uint32_t)&SPI1->DR;
  DMA2_Stream5->M0AR = (uint32_t)s_tx_dummy;
  DMA2_Stream5->CR = (3u << DMA_SxCR_CHSEL_Pos)
                   | DMA_SxCR_DIR_0              /* 01 = M2P */
                   | DMA_SxCR_MINC
                   | (1u << DMA_SxCR_PL_Pos);    /* PL=10b HIGH */
  DMA2_Stream5->FCR = 0;

  /* NVIC for RX stream0 */
  NVIC_SetPriority(DMA2_Stream0_IRQn, 1);
  NVIC_EnableIRQ(DMA2_Stream0_IRQn);

  /* Ensure SPI1 RX threshold = 8-bit for DMA, requests off initially */
  SPI1->CR2 |= SPI_CR2_FRXTH;
  SPI1->CR2 &= ~(SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN);

  s_busy = 0;
}

/* Called from EXTI DRDY ISR: starts one framed 3-byte read with CS pulse */
void ads1256_dma_kick(void)
{
  if (s_busy) return;
  s_busy = 1;

  /* Clean state */
  dma_stream_disable_and_wait(DMA2_Stream0);   /* RX */
  dma_stream_disable_and_wait(DMA2_Stream5);   /* TX */
  clear_rx_flags();
  clear_tx_flags();

  /* Program counts: exactly 3 bytes in/out */
  DMA2_Stream0->M0AR = (uint32_t)s_rx3;
  DMA2_Stream0->NDTR = 3;
  DMA2_Stream5->NDTR = 3;

  /* Make sure SPI is idle and RX FIFO empty (avoid overrun) */
  while (SPI1->SR & SPI_SR_BSY) { }
  while (SPI1->SR & SPI_SR_RXNE) { (void)*(volatile uint8_t*)&SPI1->DR; }
  { volatile uint16_t dr = SPI1->DR; (void)dr; volatile uint16_t sr = SPI1->SR; (void)sr; }

  /* Gate off DMA requests until both streams are armed */
  SPI1->CR2 &= ~(SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN);

  /* Frame begin: select ADC */
  ADS1256_GPIOD->BSRR = (1U << (ADS1256_PIN_CS + 16));

  /* CS -> first SCK pad (t11/t6) */
  spin_nops(CS2SCK_NOP);

  /* Arm RX first (to catch first byte), then TX */
  DMA2_Stream0->CR |= DMA_SxCR_EN;   /* RX */
  __DSB(); __ISB();
  spin_nops(4);                      /* tiny arm pad */
  DMA2_Stream5->CR |= DMA_SxCR_EN;   /* TX */
  __DSB(); __ISB();

  /* Allow SPI to issue DMA requests (TXE fires, SCK starts) */
  SPI1->CR2 |= (SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN);
}

/* RX transfer-complete ISR */
void ads1256_dma_irq_rx(void)
{
  /* TC for Stream0 -> LOW bank */
  if (DMA2->LISR & DMA_LISR_TCIF0) {
    DMA2->LIFCR = DMA_LIFCR_CTCIF0;
  }
  /* Clear possible RX errors */
  if (DMA2->LISR & (DMA_LISR_TEIF0 | DMA_LISR_DMEIF0 | DMA_LISR_FEIF0 | DMA_LISR_HTIF0)) {
    DMA2->LIFCR = (DMA_LIFCR_CTEIF0 | DMA_LIFCR_CDMEIF0 | DMA_LIFCR_CFEIF0 | DMA_LIFCR_CHTIF0);
  }

  /* Stop streams and requests */
  dma_stream_disable_and_wait(DMA2_Stream0);
  dma_stream_disable_and_wait(DMA2_Stream5);
  SPI1->CR2 &= ~(SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN);

  /* tiny hold after last SCK, then deassert CS (with your post-hold) */
  spin_nops(LASTSCK2CSH_NOP);
  ADS1256_GPIOD->BSRR = (1U << ADS1256_PIN_CS);
  for (volatile int i=0;i<CS_PAD_NOP;i++) __NOP();

  /* Ensure DMA writes are visible before we pack the sample */
  __DSB();

  /* Pack 24-bit sample (MSB-first) */
  uint8_t b0 = s_rx3[0], b1 = s_rx3[1], b2 = s_rx3[2];
  int32_t v  = ((int32_t)b0 << 16) | ((int32_t)b1 << 8) | (int32_t)b2;
  if (v & 0x00800000) v |= 0xFF000000;

  if (s_cb) s_cb(v);

  s_busy = 0;

  /* Re-arm EXTI1 for the next DRDY edge */
  EXTI->PR  = (1u << 1);
  EXTI->IMR |= (1u << 1);
}

/* IRQ wrapper so the vector table can call into our driver */
void DMA2_Stream0_IRQHandler(void)
{
  ads1256_dma_irq_rx();
}
