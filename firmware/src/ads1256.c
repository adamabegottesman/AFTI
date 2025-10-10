#include "ads1256.h"
#include "timing.h"
#include "ads1256_dma.h"
#include "core_cm4.h"
#include <stdint.h>

/* ===== SPI timeouts (ms) used during register writes ===== */
#define SPI_TXE_TIMEOUT_MS   1u
#define SPI_RXNE_TIMEOUT_MS  1u
#define SPI_BSY_TIMEOUT_MS   1u

/* ===== ADS1256 commands / registers ===== */
#define CMD_RDATAC   0x03
#define CMD_SDATAC   0x0F
#define CMD_WREG     0x50
#define CMD_SELFCAL  0xF0
#define CMD_RESET    0xFE

#define REG_STATUS   0x00
#define REG_MUX      0x01
#define REG_ADCON    0x02
#define REG_DRATE    0x03

/* ===== Ring buffer (keeps app unchanged) ===== */
#define RB_SIZE 128
static int32_t rb[RB_SIZE];
static volatile uint16_t rb_w = 0, rb_r = 0;

/* ===== Driver state ===== */
static ads1256_cfg_t g_cfg;
volatile uint32_t g_ads_irq_count  = 0;

/* ===== DMA -> ring buffer callback ===== */
static void ads_on_dma_sample(int32_t v)
{
  const uint16_t w = rb_w;
  const uint16_t next_w = (uint16_t)((w + 1) & (RB_SIZE - 1));
  if (next_w != rb_r) { rb[w] = v; rb_w = next_w; }
  g_ads_irq_count++;
}

/* ===== Helpers ===== */
static inline bool wait_drdy_low_timeout(uint32_t timeout_ms)
{
  return (wait_flag_with_timeout(&ADS1256_GPIOB->IDR, ADS1256_PIN_DRDY, 0, timeout_ms) == 0);
}

#ifndef CS_PAD_NOP
#define CS_PAD_NOP 24  /* keep tunable; 0..64 typical */
#endif

static inline void ADS_CS_LOW(void){
  ADS1256_GPIOD->BSRR = (1U << (ADS1256_PIN_CS + 16));
  for (volatile int i=0;i<CS_PAD_NOP;i++) __NOP();
}
static inline void ADS_CS_HIGH(void){
  ADS1256_GPIOD->BSRR = (1U << ADS1256_PIN_CS);
  for (volatile int i=0;i<CS_PAD_NOP;i++) __NOP();
}

/* SPI: MODE1, /32 (~2.625 MHz @84 MHz APB2) */
static void spi1_init_mode1_fast(void){
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
  GPIOA->OSPEEDR |= (3u << (ADS1256_PIN_SCK*2))
                  | (3u << (ADS1256_PIN_MISO*2))
                  | (3u << (ADS1256_PIN_MOSI*2));
  RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

  SPI1->CR1 = 0; SPI1->CR2 = 0;
  SPI1->CR1 &= ~SPI_CR1_SPE;

  RCC->APB2RSTR |=  RCC_APB2RSTR_SPI1RST;
  RCC->APB2RSTR &= ~RCC_APB2RSTR_SPI1RST;

  SPI1->CR1 =  SPI_CR1_MSTR
             | SPI_CR1_SSM | SPI_CR1_SSI
             | SPI_CR1_CPHA               /* CPOL=0, CPHA=1 */
             | (SPI_CR1_BR_2);            /* /32 */
  SPI1->CR1 &= ~SPI_CR1_LSBFIRST;
  SPI1->CR1 &= ~SPI_CR1_DFF;
  SPI1->CR1 |= SPI_CR1_SPE;
}

static inline uint8_t spi_txrx(uint8_t v)
{
  if (wait_flag_with_timeout(&SPI1->SR, 1 /*TXE*/, 1, SPI_TXE_TIMEOUT_MS)) return 0xEE;
  *(volatile uint8_t*)&SPI1->DR = v;
  if (wait_flag_with_timeout(&SPI1->SR, 0 /*RXNE*/, 1, SPI_RXNE_TIMEOUT_MS)) return 0xEF;
  uint8_t r = *(volatile uint8_t*)&SPI1->DR;
  (void)wait_flag_with_timeout(&SPI1->SR, 7 /*BSY*/, 0, SPI_BSY_TIMEOUT_MS);
  return r;
}

static inline bool ads_wreg(uint8_t addr, uint8_t val){
  (void)wait_drdy_low_timeout(5);
  ADS_CS_LOW();
  (void)spi_txrx(CMD_WREG | (addr & 0x0F));
  (void)spi_txrx(0x00);   /* write 1 register */
  (void)spi_txrx(val);
  ADS_CS_HIGH();
  return true;
}

/* ===== pins/SPI ===== */
void ads1256_init_pins_spi(void)
{
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIODEN | RCC_AHB1ENR_GPIOBEN;

  /* SPI1 AF5 */
  ADS1256_GPIOA->MODER &= ~(3u << (ADS1256_PIN_SCK*2));
  ADS1256_GPIOA->MODER |=  (2u << (ADS1256_PIN_SCK*2));
  ADS1256_GPIOA->AFR[0] &= ~(0xFu << (ADS1256_PIN_SCK*4));
  ADS1256_GPIOA->AFR[0] |=  (5u   << (ADS1256_PIN_SCK*4));

  ADS1256_GPIOA->MODER &= ~(3u << (ADS1256_PIN_MISO*2));
  ADS1256_GPIOA->MODER |=  (2u << (ADS1256_PIN_MISO*2));
  ADS1256_GPIOA->AFR[0] &= ~(0xFu << (ADS1256_PIN_MISO*4));
  ADS1256_GPIOA->AFR[0] |=  (5u   << (ADS1256_PIN_MISO*4));

  ADS1256_GPIOA->MODER &= ~(3u << (ADS1256_PIN_MOSI*2));
  ADS1256_GPIOA->MODER |=  (2u << (ADS1256_PIN_MOSI*2));
  ADS1256_GPIOA->AFR[0] &= ~(0xFu << (ADS1256_PIN_MOSI*4));
  ADS1256_GPIOA->AFR[0] |=  (5u   << (ADS1256_PIN_MOSI*4));

  /* CS */
  ADS1256_GPIOD->MODER  &= ~(3u << (ADS1256_PIN_CS*2));
  ADS1256_GPIOD->MODER  |=  (1u << (ADS1256_PIN_CS*2));
  ADS1256_GPIOD->BSRR    = (1U << ADS1256_PIN_CS);
  ADS1256_GPIOD->OSPEEDR &= ~(3u << (ADS1256_PIN_CS*2));
  ADS1256_GPIOD->OSPEEDR |=  (3u << (ADS1256_PIN_CS*2));   /* very high speed */

  /* PDWN high */
  ADS1256_GPIOD->MODER &= ~(3u << (ADS1256_PIN_PDWN*2));
  ADS1256_GPIOD->MODER |=  (1u << (ADS1256_PIN_PDWN*2));
  ADS1256_GPIOD->BSRR   = (1U << ADS1256_PIN_PDWN);

  /* DRDY input + EXTI1 (pull-up) */
  ADS1256_GPIOB->MODER &= ~(3u << (ADS1256_PIN_DRDY*2));
  ADS1256_GPIOB->PUPDR &= ~(3u << (ADS1256_PIN_DRDY*2));
  ADS1256_GPIOB->PUPDR |=  (1u << (ADS1256_PIN_DRDY*2));

  RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
  SYSCFG->EXTICR[0] &= ~(0xFu << 4);
  SYSCFG->EXTICR[0] |=  (0x1u << 4);    /* PB1 -> EXTI1 */
  EXTI->RTSR &= ~(1u << 1);
  EXTI->FTSR |=  (1u << 1);             /* falling edge */
  EXTI->IMR  &= ~(1u << 1);
  NVIC_DisableIRQ(EXTI1_IRQn);

  /* SPI */
  spi1_init_mode1_fast();
}

/* ===== init (pruned) ===== */
bool ads1256_init(const ads1256_cfg_t *cfg)
{
  g_cfg = *cfg;

  /* Wake & reset */
  ADS1256_GPIOD->BSRR = (1U << ADS1256_PIN_PDWN);  /* SYNC/PDWN HIGH = run */
  for (volatile int i=0;i<8000;i++) __NOP();       /* short analog settle */

  ADS_CS_LOW(); (void)spi_txrx(CMD_RESET); ADS_CS_HIGH();
  (void)wait_drdy_low_timeout(5);

  /* Leave RDATAC before touching regs */
  ADS_CS_LOW(); (void)spi_txrx(CMD_SDATAC); ADS_CS_HIGH();

  /* Program regs */
  if (!ads_wreg(REG_STATUS, 0x06)) return false;                   /* MSB, ACAL=1, BUFEN=1 */
  if (!ads_wreg(REG_MUX,    g_cfg.mux)) return false;              /* channel */
  if (!ads_wreg(REG_ADCON, (0x00 | (g_cfg.pga & 0x07)))) return false; /* CLKOUT off, PGA */
  if (!ads_wreg(REG_DRATE, (uint8_t)g_cfg.drate)) return false;    /* data rate */

  /* Self-cal and wait */
  ADS_CS_LOW(); (void)spi_txrx(CMD_SELFCAL); ADS_CS_HIGH();
  (void)wait_drdy_low_timeout(10);

  /* Hook DMA callback before streaming */
  ads1256_dma_init(ads_on_dma_sample);

  /* Arm EXTI on DRDY */
  EXTI->PR  = (1u << 1);
  NVIC_ClearPendingIRQ(EXTI1_IRQn);
  NVIC_EnableIRQ(EXTI1_IRQn);

  /* Enter RDATAC, keep CS low */
  ADS_CS_LOW();
  (void)spi_txrx(CMD_RDATAC);
  EXTI->PR  = (1u << 1);
  EXTI->IMR |= (1u << 1);

  return true;
}

/* quick MUX change under RDATAC (leave->write->enter) */
void ads1256_set_mux(uint8_t mux)
{
  (void)spi_txrx(CMD_SDATAC);
  ADS_CS_HIGH();
  (void)wait_drdy_low_timeout(1);

  ADS_CS_LOW();
  (void)spi_txrx(CMD_WREG | REG_MUX);
  (void)spi_txrx(0x00);
  (void)spi_txrx(mux);
  ADS_CS_HIGH();
  (void)wait_drdy_low_timeout(1);

  ADS_CS_LOW();
  (void)spi_txrx(CMD_RDATAC);
  EXTI->PR = (1u << 1);
}

/* ===== EXTI ISR: mask EXTI, micro-pad, kick DMA ===== */
void ads1256_on_drdy_isr(void)
{
  EXTI->PR  = (1u << 1);
  EXTI->IMR &= ~(1u << 1);

  if (ads1256_dma_is_busy()) return;

  extern void ads1256_dma_drdy2cs_pad(void);
  ads1256_dma_drdy2cs_pad();

  ads1256_dma_kick();
}

/* Non-blocking fetch */
bool ads1256_read_latest(int32_t *out)
{
  if (rb_r == rb_w) return false;
  *out = rb[rb_r];
  rb_r = (rb_r + 1) & (RB_SIZE - 1);
  return true;
}

/* Vector */
void EXTI1_IRQHandler(void) { ads1256_on_drdy_isr(); }
