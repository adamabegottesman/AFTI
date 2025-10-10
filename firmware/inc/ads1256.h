#ifndef ADS1256_H
#define ADS1256_H

/*
===============================================================================
WIRING — STM32F407 DISC1  <->  ADS1256 (HiLetgo breakout: no /RESET, has SYNC/PDWN)
-------------------------------------------------------------------------------
POWER
  STM32 3V3  ----->  ADS1256 VCC  (use 3.3 V with F407)
  STM32 GND  ----->  ADS1256 GND

SPI1 (AF5 on STM32F407)
  STM32 PA5 (SPI1_SCK)  ----->  ADS1256 SCLK
  STM32 PA7 (SPI1_MOSI) ----->  ADS1256 DIN    (MCU -> ADC)
  STM32 PA6 (SPI1_MISO) ----->  ADS1256 DOUT   (ADC -> MCU)
  STM32 PD8 (GPIO)      ----->  ADS1256 CS     (/CS, active low)

Handshaking / control
  STM32 PB1 (GPIO/EXTI1) ---->  ADS1256 DRDY      (data ready, active low)
  STM32 PD9 (GPIO)       ---->  ADS1256 SYNC/PDWN (HIGH = run, LOW = sync/powerdown)

ANALOG INPUTS (single differential with mid-bias)
  ADS1256 AIN0  <--- Your signal via 1 µF series cap; add 100 kΩ from AIN0 to 2.5 V
  ADS1256 AIN1  <--- 2.5 V reference (same “2.5 V node”)
  - Driver MUX = 0x01 (AINP=AIN0, AINN=AIN1).
  - STATUS = 0x06 (BUFEN=1, ACAL=1, MSB-first).
===============================================================================
*/

#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx.h"
#include "core_cm4.h"   // __NOP()

/* ===== Pin mapping (STM32F407 DISC1 + ADS1256 on SPI1) ===== */
#define ADS1256_SPI                   SPI1
#define ADS1256_SPI_EN()              (RCC->APB2ENR |= RCC_APB2ENR_SPI1EN)

/* SPI1 pins (AF5) on GPIOA */
#define ADS1256_GPIOA                 GPIOA
#define ADS1256_PIN_SCK               5   /* PA5  SCK  */
#define ADS1256_PIN_MISO              6   /* PA6  MISO (ADS->MCU) */
#define ADS1256_PIN_MOSI              7   /* PA7  MOSI (MCU->ADS) */

/* Control/status */
#define ADS1256_GPIOD                 GPIOD
#define ADS1256_PIN_CS                8   /* PD8  /CS  */
#define ADS1256_PIN_PDWN              9   /* PD9  SYNC/PDWN (HIGH = run) */

#define ADS1256_GPIOB                 GPIOB
#define ADS1256_PIN_DRDY              1   /* PB1  DRDY -> EXTI1 */



/* ===== Data rate / PGA ===== */
typedef enum {
  ADS1256_DR_30000SPS = 0xF0,
  ADS1256_DR_15000SPS = 0xE0,
  ADS1256_DR_7500SPS  = 0xD0,
  ADS1256_DR_3750SPS  = 0xC0,
  ADS1256_DR_2000SPS  = 0xB0,
  ADS1256_DR_1000SPS  = 0xA1,
  ADS1256_DR_500SPS   = 0x92,
  ADS1256_DR_100SPS   = 0x82
} ads1256_drate_t;

typedef enum {
  ADS1256_PGA_1 = 0, ADS1256_PGA_2, ADS1256_PGA_4, ADS1256_PGA_8,
  ADS1256_PGA_16, ADS1256_PGA_32, ADS1256_PGA_64
} ads1256_pga_t;

/* ===== Config ===== */
typedef void (*ads1256_sample_cb_t)(int32_t s);

typedef struct {
  uint8_t             mux;         /* e.g. 0x01 = AIN0 (P) – AIN1 (N) */
  ads1256_pga_t       pga;         /* 0..6 */
  ads1256_drate_t     drate;       /* data rate */
  bool                continuous;  /* true => RDATAC + EXTI path */
  ads1256_sample_cb_t on_sample;   /* optional per-sample callback */
} ads1256_cfg_t;

/* ===== Public API ===== */
void  ads1256_init_pins_spi(void);
bool  ads1256_init(const ads1256_cfg_t *cfg);    /* false if SPI/reg ops fail */
void  ads1256_start(void);                       /* DRDY ISR triggers reads */
void  ads1256_stop(void);

bool  ads1256_read_latest(int32_t *out);         /* non-blocking fetch */
void  ads1256_poll_n_samples(uint32_t n);        /* optional blocking poll */

/* Runtime helpers */
void  ads1256_pause_stream(void);
void  ads1256_resume_stream(void);
void  ads1256_set_mux(uint8_t mux);              /* quick MUX change under RDATAC */
bool  ads1256_guard_drate(void);                 /* reassert DRATE if it flips */

/* IRQ glue + simple counters */
void  ads1256_on_drdy_isr(void);
extern volatile uint32_t g_ads_irq_count;
extern volatile uint32_t g_ads_exti_count;



#endif /* ADS1256_H */
