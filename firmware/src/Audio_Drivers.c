// Bare-metal audio driver for STM32F4-DISC1 (CS43L22 codec), CMSIS only.
// PUBLIC API: see Audio_Drivers.h
//
// IMPORTANT: Behavior is preserved, but DMA now runs in true double-buffer mode.
// - SysTick provides 1 kHz timing (replaces the old TIM11 one-shot). See timing.c.
// - I2C read sequence retains the device-specific write of (MemAddress | 0x01).
//
// Quick start (call order)
// 1) myAudioSpeedUpTheSystemClock();   // 168 MHz + seed PLLI2S
// 2) initAudioTimer();                 // SysTick 1 kHz (call AFTER clocks settle)
// 3) myAudioInitialisePeripherals(out_dev, volume, fs);
//    - I2C1 pins + 100 kHz timing
//    - Codec reset pulse on PD4, probe ID (0xE0), register init
//    - I2S3 pins (PA4/PC7/PC10/PC12), I2S3 master-tx divider and MCK
//    - DMA1 Stream7 mem->SPI3->DR, IRQs, **double-buffer circular**
// 4) myAudioStartPlaying(buf, len_bytes);
//    - Starts DMA ping-pong (buf split into halves), enables I2S + TXDMA
// 5) DMA1_Stream7_IRQHandler()
//    - Uses CT bit to call audio_tx_half_isr() / audio_tx_complete_isr()

#include "./Audio_Drivers.h"
#include "audio.h"
#include "./cs43l22.h"
#include "timing.h"
#include <stddef.h>   // NULL

#define myUNUSED(X) (void)(X)

/* Addressing note (wire addresses):
 * - Project uses 8-bit I2C addresses (write=0x94 / read=0x95).
 * - If you switch to 7-bit, use 0x4A and set R/W in the transaction.
 */
#define SPI3_DR_ADDR                ((uint32_t)&(SPI3->DR))
#define BYTES_PER_SAMPLE_16         2U
#define HALF_BYTES(total_bytes)     ((total_bytes) / 2U)              // bytes per half
#define HALFWORDS_PER_HALF(bytes)   (HALF_BYTES(bytes) / BYTES_PER_SAMPLE_16)
#define CS43L22_EXPECTED_ID         0xE0U
#define externalClockFrequency      8000000U  // HSE = 8 MHz on DISC1

// Public status flags (unchanged API)
MY_AUDIO_StatusTypeDef audioI2SStatus = MY_AUDIO_RESET;
MY_AUDIO_StatusTypeDef audioDMAStatus = MY_AUDIO_RESET;
MY_AUDIO_StatusTypeDef audioI2CStatus = MY_AUDIO_RESET;
MY_AUDIO_StatusTypeDef audioDACStatus = MY_AUDIO_RESET;

// Aliases to the peripherals we use
I2C_TypeDef *AudioI2C = (I2C_TypeDef *) I2C1_BASE;
SPI_TypeDef *AudioI2S = (SPI_TypeDef *) SPI3_BASE;

/////////////////////////////////////////////////////////////////////////////////////
// PLL and system clocking (168 MHz SYSCLK from HSE=8 MHz; seed PLLI2S)

MY_AUDIO_StatusTypeDef myConfigureI2SClock(uint32_t newN, uint32_t newR)
{
  RCC->CR &= ~RCC_CR_PLLI2SON;
  if (waitForFlagWithTimeout(&RCC->CR, RCC_CR_PLLI2SON_Pos, 0, 100)) return MY_AUDIO_TIMEOUT;

  RCC->PLLI2SCFGR = (newN << RCC_PLLI2SCFGR_PLLI2SN_Pos) | (newR << RCC_PLLI2SCFGR_PLLI2SR_Pos);

  RCC->CR |= RCC_CR_PLLI2SON;
  if (waitForFlagWithTimeout(&RCC->CR, RCC_CR_PLLI2SRDY_Pos, 1, 100)) return MY_AUDIO_TIMEOUT;

  return MY_AUDIO_OK;
}

MY_AUDIO_StatusTypeDef myConfigureTheMainClockPLL(void)
{
  RCC->CR |= RCC_CR_HSEON;
  if (waitForFlagWithTimeout(&RCC->CR, RCC_CR_HSERDY_Pos, 1, 100)) return MY_AUDIO_TIMEOUT;

  RCC->CR &= ~RCC_CR_PLLON;
  if (waitForFlagWithTimeout(&RCC->CR, RCC_CR_PLLRDY_Pos, 0, 100)) return MY_AUDIO_TIMEOUT;

  uint32_t cfg = 0;
  cfg |= (8U   << RCC_PLLCFGR_PLLM_Pos);
  cfg |= (336U << RCC_PLLCFGR_PLLN_Pos);
  cfg |= (((2U >> 1U) - 1U) << RCC_PLLCFGR_PLLP_Pos); // P=2 -> 0
  cfg |= (7U   << RCC_PLLCFGR_PLLQ_Pos);
  cfg |= RCC_PLLCFGR_PLLSRC_HSE;
  RCC->PLLCFGR = cfg;

  RCC->CR |= RCC_CR_PLLON;
  if (waitForFlagWithTimeout(&RCC->CR, RCC_CR_PLLRDY_Pos, 1, 100)) return MY_AUDIO_TIMEOUT;

  return MY_AUDIO_OK;
}

MY_AUDIO_StatusTypeDef myConfigureFlashWaitStatesAndBusClocks(void)
{
  // Flash 5WS (168 MHz)
  if (FLASH_ACR_LATENCY_5WS > (FLASH->ACR & FLASH_ACR_LATENCY))
  {
    FLASH->ACR = (FLASH->ACR & ~FLASH_ACR_LATENCY) | FLASH_ACR_LATENCY_5WS;
    if (FLASH_ACR_LATENCY_5WS != (FLASH->ACR & FLASH_ACR_LATENCY)) return MY_AUDIO_ERROR;
  }

  // Safe prescalers before switch
  RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_PPRE1) | RCC_CFGR_PPRE1_DIV16;
  RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_PPRE2) | RCC_CFGR_PPRE2_DIV16;
  RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_HPRE)  | RCC_CFGR_HPRE_DIV1;

  if ((RCC->CR & RCC_CR_PLLRDY) == 0) return MY_AUDIO_ERROR;

  // SYSCLK <- PLL
  RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_PLL;
  if (waitForFlagWithTimeout(&RCC->CFGR, 3U, 1U, 100)) return MY_AUDIO_TIMEOUT;

  // Final prescalers: APB1=42 MHz, APB2=84 MHz
  RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_PPRE1) | RCC_CFGR_PPRE1_DIV4;
  RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_PPRE2) | RCC_CFGR_PPRE2_DIV2;

  // Recompute SystemCoreClock
  uint32_t pllm   = RCC->PLLCFGR & RCC_PLLCFGR_PLLM;
  uint32_t pllp   = ((((RCC->PLLCFGR & RCC_PLLCFGR_PLLP) >> RCC_PLLCFGR_PLLP_Pos) + 1U) * 2U);
  uint32_t pllsrc = (RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC) ? 8000000U : 16000000U;
  uint32_t pllvco = (uint32_t)(((uint64_t)pllsrc * ((RCC->PLLCFGR & RCC_PLLCFGR_PLLN) >> RCC_PLLCFGR_PLLN_Pos)) / (uint64_t)pllm);
  uint32_t sysclk = pllvco / pllp;
  SystemCoreClock = sysclk >> AHBPrescTable[(RCC->CFGR & RCC_CFGR_HPRE) >> RCC_CFGR_HPRE_Pos];

  return MY_AUDIO_OK;
}

MY_AUDIO_StatusTypeDef myAudioSpeedUpTheSystemClock(void)
{
  // Scale 1
  RCC->APB1ENR |= RCC_APB1ENR_PWREN;
  while ((RCC->APB1ENR & RCC_APB1ENR_PWREN) == 0) {}
  PWR->CR = (PWR->CR & ~PWR_CR_VOS) | PWR_CR_VOS;

  if (myConfigureTheMainClockPLL()             != MY_AUDIO_OK) return MY_AUDIO_ERROR;
  if (myConfigureFlashWaitStatesAndBusClocks() != MY_AUDIO_OK) return MY_AUDIO_ERROR;

  // Seed PLLI2S (exact Fs set later)
  if (myConfigureI2SClock(192, 2) != MY_AUDIO_OK) return MY_AUDIO_ERROR;

  return (SystemCoreClock == 168000000U) ? MY_AUDIO_OK : MY_AUDIO_ERROR;
}

/////////////////////////////////////////////////////////////////////////////////////
// I2C1 (blocking, 7-bit addressing, Std-mode ≤100 kHz)

const uint32_t I2CMaxTimeOut     = 0x1000;
const uint32_t I2CDefaultTimeOut = 25;

MY_AUDIO_StatusTypeDef configureAudioDACI2CRegisters(void)
{
  I2C1->CR1 &= ~I2C_CR1_PE;

  uint32_t pclk1 = (SystemCoreClock >> APBPrescTable[(RCC->CFGR & RCC_CFGR_PPRE1) >> RCC_CFGR_PPRE1_Pos]);

  I2C1->CR2   = (pclk1 / 1000000U) & I2C_CR2_FREQ; // MHz
  I2C1->TRISE = (I2C1->CR2 & I2C_CR2_FREQ) + 1U;   // Std-mode: Freq + 1
  I2C1->CCR   = (pclk1 / (100000U * 2U));          // CCR = pclk1/(2*Fs)

  I2C1->CR1 |= I2C_CR1_PE;
  audioI2CStatus = MY_AUDIO_OK;
  return MY_AUDIO_OK;
}

static void setupClocksAndGPIOForAudioI2C(void)
{
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
  (void)RCC->AHB1ENR;

  // PB6 (SCL) AF4 OD PU Fast
  GPIOB->MODER   &= ~GPIO_MODER_MODER6;   GPIOB->MODER   |=  GPIO_MODER_MODER6_1;
  GPIOB->OTYPER  |=  GPIO_OTYPER_OT6;
  GPIOB->OSPEEDR &= ~GPIO_OSPEEDER_OSPEEDR6; GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR6_1;
  GPIOB->PUPDR   &= ~GPIO_PUPDR_PUPDR6;   GPIOB->PUPDR   |=  GPIO_PUPDR_PUPDR6_0;
  GPIOB->AFR[0]  &= ~(0xFu << (6U*4U));   GPIOB->AFR[0]  |=  (4U << (6U*4U));

  // PB9 (SDA) AF4 OD PU Fast
  GPIOB->MODER   &= ~GPIO_MODER_MODER9;   GPIOB->MODER   |=  GPIO_MODER_MODER9_1;
  GPIOB->OTYPER  |=  GPIO_OTYPER_OT9;
  GPIOB->OSPEEDR &= ~GPIO_OSPEEDER_OSPEEDR9; GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR9_1;
  GPIOB->PUPDR   &= ~GPIO_PUPDR_PUPDR9;   GPIOB->PUPDR   |=  GPIO_PUPDR_PUPDR9_0;
  GPIOB->AFR[1]  &= ~(0xFu << ((9U-8U)*4U)); GPIOB->AFR[1] |= (4U << ((9U-8U)*4U));

  RCC->APB1ENR  |= RCC_APB1ENR_I2C1EN;
  RCC->APB1RSTR |= RCC_APB1RSTR_I2C1RST;
  RCC->APB1RSTR &= ~RCC_APB1RSTR_I2C1RST;
}

static void setupAudioI2CPeripheral(void)
{
  if (audioI2CStatus == MY_AUDIO_RESET)
  {
    setupClocksAndGPIOForAudioI2C();
    configureAudioDACI2CRegisters();
  }
}

void setupResetForAudioDAC(void)
{
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
  (void)RCC->AHB1ENR;

  // PD4 output, PP, Fast, no pull
  GPIOD->MODER   &= ~GPIO_MODER_MODER4;
  GPIOD->MODER   |=  GPIO_MODER_MODER4_0;
  GPIOD->OTYPER  &= ~GPIO_OTYPER_OT4;
  GPIOD->OSPEEDR &= ~GPIO_OSPEEDER_OSPEEDR4;
  GPIOD->OSPEEDR |=  GPIO_OSPEEDER_OSPEEDR4_1;
  GPIOD->PUPDR   &= ~GPIO_PUPDR_PUPDR4;

  // Reset pulse
  GPIOD->BSRR = GPIO_BSRR_BR4; audioDelay(5);
  GPIOD->BSRR = GPIO_BSRR_BS4; audioDelay(5);

  audioDACStatus = MY_AUDIO_RESET;
}

// --- I2C wait helpers (polling) ---
static MY_AUDIO_StatusTypeDef waitForI2CMasterAddressFlagUntilTimeout(uint32_t Timeout)
{
  setAudioTimer(Timeout);
  while ((I2C1->SR1 & I2C_SR1_ADDR) == 0)
  {
    if (I2C1->SR1 & I2C_SR1_AF) { I2C1->CR1 |= I2C_CR1_STOP; I2C1->SR1 &= ~I2C_SR1_AF; audioI2CStatus = MY_AUDIO_ERROR;   return MY_AUDIO_ERROR; }
    if (hasAudioTimerFinished()) { audioI2CStatus = MY_AUDIO_TIMEOUT; return MY_AUDIO_TIMEOUT; }
  }
  return MY_AUDIO_OK;
}
static MY_AUDIO_StatusTypeDef waitForI2CTXEFlagUntilTimeout(uint32_t Timeout)
{
  setAudioTimer(Timeout);
  while ((I2C1->SR1 & I2C_SR1_TXE) == 0)
  {
    if (I2C1->SR1 & I2C_SR1_AF) { I2C1->CR1 |= I2C_CR1_STOP; I2C1->SR1 &= ~I2C_SR1_AF; return MY_AUDIO_ERROR; }
    if (hasAudioTimerFinished()) return MY_AUDIO_TIMEOUT;
  }
  return MY_AUDIO_OK;
}
static MY_AUDIO_StatusTypeDef waitForI2CBTFFlagUntilTimeout(uint32_t Timeout)
{
  setAudioTimer(Timeout);
  while ((I2C1->SR1 & I2C_SR1_BTF) == 0)
  {
    if (I2C1->SR1 & I2C_SR1_AF) { I2C1->SR1 &= ~I2C_SR1_AF; audioI2CStatus = MY_AUDIO_ERROR;   return MY_AUDIO_ERROR; }
    if (hasAudioTimerFinished()) { audioI2CStatus = MY_AUDIO_TIMEOUT; return MY_AUDIO_TIMEOUT; }
  }
  return MY_AUDIO_OK;
}
static MY_AUDIO_StatusTypeDef myI2C_WaitOnRXNEFlagUntilTimeout(uint32_t Timeout)
{
  setAudioTimer(Timeout);
  while ((I2C1->SR1 & I2C_SR1_RXNE) == 0)
  {
    if (I2C1->SR1 & I2C_SR1_STOPF) { I2C1->SR1 &= ~I2C_SR1_STOPF; audioI2CStatus = MY_AUDIO_ERROR; return MY_AUDIO_ERROR; }
    if (hasAudioTimerFinished())   { audioI2CStatus = MY_AUDIO_TIMEOUT; return MY_AUDIO_TIMEOUT; }
  }
  return MY_AUDIO_OK;
}
// Clear ADDR: read SR1 then SR2 (order matters)
static void clearTheADDRFlag(I2C_TypeDef *I2Cx)
{
  __IO uint32_t tmpreg = I2Cx->SR1; (void)tmpreg;
  tmpreg = I2Cx->SR2; (void)tmpreg;
}

// ---- I2C read/write (blocking) -------------------------------------------------
MY_AUDIO_StatusTypeDef readDataFromI2CPeripheral(I2C_TypeDef * I2Cx, uint16_t DevAddress, uint16_t MemAddress, uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
  if (audioI2CStatus == MY_AUDIO_OK)
  {
    audioI2CStatus = MY_AUDIO_BUSY;

    if (waitForFlagWithTimeout(&I2Cx->SR2, I2C_SR2_BUSY_Pos, 0, I2CDefaultTimeOut)) return MY_AUDIO_BUSY;

    I2Cx->CR1 |= I2C_CR1_ACK;

    // START (write)
    I2Cx->CR1 |= I2C_CR1_START;
    if (waitForFlagWithTimeout(&I2Cx->SR1, I2C_SR1_SB_Pos, 1, Timeout)) return MY_AUDIO_TIMEOUT;

    I2Cx->DR = (uint8_t)(DevAddress & 0xFEu); // write address

    MY_AUDIO_StatusTypeDef rv = waitForI2CMasterAddressFlagUntilTimeout(Timeout);
    if (rv != MY_AUDIO_OK) return rv;

    clearTheADDRFlag(I2Cx);

    // index write (device-specific: MemAddress | 0x01)
    rv = waitForI2CTXEFlagUntilTimeout(Timeout);
    if (rv != MY_AUDIO_OK) { if (rv == MY_AUDIO_ERROR) I2Cx->CR1 |= I2C_CR1_STOP; return rv; }
    I2Cx->DR = (uint8_t)(MemAddress | 0x01u);

    rv = waitForI2CTXEFlagUntilTimeout(Timeout);
    if (rv != MY_AUDIO_OK) { if (rv == MY_AUDIO_ERROR) I2Cx->CR1 |= I2C_CR1_STOP; return rv; }

    // RESTART (read)
    I2Cx->CR1 |= I2C_CR1_START;
    if (waitForFlagWithTimeout(&I2Cx->SR1, I2C_SR1_SB_Pos, 1, Timeout)) return MY_AUDIO_TIMEOUT;

    I2Cx->DR = (uint8_t)(DevAddress | 0x01u); // read address

    rv = waitForI2CMasterAddressFlagUntilTimeout(Timeout);
    if (rv != MY_AUDIO_OK) return rv;

    if (Size == 0)
    {
      clearTheADDRFlag(I2Cx);
      I2Cx->CR1 |= I2C_CR1_STOP;
      return MY_AUDIO_OK;
    }
    else if (Size == 1)
    {
      I2Cx->CR1 &= ~I2C_CR1_ACK;
      clearTheADDRFlag(I2Cx);
      I2Cx->CR1 |= I2C_CR1_STOP;

      rv = myI2C_WaitOnRXNEFlagUntilTimeout(Timeout);
      if (rv != MY_AUDIO_OK) { audioI2CStatus = rv; return rv; }

      *pData++ = I2Cx->DR;
      return MY_AUDIO_OK;
    }
    else if (Size == 2)
    {
      I2Cx->CR1 &= ~I2C_CR1_ACK;
      I2Cx->CR1 |= I2C_CR1_POS;
      clearTheADDRFlag(I2Cx);

      if (waitForFlagWithTimeout(&I2Cx->SR1, I2C_SR1_BTF_Pos, 1, Timeout))
      { audioI2CStatus = MY_AUDIO_TIMEOUT; return MY_AUDIO_TIMEOUT; }

      I2Cx->CR1 |= I2C_CR1_STOP;
      *pData++ = I2Cx->DR;
      *pData++ = I2Cx->DR;
    }
    else
    {
      return MY_AUDIO_ERROR; // this project only uses 0/1/2 here
    }
  }
  return MY_AUDIO_OK;
}

static void resetTheI2CDriver(void)
{
  RCC->APB1RSTR |=  RCC_APB1RSTR_I2C1RST;
  RCC->APB1RSTR &= ~RCC_APB1RSTR_I2C1RST;
  configureAudioDACI2CRegisters();
}

MY_AUDIO_StatusTypeDef writeDataToI2CPeripheral(I2C_TypeDef *I2Cx, uint16_t DevAddress, uint16_t MemAddress, uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
  if (waitForFlagWithTimeout(&I2Cx->SR2, I2C_SR2_BUSY_Pos, 0, 25)) return MY_AUDIO_BUSY;

  if ((I2Cx->CR1 & I2C_CR1_PE) == 0) I2Cx->CR1 |= I2C_CR1_PE;
  I2Cx->CR1 &= ~I2C_CR1_POS;

  uint32_t remaining = Size;

  I2Cx->CR1 |= I2C_CR1_START;
  audioI2CStatus = MY_AUDIO_BUSY;

  if (waitForFlagWithTimeout(&I2Cx->SR1, I2C_SR1_SB_Pos, 1, Timeout)) return MY_AUDIO_TIMEOUT;

  I2Cx->DR = (uint8_t)(DevAddress & ~0x01u);

  MY_AUDIO_StatusTypeDef rv = waitForI2CMasterAddressFlagUntilTimeout(Timeout);
  if (rv != MY_AUDIO_OK) return rv;

  clearTheADDRFlag(I2Cx);

  rv = waitForI2CTXEFlagUntilTimeout(Timeout);
  if (rv != MY_AUDIO_OK) { if (rv == MY_AUDIO_ERROR) I2Cx->CR1 |= I2C_CR1_STOP; return rv; }

  I2Cx->DR = (uint8_t)(MemAddress & 0x00FFu);

  while (remaining > 0U)
  {
    rv = waitForI2CTXEFlagUntilTimeout(Timeout);
    if (rv != MY_AUDIO_OK) { if (rv == MY_AUDIO_ERROR) I2Cx->CR1 |= I2C_CR1_STOP; return rv; }

    I2Cx->DR = *pData++;
    --remaining;

    if ((I2Cx->SR1 & I2C_SR1_BTF) && (remaining > 0U))
    {
      I2Cx->DR = *pData++;
      --remaining;
    }
  }

  rv = waitForI2CBTFFlagUntilTimeout(Timeout);
  if (rv != MY_AUDIO_OK) { if (rv == MY_AUDIO_ERROR) I2Cx->CR1 |= I2C_CR1_STOP; return rv; }

  I2Cx->CR1 |= I2C_CR1_STOP;

  audioI2CStatus = MY_AUDIO_OK;
  return MY_AUDIO_OK;
}

uint32_t myI2Cx_WriteData(uint8_t Addr, uint8_t Reg, uint8_t Value)
{
  uint32_t timeout = 1000U;
  I2C_TypeDef *thisI2C = AudioI2C;
  MY_AUDIO_StatusTypeDef status = writeDataToI2CPeripheral(thisI2C, Addr, (uint16_t)Reg, &Value, 1, timeout);
  if (status != MY_AUDIO_OK) { audioI2CStatus = MY_AUDIO_ERROR; resetTheI2CDriver(); return 0; }
  audioI2CStatus = MY_AUDIO_OK;
  return 1;
}

uint32_t checkAudioDAC_ID(uint8_t deviceAddress)
{
  uint8_t value = 0;
  MY_AUDIO_StatusTypeDef status = readDataFromI2CPeripheral(I2C1, deviceAddress, (uint16_t)CS43L22_CHIPID_ADDR, &value, 1, I2CMaxTimeOut);
  if (status == MY_AUDIO_ERROR) resetTheI2CDriver();
  value = (value & CS43L22_ID_MASK);
  return (uint32_t)value;
}

// Minimal BSP hooks required by cs43l22.c
extern void    AUDIO_IO_DeInit(void) {}
extern void    AUDIO_IO_Init(void) {}
extern uint8_t AUDIO_IO_Read(uint8_t DeviceAddr, uint8_t chipIdAddress) { myUNUSED(DeviceAddr); myUNUSED(chipIdAddress); return 0; }
extern void    AUDIO_IO_Write(uint8_t deviceAddr, uint8_t deviceRegister, uint8_t data) { myI2Cx_WriteData(deviceAddr, deviceRegister, data); }

//////////////////////////////////////////////////////////////////////////////////
// I2S + DMA  (SPI3 / I2S3 pins AF6: PA4=WS, PC7=MCK, PC10=CK, PC12=SD)
// DMA1 Stream7 Channel 0 -> SPI3_TX (DBM + CIRC)

static void dma1_stream7_disable_and_clear(void)
{
  DMA_Stream_TypeDef *s = DMA1_Stream7;

  if (s->CR & DMA_SxCR_EN) {
    s->CR &= ~DMA_SxCR_EN;
    while (s->CR & DMA_SxCR_EN) { /* wait until EN clears (RM0090) */ }
  }

  DMA1->HIFCR = DMA_HIFCR_CFEIF7 | DMA_HIFCR_CDMEIF7 | DMA_HIFCR_CTEIF7
              | DMA_HIFCR_CHTIF7 | DMA_HIFCR_CTCIF7;
}

/* Start SPI3 TX with DMA1 Stream7 in true DBM + CIRC mode.
 * - buf: full interleaved-stereo buffer (two equal halves back-to-back).
 * - bytes_total: **in BYTES**; must be even and divisible by 2.
 *
 * NOTE: With DBM, NDTR is the number of elements in **one half** (not whole).
 * This function fixes that: NDTR = halfwords_per_half = (bytes_total/2)/2.
 */
static void audio_dma_start_dbm(int16_t *buf, uint32_t bytes_total)
{
  DMA_Stream_TypeDef *s = DMA1_Stream7;

  // Disable/clear
  if (s->CR & DMA_SxCR_EN) {
    s->CR &= ~DMA_SxCR_EN;
    while (s->CR & DMA_SxCR_EN) { /* wait */ }
  }
  DMA1->HIFCR = DMA_HIFCR_CFEIF7 | DMA_HIFCR_CDMEIF7 | DMA_HIFCR_CTEIF7
              | DMA_HIFCR_CHTIF7 | DMA_HIFCR_CTCIF7;

  // Split buffer into two halves
  uint32_t half_bytes = HALF_BYTES(bytes_total);
  s->PAR  = (uint32_t)&(SPI3->DR);
  s->M0AR = (uint32_t)buf;
  s->M1AR = (uint32_t)((uint8_t*)buf + half_bytes);

  // NDTR: halfwords per HALF (DBM rule)
  s->NDTR = HALFWORDS_PER_HALF(bytes_total);

  // DBM must already be configured in setupDMAForI2SPeripheral(); ensure CT=0
  s->CR &= ~DMA_SxCR_CT;

  // Enable interrupts (HT/TC/TE/DME)
  s->CR |= DMA_SxCR_HTIE | DMA_SxCR_TCIE | DMA_SxCR_TEIE | DMA_SxCR_DMEIE;

  // GO
  s->CR |= DMA_SxCR_EN;

  // Make sure I2S and TXDMA request are on
  if ((SPI3->I2SCFGR & SPI_I2SCFGR_I2SE) == 0) SPI3->I2SCFGR |= SPI_I2SCFGR_I2SE;
  if ((SPI3->CR2     & SPI_CR2_TXDMAEN) == 0)  SPI3->CR2     |= SPI_CR2_TXDMAEN;
}

void setupI2SPeripheral(uint32_t audioSamplingFreq)
{
  uint32_t tmpreg = 0U, i2sdiv = 2U, i2sodd = 0U, packetlength = 16U;
  uint32_t tmp = 0U, i2sclk = 0U, vcoinput = 0U, vcooutput = 0U;

  RCC->APB1ENR |= RCC_APB1ENR_SPI3EN;

  // GPIO clocks + AF6 mux (PA4/PC7/PC10/PC12)
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOCEN;
  (void)RCC->AHB1ENR;

  // PA4 = WS
  GPIOA->MODER   &= ~GPIO_MODER_MODER4;   GPIOA->MODER   |=  GPIO_MODER_MODER4_1;
  GPIOA->OTYPER  &= ~GPIO_OTYPER_OT4;     GPIOA->PUPDR   &= ~GPIO_PUPDR_PUPDR4;
  GPIOA->OSPEEDR &= ~GPIO_OSPEEDER_OSPEEDR4;
  GPIOA->AFR[0]  &= ~(0xFu << (4U*4U));   GPIOA->AFR[0]  |=  (6U << (4U*4U));

  // PC7 = MCK
  GPIOC->MODER   &= ~GPIO_MODER_MODER7;   GPIOC->MODER   |=  GPIO_MODER_MODER7_1;
  GPIOC->OTYPER  &= ~GPIO_OTYPER_OT7;     GPIOC->PUPDR   &= ~GPIO_PUPDR_PUPDR7;
  GPIOC->OSPEEDR &= ~GPIO_OSPEEDER_OSPEEDR7;
  GPIOC->AFR[0]  &= ~(0xFu << (7U*4U));   GPIOC->AFR[0]  |=  (6U << (7U*4U));

  // PC10 = CK
  GPIOC->MODER   &= ~GPIO_MODER_MODER10;  GPIOC->MODER   |=  GPIO_MODER_MODER10_1;
  GPIOC->OTYPER  &= ~GPIO_OTYPER_OT10;    GPIOC->PUPDR   &= ~GPIO_PUPDR_PUPDR10;
  GPIOC->OSPEEDR &= ~GPIO_OSPEEDER_OSPEEDR10;
  GPIOC->AFR[1]  &= ~(0xFu << ((10U-8U)*4U)); GPIOC->AFR[1] |= (6U << ((10U-8U)*4U));

  // PC12 = SD
  GPIOC->MODER   &= ~GPIO_MODER_MODER12;  GPIOC->MODER   |=  GPIO_MODER_MODER12_1;
  GPIOC->OTYPER  &= ~GPIO_OTYPER_OT12;    GPIOC->PUPDR   &= ~GPIO_PUPDR_PUPDR12;
  GPIOC->OSPEEDR &= ~GPIO_OSPEEDER_OSPEEDR12;
  GPIOC->AFR[1]  &= ~(0xFu << ((12U-8U)*4U)); GPIOC->AFR[1] |= (6U << ((12U-8U)*4U));

  // Reset I2S regs
  SPI3->I2SCFGR = 0;
  SPI3->I2SPR   = 0;

  // Packet length: 16-bit stereo -> 32 bits/frame
  packetlength *= 2U;

  // I2S kernel clock = PLLI2S VCO / R
  vcoinput  = externalClockFrequency / (RCC->PLLCFGR & RCC_PLLCFGR_PLLM);
  vcooutput = vcoinput * ((RCC->PLLI2SCFGR & RCC_PLLI2SCFGR_PLLI2SN) >> RCC_PLLI2SCFGR_PLLI2SN_Pos);
  i2sclk    = (uint32_t)(vcooutput / ((RCC->PLLI2SCFGR & RCC_PLLI2SCFGR_PLLI2SR) >> RCC_PLLI2SCFGR_PLLI2SR_Pos));

  // Divider with rounding → I2SPR: I2SDIV + ODD
  tmp = (uint32_t)(((((i2sclk / (packetlength * 8U)) * 10U) / audioSamplingFreq)) + 5U);
  tmp = tmp / 10U;

  i2sodd = (uint16_t)(tmp & 1U);
  i2sdiv = (uint16_t)((tmp - i2sodd) / 2U);
  i2sodd <<= 8U;

  // Divider + enable MCK
  SPI3->I2SPR = (i2sdiv & SPI_I2SPR_I2SDIV) | (i2sodd ? SPI_I2SPR_ODD : 0U) | SPI_I2SPR_MCKOE;

  // I2S Master Tx (Philips, 16/16)
  tmpreg = SPI_I2SCFGR_I2SMOD | SPI_I2SCFGR_I2SCFG_1;
  SPI3->I2SCFGR = tmpreg;

  audioI2SStatus = MY_AUDIO_OK;
}

void setupDMAForI2SPeripheral(void)
{
  // DMA clock
  RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;
  audioDelay(1);

  DMA_Stream_TypeDef *s = DMA1_Stream7;

  // Disable + clear
  dma1_stream7_disable_and_clear();

  // SPI3_TX on DMA1 Stream7 Channel 0, mem->periph, 16-bit/16-bit, MINC
  s->CR &= ~DMA_SxCR_CHSEL;                       // Channel 0
  s->CR &= ~DMA_SxCR_PFCTRL;                      // DMA flow controller
  s->CR &= ~DMA_SxCR_DIR; s->CR |= DMA_SxCR_DIR_0;
  s->CR |=  DMA_SxCR_MINC;  s->CR &= ~DMA_SxCR_PINC;
  s->CR &= ~DMA_SxCR_MSIZE; s->CR |= DMA_SxCR_MSIZE_0; // 16-bit
  s->CR &= ~DMA_SxCR_PSIZE; s->CR |= DMA_SxCR_PSIZE_0; // 16-bit

  // **DBM + CIRC**
  s->CR |=  DMA_SxCR_CIRC;
  s->CR |=  DMA_SxCR_DBM;
  s->CR &= ~DMA_SxCR_CT;                           // start targeting M0

  s->CR &= ~DMA_SxCR_PL;    s->CR |= DMA_SxCR_PL_1; // high priority

  // FIFO: full threshold, direct mode disabled
  s->FCR |=  DMA_SxFCR_DMDIS;
  s->FCR &= ~DMA_SxFCR_FTH; s->FCR |= DMA_SxFCR_FTH;

  // Peripheral address is fixed: SPI3->DR
  s->PAR  = (uint32_t)&(SPI3->DR);

  // M0AR/M1AR/NDTR set at start
  uint32_t pg = NVIC_GetPriorityGrouping();
  NVIC_SetPriority(DMA1_Stream7_IRQn, NVIC_EncodePriority(pg, 0x0E, 0));
  NVIC_EnableIRQ(DMA1_Stream7_IRQn);

  audioDMAStatus = MY_AUDIO_OK;
}

// PLLI2S presets by sampling rate
const uint32_t myI2SFreq[8]  = { 8000, 11025, 16000, 22050, 32000, 44100, 48000, 96000 };
const uint32_t myI2SPLLN[8]  = {  256,   429,   213,   429,   426,   271,   258,   344 };
const uint32_t myI2SPLLR[8]  = {    5,     4,     4,     4,     4,     6,     3,     1 };

void configureI2SClockPLL(uint32_t AudioFreq)
{
  uint8_t idx = 5; // 44.1 k default
  for (uint8_t i = 0; i < 8; ++i) { if (myI2SFreq[i] == AudioFreq) idx = i; }

  RCC->CR &= ~RCC_CR_PLLI2SON;
  while (RCC->CR & RCC_CR_PLLI2SRDY) {}

  RCC->PLLI2SCFGR = (RCC->PLLI2SCFGR & ~RCC_PLLI2SCFGR_PLLI2SN) | (myI2SPLLN[idx] << RCC_PLLI2SCFGR_PLLI2SN_Pos);
  RCC->PLLI2SCFGR = (RCC->PLLI2SCFGR & ~RCC_PLLI2SCFGR_PLLI2SR) | (myI2SPLLR[idx] << RCC_PLLI2SCFGR_PLLI2SR_Pos);

  RCC->CR |= RCC_CR_PLLI2SON;
  while (!(RCC->CR & RCC_CR_PLLI2SRDY)) {}
}

MY_AUDIO_StatusTypeDef myAudioInitialisePeripherals(uint16_t OutputDevice, uint8_t Volume, uint32_t audioSamplingFreq)
{
  MY_AUDIO_StatusTypeDef ret = MY_AUDIO_OK;

  audioDACStatus = audioDMAStatus = audioI2CStatus = audioI2SStatus = MY_AUDIO_RESET;

  configureI2SClockPLL(audioSamplingFreq);
  setupDMAForI2SPeripheral();
  setupI2SPeripheral(audioSamplingFreq);
  setupAudioI2CPeripheral();
  setupResetForAudioDAC();

  // Probe codec ID
  uint32_t id = checkAudioDAC_ID(DAC_ADDR_ON_I2C);

  if (id == CS43L22_EXPECTED_ID) {
    cs43l22_Init(DAC_ADDR_ON_I2C, OutputDevice, Volume, audioSamplingFreq);
    audioDACStatus = MY_AUDIO_OK;
  } else {
    ret = MY_AUDIO_ERROR;
  }
  return ret;
}

////////////////////////////////////////////////////////////////////////////
// Playback helpers (DBM + CIRC)

/* Seed DBM+CIRC with a contiguous buffer split into two halves.
 * API: PBSIZE is **BYTES** (kept from your original start function).
 */
void myAudioStartPlaying(int16_t *PlayBuff, uint32_t PBSIZE)
{
  uint8_t audioI2CAddress = DAC_ADDR_ON_I2C;
  cs43l22_Play(audioI2CAddress, (uint16_t *)&PlayBuff[0], PBSIZE);

  audio_dma_start_dbm(PlayBuff, PBSIZE);
}

/* Swap to a brand-new buffer (DBM-aware).
 * API: 'size' is **number of 16-bit samples** in the whole buffer (kept).
 * We convert to bytes internally and re-seed M0/M1 + NDTR accordingly.
 */
void myAudioChangeBuffer(int16_t *pData, uint32_t size)
{
  if ((pData == NULL) || (size < 16U)) return;

  DMA_Stream_TypeDef *s = DMA1_Stream7;

  // Disable + clear
  dma1_stream7_disable_and_clear();

  // Convert samples -> bytes (whole buffer), split into halves
  uint32_t bytes_total = size * BYTES_PER_SAMPLE_16;
  uint32_t half_bytes  = HALF_BYTES(bytes_total);

  s->M0AR = (uint32_t)pData;
  s->M1AR = (uint32_t)((uint8_t*)pData + half_bytes);

  // NDTR = halfwords per half
  s->NDTR = HALFWORDS_PER_HALF(bytes_total);

  // Restart with M0
  s->CR &= ~DMA_SxCR_CT;

  // Re-enable interrupts and stream
  s->CR |=  DMA_SxCR_HTIE | DMA_SxCR_TCIE | DMA_SxCR_TEIE | DMA_SxCR_DMEIE;
  DMA1->HIFCR = DMA_HIFCR_CFEIF7 | DMA_HIFCR_CDMEIF7 | DMA_HIFCR_CTEIF7
              | DMA_HIFCR_CHTIF7 | DMA_HIFCR_CTCIF7;
  s->CR  |= DMA_SxCR_EN;

  // Ensure I2S/TXDMA are enabled
  if ((AudioI2S->I2SCFGR & SPI_I2SCFGR_I2SE) == 0) AudioI2S->I2SCFGR |= SPI_I2SCFGR_I2SE;
  if ((AudioI2S->CR2     & SPI_CR2_TXDMAEN) == 0)  AudioI2S->CR2     |= SPI_CR2_TXDMAEN;
}

/* DMA ISR for DBM+CIRC:
 * We key off TCIF and use CT to determine which half just finished.
 * - CT=1: DMA now targets M1 → the transfer just finished into M0 → half-callback
 * - CT=0: DMA now targets M0 → the transfer just finished into M1 → complete-callback
 */
void DMA1_Stream7_IRQHandler(void)
{
  uint32_t hisr = DMA1->HISR;
  uint32_t cr   = DMA1_Stream7->CR;

  // Clear FEIF7, DMEIF7, TEIF7, HTIF7, TCIF7
  DMA1->HIFCR = DMA_HIFCR_CFEIF7 | DMA_HIFCR_CDMEIF7 | DMA_HIFCR_CTEIF7
              | DMA_HIFCR_CHTIF7 | DMA_HIFCR_CTCIF7;

  if (hisr & (DMA_HISR_FEIF7 | DMA_HISR_DMEIF7 | DMA_HISR_TEIF7)) {
    audioDMAStatus = MY_AUDIO_ERROR;
    return;
  }

  if (hisr & DMA_HISR_TCIF7) {
    if (cr & DMA_SxCR_CT) {
      audio_tx_half_isr();        // M0 just finished
    } else {
      audio_tx_complete_isr();    // M1 just finished
    }
  }
}
