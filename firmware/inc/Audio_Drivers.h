// Bare-metal audio drivers for STM32F4-DISC1 (CS43L22 codec), CMSIS-only.
// HAL-independent; uses SysTick internally for 1 kHz timing.
//
// System resources used internally: I2C1, I2S3 (SPI3), DMA1 Stream7 (SPI3_TX),
// SysTick (1 kHz), and GPIOs: PA4 (WS), PC7 (MCK), PC10 (CK), PC12 (SD),
// PB6 (I2C1_SCL), PB9 (I2C1_SDA), PD4 (CODEC_RESET).
//
// Quick start:
//   1) myAudioSpeedUpTheSystemClock();                 // 168 MHz + seed PLLI2S
//   2) myAudioInitialisePeripherals(OUTPUT_DEVICE_AUTO, 80, AUDIO_FREQUENCY_44K);
//   3) myAudioStartPlaying((int16_t*)PlayBuff, PBSIZE_BYTES); // DBM + CIRC
//
// Notes:
//   • Buffer is interleaved stereo 16-bit: L,R,L,R,…
//   • myAudioStartPlaying(..., PBSIZE_BYTES) expects a BYTE count for the WHOLE buffer;
//     DMA splits it into two equal halves (ping/pong).
//   • myAudioChangeBuffer(..., size_samples) expects a count of 16-bit SAMPLES
//     for the whole buffer (not bytes).
//   • DMA ISR will call audio_tx_half_isr() and audio_tx_complete_isr() that you implement.

#ifndef MY_AUDIO_DRIVERS_H
#define MY_AUDIO_DRIVERS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "stm32f407xx.h"
#include "system_stm32f4xx.h"
#include "./cs43l22.h"

/* -----------------------------------------------------------------------------
 * Codec I2C address (8-bit on-wire form: write=0x94 / read=0x95).
 * For 7-bit addressing, use 0x4A and set R/W in the transaction.
 * -------------------------------------------------------------------------- */
#define DAC_ADDR_ON_I2C   0x94U

/* Status codes returned by driver functions */
typedef enum
{
  MY_AUDIO_OK       = 0x00U,
  MY_AUDIO_ERROR    = 0x01U,
  MY_AUDIO_BUSY     = 0x02U,
  MY_AUDIO_TIMEOUT  = 0x03U,
  MY_AUDIO_RESET    = 0x04U
} MY_AUDIO_StatusTypeDef;

/* =============================================================================
 *                           Low-level I2C access (blocking)
 * -----------------------------------------------------------------------------
 * Reusable helpers to read/write 7-bit I2C devices.
 *  - DevAddress: 8-bit on-wire device address (LSB = R/W).
 *  - MemAddress: 8-bit register address.
 *  - Size: number of data bytes to transfer (read supports 0/1/2 here).
 *  - Timeout: milliseconds.
 * =========================================================================== */
MY_AUDIO_StatusTypeDef writeDataToI2CPeripheral(I2C_TypeDef *I2Cx,
                                                uint16_t DevAddress,
                                                uint16_t MemAddress,
                                                uint8_t *pData,
                                                uint16_t Size,
                                                uint32_t Timeout);

MY_AUDIO_StatusTypeDef readDataFromI2CPeripheral(I2C_TypeDef *I2Cx,
                                                 uint16_t DevAddress,
                                                 uint16_t MemAddress,
                                                 uint8_t *pData,
                                                 uint16_t Size,
                                                 uint32_t Timeout);

/* =============================================================================
 *                             High-level audio API
 * -----------------------------------------------------------------------------
 * myAudioSpeedUpTheSystemClock():
 *   - Sets VOS Scale 1 and configures main PLL to 168 MHz from HSE=8 MHz.
 *   - Seeds PLLI2S (N=192, R=2) so I2S divider math has a valid source.
 *
 * myAudioInitialisePeripherals(OutputDevice, Volume, AudioFreq):
 *   - Programs PLLI2S for the requested Fs, configures I2S3 pins/divider,
 *     sets up DMA1 Stream7 for SPI3_TX (double-buffer + circular), configures
 *     I2C1 pins/timing, pulses codec RESET, probes codec ID, and calls cs43l22_Init().
 *
 * myAudioStartPlaying(PlayBuff, PBSIZE_BYTES):
 *   - Calls cs43l22_Play(), then starts DMA in **double-buffer circular** mode
 *     (buffer split into two halves) and enables I2S + TXDMA.
 *   - PBSIZE_BYTES is a BYTE count for the whole interleaved 16-bit stereo buffer.
 *
 * myAudioChangeBuffer(PlayBuff, size_samples):
 *   - Re-seeds the DMA double-buffer addresses to a NEW buffer.
 *   - size_samples is the total number of 16-bit samples (L+R) in the buffer.
 * =========================================================================== */
MY_AUDIO_StatusTypeDef myAudioSpeedUpTheSystemClock(void);
MY_AUDIO_StatusTypeDef myAudioInitialisePeripherals(uint16_t OutputDevice,
                                                    uint8_t  Volume,
                                                    uint32_t AudioFreq);
void myAudioStartPlaying(int32_t *PlayBuff, uint32_t PBSIZE_BYTES);
void myAudioChangeBuffer (int32_t *PlayBuff, uint32_t size_samples);

/* =============================================================================
 *                                User callbacks
 * -----------------------------------------------------------------------------
 * Implement these in your application; the DMA ISR will call them:
 *   - audio_tx_half_isr()      // first half completed (ping ready)
 *   - audio_tx_complete_isr()  // second half completed (pong ready)
 * =========================================================================== */
extern void audio_tx_half_isr(void);
extern void audio_tx_complete_isr(void);

/* =============================================================================
 *                    Hooks required by cs43l22.c (BSP compatibility)
 * -----------------------------------------------------------------------------
 * Provided by the driver .c; declared here for completeness.
 * AUDIO_IO_Read is unused in this project (returns 0).
 * =========================================================================== */
extern void    AUDIO_IO_DeInit(void);
extern void    AUDIO_IO_Init(void);
extern uint8_t AUDIO_IO_Read(uint8_t DeviceAddr, uint8_t chipIdAddress);
extern void    AUDIO_IO_Write(uint8_t deviceAddr, uint8_t deviceRegister, uint8_t data);

#ifdef __cplusplus
}
#endif

#endif /* MY_AUDIO_DRIVERS_H */
