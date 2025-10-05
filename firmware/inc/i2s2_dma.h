#ifndef I2S2_DMA_H
#define I2S2_DMA_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* -------------------------------------------------------------------------- */
/* I2S2 + DMA playback (PCM5102 #1)                                           */
/* Format: Philips I2S, Master TX, 24-in-32, stereo interleaved (L,R,...)     */
/* DMA:    DMA1 Stream4 (SPI2_TX)                                             */
/* Pins:   PB12=WS, PB13=CK, PB15=SD, (optional) PC6=MCLK                     */
/* Notes:  - MCLK can be disabled at build-time via PCM5102_USE_MCLK=0        */
/*         - audio_fs_hz must match your PLLI2S config (see i2s_pll_set_fs)   */
/*         - Buffer must be an even number of 32-bit words (pairs of L/R)     */
/* -------------------------------------------------------------------------- */

/** One-time base init of I2S2 + DMA1 Stream4 (no buffer yet).
 *  - Enables clocks, configures GPIOs and NVIC, resets SPI2/I2S2 registers.
 *  - Safe to call once early in bring-up. */
void dma_i2s2_init(void);

/** Configure I2S2 for Philips I2S, Master TX, 24-bit in 32-bit channel frames.
 *  - Programs prescaler for the requested sample rate.
 *  - If PCM5102_USE_MCLK=1 (default), enables MCLK at 256×Fs.
 *  - If PCM5102_USE_MCLK=0, runs without MCLK; BCK ≈ 64×Fs. */
void i2s2_init(uint32_t audio_fs_hz);

/** Start circular single-buffer playback via DMA1 Stream4 (SPI2_TX).
 *  @param buf          Base address of stereo interleaved samples (int32_t).
 *                      Each channel is 24-in-32 (left-justified in 32-bit word).
 *  @param bytes_total  Total size of the buffer in bytes; MUST be a multiple of 8
 *                      (two 32-bit words per L/R frame).
 *
 *  Notes:
 *   - The driver installs half/complete IRQs; your app’s callbacks should
 *     write only the half just released by DMA.
 *   - To restart with a new buffer, stop the stream first (disable/clear). */
void audio2_dma_start(int32_t *buf, uint32_t bytes_total);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* I2S2_DMA_H */
