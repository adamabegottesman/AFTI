#ifndef I2S3_DMA_PCM_H
#define I2S3_DMA_PCM_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* -------------------------------------------------------------------------- */
/* I2S3 + DMA playback (PCM5102 #2)                                           */
/* Format: Philips I2S, Master TX, 24-in-32, stereo interleaved (L,R,...)     */
/* DMA:    DMA1 Stream7 (SPI3_TX)                                             */
/* Pins:   PA4=WS, PC10=CK, PC12=SD, (optional) PC7=MCLK  — all AF6           */
/* Notes:  - MCLK can be disabled at build-time via PCM5102B_USE_MCLK=0       */
/*         - audio_fs_hz must match your PLLI2S config (see i2s_pll_set_fs)   */
/*         - Buffer must be an even number of 32-bit words (pairs of L/R)     */
/* -------------------------------------------------------------------------- */

/** One-time base init of I2S3 + DMA1 Stream7 (no buffer yet).
 *  - Enables clocks, configures GPIOs and NVIC, resets SPI3/I2S3 registers.
 *  - Safe to call once early in bring-up. */
void dma_i2s3_init_pcm(void);

/** Configure I2S3 for Philips I2S, Master TX, 24-in-32 channel frames.
 *  - Programs prescaler for the requested sample rate.
 *  - If PCM5102B_USE_MCLK=1 (default), enables MCLK at 256×Fs.
 *  - If PCM5102B_USE_MCLK=0, runs without MCLK; BCK ≈ 64×Fs. */
void i2s3_init_pcm(uint32_t audio_fs_hz);

/** Start circular single-buffer playback via DMA1 Stream7 (SPI3_TX).
 *  @param buf          Base address of stereo interleaved samples (int32_t).
 *                      Each channel is 24-in-32 (left-justified in 32-bit word).
 *  @param bytes_total  Total size of the buffer in bytes; MUST be a multiple of 8
 *                      (two 32-bit words per L/R frame).
 *
 *  Notes:
 *   - The driver installs half/complete IRQs; your app’s callbacks should
 *     write only the half just released by DMA.
 *   - To restart with a new buffer, stop the stream first (disable/clear). */
void audio3_dma_start(int32_t *buf, uint32_t bytes_total);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* I2S3_DMA_PCM_H */
