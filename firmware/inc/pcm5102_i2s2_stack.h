#ifndef PCM5102_I2S2_STACK_H
#define PCM5102_I2S2_STACK_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Thin wrapper around the CMSIS I2S2 + DMA driver (i2s2_dma.*).
   - Initialize I2S2 clocks + GPIO + DMA
   - Configure I2S2 for a given Fs
   - Start TX from an interleaved int32 stereo buffer
   - Expose a tiny callback hook so the DMA ISR can dispatch to either:
       * default DDS fillers:    audio2_on_* (from dds_tone_i2s2.c)
       * ADC monitor fillers:    adcmon2_on_* (from adcmon_i2s2_app.c)
*/

/* Init I2S2 + DMA base (no buffer yet). */
int  audio2_init(uint32_t fs_hz);

/* Start I2S2 DMA from an interleaved int32 stereo buffer.
   play_buf_bytes must be a multiple of 8 (two 32-bit words per LR frame). */
void audio2_start(int32_t *play_buf, uint32_t play_buf_bytes);

/* Optional dynamic filler selection (implemented in i2s2_dma.c) */
typedef void (*i2s_cb_t)(void);
void audio2_set_callbacks(i2s_cb_t on_half, i2s_cb_t on_full);

#ifdef __cplusplus
}
#endif

#endif /* PCM5102_I2S2_STACK_H */
