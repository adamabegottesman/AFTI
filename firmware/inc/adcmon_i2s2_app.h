#pragma once
#include <stdint.h>
#include <stdbool.h>

/* ADS1256 monitor -> I2S2 (PCM5102)
   - Pops samples via ads1256_read_latest()
   - Tiny linear interpolation resampler (ADS src_hz -> I2S2 Fs)
   - Writes mono duplicated to stereo into the SAME buffer I2S2 DMA uses
   - Output format matches your audio2_app: 16-bit-in-32 (low 16 bits) by default
*/

/* If your I2S2 path should use 24-in-32 samples, set to 1. */
#ifndef I2S2_USE_24IN32
#define I2S2_USE_24IN32  0
#endif

/* Optional software gain (left shift before 16-bit clip) */
#ifndef MON_GAIN_SHIFT
#define MON_GAIN_SHIFT   0     /* try 0..4 for +0..24 dB */
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* One-time setup & zero prefill */
void adcmon2_app_prefill(void);

/* Bind the monitor to the buffer I2S2 DMA actually transmits.
   Typically pass audio2_playbuff() and AUDIO_PB_WORDS. */
void adcmon2_bind_output_buffer(int32_t *out_buf, uint32_t out_words);

/* Tell monitor the ADS1256 source rate in Hz (e.g., 500, 7500, 30000) */
void adcmon2_set_src_rate_hz(uint32_t src_hz);

/* Optional: reset resampler phase when switching in/out */
void adcmon2_reset_phase(void);

/* DMA callbacks: install these via audio2_set_callbacks() */
void adcmon2_on_half_transfer(void);
void adcmon2_on_transfer_complete(void);

#ifdef __cplusplus
}
#endif
