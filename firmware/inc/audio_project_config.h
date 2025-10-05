// audio_project_config.h — one place for project-wide audio knobs
#ifndef AUDIO_PROJECT_CONFIG_H
#define AUDIO_PROJECT_CONFIG_H

/* =============================== Sample rate =============================== */
/* Target Fs in Hz. Must be one of the PLLI2S presets chosen in bsp_clock.c:
   { 8000, 11025, 16000, 22050, 32000, 44100, 48000, 96000 } */
#define AUDIO_FS_HZ             96000u

/* ============================ Playback buffering ========================== */
/* Total play buffer size in 32-bit words (int32_t slots), interleaved L,R.
   Must be even (pairs of L/R words). Typical sizes: 2048, 4096, 8192. */
#define AUDIO_PB_WORDS          4096u

/* =============================== Tone generator =========================== */
/* Peak amplitude for 24-in-32 output (headroom below full-scale to avoid DAC
   clipping on interpolation). 30000 ≈ −20 dBFS. */
#define AUDIO_APP_PEAK          30000

/* Sine LUT resolution: size = 2^AUDIO_APP_SINE_LUT_BITS entries.
   10 → 1024 points (fast, tiny); 12 → 4096 (smoother); 14 → 16384 (largest). */
#define AUDIO_APP_SINE_LUT_BITS 10u

/* DDS fractional bits used for linear interpolation between LUT points.
   8 is a good balance of quality vs. cost. */
#define AUDIO_APP_PHASE_FRAC_BITS 8u

/* =============================== Output controls ========================== */
/* Mirrors simple output/volume semantics if a codec needs them.
   (The PCM5102 paths ignore device selection but keep volume semantics if you
   add a digital gain stage later.) */
#define AUDIO_VOLUME            80
#define AUDIO_OUTPUT_DEVICE     OUTPUT_DEVICE_BOTH

/* =============================== I2S options ============================== */
/* Master clock to PCM5102s:
   - 1: provide MCLK = 256×Fs (PC6 for I2S2, PC7 for I2S3)
   - 0: no MCLK (PCM5102 can lock from BCK/LRCK)
   You can override in a per-file build, but centralizing here keeps both DACs
   in sync. */
#define PCM5102_USE_MCLK        1   /* I2S2 / PCM #1 */
#define PCM5102B_USE_MCLK       1   /* I2S3 / PCM #2 */

/* DMA IRQ priorities (lower is higher priority). Keep them modest so they
   preempt foreground but don’t starve other ISRs. */
#define AUDIO2_DMA_IRQ_PRIO     14  /* DMA1 Stream4 (I2S2 TX) */
#define AUDIO3_DMA_IRQ_PRIO     14  /* DMA1 Stream7 (I2S3 TX) */

/* =============================== Sanity checks ============================ */
#if (AUDIO_PB_WORDS % 2u) != 0u
# error "AUDIO_PB_WORDS must be even (pairs of L/R 32-bit words)."
#endif

#if (AUDIO_APP_SINE_LUT_BITS < 8u) || (AUDIO_APP_SINE_LUT_BITS > 14u)
# error "AUDIO_APP_SINE_LUT_BITS out of range (8..14 recommended)."
#endif

#if (AUDIO_APP_PHASE_FRAC_BITS < 4u) || (AUDIO_APP_PHASE_FRAC_BITS > 12u)
# error "AUDIO_APP_PHASE_FRAC_BITS out of range (4..12 recommended)."
#endif

#endif /* AUDIO_PROJECT_CONFIG_H */
