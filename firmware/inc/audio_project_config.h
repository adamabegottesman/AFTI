// audio_project_config.h — one place for project-wide audio knobs
#ifndef AUDIO_PROJECT_CONFIG_H
#define AUDIO_PROJECT_CONFIG_H

/* Target peak for ADC->I2S2 monitor when converting 24-bit to 16-bit. */
#define MON_TARGET_PEAK_16   30000   /* try 20000..31000 to taste */


/* =============================== Sample rate =============================== */
/* Target Fs in Hz. Must be one of the PLLI2S presets chosen in bsp_clock.c:
   { 8000, 11025, 16000, 22050, 32000, 44100, 48000, 96000 } */
#define AUDIO_FS_HZ             96000u

/* ============================ Playback buffering ========================== */
/* Total play buffer size in 32-bit words (int32_t slots), interleaved L,R.
   Must be even (pairs of L/R words). Typical sizes: 2048, 4096, 8192. */
#define AUDIO_PB_WORDS          4096u

/* =============================== Tone generator =========================== */
/* Legacy/global DDS peak (kept for compatibility) */
#define AUDIO_APP_PEAK          10000

/* Per-path DDS peaks (set as you like). If you prefer to override from the
   project/Makfile, wrap these in #ifndef. For now we set them explicitly. */
#define AUDIO2_APP_PEAK         10000   /* I2S2 / PCM5102 #1 DDS */
#define AUDIO3_APP_PEAK         10000   /* I2S3 / PCM5102 #2 DDS */

/* Sine LUT resolution: size = 2^AUDIO_APP_SINE_LUT_BITS entries.
   10 → 1024 points (fast, tiny); 12 → 4096 (smoother); 14 → 16384 (largest). */
#define AUDIO_APP_SINE_LUT_BITS 12u

/* DDS fractional bits used for linear interpolation between LUT points.
   8 is a good balance of quality vs. cost. */
#define AUDIO_APP_PHASE_FRAC_BITS 8u


/* =============================== I2S options ============================== */
/* Master clock to PCM5102s:
   - 1: provide MCLK = 256×Fs (PC6 for I2S2, PC7 for I2S3)
   - 0: no MCLK (PCM5102 can lock from BCK/LRCK) */
#define PCM5102_USE_MCLK        1   /* I2S2 / PCM #1 */
#define PCM5102B_USE_MCLK       1   /* I2S3 / PCM #2 */

/* DMA IRQ priorities (lower is higher priority). */
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
