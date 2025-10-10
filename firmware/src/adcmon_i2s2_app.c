// adcmon_i2s2_app.c — ADS1256 monitor → I2S2 (PCM5102)
//
// - Pulls ADS1256 samples via ads1256_read_latest()
// - Linear interpolates from src_hz (ADS) to AUDIO_FS_HZ (I2S2)
// - One-pole IIR de-clicker in 24-bit domain
// - Writes mono duplicated to stereo into the bound I2S2 TX buffer
//
// Default output format matches audio2_app: int16 (~±30000) stored in int32 slots.
// Define I2S2_USE_24IN32=1 for true 24-in-32 output.

#include <stdint.h>
#include <stddef.h>
#include "audio_project_config.h"   // AUDIO_FS_HZ, AUDIO_PB_WORDS
#include "ads1256.h"                // ads1256_read_latest()
#include "adcmon_i2s2_app.h"

/* ===== Build-time knobs ===== */

/* 0: output int16 stored in int32 slots (matches your DDS path)
 * 1: output true 24-bit samples left-justified in 32-bit words */
#ifndef I2S2_USE_24IN32
#define I2S2_USE_24IN32  1
#endif

/* Software gain (pre-map): positive=boost, negative=atten */
#ifndef MON_GAIN_SHIFT
#define MON_GAIN_SHIFT   0
#endif

/* Target peak when mapping 24-bit to 16-bit (matches DDS loudness) */
#ifndef MON_TARGET_PEAK_16
#define MON_TARGET_PEAK_16  30000   /* ~-1.1 dBFS in 16-bit domain */
#endif

/* One-pole IIR smoothing: y += (x - y) / 2^MON_SMOOTH_SHIFT
 * 5 => 1/32 (gentle, kills random clicks). */
#ifndef MON_SMOOTH_SHIFT
#define MON_SMOOTH_SHIFT  2
#endif

/* Optional median-of-3 on raw ADS samples (helps bursty spikes). */
#ifndef MON_MEDIAN3
#define MON_MEDIAN3  1
#endif

/* ===== Private state ===== */
static int32_t  *s_out_buf   = NULL;   /* I2S2 TX buffer (interleaved int32 L,R,...) */
static uint32_t  s_out_words = 0;      /* number of int32 words in buffer            */

static uint32_t  dst_fs_hz = AUDIO_FS_HZ;  /* I2S2 sample-rate (Hz) */
static uint32_t  src_hz    = 2000u;        /* ADS rate (set by user) */

/* Phase accumulator: step = src_hz / dst_fs_hz in Q32 */
static uint32_t  step_q32  = 0;
static uint32_t  phase_q32 = 0;

/* Current/next ADC samples; look-ahead for lerp */
static int32_t   s_cur     = 0;
static int32_t   s_next    = 0;
static uint8_t   have_next = 0;

/* IIR state in 24-bit domain */
static int32_t   s_iir24   = 0;

#if MON_MEDIAN3
/* Median-of-3 scratch */
static int32_t m0 = 0, m1 = 0, m2 = 0;
static inline int32_t median3(int32_t a, int32_t b, int32_t c){
  int32_t x=a, y=b, z=c;
  if (x>y){ int32_t t=x; x=y; y=t; }
  if (y>z){ int32_t t=y; y=z; z=t; }
  if (x>y){ int32_t t=x; x=y; y=t; }
  return y;
}
#endif

/* ===== Helpers ===== */

static inline uint32_t half_words(void){
  return (s_out_words ? (s_out_words / 2u) : 0u);
}

/* Clamp to signed 24-bit */
static inline int32_t clip24(int32_t x){
  if (x >  0x7FFFFF) return  0x7FFFFF;
  if (x < -0x800000) return -0x800000;
  return x;
}

/* Map 24-bit centered to 16-bit with clipping (match DDS amplitude) */
static inline int16_t map24_to_target16(int32_t s24){
  /* s24 full-scale ≈ ±(2^23-1); scale to MON_TARGET_PEAK_16. */
  int64_t y = ((int64_t)s24 * (int64_t)MON_TARGET_PEAK_16) >> 23;
  if (y >  32767) y =  32767;
  if (y < -32768) y = -32768;
  return (int16_t)y;
}

/* Linear interpolation with frac in Q15 */
static inline int32_t lerp_q15(int32_t a, int32_t b, uint32_t frac_q15){
  return a + (int32_t)(((int64_t)(b - a) * (int64_t)frac_q15) >> 15);
}

/* Refresh Q32 step */
static inline void update_step(void){
  if (src_hz == 0u) { step_q32 = 0u; return; }
  step_q32 = (uint32_t)(((uint64_t)src_hz << 32) / (uint64_t)dst_fs_hz);
}

/* ===== Public API ===== */

void adcmon2_bind_output_buffer(int32_t *out_buf, uint32_t out_words){
  s_out_buf   = out_buf;
  s_out_words = out_words;
}

void adcmon2_set_src_rate_hz(uint32_t hz){
  src_hz = (hz == 0u) ? 1u : hz;
  update_step();
}

void adcmon2_reset_phase(void){
  phase_q32 = 0u;
  have_next = 0u;
  s_cur = s_next = 0;
  s_iir24 = 0;
#if MON_MEDIAN3
  m0 = m1 = m2 = 0;
#endif
}

void adcmon2_app_prefill(void){
  adcmon2_reset_phase();
  if (s_out_buf && s_out_words){
    for (uint32_t i=0;i<s_out_words;i++) s_out_buf[i] = 0;
  }
}

/* ===== Core fill ===== */

static void fill_range_from_adc(uint32_t start_word, uint32_t end_word)
{
  if (!s_out_buf || s_out_words == 0u) return;

  /* Align to L on entry; ensure end on frame boundary (pairs of words) */
  if (start_word & 1u) start_word++;
  if (end_word   & 1u) end_word--;

  for (uint32_t w = start_word; w + 1u < end_word; w += 2u) {

    /* Ensure we have a look-ahead sample for interpolation */
    while (!have_next){
      int32_t tmp;
      if (!ads1256_read_latest(&tmp)) { tmp = s_cur; break; }  /* hold-last */
#if MON_MEDIAN3
      m0 = m1; m1 = m2; m2 = tmp;
      tmp = median3(m0, m1, m2);
#endif
      s_next    = tmp;
      have_next = 1u;
    }

    /* Fraction from phase (top 15 frac bits) */
    const uint32_t frac_q15 = (phase_q32 >> 17) & 0x7FFFu;
    int32_t s_lerp = lerp_q15(s_cur, s_next, frac_q15);

    /* Optional software gain (pre-map) */
#if (MON_GAIN_SHIFT > 0)
    s_lerp <<= MON_GAIN_SHIFT;
#elif (MON_GAIN_SHIFT < 0)
    s_lerp >>= (-MON_GAIN_SHIFT);
#endif

    /* One-pole IIR de-clicker */
    s_iir24 += ((s_lerp - s_iir24) >> MON_SMOOTH_SHIFT);
    s_lerp   = s_iir24;

#if I2S2_USE_24IN32
    const int32_t s24 = clip24(s_lerp);
    s_out_buf[w+0] = s24;   /* L */
    s_out_buf[w+1] = s24;   /* R */
#else
    const int16_t v16 = map24_to_target16(s_lerp);
    s_out_buf[w+0] = (int32_t)v16;  /* L */
    s_out_buf[w+1] = (int32_t)v16;  /* R */
#endif

    /* Advance phase; on wrap, accept next input sample */
    const uint32_t prev = phase_q32;
    phase_q32 += step_q32;
    if (phase_q32 < prev) {
      s_cur = s_next;
      have_next = 0u;
    }
  }
}

/* ===== DMA callbacks ===== */

void adcmon2_on_half_transfer(void){
  if (!s_out_words) return;
  fill_range_from_adc(0u, half_words());
}

void adcmon2_on_transfer_complete(void){
  if (!s_out_words) return;
  fill_range_from_adc(half_words(), s_out_words);
}
