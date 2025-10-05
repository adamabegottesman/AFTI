// audio2_app.c — PCM5102 #1 tone generator (I2S2 path)
// Owns a circular interleaved stereo buffer (L,R,...) and fills the half
// just released by DMA. DDS uses a compact sine LUT + linear interpolation.

#include <stdint.h>
#include <math.h>
#include "audio2_app.h"
#include "audio_project_config.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* =============================== Configuration ============================= */
/* These mirror audio_app.c so both DAC paths sound identical. */

#ifndef AUDIO_APP_PBSIZE
#define AUDIO_APP_PBSIZE   AUDIO_PB_WORDS
#endif

#ifndef AUDIO_APP_PEAK
#define AUDIO_APP_PEAK     30000           /* headroom below 24-bit full-scale */
#endif

#ifndef AUDIO_APP_FS_HZ
#define AUDIO_APP_FS_HZ    AUDIO_FS_HZ     /* must match i2s*_init(Fs) */
#endif

/* LUT resolution and DDS phase partition:
   - Index uses top AUDIO_APP_SINE_LUT_BITS of the 32-bit phase
   - Fraction uses AUDIO_APP_PHASE_FRAC_BITS below that for interp */
#ifndef AUDIO_APP_SINE_LUT_BITS
#define AUDIO_APP_SINE_LUT_BITS   10u      /* 2^10 = 1024 entries */
#endif
#define AUDIO_APP_SINE_SIZE        (1u << AUDIO_APP_SINE_LUT_BITS)

#ifndef AUDIO_APP_PHASE_FRAC_BITS
#define AUDIO_APP_PHASE_FRAC_BITS  8u
#endif
#define AUDIO_APP_PHASE_INT_BITS   (AUDIO_APP_SINE_LUT_BITS)
#define AUDIO_APP_PHASE_SHIFT_IDX  (32u - AUDIO_APP_PHASE_INT_BITS)
#define AUDIO_APP_PHASE_SHIFT_FRAC (32u - AUDIO_APP_PHASE_INT_BITS - AUDIO_APP_PHASE_FRAC_BITS)
#define AUDIO_APP_FRAC_MASK        ((1u << AUDIO_APP_PHASE_FRAC_BITS) - 1u)

/* Buffer must be even number of 32-bit words (pairs of L/R). */
#if (AUDIO_APP_PBSIZE % 2) != 0
# error "AUDIO_APP_PBSIZE must be even (pairs of L/R 32-bit slots)."
#endif

/* ============================== Private state ============================== */

static int32_t  Play2[AUDIO_APP_PBSIZE];     /* circular play buffer (private) */
static uint32_t dds_phase2 = 0;              /* 32-bit phase accumulator       */
static uint32_t dds_step2  = 0;              /* set via audio2_set_frequency   */
static int32_t  SineLUT[AUDIO_APP_SINE_SIZE];/* centered integer sine table    */

static inline uint32_t half_words(void)
{
  return (uint32_t)(AUDIO_APP_PBSIZE / 2u);
}

/* =============================== Internal helpers ========================== */

static void build_sine_lut_once(void)
{
  static uint8_t built = 0u;
  if (built) return;

  for (uint32_t i = 0; i < AUDIO_APP_SINE_SIZE; ++i) {
    const float a = (float)(2.0 * M_PI) * (float)i / (float)AUDIO_APP_SINE_SIZE;
    /* Scale to ~Q1.23 range (placed in 32-bit slot for 24-in-32 output). */
    SineLUT[i] = (int32_t)lrintf((float)AUDIO_APP_PEAK * sinf(a));
  }
  built = 1u;
}

/* Next sample from DDS using linear interpolation between LUT points. */
static inline int32_t dds_next_sample_interp(void)
{
  const uint32_t idx  = (dds_phase2 >> AUDIO_APP_PHASE_SHIFT_IDX)  & (AUDIO_APP_SINE_SIZE - 1u);
  const uint32_t frac = (dds_phase2 >> AUDIO_APP_PHASE_SHIFT_FRAC) & AUDIO_APP_FRAC_MASK;
  dds_phase2 += dds_step2;

  const uint32_t idx2 = (idx + 1u) & (AUDIO_APP_SINE_SIZE - 1u);
  const int32_t  s0   = SineLUT[idx];
  const int32_t  s1   = SineLUT[idx2];
  const int32_t  diff = s1 - s0;

  return s0 + (int32_t)(((int64_t)diff * (int64_t)frac) >> AUDIO_APP_PHASE_FRAC_BITS);
}

/* Fill [start_word, end_word) with interleaved LR frames (int32 per slot). */
static void fill_range(uint32_t start_word, uint32_t end_word)
{
  /* Align to L on entry; ensure end is on frame boundary. */
  if ((start_word & 1u) != 0u) start_word++;
  if ((end_word   & 1u) != 0u) end_word--;

  for (uint32_t w = start_word; w + 1u < end_word; w += 2u) {
    const int32_t s = dds_next_sample_interp();
    Play2[w + 0] = s;  /* Left  */
    Play2[w + 1] = s;  /* Right (mono duplicated) */
  }
}

/* ================================= Public API ============================== */

void audio2_app_prefill(void)
{
  build_sine_lut_once();
  audio2_set_frequency_hz(2000u);  /* default tone */
  fill_range(0u, (uint32_t)(sizeof(Play2) / sizeof(Play2[0])));
}

int32_t* audio2_playbuff(void)
{
  return Play2;
}

uint32_t audio2_playbuff_bytes(void)
{
  return (uint32_t)sizeof(Play2);
}

/* Exact DDS step: step = hz * 2^32 / Fs (64-bit math prevents overflow). */
void audio2_set_frequency_hz(uint32_t hz)
{
  const uint64_t step = (((uint64_t)hz) << 32) / (uint64_t)AUDIO_APP_FS_HZ;
  dds_step2 = (uint32_t)step;
}

/* ISR callbacks bound from the I2S2/DMA driver. Keep them tiny & nonblocking. */
void audio2_on_half_transfer(void)
{
  fill_range(0u, half_words());
}

void audio2_on_transfer_complete(void)
{
  fill_range(half_words(), (uint32_t)(sizeof(Play2) / sizeof(Play2[0])));
}
