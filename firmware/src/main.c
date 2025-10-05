// main.c — explicit bring-up for dual PCM5102 outputs on I2S2 & I2S3
// - System clock to 168 MHz, 1 kHz SysTick
// - Two independent tone generators (audio2_app / audio3_app)
// - Clean, circular DMA playback on both I2S blocks

#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx.h"                 // CMSIS core
#include "bsp_clock.h"                 // system_clock_init_168mhz(), timing_init_1khz()
#include "timing.h"                    // timing_now_ms(), etc.
#include "audio_project_config.h"      // AUDIO_FS_HZ, etc.

#include "audio2_app.h"
#include "audio_stack_pcm5102.h"       // PCM #1 on I2S2 (SPI2/DMA1-Stream4)
#include "audio3_app.h"
#include "audio_stack_pcm5102_b.h"     // PCM #2 on I2S3 (SPI3/DMA1-Stream7)

/* Idle policy: WFI in Release to keep ISR jitter low; NOP under debugger. */
#ifdef DEBUG
  #define APP_IDLE() do { __NOP(); } while (0)
#else
  #define APP_IDLE() do { __WFI(); } while (0)
#endif

/* How long each tone plays before switching (ms). */
#define TONE_DWELL_MS  1000u   /* change to 5000u etc. if you prefer */

//------------------------------------------------------------------------------
// Dataflow
//   system_clock(168 MHz) ──► i2s_pll_set_fs(Fs)
//                                  │
//                 ┌────────────┬───┴────┬────────────┐
//                 │            │        │            │
//          I2S2 + DMA   ─────► LRCK/BCK/SD  ─────►  PCM5102 #1 (analog out)
//          I2S3 + DMA   ─────► LRCK/BCK/SD  ─────►  PCM5102 #2 (analog out)
//        (HT/TC IRQs)            (Philips I2S, 24-in-32, stereo)
//             ▲                                   ▲
//             │                                   │
//      audio2_app fills half                audio3_app fills half
//------------------------------------------------------------------------------

int main(void)
{
    // 0) Core timing
    system_clock_init_168mhz();    // 168 MHz core; seeds PLLI2S for I2S
    timing_init_1khz();            // 1 kHz SysTick for delays/timeouts

    // 1) PCM #1 on I2S2
    audio2_app_prefill();
    audio2_init(AUDIO_FS_HZ);
    audio2_start(audio2_playbuff(), audio2_playbuff_bytes());

    // 2) PCM #2 on I2S3
    audio3_app_prefill();
    audio3_init(AUDIO_FS_HZ);
    audio3_start(audio3_playbuff(), audio3_playbuff_bytes());

    // 3) Foreground: cycle 1000 / 2000 / 4000 Hz every TONE_DWELL_MS
    static const uint32_t kFreqs[] = { 1000u, 2000u, 4000u };
    uint32_t idx = 0u;

    audio2_set_frequency_hz(kFreqs[idx]);
    audio3_set_frequency_hz(kFreqs[idx]);

    uint32_t next_switch_ms = timing_now_ms() + TONE_DWELL_MS;

    while (true)
    {
        // wrap-safe time check
        if ((int32_t)(timing_now_ms() - next_switch_ms) >= 0)
        {
            idx = (idx + 1u) % (sizeof(kFreqs)/sizeof(kFreqs[0]));
            audio2_set_frequency_hz(kFreqs[idx]);
            audio3_set_frequency_hz(kFreqs[idx]);
            next_switch_ms += TONE_DWELL_MS;
        }

        APP_IDLE();  // low-jitter idle (WFI in Release, NOP in Debug)
    }
}
