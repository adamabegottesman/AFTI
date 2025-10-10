#include <adcmon_i2s2_app.h>
#include <stdint.h>
#include <stdio.h>

#include "stm32f4xx.h"
#include "bsp_clock.h"
#include "timing.h"
#include "audio_project_config.h"
#include "debug_print.h"

#include "ads1256.h"
#include "dds_tone_i2s3.h"
#include "dds_tone_i2s2.h"
#include "pcm5102_i2s3_stack.h"
#include "pcm5102_i2s2_stack.h"

#ifndef SWO_CLK
#define SWO_CLK 1894700
#endif

int main(void)
{
  /* ===== 0) Clocks, timing, and SWO printf ===== */
  system_clock_init_168mhz();
  timing_init_1khz();
  DebugPrint_Init(SWO_CLK);
  printf("\r\n=== Boot ===\r\n");

  /* ===== 1) ADS1256 bring-up (pins + SPI) ===== */
  ads1256_init_pins_spi();

  /* ===== 2) ADS1256 config & init ===== */
  ads1256_cfg_t cfg = {
    .mux        = 0x06,                 /* AIN0(+) only */
    .pga        = ADS1256_PGA_8,
    .drate      = ADS1256_DR_30000SPS,  /* match monitor src rate below */
    .continuous = true,
    .on_sample  = NULL
  };
  ads1256_init(&cfg);

  /* ===== 3) I2S3: DDS tone (unchanged) ===== */
  audio3_app_prefill();
  audio3_init(AUDIO_FS_HZ);
  audio3_start(audio3_playbuff(), audio3_playbuff_bytes());
  audio3_set_frequency_hz(2000u);       /* tone on I2S3 only */

  /* ===== 4) I2S2: ADS1256 monitor =====
     Prepare monitor, bind to I2S2 TX buffer, and hook DMA callbacks. */
  adcmon2_app_prefill();
  adcmon2_set_src_rate_hz(30000u);      /* keep in lockstep with DRATE */
  adcmon2_bind_output_buffer(audio2_playbuff(), AUDIO_PB_WORDS);
  audio2_set_callbacks(adcmon2_on_half_transfer, adcmon2_on_transfer_complete);

  /* Init / start I2S2 using the usual audio2 buffer */
  audio2_init(AUDIO_FS_HZ);
  audio2_start(audio2_playbuff(), audio2_playbuff_bytes());

  /* ===== 5) Idle ===== */
  while (1) {
    /* spin */
  }
}
