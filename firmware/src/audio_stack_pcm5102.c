// audio_stack_pcm5102.c — thin init/start layer for PCM5102 (no I2C)
#include "audio_stack_pcm5102.h"
#include "i2s2_dma.h"

int audio2_init(uint32_t fs_hz)
{
  dma_i2s2_init();
  i2s2_init(fs_hz);
  return 0;
}

void audio2_start(int32_t *play_buf, uint32_t play_buf_bytes)
{
  if ((play_buf_bytes & 0x7u) != 0u) return;
  audio2_dma_start(play_buf, play_buf_bytes);
}
