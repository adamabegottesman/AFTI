// audio_stack_pcm5102_b.c — thin init/start for PCM5102 #2 (I2S3)
#include "audio_stack_pcm5102_b.h"
#include "i2s3_dma_pcm.h"

int audio3_init(uint32_t fs_hz)
{
  dma_i2s3_init_pcm();
  i2s3_init_pcm(fs_hz);
  return 0;
}

void audio3_start(int32_t *play_buf, uint32_t play_buf_bytes)
{
  if ((play_buf_bytes & 0x7u) != 0u) return;
  audio3_dma_start(play_buf, play_buf_bytes);
}
