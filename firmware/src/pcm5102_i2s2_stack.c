// pcm5102_i2s2_stack.c — thin init/start layer for I2S2 + PCM5102
// CMSIS-only: uses i2s2_dma.c (no HAL)

#include "pcm5102_i2s2_stack.h"
#include "i2s2_dma.h"    // dma_i2s2_init(), i2s2_init(), audio2_dma_start()
#include <stddef.h>

int audio2_init(uint32_t fs_hz)
{
  dma_i2s2_init();
  i2s2_init(fs_hz);
  return 0;
}

void audio2_start(int32_t *play_buf, uint32_t play_buf_bytes)
{
  if (play_buf == NULL) return;
  if ((play_buf_bytes & 0x7u) != 0u) return;   // must be multiple of 8 bytes
  audio2_dma_start(play_buf, play_buf_bytes);
}

/* NOTE:
   - audio2_set_callbacks(on_half, on_full) is declared in this stack header
     but DEFINED in i2s2_dma.c next to the ISR — do not define it here to avoid
     multiple-definition at link time.
*/
