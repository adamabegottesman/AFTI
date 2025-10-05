#ifndef AUDIO_STACK_PCM5102_B_H
#define AUDIO_STACK_PCM5102_B_H
#include <stdint.h>

int  audio3_init(uint32_t fs_hz);
void audio3_start(int32_t *play_buf, uint32_t play_buf_bytes);

#endif /* AUDIO_STACK_PCM5102_B_H */
