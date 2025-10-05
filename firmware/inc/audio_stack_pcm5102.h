#ifndef AUDIO_STACK_PCM5102_H
#define AUDIO_STACK_PCM5102_H
#include <stdint.h>

int  audio2_init(uint32_t fs_hz);
void audio2_start(int32_t *play_buf, uint32_t play_buf_bytes);

#endif /* AUDIO_STACK_PCM5102_H */
