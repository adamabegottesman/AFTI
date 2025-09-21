#ifndef INC_AUDIO_H_
#define INC_AUDIO_H_

#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// 1) Init all audio-related peripherals (GPIO/I2C/I2S/DMA/codec), same as your current init
void audio_init_peripherals(uint16_t *workingBuf, uint32_t workingLen,
                            uint16_t *sineBuf,    uint32_t sineLen);

// 2) Start playback (same behavior as your start function)
void audio_start_playback(uint16_t *buf, uint32_t len);

// 3) Change the active TX buffer (DMA double-buffer style)
void audio_change_buffer(uint16_t *buf, uint32_t len);

// 4) DMA callbacks (to be called from IRQ handlers)
void audio_tx_half_isr(void);
void audio_tx_complete_isr(void);

// (optional later) configure audio PLL/I2S
void audio_plli2s_config(uint32_t n, uint32_t r);

#ifdef __cplusplus
}
#endif


#endif /* INC_AUDIO_H_ */
