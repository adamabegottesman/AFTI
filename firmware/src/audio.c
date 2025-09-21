#include "audio.h"

// Declare your existing symbols (no code changes to your old files)
extern void myAudioInitialisePeripherals(uint16_t*, uint32_t, uint16_t*, uint32_t);
extern void myAudioStartPlaying(uint16_t*, uint32_t);
extern void myAudioChangeBuffer(uint16_t*, uint32_t);
extern void myAudioHalfTransferCallback(void);
extern void myAudioTransferCompleteCallback(void);
extern void myConfigureI2SClock(uint32_t n, uint32_t r);

void audio_init_peripherals(uint16_t *workingBuf, uint32_t workingLen,
                            uint16_t *sineBuf,    uint32_t sineLen)
{
  myAudioInitialisePeripherals(workingBuf, workingLen, sineBuf, sineLen);
}

void audio_start_playback(uint16_t *buf, uint32_t len)
{
  myAudioStartPlaying(buf, len);
}

void audio_change_buffer(uint16_t *buf, uint32_t len)
{
  myAudioChangeBuffer(buf, len);
}

void audio_tx_half_isr(void)
{
  myAudioHalfTransferCallback();
}

void audio_tx_complete_isr(void)
{
  myAudioTransferCompleteCallback();
}

void audio_plli2s_config(uint32_t n, uint32_t r)
{
  myConfigureI2SClock(n, r);
}
