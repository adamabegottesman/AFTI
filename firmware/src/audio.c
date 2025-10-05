#include "audio.h"

// Driver API (must match Audio_Drivers.h / .c)
extern void myAudioInitialisePeripherals(int32_t*, uint32_t, int32_t*, uint32_t);
extern void myAudioStartPlaying        (int32_t*, uint32_t /*PBSIZE bytes*/);
extern void myAudioChangeBuffer        (int32_t*, uint32_t /*size_words*/);
extern void myAudioHalfTransferCallback(void);
extern void myAudioTransferCompleteCallback(void);
extern void myConfigureI2SClock(uint32_t n, uint32_t r);


void audio_init_peripherals(int32_t *workingBuf, uint32_t workingLen,
                            int32_t *sineBuf,    uint32_t sineLen)
{
  myAudioInitialisePeripherals(workingBuf, workingLen, sineBuf, sineLen);
}

void audio_start_playback(int32_t *buf, uint32_t len)
{
  myAudioStartPlaying(buf, len);
}

void audio_change_buffer(int32_t *buf, uint32_t len)
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
