#ifndef AUDIO3_APP_H
#define AUDIO3_APP_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* -------------------------------------------------------------------------- */
/* PCM5102 #2 tone generator (I2S3 path)                                      */
/* Owns a circular, interleaved stereo buffer (L,R,...) with 24-in-32 samples */
/* The I2S3/DMA driver calls the half/complete callbacks from ISR context.    */
/* -------------------------------------------------------------------------- */

/** Prefill the entire circular playback buffer (builds sine LUT as needed). */
void      audio3_app_prefill(void);

/** Opaque accessors so callers don't need global symbols or sizes. */
int32_t*  audio3_playbuff(void);       /* pointer to interleaved L/R buffer   */
uint32_t  audio3_playbuff_bytes(void); /* total byte size of the play buffer  */

/** Change the DDS frequency from foreground (keeps phase continuity). */
void      audio3_set_frequency_hz(uint32_t hz);

/** ISR callbacks invoked by the I2S3 DMA handler. Keep them short/nonblocking. */
void      audio3_on_half_transfer(void);
void      audio3_on_transfer_complete(void);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* AUDIO3_APP_H */
