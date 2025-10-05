/*
 * myCode.h
 *
 *  Header for the STM32CubeIDE getting-started audio project
 *  on the STM32F407 Discovery board. Include this from main.c.
 */

#ifndef INC_MYCODE_H_
#define INC_MYCODE_H_

#include "Audio_Drivers.h"
#include "audio_replacement.h"



// Public API
void bsp_board_init(void);
void app_init_buffers_and_start(void);
void app_loop(void);

// Audio DMA callbacks (called by audio driver/ISR)
void myAudioHalfTransferCallback(void);
void myAudioTransferCompleteCallback(void);

#endif /* INC_MYCODE_H_ */
