#ifndef INC_DEBUG_PRINT_H_
#define INC_DEBUG_PRINT_H_

#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Call once after clocks are set and SystemCoreClock is correct.
void DebugPrint_Init(uint32_t swo_baud);
uint32_t DebugPrint_InitAuto(uint32_t desired_swo_hz);
// Send one character / a buffer (non-blocking; may drop if viewer not started)
void DebugPrint_PutChar(uint8_t ch);
void DebugPrint_Write(const void* buf, uint32_t len);

// printf retargets
int __io_putchar(int ch);
int _write(int fd, const char* buf, int len);

#ifdef __cplusplus
}
#endif



#endif /* INC_DEBUG_PRINT_H_ */
