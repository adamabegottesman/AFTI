#include "myCode.h"
#include "bsp_clock.h"
#include "audio.h"
#include "debug_print.h"
#include <stdio.h>
#define SWO_CLK 1894700

int main(void)
{
	SystemInit();
	bsp_clock_init_168mhz();
	bsp_board_init();
	app_init_buffers_and_start();
	DebugPrint_Init(SWO_CLK);
	printf("PPrinting enabled!\n");

  while (1)
  {
	app_loop();

  }
}
