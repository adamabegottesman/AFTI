#include "myCode.h"
#include "bsp_clock.h"
#include "audio.h"
//#include "debug_print.h"
#include <stdio.h>
#include "uart.h"

#define GPIODEN				   (1U<<3)
#define GPIOD_14			   (1U<<14)

#define LED_PIN				   GPIOD_14

char key;

#define SWO_CLK 1894700

int main(void)
{
	SystemInit();
	bsp_clock_init_168mhz();
	bsp_board_init();
	app_init_buffers_and_start();
	//DebugPrint_Init(SWO_CLK);
	//printf("PPrinting enabled!\n");
	uart2_rxtx_init();

	/*1.Enable clock access to GPIOA*/
	RCC->AHB1ENR |= GPIODEN;

	/*2.Set PD14 as output pin*/
	GPIOD->MODER |= (1U<<28);
	GPIOD->MODER &= ~(1U<<29);

  while (1)
  {
	 app_loop();
  }

}

