# Firmware (STM32F407, CMSIS-only)

Minimal, no-HAL firmware targeting STM32F407 Discovery with CS43L22 codec.

## Features (current)
- System clock @168 MHz (HSE → PLL)
- I2C1 control of CS43L22
- I2S3 TX with **DMA ping/pong**
- SWO printf (PB3), SysTick 1 kHz
- Sine/DDS scaffolding

## Build (STM32CubeIDE)
1. Open this folder as a project.
2. Build `stm32f407-audio` target.
3. Flash with CubeProgrammer.

## Notes
- Ensure PLLI2S is configured for the selected sample rate.
- SWO bitrate 2 MHz (set in debugger).