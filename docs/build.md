# Build & Flash

## Toolchain options
- **STM32CubeIDE** (recommended)
- `arm-none-eabi-gcc` + `make`
- ST-LINK CLI or CubeProgrammer

## SWO logging
- Pin: PB3 (TRACESWO)
- Suggested bitrate: 2 MHz

## Flashing (CLI example)
```bash
# Using STM32_Programmer_CLI
STM32_Programmer_CLI -c port=SWD -w build/firmware.bin 0x08000000 -v -rst
```
