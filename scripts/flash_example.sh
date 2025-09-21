#!/usr/bin/env bash
set -euo pipefail
STM32_Programmer_CLI -c port=SWD -w build/firmware.bin 0x08000000 -v -rst
