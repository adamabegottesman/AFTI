# Architecture Overview

**Goal**: artefact‑free EEG during Temporal Interference (TI) stimulation.

- **MCU**: STM32F407 @168 MHz
- **Audio path**: I2C1 (codec ctrl), I2S3 TX + DMA ping/pong
- **Timing/logging**: SysTick 1 kHz, SWO printf
- **Signal gen**: DDS/sine scaffolding (for TI experiments)
- **Planned**: ADC EEG path, adaptive canceller (LMS/RLS), synchronization & metrics.
