# AFTI: Artefact‑Free TI EEG – Starter Repo

This repository documents and hosts firmware and docs for building an **artefact‑free EEG recording** pipeline during **Temporal Interference (TI) stimulation**.

> **Status**: firmware groundwork in place (MCU, audio/DMA, timing/logging, DDS scaffolding). EEG capture and adaptive artefact cancellation TBD.

## Quick start
1. **Clone**:  
   ```bash
   git clone https://github.com/<your-org>/afti.git
   cd afti
   ```
2. **Install tools** (suggested):
   - STM32CubeIDE (F4 series) or arm-none-eabi-gcc + make
   - ST-LINK CLI or STM32CubeProgrammer
3. **Build & flash** (CubeIDE):
   - Open `firmware/` as workspace project.
   - Build the `stm32f407-audio` target.
   - Flash to an STM32F407 Discovery board.
4. **Run**:
   - Observe SWO logs at 2 MHz on `PB3`.
   - Audio codec (CS43L22) streams from DMA ping/pong buffer.

## Repo structure
```
.
├─ firmware/                 # Source for STM32F4 (CMSIS-only, no HAL)
│  ├─ src/
│  ├─ inc/
│  ├─ linker/
│  ├─ third_party/
│  └─ README.md
├─ docs/                     # Living documentation
│  ├─ build.md               # Toolchain, flashing, debug
│  ├─ architecture.md        # System & signal-path overview
│  ├─ hardware.md            # Board, codec, pinout, clocks
│  ├─ stimulation.md         # TI wave synthesis plan
│  ├─ eeg.md                 # EEG acquisition plan
│  └─ adaptive-canceller.md  # Canceller design & metrics
├─ examples/                 # Minimal buffer-fill & sine/DDS demo
├─ scripts/                  # Helper scripts (flash, format, checks)
├─ .github/
│  ├─ ISSUE_TEMPLATE/
│  │  ├─ bug_report.md
│  │  └─ feature_request.md
│  └─ workflows/
│     └─ ci.yml             # Lint docs & structure checks
├─ CHANGELOG.md
├─ CONTRIBUTING.md
├─ CODE_OF_CONDUCT.md
├─ LICENSE
└─ README.md
```

## Roadmap (high‑level)
- [x] MCU bring-up, clocks @168 MHz, SWO logs
- [x] Audio path (I2C + I2S + DMA ping/pong)
- [ ] EEG ADC front-end & synchronization
- [ ] Adaptive artefact canceller (LMS/RLS) with tests
- [ ] Integrated end-to-end demo (stim + record + cancel)
- [ ] Bench & in‑situ validation, metrics & plots

## How to contribute
See [CONTRIBUTING.md](CONTRIBUTING.md). Please open issues with minimal repro details and expected outcomes.

## License
MIT – see [LICENSE](LICENSE).
