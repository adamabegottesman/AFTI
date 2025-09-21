# Contributing to AFTI

Thanks for helping build artefact‑free TI EEG!

## Ways to contribute
- File **bug reports** with steps to reproduce and expected behavior.
- Propose **features** with a clear problem statement and success criteria.
- Improve **docs** (build steps, diagrams, measurements).
- Submit **PRs** for firmware, tests, or examples.

## Workflow
1. **Fork** and create a branch: `feat/<short-name>`.
2. Run local checks (`scripts/checks.sh`) before opening a PR.
3. Open a PR that links to an issue. Describe **what** and **why**.
4. One reviewer approval + passing CI required.

## Coding guidelines (firmware)
- CMSIS-first, no HAL. Prefer explicit register access.
- Keep ISRs short; use DMA and double-buffering patterns.
- Document clock tree assumptions (PLL, PLLI2S, MCLK).
- Provide minimal examples in `examples/` for new subsystems.
- Add unit/host tests where feasible (DSP, algorithms).

## Commit style
Follow Conventional Commits:
- `feat:`, `fix:`, `docs:`, `refactor:`, `test:`, `build:`, `ci:`

## Development environment
- STM32CubeIDE **or** `arm-none-eabi-gcc` + `make`
- ST-LINK CLI or CubeProgrammer for flashing
