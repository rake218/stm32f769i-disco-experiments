# STM32F769I-DISCO Learning Lab

This repository contains hands-on experiments using the **STM32F769I-DISCO** board
with **STM32CubeIDE**.

The purpose of this repository is to learn STM32 hardware and software behavior
step by step, while documenting:
- requirements
- architecture and design decisions
- implementation details
- verification steps
- results and learnings

Each experiment is self-contained and reproducible on any PC using STM32CubeIDE.

---

## Hardware
- Board: STM32F769I-DISCO
- MCU: STM32F769NIH6 (ARM Cortex-M7)

---

## Software
- STM32CubeIDE (Windows)
- STM32 HAL
- FreeRTOS (used in later projects)
- On-board ST-LINK debugger

---
## How to Use This Repository

- Clone the repository
- Open STM32CubeIDE
- Import the project from projects/<experiment>/cubeide
- Build, flash, and debug on STM32F769I-DISCO

---
## Repository Structure

```text
projects/
└── <experiment_name>/
    ├── docs/        → requirements, design, verification, results
    └── cubeide/     → STM32CubeIDE project (buildable)

---

