# STM32F769I-DISCO – FreeRTOS Minimal Baseline

> **Project intent:**  
> Build a clean, minimal STM32 + FreeRTOS baseline to understand hardware behavior, HAL internals, RTOS interactions, performance bottlenecks, and debugging pitfalls.

---

## 1. Project Overview

This project is a **from-scratch STM32CubeIDE project** for the **STM32F769I-DISCO** board with:

- **FreeRTOS enabled**
- **Only essential peripherals active**
- **User LED blink as verification**

The focus is **learning by reduction**: remove everything unnecessary, then add features incrementally with understanding.

---

## 2. Learning Goals

By working on this project, the following knowledge is targeted:

- STM32F7 boot flow (Reset → HAL → Clock → RTOS)
- HAL vs hardware responsibilities
- FreeRTOS scheduling and task behavior
- Peripheral initialization pitfalls
- Debugging startup hangs and RTOS issues
- Memory and performance impact of HAL + RTOS

---

## 3. System Requirements

### Functional Requirements

| ID | Requirement |
|----|------------|
| FR-01 | System shall boot on STM32F769I-DISCO |
| FR-02 | FreeRTOS scheduler shall start successfully |
| FR-03 | At least one RTOS task shall execute |
| FR-04 | User LED shall toggle periodically |

### Non-Functional Requirements

| ID | Requirement |
|-----|------------|
| NFR-01 | Project shall be debuggable using STM32CubeIDE |
| NFR-02 | Project shall be portable across PCs |
| NFR-03 | Unused peripherals shall be disabled |
| NFR-04 | Design shall favor clarity over optimization |

---

## 4. Hardware Context

- **Board:** STM32F769I-DISCO  
- **MCU:** STM32F769NIH6 (Cortex-M7)  
- **Debugger:** On-board ST-LINK  

### LED Behavior (Important)

- User LEDs are **active-low**
- Writing `0` → LED ON  
- Writing `1` → LED OFF  

📘 Reference: *UM2033 – Section 5.16 (Buttons and LEDs)*

---

## 5. Design Decisions

### 5.1 Project Creation

- Created using **Board Selector** in STM32CubeIDE  
**Why:** Ensures correct pin mapping, clock sources, and LED polarity.

### 5.2 Peripheral Strategy

#### Enabled
- RCC
- SYS
- GPIO
- Cortex-M7 core
- FreeRTOS (CMSIS-RTOS v1)

#### Disabled (Intentionally)
- Ethernet
- LCD / DSI / LTDC / DMA2D
- USB
- Audio (SAI / SPDIF)
- Camera (DCMI)
- SDMMC / QSPI
- RTC
- CRC
- ADCs
- All communication peripherals

**Why:**  
Unused peripherals often block startup, wait on clocks, or cause HAL deadlocks.

---

## 6. RTOS Architecture

### Execution Model
- FreeRTOS controls execution
- No application logic in `while(1)`

### Tasks

| Task | Purpose |
|------|---------|
| DefaultTask | Toggle user LED |

### Timing Rules
- `osDelay()` inside tasks  
- `HAL_Delay()` avoided after scheduler start

---

## 7. Project Structure

```
02_freertos_minimal/
├── README.md
├── cubeide/
│   ├── Core/
│   ├── Drivers/
│   ├── Middlewares/
│   ├── STM32F769I.ioc
│   └── *.launch
└── .gitignore
```

---

## 8. Build & Debug

### Clone
```bash
git clone <repo-url>
```

### Import
STM32CubeIDE → File → Import → Existing STM32CubeIDE Project

### Build
Click **Build**

### Debug
Click **Debug**

Expected: LED blinking.

---

## 9. Verification Criteria

| Check | Expected |
|------|----------|
| Build | Success |
| Debug | No hang |
| Scheduler | Running |
| LED | Blinks |
| Faults | None |

---

## 10. Known Issues

- Startup hangs due to unused peripherals
- ST-LINK server conflicts
- RTC/Ethernet common blockers

---

## 11. Commit Guidelines

```
[02] freertos-baseline: initial bring-up
[02] freertos-baseline: LED blink verified
```

---

## 12. Future Work

- UART logging
- Multi-task scheduling
- Stack analysis
- Interrupt to task signaling
- Memory footprint analysis

---

## 13. Final Note

> This project is a learning lab, not production firmware.
