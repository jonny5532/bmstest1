# GitHub Copilot Instructions for RP2350 BMS Project

## Project Overview
This is a C firmware project for a Battery Management System (BMS) running on the Raspberry Pi Pico (RP2350). It uses the Pico SDK and CMake build system.

## Architecture & Patterns

### 1. Global Data Model (`model.h`)
- **Central State**: The system state is centralized in a global singleton `bms_model_t model`.
- **Usage**: All components (state machines, hardware drivers) read from and write to this global structure.
- **Fields**: Contains sensor data (`current_mA`, `cell_voltages_mv`), timestamps (`current_millis`), and state machine contexts (`contactor_sm`).

### 2. State Machines (`state_machines/`)
- **Framework**: Logic is implemented using a lightweight state machine framework defined in `state_machines/base.h`.
- **Structure**:
  - Use `sm_t` for the state machine context.
  - Implement a `_tick(bms_model_t *model)` function called in the main loop.
  - Use `state_transition(sm, NEW_STATE)` to change states.
  - Use `state_timeout(sm, timeout_ms)` for time-based transitions.
- **Example**: See `state_machines/contactors.c` for a reference implementation.

### 3. Hardware Abstraction (`hw/`)
- **Drivers**: Peripheral drivers are located in `hw/` (e.g., `duart.c`, `internal_adc.c`, `pwm.c`).
- **Time**: Use `hw/time.h` for system time. `millis_t` is the standard time type.
- **Safety**: Always check data freshness using `millis_recent_enough(timestamp, max_age_ms)` before acting on sensor data.

### 4. Communication Protocols
- **ISOSPI**: Implemented in `isospi/` using a custom PIO program (`isospi_master.pio`).
- **BATMan**: The battery monitoring protocol (likely for LTC68xx chips) is in `batman/`. It uses the ISOSPI driver.
- **DUART**: A custom DMA-driven Dual UART implementation (`hw/duart.c`) handles external communication using ring buffers and packet framing.

## Developer Workflow

### Build System
- **CMake**: The project uses standard CMake with the Pico SDK.
- **Commands**:
  ```bash
  mkdir build && cd build
  cmake ..
  make -j4
  ```
- **Output**: The build produces `bms.uf2` for flashing.

### Debugging
- **USB Output**: `stdio_usb_init()` is used. `printf` output goes to the USB serial connection.
- **Logging**: Use `printf` for debug logs.

## Coding Conventions
- **C Standard**: C11.
- **Naming**: `snake_case` for functions and variables.
- **Safety First**:
  - Validate sensor data age.
  - Handle timeouts in state machines.
  - Use `volatile` and `atomic` types where appropriate for interrupt-shared data (though the current codebase relies heavily on the main loop).

## Key Files
- `bms.c`: Main entry point and loop.
- `model.h`: Global state definition.
- `state_machines/base.h`: State machine primitives.
- `isospi/isospi_master.pio`: PIO assembly for ISOSPI.
