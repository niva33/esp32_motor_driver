# ESP32 Motor Driver (omni / bdc) — esp32_motor_driver

Compact, well-organized ESP-IDF project for driving omni/BDC motors on ESP32-family MCUs. This repository contains firmware, BSP layers, libraries, examples, and tools used to build, flash and test a motor driver on ESP32 devices (project developed for personal use and experimentation).

## Highlights
- ESP-IDF based project (tested with ESP-IDF v5.x — workspace includes v5_5_1 layout)
- Modular layout: `app/`, `main/`, `bsp/`, `libs/`, `service/`, and `testing/` directories
- Build & flash using `idf.py` (recommended) or CMake/Ninja workflows
- Includes motor control algorithms (PID), encoder support, and convenient console utilities

## Repository layout

- `app/` — application entry & core modules (omni motor control, app-level CMake)
- `bsp/` — board support package and low-level hardware glue (GPIO, PWM, ADC mappings)
- `main/` — main application (C entry, example usage)
- `service/` — auxiliary services and daemons used by the firmware
- `libs/` — reusable libraries (bdc_motor, pid_ctl, ether_, ...)
- `testing/`, `ultilis/` — test utilities, tuners, and console helpers
- `build/` — build output (ignored in version control normally)
- `sdkconfig`, `CMakeLists.txt` — project config and top-level build files

See the top-level `CMakeLists.txt` and per-folder `CMakeLists.txt` for build wiring.

## Quick contract

- Inputs: compiled firmware built by the ESP-IDF toolchain; GPIO/power connections from the host to motors/encoders.
- Outputs: motor control PWM, encoder readouts, console diagnostics, optional telemetry via serial or network.
- Success criteria: firmware builds without errors for the selected target, flashes to device, and basic motor commands produce expected motor movement.

## Prerequisites

- Windows 10/11 (PowerShell) or any platform supported by ESP-IDF
- ESP-IDF toolchain installed (tested with ESP-IDF v5.x). Follow official setup:

	- Install Python 3.10+ and required packages
	- Install the ESP-IDF tools and set up environment variables

See https://docs.espressif.com/projects/esp-idf for detailed setup steps.

If you have the `esp-idf` extension installed or use the IDF Command Prompt, the following commands assume your environment is active.

## Build & Flash (PowerShell)

Open a PowerShell session with ESP-IDF environment activated (or run the `export`/`set` scripts provided by your ESP-IDF install). Replace <PORT> with your device COM port (e.g., COM3).

Build the project:

```powershell
# From repository root
idf.py set-target esp32s3
idf.py build
```

Flash the firmware and monitor serial output:

```powershell
idf.py -p COM3 flash monitor
```

If you prefer a separate flash step:

```powershell
idf.py -p COM3 flash
idf.py monitor
```

Notes:
- If you use a specific ESP32 variant (esp32s3, esp32s2, etc.), set the target appropriately: `idf.py set-target esp32s3`.
- If the project is pre-configured with a `sdkconfig`, you can inspect or tweak it with `idf.py menuconfig`.

## Configuration

- `sdkconfig` contains build-time configuration (peripherals enabled, driver options, FreeRTOS settings). Run:

```powershell
idf.py menuconfig
```

to adjust pin mappings, PWM frequency, PID defaults, or other options.

## Wiring / Hardware

This repository supports typical motor driver setups (PWM outputs to motor driver, encoder inputs, power sensing). Typical connections:

- Motor driver PWM inputs: connect to configured PWM-capable GPIOs
- Motor driver direction inputs: connect to GPIOs if using H-bridge direction pins
- Encoder A/B channels: connect to GPIOs configured for pulse counting or interrupts
- Power/Ground: common ground between ESP32 board and motor driver; ensure motor power supply can handle stall currents

Pin mappings are defined in the BSP headers under `bsp/include/omni_bsp.h` and `bsp/omni_bsp.c`. Inspect and adjust there or in `menuconfig` as needed.

Safety
- Add motor disconnect / kill switch and current limiting in hardware where possible.
- Start with no-load tests and low PWM duty to verify direction and encoder counts before applying full power.

## Usage

- Use the console utilities under `ultilis/omni_console` or the `main`/`app` entry to send commands to the motor controller.
- Typical workflow:
	1. Build & flash
	2. Open monitor: `idf.py monitor`
	3. From the console, run commands like `motor start`, `motor stop`, `motor set-speed <id> <rpm>` (commands depend on the console implementation in `ultilis`).

Refer to `service/` and `app/` code for exact CLI commands and RPC/IPC endpoints.


## Tests & PID Tuning

- `testing/pid_tuning/` contains utilities for tuning PID loops. You can run on-device tests and log results to serial.
- Implement unit tests or hardware-in-the-loop tests under `testing/` as needed.

## Development notes

- To add a library, place it under `libs/` and add a matching `CMakeLists.txt` to include it in the build.
- Keep hardware-specific pin mappings in `bsp/` so higher-level code stays portable.

## CI / Build hints

- The project uses CMake with ESP-IDF. For automated builds, ensure the CI runner installs ESP-IDF and exports environment variables before calling `idf.py build`.

## Contributing

1. Fork the repo
2. Create a branch for your change
3. Make changes and add tests if applicable
4. Run `idf.py build` and verify changes
5. Open a pull request with a clear description

Please follow existing code style and add a short test where possible.

## License

This repository does not include a license file by default. If you want to publish or share the code, add a `LICENSE` file (MIT, Apache-2.0, or whichever license you prefer).

## Contact / Authors

Repo owner: vlata135 

— End of README —

