# EV — Build & Flash

Getting `ev_firmware.uf2` onto the Pico 2. Two paths; pick one.

## Prerequisites

- The project folder (`code/`) — you have it.
- A **Pico 2 (RP2350)** and a USB cable (data, not charge-only).
- Pico SDK **≥ 2.0** (RP2350 support). The bundled `CMakeLists.txt` sets
  `PICO_BOARD pico2` and will fetch the SDK from git if `PICO_SDK_PATH` isn't set.

## Path A — VS Code + Raspberry Pi Pico extension (easiest)

The `CMakeLists.txt` already keeps the extension's header block, so:

1. Install the **Raspberry Pi Pico** extension in VS Code (it installs the SDK,
   toolchain, and CMake tools for you).
2. Open the `code/` folder in VS Code.
3. When prompted, select the SDK version (**2.0+**) and board **Pico 2 / RP2350**.
4. Hit **Compile** (or the extension's build button). Output lands in `build/`.
5. To flash: hold **BOOTSEL** on the Pico while plugging in USB (it mounts as a drive
   `RPI-RP2`), then use the extension's **Run/Flash**, or drag `build/ev_firmware.uf2`
   onto the drive.

## Path B — Command line

```bash
cd code
mkdir build && cd build
cmake ..            # PICO_BOARD is pinned to pico2; fetches SDK if needed
make -j             # produces ev_firmware.uf2
```
If you have the SDK locally, `export PICO_SDK_PATH=/path/to/pico-sdk` first to skip
the git fetch.

**Flash:** hold **BOOTSEL**, plug in USB → the Pico mounts as `RPI-RP2` →
copy the uf2:
```bash
cp ev_firmware.uf2 /path/to/RPI-RP2      # or drag-and-drop in a file manager
```
The Pico reboots and runs it.

## Serial monitor (you'll live here during bring-up)

USB stdio is **on**, UART **off** (set in `CMakeLists.txt`). After flashing, open the
Pico's USB serial port at any baud (USB CDC ignores baud):

- VS Code: the extension's **Serial Monitor**, or
- `screen /dev/tty.usbmodem* 115200` (macOS) / `minicom` / PuTTY, or
- the Arduino IDE serial monitor pointed at the Pico's port.

You should see the boot banner:
```
=== EV firmware boot (Pico 2) ===
Initializing BNO085 (Game Rotation Vector, mag-free)...
IMU I2C connection established (SDA=GP4, SCL=GP5).
IMU ready.
```
If you get that far, hardware bring-up begins — go to `BRINGUP_AND_TUNING.md`.

## First-compile gotchas (expected, minor)

- If `cmake` can't find the SDK: set `PICO_SDK_PATH` or let it fetch from git
  (needs internet the first time).
- If a `hardware_*` library is missing at link time, confirm it's in the
  `target_link_libraries(...)` list in `CMakeLists.txt` (currently: `pico_stdlib
  pico_multicore hardware_pio hardware_i2c hardware_pwm hardware_adc`).
- The firmware was compile-checked against mock headers, not a real ARM build, so a
  stray include or type nudge is possible — send me any compiler error and I'll fix
  it fast.

## Sanity note about the servo PWM clock

`steer.cpp` assumes a **150 MHz** system clock to get a 1 µs PWM tick (`clkdiv 150`,
`wrap 20000` → 50 Hz). 150 MHz is the Pico 2 default. If you change the system clock,
adjust `clkdiv` so the servo frame stays 20 ms.
