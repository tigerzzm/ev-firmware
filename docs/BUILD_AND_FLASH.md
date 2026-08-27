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

## Path B — Command line (VERIFIED 2026-08-22)

The Pico VS Code extension already installed cmake, ninja, the ARM toolchain, the SDK
and picotool under `~/.pico-sdk/` — but **none of them are on your PATH**, so a bare
`cmake ..` fails with "command not found". Use the extension's copies:

```bash
cd ~/Documents/EV_2026/ev-firmware
export PATH="$HOME/.pico-sdk/cmake/v4.3.4/bin:$HOME/.pico-sdk/ninja/v1.13.2:$PATH"
export PICO_SDK_PATH="$HOME/.pico-sdk/sdk/2.3.0"
export PICO_TOOLCHAIN_PATH="$HOME/.pico-sdk/toolchain/15_2_Rel1"

cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release \
  -DPICO_SDK_PATH="$PICO_SDK_PATH" \
  -DPICO_TOOLCHAIN_PATH="$PICO_TOOLCHAIN_PATH" \
  -DPICO_BOARD=pico2

cmake --build build          # -> build/ev_firmware.uf2
```

Version numbers in those paths will drift as the extension updates — check with
`ls ~/.pico-sdk/cmake ~/.pico-sdk/ninja ~/.pico-sdk/toolchain ~/.pico-sdk/sdk`.

Last verified build: **117/117 targets, zero warnings**, 63.6 KB flash / 7.9 KB RAM.

**Flash:** hold **BOOTSEL**, plug in USB → the Pico mounts as `RPI-RP2` →
copy the uf2:
```bash
cp build/ev_firmware.uf2 /Volumes/RPI-RP2     # macOS; drag-and-drop also works
```
Or use picotool (no BOOTSEL needed if the board is already running this firmware):
```bash
~/.pico-sdk/picotool/2.3.0/picotool/picotool load -x build/ev_firmware.uf2
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
