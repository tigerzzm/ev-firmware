# host_check — offline compile & math checks

Not a real RP2350 build. This harness lets you sanity-check the firmware on any
machine with `g++`, without the Pico SDK or ARM toolchain:

- **Syntax/type check** of every module against **mock** pico-sdk / BNO08x headers
  in `stubs/` (matching only the API surface the firmware uses).
- **Unit tests** for the hardware-independent math (arc geometry, angle utils, the
  speed trapezoid), compiled and run natively.

Run it:

```bash
bash host_check/run.sh
```

Green here means "no compile/type errors and the math is correct" — it does NOT
replace building with the real SDK (`cmake && make`) or bench testing. The stubs are
deliberately minimal; if you add a new pico-sdk call, extend the matching stub.
