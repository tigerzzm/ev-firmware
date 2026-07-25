#!/usr/bin/env bash
# Host compile-check + math unit tests for the EV firmware.
# Not a real ARM build — it uses mock Pico/BNO08x headers (host_check/stubs) to
# catch syntax/type errors, and natively runs the pure-math unit tests.
set -u
cd "$(dirname "$0")/.."          # repo root
STUBS=host_check/stubs
fail=0
echo "== syntax check (mock headers) =="
# imu.cpp checked from the stubs dir so its BNO08x quote-includes hit the stubs
cp src/imu.cpp "$STUBS/__imu.cpp"
g++ -std=gnu++17 -fsyntax-only -Wall -Wextra -Wno-unused-parameter -I"$STUBS" -Isrc "$STUBS/__imu.cpp" \
  && echo "OK   src/imu.cpp" || fail=1
rm -f "$STUBS/__imu.cpp"
for f in odo pose drive path steer speed ui lcd sm main utils pid motor_controller; do
  g++ -std=gnu++17 -fsyntax-only -Wall -Wextra -Wno-unused-parameter -I"$STUBS" -Isrc "src/$f.cpp" \
    && echo "OK   src/$f.cpp" || fail=1
done
echo; echo "== math unit tests =="
g++ -std=gnu++17 -Isrc host_check/test_math.cpp src/path.cpp src/utils.cpp src/speed.cpp src/motor_controller.cpp -o /tmp/ev_test_math \
  && /tmp/ev_test_math || fail=1
echo; [ $fail -eq 0 ] && echo "== ALL CHECKS PASS ==" || { echo "== FAILURES ABOVE =="; exit 1; }
