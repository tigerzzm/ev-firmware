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
for f in odo pose drive plan steer steer_law speed ui lcd sm cal main utils; do
  g++ -std=gnu++17 -fsyntax-only -Wall -Wextra -Wno-unused-parameter -I"$STUBS" -Isrc "src/$f.cpp" \
    && echo "OK   src/$f.cpp" || fail=1
done
echo; echo "== math unit tests =="
# Built in a private temp dir, not a fixed /tmp path, so two harness runs at once
# can't overwrite or run each other's binary.
bin=$(mktemp -d) || { echo "FAIL could not create a temp directory"; exit 1; }
[ -n "$bin" ] && [ -d "$bin" ] || { echo "FAIL bad temp directory '$bin'"; exit 1; }
g++ -std=gnu++17 -Wall -Wextra -Isrc host_check/test_math.cpp src/plan.cpp src/speed.cpp src/steer_law.cpp src/utils.cpp -o "$bin/test_math" \
  && "$bin/test_math" || fail=1
rm -rf "${bin:?}"

# config.h ships the car's geometry at 0, which multiplies every roller -> rear
# axle / rear MP offset in the steering dispatch by zero. Re-run the tests with a
# plausible measured car, once per steering-law combination, so those transforms
# and every law selection are actually exercised. (Copies the sources so the
# edited config.h is the one their quoted #includes find.)
echo; echo "== math unit tests: measured geometry, every steering-law combination =="
# Never let a failed mktemp turn the cleanup below into `rm -rf /*`: stop unless
# we really have a fresh directory, and use ${tmp:?} so an empty value aborts.
tmp=$(mktemp -d) || { echo "FAIL could not create a temp directory"; exit 1; }
[ -n "$tmp" ] && [ -d "$tmp" ] || { echo "FAIL bad temp directory '$tmp'"; exit 1; }
for fwd in Stanley PurePursuit; do for rev in AimAtTarget PurePursuit Stanley; do
  rm -rf "${tmp:?}"/*; cp src/*.h src/plan.cpp src/speed.cpp src/steer_law.cpp src/utils.cpp host_check/test_math.cpp "$tmp"/
  # Match whatever config.h currently holds, then CHECK each edit took: a silent
  # no-op would test the wrong law under the right label.
  sed -i.bak -E "s/(STEER_FWD_LAW = SteerLaw::)[A-Za-z]+;/\1$fwd;/; s/(STEER_REV_LAW = RevSteerLaw::)[A-Za-z]+;/\1$rev;/; s/(constexpr float WHEELBASE_L_M +=) [0-9.]+f;/\1 0.44f;/; s/(constexpr float ENC_FROM_MP_M +=) [0-9.]+f;/\1 0.50f;/" "$tmp/config.h"
  for want in "STEER_FWD_LAW = SteerLaw::$fwd;" "STEER_REV_LAW = RevSteerLaw::$rev;" "WHEELBASE_L_M    = 0.44f;" "ENC_FROM_MP_M    = 0.50f;"; do
    grep -qF "$want" "$tmp/config.h" || { echo "FAIL harness could not set: $want"; fail=1; }
  done
  if (cd "$tmp" && g++ -std=gnu++17 -Wall -Wextra -Werror -I. test_math.cpp plan.cpp speed.cpp steer_law.cpp utils.cpp -o t && ./t > out.txt); then
    echo "OK   fwd=$fwd rev=$rev"
  else
    echo "FAIL fwd=$fwd rev=$rev"; grep FAIL "$tmp/out.txt" 2>/dev/null | head -5; fail=1
  fi
done; done
rm -rf "${tmp:?}"

echo; [ $fail -eq 0 ] && echo "== ALL CHECKS PASS ==" || { echo "== FAILURES ABOVE =="; exit 1; }
