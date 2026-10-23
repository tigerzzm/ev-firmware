#pragma once
//==============================================================================
// odo_split — attributes encoder counts to forward or reverse travel, so each
// direction can have its own scale (M_PER_COUNT / M_PER_COUNT_REV).
// Pure logic, header-only, tested in host_check.
//
// Splitting by the sign of each sample alone has a flaw: a roller sitting on a
// quadrature edge at a standstill produces +1/-1 pairs, and with two different
// scales every pair would move the distance by (M_PER_COUNT - M_PER_COUNT_REV)
// even though the net count is unchanged. So a change of direction is accepted
// only after `hyst` counts against the current one. Until then those counts are
// held as `pending` — still included in the distance, so the net is always exact
// — and returning motion cancels them first.
//==============================================================================
#include <cstdint>

struct DirSplit {
  int32_t fwd     = 0;   // counts attributed to forward travel (>= 0)
  int32_t rev     = 0;   // counts attributed to reverse travel (<= 0)
  int32_t pending = 0;   // signed counts AGAINST `dir`, not yet a confirmed reversal
  int32_t dir     = +1;  // current direction of travel, +1 or -1

  void add(int32_t d, int32_t hyst) {
    if (d == 0) return;
    int32_t s = d * dir;                  // > 0: with the current direction
    int32_t p = pending * dir;            // <= 0: pending, in the same frame
    if (s > 0) {
      const int32_t cancel = (s < -p) ? s : -p;   // returning motion cancels pending first
      p += cancel;
      s -= cancel;
      attribute(dir, s);
    } else {
      p += s;                             // more motion against the current direction
      if (-p >= hyst) {                   // confirmed: direction has changed
        dir = -dir;
        attribute(dir, -p);
        p = 0;
      }
    }
    pending = p * dir;
  }

  int32_t net() const { return fwd + rev + pending; }

  // Distance in metres. Pending counts are motion against `dir`, so they take the
  // other direction's scale.
  float metres(float mFwd, float mRev) const {
    return (float)fwd * mFwd + (float)rev * mRev + (float)pending * (pending > 0 ? mFwd : mRev);
  }

 private:
  void attribute(int32_t direction, int32_t counts) {   // counts >= 0
    if (direction > 0) fwd += counts; else rev -= counts;
  }
};
