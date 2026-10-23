#pragma once
//==============================================================================
// plan — turns the announced targets into the numbers the run executes.
// Pure math (no hardware), so host_check tests it directly.
//
// Geometry (docs/design/EV_Detailed_Design.md §0). x = REAR MP position along the
// centerline, Start = 0, Target = D. The MP starts over the Start dot (§4.d.vii).
//
//   xContact     = xBottle - R_BOTTLE - L_P        pusher face touches the bottle
//   captureStart = xContact - CAPTURE_LEAD         slow down to V_CAPTURE here
//   pushStart    = xContact + SEAT_MARGIN          bottle seated; start pushing
//   xF           = D + B + A_PAST + btlOfs - S_SLIDE - L_P
//                                                  forward stop: bottle's near side
//                                                  ends A_PAST past the Bottle Line
//   xFinal       = D + mpOfs                       reverse stop: MP on the Target
//
// The final stop depends only on NET encoder travel, so a forward overshoot moves
// the bottle (1 pt/cm) but not the final MP (2 pt/cm).
//==============================================================================

// What the operator enters after impound (plus the level preset).
struct RunInputs {
  float D;        // Target Distance, m (5-9)
  float B;        // Bottle Line Distance past the Target, m (1-3)
  float T;        // Target Time, s (10-20)
  float xBottle;  // bottle CENTRE from Start, m (2/3/4 by level)
  float mpOfs;    // run-2 correction to the final MP stop, m
  float btlOfs;   // run-2 correction to the forward (bottle) stop, m
};

// Measured car dimensions. Separate from config.h so tests can pass their own.
struct VehicleGeom {
  float lP;       // pusher contact face -> rear MP, m
};
VehicleGeom vehicleFromConfig();

// Segment speeds the run will actually use (plan may raise them to fit T).
struct SegmentSpeeds {
  float vCruise;
  float vPush;
  float vRev;
};

struct RunPlan {
  bool        valid;       // false -> do not arm; `error` says why
  const char *error;       // shown on the LCD: <= 16 chars (host_check enforces it)

  RunInputs   in;
  float xContact, captureStart, pushStart;
  float xF;                // forward stop, MP x
  float xFinal;            // final stop, MP x
  float rev;               // reverse leg length = xF - xFinal
  float path;              // total distance driven = xF + rev

  SegmentSpeeds v;
  float tMotion;           // estimated motion time incl. turnaround + T_BIAS_S
  float delayS;            // start delay so that delay + tMotion = T
  bool  timeFits;          // false -> even max speeds can't finish inside T
};

RunPlan planRun(const RunInputs &in, const VehicleGeom &veh = vehicleFromConfig());

// Motion time for the plan's geometry at the given segment speeds (no T_BIAS_S).
// Uses the same constant-acceleration profiles that speed.cpp commands.
float planMotionTime(const RunPlan &p, const SegmentSpeeds &v);

// Time to cover distance s starting at v0, cruising at vc, ending at v1, with
// acceleration aUp and deceleration aDown. Falls back to a triangle when s is too
// short to reach vc. Exposed for testing.
float segmentTime(float s, float v0, float vc, float v1, float aUp, float aDown);

// Bottle centre position for a level index: 0 regional, 1 state, 2 national.
float bottleXForLevel(int level);
