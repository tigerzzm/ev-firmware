#pragma once
//==============================================================================
// path — the planned constant-curvature arc through (0,0) -> (D/2, h) -> (D,0).
// Math straight from claude/EV_Detailed_Design.md §0. Deterministic; no robotour
// grid/interpolation/reverse logic (all of that was dropped as irrelevant).
//
//   x = forward along centerline (Start->Target), y = left (positive).
//   h = sideways bulge (sagitta) chosen to thread the can gate (~90-95 cm).
//==============================================================================
#include "position.h"

struct ArcPlan {
  float D;    // target distance (chord), m
  float h;    // bulge / sagitta, m
  float R;    // radius, m
  float L;    // arc length (the STOP TARGET), m
  float k;    // curvature 1/R (left +)
  float Cx, Cy;   // arc centre
};

// Build the plan from target distance D and gate bulge h (both metres).
// h<=0 degenerates to a straight line (k=0, L=D).
ArcPlan pathPlan(float D, float h);

// Cross-track error to the reference circle: >0 = outside the arc.
float pathCrosstrack(const ArcPlan &p, const Position &pose);

// Arc length still remaining given along-path distance travelled.
float pathArcRemaining(const ArcPlan &p, float distanceTravelled);
