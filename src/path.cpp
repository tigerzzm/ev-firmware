#include "path.h"
#include <cmath>

ArcPlan pathPlan(float D, float h) {
  ArcPlan p{};
  p.D = D;
  p.h = h;
  if (h <= 1e-4f) {
    // straight line
    p.R = 0.0f; p.k = 0.0f; p.L = D;
    p.Cx = D * 0.5f; p.Cy = 0.0f;
    return p;
  }
  p.R  = h * 0.5f + (D * D) / (8.0f * h);   // radius
  float phi = 2.0f * std::asin(D / (2.0f * p.R));  // arc angle
  p.L  = p.R * phi;                          // arc length = stop target
  p.k  = 1.0f / p.R;                         // curvature (left turn)
  p.Cx = D * 0.5f;
  p.Cy = h - p.R;                            // < 0, centre is right of centerline
  return p;
}

float pathCrosstrack(const ArcPlan &p, const Position &pose) {
  if (p.k == 0.0f) {
    // straight line along x-axis: cross-track is just y
    return (float)pose.y;
  }
  float dxc = (float)pose.x - p.Cx;
  float dyc = (float)pose.y - p.Cy;
  float range = std::hypot(dxc, dyc);
  return range - p.R;    // >0 outside the arc
}

float pathArcRemaining(const ArcPlan &p, float distanceTravelled) {
  float rem = p.L - distanceTravelled;
  return rem > 0.0f ? rem : 0.0f;
}
