// Host unit tests for the hardware-independent math (path arc geometry, angle
// utils, speed trapezoid). Compiled natively; no Pico headers needed.
#include "path.h"
#include "utils.h"
#include "speed.h"
#include "config.h"
#include <cstdio>
#include <cmath>
static int fails=0;
static void chk(const char*n,double got,double want,double tol){bool ok=std::fabs(got-want)<=tol;printf("%-30s got=%.5f want=%.5f %s\n",n,got,want,ok?"ok":"FAIL");if(!ok)fails++;}
int main(){
  ArcPlan p = pathPlan(8.0f, 0.925f);
  printf("Arc(D=8,h=0.925): R=%.4f L=%.4f k=%.5f C=(%.3f,%.3f)\n",p.R,p.L,p.k,p.Cx,p.Cy);
  chk("start on circle", std::hypot(0-p.Cx,0-p.Cy), p.R, 1e-3);
  chk("end on circle",   std::hypot(p.D-p.Cx,0-p.Cy), p.R, 1e-3);
  chk("apex on circle",  std::hypot(p.D/2-p.Cx,p.h-p.Cy), p.R, 1e-3);
  chk("crosstrack@apex",  pathCrosstrack(p, Position{p.D/2,p.h,0}), 0.0, 1e-3);
  chk("L>chord",          (p.L>p.D)?1:0, 1.0, 0.0);
  chk("arcRemaining@L",   pathArcRemaining(p, p.L), 0.0, 1e-6);
  ArcPlan s = pathPlan(8.0f, 0.0f);
  chk("straight k=0",     s.k, 0.0, 1e-9);
  chk("straight L=D",     s.L, 8.0, 1e-6);
  chk("straight crosstrack",pathCrosstrack(s,Position{4,0.1,0}), 0.1, 1e-6);
  float L=p.L,T=15.0f; double time=0; int N=20000;
  for(int i=0;i<N;i++){float ss=(float)L*i/N; float v=speedTrapezoid(ss,L,T); time += (L/N)/v;}
  printf("trapezoid est cruise time=%.2fs (target %.1f)\n",time,T);
  chk("cruise time near T", time, T, 3.0);
  chk("angleError(350,10)", utils::angleError(350,10,false), -20.0, 1e-6);
  chk("angleSquish(-90)",   utils::angleSquish(-90,false), 270.0, 1e-6);
  chk("slew clamps",        utils::slew(100,0,5), 5.0, 1e-6);

  // ---- ground-speed low-pass (odo.cpp uses this for the free-roller estimate) --
  chk("lowpass dt=0 holds",  utils::lowpass(1.0f, 9.0f, 0.0f, 0.1f), 1.0, 1e-6);
  chk("lowpass tau=0 passes",utils::lowpass(1.0f, 9.0f, 0.02f, 0.0f), 9.0, 1e-6);
  chk("lowpass dt==tau",     utils::lowpass(0.0f, 1.0f, 0.1f, 0.1f), 0.5, 1e-6);
  // A step held for 5 tau must settle close to the new value and never overshoot
  // it (a first-order lag has no overshoot). The discrete alpha = dt/(tau+dt) is
  // marginally slower than the continuous e^(-t/tau), so 5 tau lands at ~98.9%.
  {
    float v = 0.0f; const float dt = 0.02f, tau = SPEED_FILTER_TAU_S;
    bool overshot = false;
    for (int i = 0; i < (int)(5.0f * tau / dt); i++) {
      v = utils::lowpass(v, 1.0f, dt, tau);
      if (v > 1.0f) overshot = true;
    }
    chk("lowpass settles 5tau", v, 1.0, 0.02);
    chk("lowpass no overshoot", overshot ? 1 : 0, 0.0, 0.0);
  }
  // Constant-speed ramp: differentiating a straight distance ramp and filtering it
  // must converge on the true speed (this is exactly what odoSpeedTick does).
  {
    const float trueV = 1.25f, dt = 0.02f;
    float v = 0.0f, dist = 0.0f, last = 0.0f;
    for (int i = 0; i < 100; i++) {
      dist += trueV * dt;
      v = utils::lowpass(v, (dist - last) / dt, dt, SPEED_FILTER_TAU_S);
      last = dist;
    }
    chk("speed est tracks ramp", v, trueV, 1e-3);
  }
  printf("\n%s (%d failures)\n", fails? "TESTS FAILED":"ALL MATH TESTS PASS", fails);
  return fails?1:0;
}
