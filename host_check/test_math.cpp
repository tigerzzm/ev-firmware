// Host unit tests for the hardware-independent math (path arc geometry, angle
// utils, speed trapezoid). Compiled natively; no Pico headers needed.
#include "path.h"
#include "utils.h"
#include "speed.h"
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
  printf("\n%s (%d failures)\n", fails? "TESTS FAILED":"ALL MATH TESTS PASS", fails);
  return fails?1:0;
}
