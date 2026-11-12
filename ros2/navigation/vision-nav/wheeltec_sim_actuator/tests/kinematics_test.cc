#include <cassert>
#include <cmath>
#include "wheeltec_sim_actuator/kinematics.hh"
#include "wheeltec_sim_actuator/simulation_clock.hh"
int main() {
  wheeltec::CommandLease lease;
  assert(lease.Resolve(2).first==0);
  lease.Set(.2,.1,2);
  assert(lease.Resolve(2.20).first==.2);
  assert(lease.Resolve(2.251).first==0);
  assert(lease.Resolve(2.10,true).first==0);
  assert(lease.Resolve(1.9).first==0);
  auto t=wheeltec::Ackermann(.2,.2,.34,.30,.065,.65,.6);
  assert(t[4]>t[5] && t[0]<t[1]);
  // Front wheel vector must be tangent to its circular path (no lateral slip).
  for (size_t s=0;s<2;++s) {
    const double y=s==0?.15:-.15;
    assert(std::abs(t[s+2]*.065*std::cos(t[s+4])-(.2-y*.2))<1e-10);
    assert(std::abs(t[s+2]*.065*std::sin(t[s+4])-.34*.2)<1e-10);
  }
  auto reverse=wheeltec::Ackermann(-.2,-.2,.34,.30,.065,.65,.6);
  assert(reverse[0]<0 && reverse[4]==t[4]);
  auto spin=wheeltec::Ackermann(0,1,.34,.30,.065,.65,.6);
  for(double value:spin) assert(value==0);
  auto sharp=wheeltec::Ackermann(.3,2,.34,.30,.065,.65,.6);
  assert(sharp[4]<=.65);
  wheeltec::SimulationClock clock;
  int publications=0;
  for (std::int64_t ns=1000000;ns<=1000000000;ns+=1000000)
    if (clock.Due(ns,false)) ++publications;
  assert(publications==100);  // 1kHz physics -> 100 original clock stamps per sim s
  for (int i=0;i<1000;++i) assert(!clock.Due(1000000000,true));
  assert(clock.Due(1001000000,false));
  assert(clock.publishedNs==1001000000);  // acquisition time, no period rounding
  assert(!clock.Due(1002000000,false));
  assert(clock.Due(0,true));              // rewind visible even while paused
  assert(!clock.Due(0,true));
  assert(clock.Due(10000000,false));
  assert(!clock.Due(-1,false));
  wheeltec::SimulationClock initiallyPaused;
  assert(initiallyPaused.Due(0,true));
  assert(!initiallyPaused.Due(0,true));
}
