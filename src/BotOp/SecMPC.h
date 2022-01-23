#pragma once

#include <KOMO/komo.h>
#include <Control/timingMPC.h>

//===========================================================================

//A wrapper of KOMO to optimize waypoints for a given sequence of constraints
struct WaypointMPC{
  KOMO& komo;
  arr qHome;
  uint steps=0;
  //result
  arr path;
  arr tau;
  bool feasible=false;

  WaypointMPC(KOMO& _komo, const arr& qHome={});

  void reinit(const rai::Configuration& C);
  void solve();
};

//===========================================================================

struct SecMPC{
  WaypointMPC pathMPC;
  TimingMPC timingMPC;
  int subSeqStart=0, subSeqStop=-1;
  rai::String msg;
  double precision = .1;
  double ctrlTimeLast = -1.;
  double tauCutoff = -.1;

  SecMPC(KOMO& komo, int subSeqStart=0, int subSeqStop=-1, double timeCost=1e0, double ctrlCost=1e0);

  void updateWaypoints(const rai::Configuration& C);
  void updateTiming(const rai::Configuration& C, const ObjectiveL& phi, double ctrlTime, const arr& q_real, const arr& qDot_real, const arr& q_ref, const arr& qDot_ref);
  void cycle(const rai::Configuration& C, const arr& q_ref, const arr& qDot_ref, const arr& q_real, const arr& qDot_real, double ctrlTime);
  rai::CubicSplineCtor getSpline(double realtime);
  void report(const rai::Configuration& C);
};






