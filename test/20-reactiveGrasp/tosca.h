#pragma once

#include <KOMO/komo.h>
#include <Control/timingMPC.h>

//===========================================================================

//A wrapper of KOMO to optimize waypoints for a given sequence of constraints
struct WaypointMPC{
  KOMO komo;
  uint steps=0;
  //result
  arr path;
  arr tau;
  bool feasible=false;

  WaypointMPC(rai::Configuration& C, rai::Array<ObjectiveL> _phiflag, rai::Array<ObjectiveL> _phirun, const arr& qHome={});

  void reinit(const rai::Configuration& C);
  void solve();
};

//===========================================================================

struct TOSCA{
  WaypointMPC pathMPC;
  TimingMPC timingMPC;
  rai::String msg;
  double ctrlTimeLast = -1.;

  TOSCA(rai::Configuration& C, rai::Array<ObjectiveL> _phiflag, rai::Array<ObjectiveL> _phirun, const arr& qHome={});

  void updateWaypoints(const rai::Configuration& C);
  void updateTiming(double ctrlTime, const arr& q_real, const arr& qDot_real);
  void cycle(const rai::Configuration& C, const arr& q_real, const arr& qDot_real, double ctrlTime);
  void report(const rai::Configuration& C, const rai::Array<ObjectiveL>& phiflag, const rai::Array<ObjectiveL>& phirun);
};






