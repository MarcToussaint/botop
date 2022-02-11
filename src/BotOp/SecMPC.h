#pragma once

#include "WaypointMPC.h"

#include <Control/timingMPC.h>

//===========================================================================

struct SecMPC{
  WaypointMPC pathMPC;
  TimingMPC timingMPC;
  int subSeqStart=0, subSeqStop=-1;
  bool setNextWaypointTangent;
  rai::String msg;
  double precision = .1;
  double ctrlTimeLastUpdate = -1.;
  double tauCutoff = .0;
  arr q_refAdapted, qDot_refAdapted;
  bool phaseSwitch=false;

  SecMPC(KOMO& komo, int subSeqStart=0, int subSeqStop=-1, double timeCost=1e0, double ctrlCost=1e0, bool _setNextWaypointTangent=true);

  void updateWaypoints(const rai::Configuration& C);
  void updateTiming(const rai::Configuration& C, const ObjectiveL& phi, double ctrlTime, const arr& q_real, const arr& qDot_real, const arr& q_ref, const arr& qDot_ref);
  void cycle(const rai::Configuration& C, const arr& q_ref, const arr& qDot_ref, const arr& q_real, const arr& qDot_real, double ctrlTime);
  rai::CubicSplineCtor getSpline(double realtime);
  void report(const rai::Configuration& C);
};






