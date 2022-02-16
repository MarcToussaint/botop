#pragma once

#include "WaypointMPC.h"
#include "ShortPathMPC.h"

#include <Control/timingMPC.h>

//===========================================================================

struct SecMPC{
  WaypointMPC pathMPC;
  TimingMPC timingMPC;
  ShortPathMPC shortMPC;

  int subSeqStart=0, subSeqStop=-1;
  bool setNextWaypointTangent;
  rai::String msg;
  double precision = .1;
  double tauCutoff = .0;

  double ctrlTimeDelta = 0.;
  double ctrlTime_atLastUpdate = -1.;
  arr q_ref_atLastUpdate, qDot_ref_atLastUpdate, q_refAdapted;
  bool phaseSwitch=false;

  SecMPC(KOMO& komo, int subSeqStart=0, int subSeqStop=-1, double timeCost=1e0, double ctrlCost=1e0, bool _setNextWaypointTangent=true);

  void updateWaypoints(const rai::Configuration& C);
  void updateTiming(const rai::Configuration& C, const ObjectiveL& phi, const arr& q_real);
  void updateShortPath(const rai::Configuration& C, const arr& q_ref, const arr& qDot_ref);
  void cycle(const rai::Configuration& C, const arr& q_ref, const arr& qDot_ref, const arr& q_real, const arr& qDot_real, double ctrlTime);
  rai::CubicSplineCtor getSpline(double realtime, bool prependRef=false);
  rai::CubicSplineCtor getShortPath(double realtime);
  rai::CubicSplineCtor getShortPath2(double realtime);
  void report(const rai::Configuration& C);
};






