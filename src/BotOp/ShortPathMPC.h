#pragma once

#include <KOMO/komo.h>

struct ShortPathMPC{
  KOMO komo;
  arr qHome;
  uint steps=0;

  //results
  arr path;
  arr tau;
  bool feasible=false;
  rai::String msg;

  ShortPathMPC(rai::Configuration& C, double timingScale=1.);

  void reinit(const arr& x, const arr& v);  //update robot state
  void reinit(const rai::Configuration& C); //update object movements

  void solve();
};
