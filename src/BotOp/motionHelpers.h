#pragma once

#include <Kin/kin.h>
#include <KOMO/komo.h>

//===========================================================================

struct Avoid{
  arr times;
  StringA frames;
  double dist;
};

//===========================================================================

void addBoxPickObjectives(KOMO& komo, double time, rai::ArgWord dir, const char* boxName, const arr& boxSize, const char* gripperName, const char* palmName, const char* tableName);
void addBoxPlaceObjectives(KOMO& komo, double time, rai::ArgWord dir, const char* boxName, const arr& boxSize, const char* gripperName, const char* palmName);
arr getStartGoalPath(rai::Configuration& C, const arr& qTarget, const arr& qHome, const rai::Array<Avoid>& avoids={}, const char* endeff=0, bool endeffApproach=false, bool endeffRetract=false);

//===========================================================================

arr getLoopPath(rai::Configuration& C);

