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
arr getStartGoalPath(rai::Configuration& C, const arr& qTarget, const arr& qHome, const rai::Array<Avoid>& avoids={}, StringA endeffectors={}, bool endeffApproach=false, bool endeffRetract=false);
arr getBoxPnpKeyframes(const rai::Configuration& C,
                       rai::ArgWord pickDirection, rai::ArgWord placeDirection,
                       const char* boxName, const char* gripperName, const char* palmName, const char* tableName,
                       const arr& qHome);

//===========================================================================

arr getLoopPath(rai::Configuration& C);

