#pragma once

#include <Kin/kin.h>
#include <KOMO/komo.h>

//===========================================================================

arr getLoopPath(rai::Configuration& C);
void addBoxPickObjectives_botop(KOMO& komo, double time, rai::ArgWord dir, const char* boxName, const arr& boxSize, const char* gripperName, const char* palmName, const char* tableName);
void addBoxPlaceObjectives_botop(KOMO& komo, double time, rai::ArgWord dir, const char* boxName, const arr& boxSize, const char* gripperName, const char* palmName);
arr getBoxPnpKeyframes(const rai::Configuration& C,
                       str pickDirection, str placeDirection,
                       const char* boxName, const char* gripperName, const char* palmName, const char* tableName,
                       const arr& qHome);
arr getBoxPnpKeyframes_new(rai::Configuration& C, str graspDirection, str placeDirection, str box, str gripper, str palm, str table, const arr& qHome);

//===========================================================================

void getGraspLinePlane(arr& xLine, arr& yzPlane, FeatureSymbol& xyScalarProduct, FeatureSymbol& xzScalarProduct,
                       const rai::ArgWord& dir);
