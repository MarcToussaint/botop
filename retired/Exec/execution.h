#pragma once

#include <KOMO/komo.h>
#include <memory>
#include <Sim/simulationIO.h>
#include "robotio.h"

/** This is a high helper for LGP execution
 *
 * this code is not generic, but very specific to our LGP execution experiments
 */

template<class T> using ptr = std::shared_ptr<T>;

struct LGPExecution{
  SimulationIO S;
  RobotIO R;
  rai::KinematicWorld K;
  StringA allJoints, armJoints, gripperJoints;
  arr q_home;
  bool guiPauses = true;
  bool useRealRobot=false;
  bool useSimulation=false;
  double timeScale=1.;

  // the object poses are initially set to unknown/random
  LGPExecution(bool _useRealRobot, bool _useSimulation, const char* modelFile = "../model/model.g");

  void startSimulation();
  void stopSimulation();

  //-- sync the model's state with the real (or sim) state
  void syncModelJointStateWithRealOrSimulation();
  void syncModelObjectPosesWithRealOrSimulation(uint average);

  void setModelJointState(const StringA& joints, const arr& q);

  //-- planning

  // We plan a full pick-and-place of one object using LGP (1st use of KOMO)
  ptr<KOMO> planSkeleton(const Skeleton& S);

  // We retrieve the key frame poses
  arr getJointConfiguration(ptr<KOMO> plan, double phase);
  rai::Transformation getObjectPose(ptr<KOMO> plan, double phase, const char* obj);

  // Use 1. keyframe as regularization for a high-precision grasp pose IK (2nd use of KOMO)
  arr computePreciseGrasp(const arr& roughPose, const char* obj);

  // Use the 2. keyframe as regularization for a high-prediction placement IK
  arr computePrecisePlace(const arr& roughPose, const char* obj, const char* onto, const rai::Transformation& Q);

  // For this grasp pose, generate a nice motion with final downward motion (3nd use of KOMO)
  std::pair<arr, arr> computePreciseMotion(const arr& to, bool initialUp, bool finalDown);

  //-- validation
  void displayAndValidate(const StringA& joints, const arr& q, const arr& tau);

  //-- execution
  void executeMotion(const StringA& joints, const arr& q, const arr& tau);
  void waitForCompletion();
  void closeGripper();
  void openGripper();
  void attach(const char* a, const char* b);
  void homing(bool initialUp);
};
