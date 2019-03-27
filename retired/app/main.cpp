#include "sim.h"
#include "komo_fine.h"
#include "filter.h"
#include "help.h"

#include <ros/ros.h>
#include <RosCom/spinner.h>
#include <Kin/kin.h>

//===============================================================================

void execAction(const StringA& action, const rai::Transformation& rel=NoTransformation){
  Msg_MotionReference ref = planPath_IK(action, rel);
  Var<Msg_MotionReference>("MotionReference").set() = ref;
  Var<double> ttg("timeToGo");
  callTrajectoryService(ref);
  rai::wait(1.);
  for(;;){ ttg.waitForNextRevision(); if(ttg.get()<=0.) break;  }
}

void execGripper(double q, const StringA& kinSwitch={}){
  Var<double>("gripperReference").set() = q;
  if(kinSwitch.N)  Var<StringA>("switches").set() = kinSwitch;
  rai::wait(.3);
}

void go(int probId=1) {
  //-- load the problem description into the 'world' variable
  Var<rai::KinematicWorld>("world").set()->init(STRING("problem-0"<<probId<<".g"));

  //-- load the robot joints into a global variable
  StringA joints = Var<rai::KinematicWorld>("world").get()->getJointNames();
  joints.removeValue("slider1Joint", false);
  joints.removeValue("wsg_50_base_joint_gripper_left");
  Var<StringA>("jointNames").set() = joints;

  //-- start a simulation
  KinSim sim;
  sim.threadLoop();

  //-- start the filter
  FilterSimple filter;
  filter.threadLoop();

  rai::wait(.1);

  {
    Var<arr> q("currentQ");
    Var<rai::KinematicWorld> K("world");
    if(maxDiff(q.get()(), K.get()->getJointState(joints)) > 1e-2){
      execAction({"home"});
    }
  }

  //wait for all objects to be localized
  filter.problemPerceived.waitForValueEq(true);

//  solveLGP();
//  return;

  for(uint l=0;;l++){
    execAction({"grasp", "endeff", "stick"});

    execGripper(.004, {"attach", "endeff", "nostick"});

    execAction({"push","stickTip","redBall","table1"}, { {.5, -.5, 1.12}, Quaternion_Id } );

    execGripper(.05, {"attach", "table1", "nostick"});
  }


}

//===============================================================================

int main(int argc, char** argv){
  rai::initCmdLine(argc, argv);

  RosCom_Spinner rs;
  rs.threadLoop();

  go();

  return 0;
}

