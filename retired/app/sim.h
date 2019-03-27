#include <Core/thread.h>
#include <Kin/kin.h>
#include <Algo/spline.h>
#include <Msg/MotionReference.h>

//===============================================================================

struct PerceptSimple;
typedef rai::Array<PerceptSimple*> PerceptSimpleL;

//===============================================================================

struct KinSim : Thread{
  rai::KinematicWorld K;
  uint pathRev=0, switchesRev=0;
  rai::Spline reference;
  double phase=0.;
  double dt, planDuration=5.;
  ofstream log;

  //input
  Var<Msg_MotionReference> ref;
  Var<StringA> switches;
  Var<double> gripper;

  //output
  Var<arr> currentQ;
  Var<double> currentGripper;
  Var<StringA> jointNames;
  Var<rai::Transformation> robotBase;
//  Var<arr> nextQ;
//  Var<rai::KinematicWorld> world;
  Var<double> timeToGo;
  Var<PerceptSimpleL> percepts_input;


  KinSim(double dt=.1);
  ~KinSim(){
    log.close();
  }

  void open(){}
  void close(){}

  void step();

};
