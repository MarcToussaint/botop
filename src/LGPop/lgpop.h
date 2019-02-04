#include <Kin/kin.h>
#include <Kin/frame.h>

#include <Control/ctrlMsg.h>
#include <Control/TaskControlThread.h>

#include <Perception/percept.h>

#include <Franka/franka.h>

struct LGPop{
  bool simulationMode;
  rai::KinematicWorld rawModel;
  arr q_home;

  //-- Variables
  Var<rai::KinematicWorld> ctrl_config; //real-synced configuration
  Var<CtrlMsg> ctrl_ref;                //control reference
  Var<CtrlMsg> ctrl_state;              //control state
  Var<CtrlTaskL> ctrl_tasks;            //set of control tasks

  Var<floatA> cam_depth; //camera output
  Var<byteA> cam_color;  //camera output
  Var<arr> cam_pose;     //camera pose
  Var<arr> cam_Fxypxy;   //camera parameters

  Var<byteA> model_segments; //output of model camera (segment IDs)
  Var<floatA> model_depth;   //output of model camera (segment IDs)

  Var<PerceptL> percepts; //percepts


  //-- list of all processes
  rai::Array<ptr<Thread>> processes;



  LGPop(bool _simulationMode=true);

  ~LGPop();

  //-- running processes
  void runRobotControllers();
  void runTaskController(int verbose=1);
  void runCamera(int verbose=1);
  void runPerception(int verbose=0);
  void runCalibration();

  void pauseProcess(const char* name, bool resume=false);


  void reportCycleTimes();

};


/*
 * waitForStableObjectPercept();
 * LGP plan
 * execute plan
 */