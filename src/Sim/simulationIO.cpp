#include "simulationIO.h"
#include <RosCom/roscom.h>
#include "simulationIO_self.h"

SimulationIO_self::SimulationIO_self(bool pubSubToROS, const char *modelFile, double dt)
    : ROS("simulator"),
      SIM(modelFile, dt),
      dt(dt),
      ref("MotionReference"),
      command("command"),
      currentQ("currentQ"),
      objectPoses("objectPoses"),
      objectNames("objectNames"),
      timeToGo("timeToGo"){

    if(pubSubToROS){
        //setup ros communication
        sub_ref = ROS.subscribe(ref);
        sub_command = ROS.subscribe(command);

        pub_timeToGo = ROS.publish(timeToGo);
        pub_objectPoses = ROS.publish(objectPoses);
        pub_currentQ = ROS.publish(currentQ);
    }
}

SimulationIO::SimulationIO(bool pubSubToROS, const char *modelFile, double dt)
  : Thread("SimulationIO", dt){
  self = new SimulationIO_self(pubSubToROS, modelFile, dt);
}

SimulationIO::~SimulationIO(){
  threadClose();
  delete self;
}

void SimulationIO::step(){
  //check for inputs:
  if(self->ref.hasNewRevision()){
    self->ref.readAccess();
    StringA joints = conv_stdStringVec2StringA(self->ref->joints); //is ignored for now!!!
    arr t = conv_arr2arr(self->ref->t);
    arr x = conv_arr2arr(self->ref->x);
    bool append = self->ref->append;
    cout <<"MotionReference revision=" <<self->ref.getRevision() <<' ' <<self->ref->revision <<endl;
    self->ref.deAccess();

    self->SIM.setUsedRobotJoints(joints);
    self->SIM.exec(x, t, append);
  }

  if(self->command.hasNewRevision()){
    StringA cmd = conv_StringA2StringA( self->command.get() );
    self->SIM.exec(cmd);
  }

  self->SIM.stepKin();

  //publish:
  self->currentQ.set() = conv_arr2arr( self->SIM.getJointState() );
  self->objectNames.set() = conv_StringA2StringA( self->SIM.getObjectNames() );
  self->objectPoses.set() = conv_arr2arr( self->SIM.getObjectPoses() );
  self->timeToGo.set() = conv_double2Float64( self->SIM.getTimeToGo() );
}

void SimulationIO::loop(){
  //loop
  Metronome tictac(self->dt);
  for(;;){
    tictac.waitForTic();
    step();
  }
}

