#include <Kin/kin.h>
#include <Control/TaskControlThread.h>
#include <Franka/franka.h>
#include <Franka/gripper.h>
#include <Franka/help.h>
#include <Franka/controlEmulator.h>

int main(int argc, char** argv) {
  Var<rai::KinematicWorld> sim_config;
  Var<rai::KinematicWorld> ctrl_config;
  Var<CtrlMsg> ctrl_ref;
  Var<CtrlMsg> ctrl_state;
  Var<CtrlTaskL> ctrl_tasks;

  {
    ctrl_config.set()->init("../../model/pandaStation.g");
    sim_config.set() = ctrl_config.get();
    arr q0 = ctrl_config.get()->getJointState();
    {
      auto set = ctrl_state.set();
      ctrl_config.set()->getJointState(set->q, set->qdot);
    }

    rai::Array<ptr<Thread>> processes;

    if(rai::getParameter<bool>("sim", false)){
      processes.append( make_shared<ControlEmulator>(sim_config, ctrl_ref, ctrl_state) );
    }else{
      processes.append( make_shared<FrankaThread>(ctrl_ref, ctrl_state, 0, franka_getJointIndices(ctrl_config.get(), 'R')) );
      processes.append( make_shared<FrankaThread>(ctrl_ref, ctrl_state, 1, franka_getJointIndices(ctrl_config.get(), 'L')) );
    }

    TaskControlThread C(ctrl_config, ctrl_ref, ctrl_state, ctrl_tasks);
    ctrl_tasks.waitForRevisionGreaterThan(20);

    {
      auto t = addCtrlTask(ctrl_tasks, ctrl_config, "upDown", FS_position, {"endeffL"}, 2.);
      arr y0 = t->y;
      t->setTarget( y0 + ARR(.0, .0, .2) );

      auto t2 = addCtrlTask(ctrl_tasks, ctrl_config, "upDown", FS_position, {"endeffR"}, 2.);
      arr y2 = t2->y;
      t2->setTarget( y2 + ARR(.0, .0, .2) );

      wait(+t +t2);

//      t->setTarget(y0);
//      t2->setTarget(y2);
//      wait(+t +t2);
    }

    {
      auto t = addCtrlTask(C.ctrl_tasks, ctrl_config, "homing", FS_qItself, {}, 2.);
      t->setTarget( q0 );
      wait(+ t);
    }
  }

  cout <<"BYE BYE" <<endl;

  return 0;
}
