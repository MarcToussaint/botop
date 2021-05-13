#include <Kin/kin.h>
#include <Control/TaskControlThread.h>
#include <Franka/franka.h>
#include <Franka/gripper.h>
#include <Franka/controlEmulator.h>
#include <Gui/viewer.h>

int main(int argc, char** argv) {
  rai::String leftRight = "right";
  rai::String cmd = "upDown";
  if(argc>1) leftRight = argv[1];
  if(argc>2) cmd = argv[2];

  CHECK(leftRight=="right" || leftRight=="left", "");

  Var<rai::KinematicWorld> ctrl_config;
  Var<CtrlMsg> ctrl_ref;
  Var<CtrlMsg> ctrl_state;

  {

#if 1
    Var<arr> q_real;
    ofstream fil("z.q");
    auto tmp = run([&](){
      arr q = ctrl_state.get()->q;
      q_real.set() = q;
      fil <<q <<endl;
      return 0;
    }, .01);
    PlotViewer plot(q_real);
#endif

    rai::KinematicWorld K(rai::raiPath("../model/pandaSingle.g"));
    K["R_panda_finger_joint1"]->joint->makeRigid();
    arr q0 = K.getJointState();

    ctrl_config.set() = K;

#if 1
    FrankaThread F(ctrl_ref, ctrl_state, (leftRight=="right"?0:1));
    FrankaGripper G((leftRight=="right"?0:1));
#else
    ControlEmulator F(ctrl_ref, ctrl_state, K);
    GripperEmulator G;
#endif

    K.setJointState(ctrl_state.get()->q);

    TaskControlThread C(ctrl_config, ctrl_ref, ctrl_state, {});
    C.ctrl_tasks.waitForRevisionGreaterThan(20);

    if(cmd=="upDown"){
      auto t = addCtrlTask(C.ctrl_tasks, ctrl_config, "upDown", FS_position, {"endeffR"}, 2.);
      arr y0 = t->y;
      t->setTarget( y0 + ARR(.0, .0, .2) );
      wait(+ t);
      t->setTarget(y0);
      wait(+ t);

    }else if(cmd=="none"){
      //    Feature *f = symbols2feature(FS_position, {"endeffR"}, ctrl_config.get());
      //    arr y,J;
      //    for(;;){
      //      f->phi(y, J, C.ctrl_config.get());
      //      cout <<pseudoInverse(~J)*C.ctrl_state.get()->u_bias <<endl;
      //      rai::wait(.1);
      //    }
      //    delete f;

      auto tmp = run([&](){
        arr q = ctrl_state.get()->q;
        cout <<"current state = " <<q <<endl;
        return 0;
      }, 1.);

      rai::wait();

    }else if(cmd=="home"){
      auto t = addCtrlTask(C.ctrl_tasks, ctrl_config, "homing", FS_qItself, {}, 4.);
      t->setTarget( q0 );
      cout <<"HERE" <<endl;
      wait(+ t);

    }else if(cmd=="calib"){
      arr q;
      if(leftRight=="right") q = ARR(0., 1.,  0., -1.5,  1.1,   2.,  -1.6);
      else                   q = ARR(0., 1.,  0., -1.5,  -1.1,   2.,  -0.);

      auto t = addCtrlTask(C.ctrl_tasks, ctrl_config, "calib", FS_qItself, {}, 4.);
      t->setTarget( q );
      wait(+ t);

    }else if(cmd=="initGripper"){
      G.homing();
      rai::wait();

    }else if(cmd=="open"){
      G.open();

    }else if(cmd=="close"){
      bool ret = G.close(20, .035);
      cout <<"grasp returned: " <<ret <<endl;

    }else if(cmd=="comply"){
      auto t = addCtrlTask(C.ctrl_tasks, ctrl_config, "homing", FS_qItself, {}, 4.);
      t->setTarget( q0 );
      //    auto t = addCtrlTask(C.ctrl_tasks, ctrl_config, "homing", FS_position, {"endeffR"}, 1.);
      auto c = addCompliance(C.ctrl_tasks, ctrl_config, "compliance", FS_position, {"endeffR"}, {0., 1., 0.});
      rai::wait();

    }else if(cmd=="vel"){
      auto t = addCtrlTask(C.ctrl_tasks, ctrl_config, "homing",
                           ptr<Feature>(new TM_LinTrans(make_shared<TM_Default>(TMT_pos, ctrl_config.get(), "endeffR"), arr(2,3,{1,0,0,0,1,0}), arr())),
                           make_shared<MotionProfile_Const>(arr()));
      auto tvel = addCtrlTask(C.ctrl_tasks, ctrl_config, "homing", FS_position, {"endeffR"},
                              make_shared<MotionProfile_ConstVel>(ARR(0,0,-.07)));
      auto c = addCompliance(C.ctrl_tasks, ctrl_config, "compliance", FS_position, {"endeffR"}, {0., 0., 1.});

      rai::wait();

    }else if(cmd=="free"){
      //note: this is not fully free: in franka.cpp the compliance is not multiplied to the final u
      auto c = addCompliance(C.ctrl_tasks, ctrl_config, "compliance", FS_qItself, {}, {1.});
      rai::wait();

    }else{
      LOG(0) <<"unkown command";
    }

    rai::wait(.1);

    cout <<"closing controller" <<endl;
  }
  cout <<"BYE BYE" <<endl;

  return 0;
}
