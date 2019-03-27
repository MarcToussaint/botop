#include "help.h"
#include <Core/thread.h>
#include "komo_fine.h"
#include <Kin/frame.h>
#include <LGP/optLGP.h>

#ifdef RAI_ROS
#include <robot_msgs/SendJointTrajectory.h>
#include <ros/ros.h>
#endif

void callTrajectoryService(const Msg_MotionReference &msg){
#ifdef RAI_ROS
  static robot_msgs::SendJointTrajectory srv;

  srv.request.trajectory.points.resize(msg.path.d0);
  double time=0.;
  for(uint t=0;t<msg.path.d0;t++){
    if(msg.tau.N==1) time += msg.tau.scalar();
    else time += msg.tau(t);

    trajectory_msgs::JointTrajectoryPoint& p = srv.request.trajectory.points[t];
    p.positions = msg.path[t].copy();
    p.velocities = zeros(msg.path.d1);
    p.accelerations = zeros(msg.path.d1);
    p.effort = zeros(msg.path.d1);
    p.time_from_start = ros::Duration(2.*time);
  }


  srv.request.trajectory.joint_names.resize(msg.path.d1);
  StringA joints = Var<StringA>("jointNames").get();
  CHECK_EQ(joints.N, msg.path.d1, "");
  for(uint i=0;i<msg.path.d1;i++){
    srv.request.trajectory.joint_names[i] = joints(i).p;
  }


  if(ros::service::call("/robot_control/SendJointTrajectory", srv)){
    ROS_INFO("Success?: %ld", (long int)srv.response.success);
  }else{
    ROS_ERROR("Failed to call service add_two_ints");
  }
#endif
}

Msg_MotionReference planPath_IK(const StringA &cmd, const rai::Transformation &where, bool fromCurrent){
  Var<rai::KinematicWorld> K("filterWorld");
  Var<rai::KinematicWorld> Ktail("kinTail");
  StringA joints = Var<StringA>("jointNames").get();


  KOMO_fineManip komo1(K.get());
  komo1.setIKOpt();
  komo1.setIKAbstractTask(cmd);
  komo1.reset();
  komo1.run();
//  cout <<komo1.getReport(false);
  arr q = komo1.getPath(joints);
  q.reshape(q.N);
//  while(komo1.displayTrajectory(.05, true));

  KOMO_fineManip komo(K.get());
  komo.setPathOpt(1., 20, 3.);
  if(cmd.N>1)
    komo.setGoTo(q, joints, cmd(1), .2, .8);
  else
    komo.setGoTo(q, joints, NULL, .2, .8);
  komo.reset();
  komo.run();
//  cout <<komo.getReport(false);
  //  while(komo.displayTrajectory(.05, true));

  Ktail.set() = *komo.configurations.elem(-1);

  arr x = komo.getPath(joints);
  Msg_MotionReference ref;
  ref.path = x;
  ref.tau = { komo.tau };
  ref.append = true;
  return ref;
}

void solveLGP(){
  rai::KinematicWorld K = Var<rai::KinematicWorld>("world").get();
  FOL_World L(FILE("fol.g"));
  initFolStateFromKin(L, K);

  int verbose=1;

  //-- prepare logic world
  
  // for(rai::Frame *f:K.frames){
  //   if(f->ats["isObject"]){ L.addObject(f->name); if(verbose>0) cout <<"LGP -- adding object " <<f->name <<endl; }
  //   if(f->ats["isAgent"]) { L.addAgent(f->name); if(verbose>0) cout <<"LGP -- adding agent " <<f->name <<endl; }
  //   if(f->ats["isTable"]) { L.addFact({"table", f->name}); if(verbose>0) cout <<"LGP -- adding table " <<f->name <<endl; }
  //   if(f->ats["isPusher"]){ L.addFact({"pusher", f->name}); if(verbose>0) cout <<"LGP -- adding pusher " <<f->name <<endl; }
  //   if(f->ats["isPartOf"]){ L.addFact({"partOf", f->name, f->ats.get<rai::String>("isPartOf")}); if(verbose>0) cout <<"LGP -- adding partOf" <<f->name <<' ' <<f->ats.get<rai::String>("isPartOf") <<endl; }
  // }

  OptLGP lgp(K, L);

//    lgp.verbose = 0;
//    rai::timerStart();
//    for(uint d=1;d<4;d++){
//      lgp.buildTree(d);
//      MNodeL all = lgp.root->getAll();
//      cout <<"d= " <<d <<" #= " <<all.N <<" t= " <<rai::timerRead(true) <<endl;
//    }

//    lgp.updateDisplay();
//    rai::wait();
  //  lgp.player();

  lgp.optFixedSequence("(grasp endeff stick) \
                       (push stickTip redBall table1) \
                       (place endeff stick table1) \
                       (grasp endeff redBall) \
                       ", 3);

//  lgp.run();


  lgp.reportEffectiveJoints();

  rai::wait();

  //  lgp.renderToVideo();
}
