#include <Franka/controlEmulator.h>
#include <Franka/franka.h>
#include <Franka/help.h>

#include <Control/SplineCtrlFeed.h>

#include <Kin/frame.h>

#include <KOMO/komo.h>
#include <Kin/F_qFeatures.h>
#include <Kin/viewer.h>
#include <Control/LeapMPC.h>

//===========================================================================

void testLeapCtrl() {
  //-- setup a configuration
  rai::Configuration C;
  C.addFile(rai::raiPath("../rai-robotModels/scenarios/pandaSingle.g"));

  //add a target in position space
  arr center = C["R_gripperCenter"]->getPosition();
  rai::Frame& target =
      C.addFrame("target")
      ->setShape(rai::ST_marker, {.1})
      .setPosition(center + arr{+.3,.0,+.2});
  C.watch(true);
  arr q0 = C.getJointState();

  //-- start a robot thread
  C.ensure_indexedJoints();
//  ControlEmulator robot(C, {});
  FrankaThreadNew robot(0, franka_getJointIndices(C,'R'));
  robot.writeData = true;

  C.setJointState(robot.state.get()->q);

  //-- define the reference feed to be a spline
  std::shared_ptr<rai::SplineCtrlReference> sp = make_shared<rai::SplineCtrlReference>();
  robot.cmd.set()->ref = sp;

  LeapMPC leap(C,1.);
  leap.komo.addObjective({1.}, FS_positionDiff, {"R_gripperCenter", "target"}, OT_eq, {1e2});

  for(uint t=0;;t++){

    if(!(t%10)){
      switch(rnd(4)){
        case 0: target.setPosition(center + arr{+.3,.0,+.2}); break;
        case 1: target.setPosition(center + arr{-.3,.0,+.2}); break;
        case 2: target.setPosition(center + arr{+.3,.0,-.2}); break;
        case 3: target.setPosition(center + arr{-.3,.0,-.2}); break;
      }
    }

    double ctrlTime;
    arr q,qDot;
    {
      auto stateGet = robot.state.get();
      ctrlTime = stateGet->time;
      q = stateGet->q;
      qDot = stateGet->qDot;
    }

    leap.reinit(C);
    leap.reinit(q, qDot); //zeros(q.N));
    leap.solve();
    rai::Graph R = leap.komo.getReport();

    double constraints = R.get<double>("ineq") + R.get<double>("eq");
    double cost = R.get<double>("sos");
//    double T=10.; //leap.tau.last();
    double dist = length(q-leap.xT);
    double vel = scalarProduct(qDot, leap.xT-q)/dist;
    double alpha = .7;
    double T = (sqrt(6.*alpha*dist+vel*vel) - vel)/alpha;

    leap.komo.view(false, STRING("LEAP proposal T:"<<T <<"\n" <<R));

    if(T>.2 && cost<1. && constraints<1e-3){
#if 0
      double now=rai::realTime();
      {
        auto stateGet = robot.state.get();
        q = stateGet->q;
        qDot = stateGet->qDot;
      }
      arr _x = cat(q, leap.xT).reshape(2,-1);
      arr _t = {now, now+T};
      sp->overrideHardRealTime(_x, _t, qDot);
#else
      sp->override(~leap.xT, {T}, ctrlTime);
#endif
    }

    C.setJointState(robot.state.get()->q);
    C.gl()->raiseWindow();
    int key = C.watch(false,STRING("time: "<<ctrlTime <<"\n[q or ESC to ABORT]"));
    if(key==13) break;
    if(key=='q' || key==27) return;
    rai::wait(.1);

  }
}

//===========================================================================

int main(int argc, char * argv[]){
  rai::initCmdLine(argc, argv);

  testLeapCtrl();

  return 0;
}
