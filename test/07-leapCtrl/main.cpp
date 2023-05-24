#include <BotOp/bot.h>
//#include <Control/ShortPathMPC.h>
#include <Control/ShortPathMPC.h>
#include <Control/timingOpt.h>
#include <Optim/NLP_Solver.h>

//===========================================================================

void testLeapCtrl() {
  //-- setup a configuration
  rai::Configuration C;
  C.addFile(rai::raiPath("../rai-robotModels/scenarios/pandaSingle.g"));
//  C.addFile(rai::raiPath("../rai-robotModels/scenarios/pandasTable.g"));
//  C.addFile("model.g");

  //add a target in position space
  arr center = C["l_gripper"]->getPosition();
  rai::Frame& target =
      C.addFrame("target")
      ->setShape(rai::ST_marker, {.1})
      .setPosition(center + arr{+.3,.0,+.2});
  C.view();

  //-- start a robot thread
  BotOp bot(C, rai::getParameter<bool>("real", false));
  bot.home(C);

  //-- create a leap controller (essentially receeding horizon KOMO solver)
  ShortPathMPC shortMPC(C, 10, .1);
//  shortMPC.komo.addObjective({}, FS_distance, {"stick", "l_gripper"}, OT_ineq, {1e1}, {-.1});
//  shortMPC.komo.addObjective({}, FS_accumulatedCollisions, {}, OT_eq, {1e0});
  shortMPC.komo.addObjective({1.}, FS_positionDiff, {"l_gripper", "target"}, OT_eq, {1e1});
  shortMPC.komo.addObjective({1.}, FS_qItself, {}, OT_eq, {1e0}, {}, 1);

  //-- iterate (with wait(.1))
  for(uint t=0;;t++){
    rai::wait(.1);

    //-- switch target randomly every second
    if(!(t%7)){
      switch((t/7)%4){
        case 0: target.setPosition(center + arr{+.3,.0,+.2}); break;
        case 1: target.setPosition(center + arr{+.3,.0,-.2}); break;
        case 2: target.setPosition(center + arr{-.3,.0,-.2}); break;
        case 3: target.setPosition(center + arr{-.3,.0,+.2}); break;
      }
    }

    //get time,q,qDot - as batch from the same mutex lock
    double ctrlTime;
    arr q,qDot;
    {
      auto stateGet = bot.robotL->state.get();
      ctrlTime = stateGet->ctrlTime;
      q = stateGet->q;
      qDot = stateGet->qDot;
    }

    //solve the leap problem
    shortMPC.reinit(C); //adopt all frames in C as prefix (also positions of objects)
    shortMPC.reinit(q, qDot);
    shortMPC.solve(true, 0);
//    rai::Graph R = shortMPC.komo.getReport();


    if(shortMPC.feasible){
      arr path = shortMPC.getPath();
      TimingProblem timingProblem(path, {}, q, qDot, 1e1, 1.); //, {}, tauInitial, optTau);
      NLP_Solver solver;
      solver
          .setProblem(timingProblem.ptr())
          .setSolver(NLPS_newton);
      solver.opt
          .set_stopTolerance(1e-4)
          .set_maxStep(1e0)
          .set_damping(1e-2);
      auto ret = solver.solve();
      //LOG(1) <<"timing f: " <<ret->f;
      arr vels, times;
      timingProblem.getVels(vels);
      times = integral(timingProblem.tau);
      //cout <<times <<endl;

      if(times.last()>.1){
        shortMPC.reinit_taus(times.last());

        double ctrlTimeNow = bot.get_t();
        times -= ctrlTimeNow - ctrlTime;
        bot.move(path, vels, times, true, ctrlTime);
      }else{
        bot.moveTo(shortMPC.path[-1]);
      }
    }else{
      shortMPC.reinit_taus(1.);
    }


    //send leap target
    if(shortMPC.feasible){
//      double T = bot.moveTo(shortMPC.path[-1]);
      //bot.move(shortMPC.path, {}, true);
      double T = bot.getTimeToEnd();
      double ctrlTime = bot.get_t();
      //    shortMPC.komo.view(false, STRING("LEAP proposal ETA:"<<T <<"\n" <<R));
      cout <<"LEAP proposal ETA:" <<T <<" \tT:"<<T-ctrlTime <<endl;
    }

    //update C
    bot.sync(C);
    if(bot.keypressed==13){ t=9; continue; }
    if(bot.keypressed=='q' || bot.keypressed==27) return;
  }
}

//===========================================================================

int main(int argc, char * argv[]){
  rai::initCmdLine(argc, argv);

  testLeapCtrl();

  LOG(0) <<" === bye bye ===\n used parameters:\n" <<rai::params() <<'\n';

  return 0;
}
