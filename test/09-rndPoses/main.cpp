#include <KOMO/pathTools.h>

#include <BotOp/bot.h>
#include <BotOp/motionHelpers.h>

//===========================================================================

void rndPoses(){
  rai::Configuration C;
  rai::String useArm = rai::getParameter<rai::String>("bot/useArm", "both");
  if(useArm=="both"){
    C.addFile(rai::raiPath("../rai-robotModels/scenarios/pandasTable.g"));
    C.addFrame("l_gripper_target") ->setShape(rai::ST_marker, {.2});
    C.addFrame("r_gripper_target") ->setShape(rai::ST_marker, {.2});
  }else{
    C.addFile(rai::raiPath("../rai-robotModels/scenarios/pandaSingle.g"));
    C.addFrame("l_gripper_target") ->setShape(rai::ST_marker, {.2});
  }

  arr qHome = C.getJointState();
  arr limits = C.getLimits();
  FrameL collisionPairs = C.getCollisionAllPairs();
  //for(uint i=0;i<collisionPairs.d0;i++) cout <<"PAIR: " <<collisionPairs(i,0)->name <<' ' <<collisionPairs(i,1)->name <<endl;

  BotOp bot(C, rai::checkParameter<bool>("real"));
  bot.home(C);
  arr bounds = ~C.getLimits();

  uint N=100;
  arr x_last = bot.get_q();
  for(uint i=0;i<N;i++){
    cout <<" ====== POSE " <<i <<" ====== " <<endl;
    arr x = bounds[0] + (bounds[1]-bounds[0]) % rand(qHome.N);

    C.setJointState(x);
    //C.watch(false, STRING("conf " <<i));
//    bool succ = checkCollisionsAndLimits(C, collisionPairs, limits, true);
    bool succ = PoseTool(C,0).checkLimitsAndCollisions(limits, {}, true);
    if(succ){
      cout <<" === pose made feasible === " <<endl;
      x = C.getJointState();
      //C.watch(true, STRING("conf " <<i));

      C.setJointState(x_last);
      arr path = getStartGoalPath(C, x, bot.qHome); //, {}, {"l_gripper", "r_gripper"}, true, true);
      if(!path.N){
        cout <<" === path infeasible === " <<endl;
        continue;
      }
      x_last = x;
      cout <<" === path feasible  === " <<endl;
      while(bot.step(C));
      cout <<" === -> executing === " <<endl;
      bot.moveAutoTimed(path, .01);
      if(bot.keypressed=='q') break;
    }else{
      cout <<" === pose infeasible === " <<endl;
    }
  }
}

//===========================================================================

int main(int argc, char * argv[]){
  rai::initCmdLine(argc, argv);

//  rnd.clockSeed();
  rnd.seed(2);
  //  driveToPoses();
  rndPoses();

  return 0;
}
