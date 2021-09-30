#include <KOMO/pathTools.h>

#include <BotOp/bot.h>
#include <BotOp/motionHelpers.h>

//===========================================================================

void collectJointData(){
  rai::Configuration C("../../rai-robotModels/scenarios/pandasTable.g");
  arr qHome = C.getJointState();
  arr qLimits = C.getLimits();

  uint jointID = rai::getParameter<int>("joint", 6);
  CHECK_LE(jointID, qHome.N, "");


  BotOp bot(C, rai::checkParameter<bool>("real"));
  bot.home(C);

  double q0 = qHome(jointID);
  double lo = qLimits(jointID, 0)+.1;
  double up = qLimits(jointID, 1)-.1;

  uint k=3;
  arr path = ~qHome;
  arr times = ARR(0.);
  double q = lo;
  double t = 1.;
  for(uint s=0;;s++){
    q += .01 * s;
    t += 1.;
    if(q>up) break;
    for(uint j=0;j<k;j++){
      path.append(qHome);
      path(-1, jointID) = q;
      times.append(t);
      t += .2;
    }
  }
  q = up;
  for(uint s=0;;s++){
    q -= .01 * s;
    t += 1.;
    if(q<lo) break;
    for(uint j=0;j<k;j++){
      path.append(qHome);
      path(-1, jointID) = q;
      times.append(t);
      t += .2;
    }
  }
  path.append(qHome);
  times.append(t+2.);
  times *= .5;

  rai::wait();

  bot.robotL->writeData=1;
  bot.move(path, times);
  while(bot.step(C)){}
  bot.robotL->writeData=0;


  rai::wait();
}

//===========================================================================

int main(int argc, char * argv[]){
  rai::initCmdLine(argc, argv);


  collectJointData();

  return 0;
}
