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


  BotOp bot(C, rai::getParameter<bool>("real", false));
  bot.home(C);

  double q0 = qHome(jointID);
  double lo = qLimits(jointID, 0)+.1;
  double up = qLimits(jointID, 1)-.1;
  if(jointID==1) up -= 1.2;

  uint k=30;
  arr path = ~qHome;
  arr times = ARR(0.);
  double q = lo;
  double v = 0.;
  double t = 10.;
  double vstep = .005;

  for(uint s=0;;s++){
    for(uint j=0;j<k;j++){
      q += .1*v;
      t += .1;
      if(q>up) break;
      path.append(qHome);  path(-1, jointID) = q;
      times.append(t);
    }
    if(q>up) break;
    v += vstep;
  }
  q = up;
  v = 0.;
  for(uint s=0;;s++){
    for(uint j=0;j<k;j++){
      q += .1*v;
      t += .1;
      if(q<lo) break;
      path.append(qHome);  path(-1, jointID) = q;
      times.append(t);
    }
    if(q<lo) break;
    v -= vstep;
  }

#if 0
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
#endif

  path.append(qHome);
  times.append(t+10.);
  times *= .5;

  rai::wait();

  bot.robotL->writeData=2;
  bot.move(path, times);
  while(bot.sync(C)){}
  bot.robotL->writeData=0;


  rai::wait();
}

//===========================================================================

int main(int argc, char * argv[]){
  rai::initCmdLine(argc, argv);


  collectJointData();

  return 0;
}
