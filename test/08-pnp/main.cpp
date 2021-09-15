#include <BotOp/bot.h>
#include <BotOp/motionHelpers.h>

//===========================================================================

void testPnp() {
  rai::Configuration C;
  C.addFile(rai::raiPath("../rai-robotModels/scenarios/pandasTable.g"));

  C.addFrame("target", "table")
      ->setJoint(rai::JT_rigid)
      .setShape(rai::ST_ssBox, {.06,.15,.09,.01})
      .setRelativePosition(arr{-.3,-.2,.095});

  arr qHome = C.getJointState();

  //-- start a robot thread
  BotOp bot(C, rai::checkParameter<bool>("real"));
  bot.home(C);

  const char* gripperName="l_gripper";
  const char* palmName="l_palm";
  const char* boxName="target";
  const char* tableName="table";
  const char* arm1Name="l_panda_coll7";
  const char* arm2Name="l_panda_coll6";

  uint L=50;
  for(uint l=0;l<=L;l++){
    //-- pick a random place direction
    rai::Enum<rai::ArgWord> placeDirection = random(rai::Array<rai::ArgWord>{rai::_yAxis, rai::_zAxis, rai::_yNegAxis, rai::_zNegAxis });
    cout <<"========= PLACEMENT " <<l <<": " <<placeDirection <<endl;

    //-- compute pnp keyframes (that box can only be picked in x-direction)
    C.setJointState(bot.get_q());
    arr keyframes = getBoxPnpKeyframes(C, rai::_xAxis, placeDirection, boxName, gripperName, palmName, tableName, qHome);
    if(!keyframes.N) continue; //infeasible

    //-- execute keyframes
    for(uint k=0;k<keyframes.d0;k++){
      C.setJointState(bot.get_q());
      arr path;
      if(l==L) k=2; //in last episode, just go home

      if(k==0){ //move to keyframes[0]
        C.attach(tableName, boxName);
        path = getStartGoalPath(C, keyframes[0], qHome, { {{.3,.7}, {palmName, boxName}, .1},
                                                          {{}, {palmName, boxName}, .0},
                                                          {{}, {arm1Name, tableName}, .0},
                                                          {{}, {arm2Name, tableName}, .0},
                                                        }, {gripperName}, true, true);
        if(!path.N) break;
      }

      if(k==1){ //move to keyframes[1]
        C.attach(gripperName, boxName);
        path = getStartGoalPath(C, keyframes[1], qHome, { {{.3,.7}, {boxName, tableName}, .05},
                                                          {{}, {boxName, tableName}, .0},
                                                          {{}, {arm1Name, tableName}, .0},
                                                          {{}, {arm2Name, tableName}, .0}
                                                        }, {gripperName}, false, false);
        if(!path.N) break;
      }

      if(k==2){ //move to home
        C.attach(tableName, boxName);
        path = getStartGoalPath(C, qHome, qHome, { {{.3,.5}, {palmName, boxName}, .1},
                                                   {{}, {palmName, boxName}, .0},
                                                   {{}, {arm1Name, tableName}, .0}
                                }, {gripperName}, false, true);
      }

      if(bot.gripperL){
        if(k==0){ bot.gripperL->open(); while(!bot.gripperL->isDone()) rai::wait(.1); }
        else if(k==1){ bot.gripperL->close(); while(!bot.gripperL->isDone()) rai::wait(.1); }
        else if(k==2){ bot.gripperL->open(); while(!bot.gripperL->isDone()) rai::wait(.1); }
      }

      //send komo as spline:
      //bot.move(path, {2.5});
      bot.moveAutoTimed(path, .003);

      while(bot.step(C));
      if(bot.keypressed=='q' || bot.keypressed==27) return;
    }
  }

}

//===========================================================================

int main(int argc, char * argv[]){
  rai::initCmdLine(argc, argv);

//  rnd.clockSeed();
  rnd.seed(0);

  testPnp();

  LOG(0) <<" === bye bye ===\n used parameters:\n" <<rai::getParameters()() <<'\n';

  return 0;
}
