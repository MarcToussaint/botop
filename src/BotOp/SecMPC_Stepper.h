#pragma once

#include "bot.h"
#include <Control/SecMPC.h>

//===========================================================================

struct SecMPC_Stepper{
  Metronome tic;
  uint stepCount = 0;
  ofstream fil;
  FrameL logPoses;

  SecMPC_Stepper( double cycleTime=.1)
    : tic(cycleTime){
    fil.open(STRING("z.SecMPC.log"));
//    fil.open(STRING("z." <<rai::date(true) <<".secMPC.log"));
  }

  bool step(rai::Configuration& C, BotOp& bot, SecMPC& mpc);
};

//===========================================================================

//helper
void randomWalkPosition(rai::Frame* f, arr& centerPos, arr& velocity, double rate=.001);


void playLog(const rai::String& logfile);
