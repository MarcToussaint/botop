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

  bool step(rai::Configuration& C, BotOp& bot, SecMPC& mpc, bool doNotExecute=false);
};

//===========================================================================

struct SecMPC_Viewer {
  uint stepCount, phase;
  double ctrlTime;
  arr q_real, waypoints, tau, shortPath, poses;
  rai::Configuration C;

  SecMPC_Viewer(const rai::Configuration& C);

  void step(SecMPC& mpc);
  void glDraw(OpenGL& gl);
};

//===========================================================================

//helper
void randomWalkPosition(rai::Frame* f, arr& centerPos, arr& velocity, double rate=.001);


void playLog(const rai::String& logfile);
