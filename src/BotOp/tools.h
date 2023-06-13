#include "bot.h"
#include <KOMO/komo.h>

struct Move_IK{
  BotOp& bot;
  rai::Configuration& C;
  bool askForOK;
  KOMO komo;
  arr qT;

  Move_IK(BotOp& _bot, rai::Configuration& _C, bool _askForOK=true) : bot(_bot), C(_C), askForOK(_askForOK){
    komo.setConfig(C, false);
    komo.setTiming(1., 1, 1., 0);
    komo.addControlObjective({}, 0, 1e-1);
  }

  KOMO& operator()(){ return komo; }

  bool go();

};

bool sense_HsvBlob(BotOp& bot, rai::Configuration& C,
                   const char* camName, const char* blobName,
                   const arr& hsvFilter, const arr& Pinv={}, int verbose=0);
