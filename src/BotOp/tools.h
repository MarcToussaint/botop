#include "bot.h"
#include <KOMO/komo.h>

struct Move_IK{
  BotOp& bot;
  rai::Configuration& C;
  int askForOK;
  KOMO komo;
  arr qT;

  Move_IK(BotOp& _bot, rai::Configuration& _C, int _askForOK=true);

  KOMO& operator()(){ return komo; }

  bool go();

};

bool sense_HsvBlob(BotOp& bot, rai::Configuration& C,
                   const char* camName, const char* blobName,
                   const arr& hsvFilter, const arr& Pinv={}, int verbose=0);
