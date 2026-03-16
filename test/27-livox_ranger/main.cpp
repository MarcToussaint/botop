#include <BotOp/bot.h>

#define ON_REAL true


void justSpin(){
  rai::Configuration C;
  // C.addFile(rai::raiPath("../rai-robotModels/ranger/ranger_simplified.g"));
  C.addFile(rai::raiPath("../rai-robotModels/scenarios/panda_ranger.g"));
  // C.addFile(rai::raiPath("../rai-robotModels/scenarios/pandaSingle.g"));

  BotOp bot(C, ON_REAL);
  bot.sync(C);

  C.view(true);
  double time = 1.;

  // arr q = arr{0., 0., -1.57, 0, -0.5, 0, -2, 0, 2, -0.5};
  // bot.moveTo(q, {time});
  // bot.wait(C);
  
  // q = arr{.1, .2, -1.57, 0, -0.5, 0, -2, 0, 2, -0.5};
  // bot.moveTo(q, {time});
  // bot.wait(C);

  // q = arr{.1, .2, -1.57, 0, -0.5, 0, -2, 0, 2.5, -0.5};
  // bot.moveTo(q, {time});
  // bot.wait(C);

  // q = arr{.1, .2, -1.57, 0, -0.5, 0, -2, 0, 2, -0.5};
  // bot.moveTo(q, {time});
  // bot.wait(C);
  
  // q = arr{.1, .2, 0, 0, -0.5, 0, -2, 0, 2, -0.5};
  // bot.moveTo(q, {time});
  // bot.wait(C);

  // q = arr{0., 0., 0., 0, -0.5, 0, -2, 0, 2, -0.5};
  // bot.moveTo(q, {time});
  // bot.wait(C);

  arr q = arr{0, 0, 0, 0, -0.5, 0, -2, 0, 2.5, -0.5};
  // arr q = arr{0, -0.5, 0, -2, 0, 2.5, -0.5};
  bot.moveTo(q, {time});
  bot.wait(C);

  q = arr{0, 0, 0, 0, -0.5, 0, -2, 0, 2, -0.5};
  // q = arr{0, -0.5, 0, -2, 0, 2, -0.5};
  bot.moveTo(q, {time});
  bot.wait(C);
  C.view(true);
}

int main(int argc, char** argv){
  rai::initCmdLine(argc, argv);

  // moveRanger();
  // moveRangerLoop();
  justSpin();

  return 0;
}
