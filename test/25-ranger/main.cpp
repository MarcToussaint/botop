#include <KOMO/komo.h>
#include <BotOp/bot.h>

#define ON_REAL true


arr getTrianglePath(rai::Configuration& C)
{
  C.addFrame("way0")->setShape(rai::ST_marker, {.3}).setPosition({.3, 0., 1.});
  C.addFrame("way1")->setShape(rai::ST_marker, {.3}).setPosition({.3, -.1, 1.});
  C.addFrame("way2")->setShape(rai::ST_marker, {.3}).setPosition({0, 0, 1.});

  KOMO komo(C, 3., 20, 2, false);

  komo.addControlObjective({}, 2, 1.);
  
  komo.addObjective({1.}, FS_positionDiff, {"ranger", "way0"}, OT_eq, {1e1, 1e1, 0});
  komo.addObjective({2.}, FS_positionDiff, {"ranger", "way1"}, OT_eq, {1e1, 1e1, 0});
  komo.addObjective({3.}, FS_positionDiff, {"ranger", "way2"}, OT_eq, {1e1, 1e1, 0});
  
  komo.opt.verbose=-1;
  komo.optimize();

  return komo.getPath_qOrg();
}

arr getYPath(rai::Configuration& C)
{
  C.addFrame("way0")->setShape(rai::ST_marker, {.3}).setPosition({.3, 0., 1.});
  C.addFrame("way2")->setShape(rai::ST_marker, {.3}).setPosition({0, 0, 1.});

  KOMO komo(C, 2., 20, 2, false);

  komo.addControlObjective({}, 2, 1.);
  
  komo.addObjective({1.}, FS_positionDiff, {"ranger", "way0"}, OT_eq, {1e1, 1e1, 0});
  komo.addObjective({2.}, FS_positionDiff, {"ranger", "way2"}, OT_eq, {1e1, 1e1, 0});
  
  komo.opt.verbose=-1;
  komo.optimize();

  return komo.getPath_qOrg();
}

arr getPathToTarget(rai::Configuration& C, double target_x, double target_y)
{
  C.addFrame("way0")->setShape(rai::ST_marker, {.3}).setPosition({target_x, target_y, 1.});
  C.addFrame("way2")->setShape(rai::ST_marker, {.3}).setPosition({0, 0, 1.});

  KOMO komo(C, 2., 20, 2, false);

  komo.addControlObjective({}, 2, 1.);
  
  komo.addObjective({1.}, FS_positionDiff, {"ranger", "way0"}, OT_eq, {1e1, 1e1, 0});
  komo.addObjective({2.}, FS_positionDiff, {"ranger", "way2"}, OT_eq, {1e1, 1e1, 0});
  
  komo.opt.verbose=-1;
  komo.optimize();

  return komo.getPath_qOrg();
}

void moveRanger(){
  rai::Configuration C;
  C.addFile(rai::raiPath("../rai-robotModels/ranger/ranger_simplified.g"));

  // arr q = getTrianglePath(C);
  arr q = getYPath(C);
  C.view(true);

  BotOp bot(C, ON_REAL);

  bot.move(q, {10.});
  bot.wait(C);
}

void moveRangerLoop(){
  rai::Configuration C;
  C.addFile(rai::raiPath("../rai-robotModels/ranger/ranger_simplified.g"));
  C.view(false);

  int point_count = 6;
  double offset = M_PI*.25;
  double mag = .3;
  for (int i = 0; i < point_count; i++) {
    double angle = static_cast<double>(i)/static_cast<double>(point_count) * M_PI*2.0 + offset;
    arr q = getPathToTarget(C, cos(angle)*mag, -sin(angle)*mag);

    BotOp bot(C, ON_REAL);

    bot.move(q, {5.});
    bot.wait(C);
  }
}

void justSpin(){
  rai::Configuration C;
  C.addFile(rai::raiPath("../rai-robotModels/ranger/ranger_simplified.g"));

  BotOp bot(C, ON_REAL);

  double time = 5.;

  arr q = arr{0., 0., -1.57};
  bot.moveTo(q, {time});
  bot.wait(C);
  
  q = arr{.1, .2, -1.57};
  bot.moveTo(q, {time});
  bot.wait(C);
  
  q = arr{.1, .2, 0};
  bot.moveTo(q, {time});
  bot.wait(C);

  q = arr{0., 0., 0.};
  bot.moveTo(q, {time});
  bot.wait(C);
}

int main(int argc, char** argv){
  rai::initCmdLine(argc, argv);

  // moveRanger();
  // moveRangerLoop();
  justSpin();

  return 0;
}
