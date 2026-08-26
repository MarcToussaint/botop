#include <BotOp/bot.h>

//===========================================================================

arr rock = {
    -0.1194, 1.2068, 1.0,    1.4042,
    -0.0093, 1.2481, 1.4073, 0.8163,
    0.1116,  1.2712, 1.3881, 1.0122,
    0.6017,  0.2976, 0.9034, 0.7929};
arr paper = {
    -0.1220, 0.4,    0.6,    -0.0769,
    0.0312,  0.4,    0.6,    -0.0,
    0.1767,  0.4,    0.6,    -0.0528,
    0.5284,  0.3693, 0.8977, 0.4863};
arr scissors = {
    0.0885, 0.4,    0.6,    -0.0704,
    0.0312, 0.4,    0.6,    -0.0,
    0.1019, 1.2375, 1.1346, 1.0244,
    1.0,    0.6331, 1.3509, 1.0};

void test_mini() {
  rai::Configuration C;
  C.addFile("allegro_cube.yml");
  // C.view(true);

  arr q_home = C.getJointState();

  BotOp bot(C, rai::getParameter<bool>("real", false));

  bot.launch_allegro();

  // rai::wait(1.);
  bot.moveTo(q_home, .5, true);
  bot.wait(C);

  double speed = .2;
  for(uint k=0; k<3; k++){
    bot.moveTo(rock, speed, true);
    bot.wait(C);
    bot.moveTo(scissors, speed, true);
    bot.wait(C);
    bot.moveTo(paper, speed, true);
    bot.wait(C);
  }

  // for(;;){
  //   int key = bot.sync(C);
  //   cout <<"q: " <<bot.get_qDot() <<endl;
  //   if(key=='q') break;
  // }

  // bot.wait(C, false, true);

  // C.view(true);


  // bot.wait(C);
}


//===========================================================================

int main(int argc, char * argv[]){
  rai::initCmdLine(argc, argv);

  test_mini();

  LOG(0) <<" === bye bye ===\n used parameters:\n" <<rai::params() <<'\n';

  return 0;
}
