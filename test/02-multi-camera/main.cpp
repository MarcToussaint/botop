#include <Core/array.h>
#include <Gui/opengl.h>
#include <RealSense/MultiRealSenseThread.h>
// #include <Gui/viewer.h>
#include <Core/thread.h>
#include <BotOp/bot.h>

void direct(){
  rai::MultiRealSenseThread RS({"102422075114", "102422071099", "825312070938"}, true, false);
  uint V = RS.cameras.N;
  std::vector<OpenGL> windows(V);

  RS.color(-1).waitForNextRevision();
  for(uint i = 0; i < V; i++) {
    auto colorGet = RS.color(i).get();
    windows[i].resize(colorGet().d1, colorGet().d0);
  }


  CycleTimer tim;
  for(;;) {
    RS.color(-1).waitForNextRevision();

    tim.cycleStart();
    int key=0;
    for(uint i = 0; i < V; i++) {
      auto colorGet = RS.color(i).get();
      key = windows[i].watchImage(colorGet(), false, 1.);
    }
    tim.cycleDone();

    if(key=='q') break;
  }

  cout <<"DISPLAY timer:   " <<tim.report() <<endl;
  cout <<"RealSense timer: " <<RS.timer.report() <<endl;

  LOG(0) <<"bye bye";
}

void botop(){
  rai::Configuration C;
  C.addFile("../15-arucoSystem/table.yml");

  rai::setParameter("botsim/verbose", 1);
  BotOp bot(C, false);


  FrameL f_cams;
  for(uint k=0;k<1;k++)  f_cams.append( C.getFrame(STRING("cam"<<k)) );
  bot.launch_MultiRealSense(f_cams, true, false);

//  rai::MultiRealSenseThread RS({"102422075114"}, true, false);

  bot.realsenses->color(-1).waitForNextRevision();

  for(;;){
    int key = bot.sync(C);
    if(key=='q') break;
  }
}

int main(int argc, char** argv) {
  rai::initCmdLine(argc, argv);

//  direct();
  botop();

  return 0;
}
