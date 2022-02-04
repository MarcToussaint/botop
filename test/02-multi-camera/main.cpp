#include <Core/array.h>
#include <Gui/opengl.h>
#include <RealSense/MultiRealSenseThread.h>
#include <Gui/viewer.h>
#include <Core/thread.h>

int main(int argc, char** argv) {
  rai::initCmdLine(argc, argv);

  rai::realsense::MultiRealSenseThread RS({"102422075114", "102422071099"}, {}, {}, true, false);
  uint V = RS.getNumberOfCameras();
  std::vector<OpenGL> windows(V);

  RS.color.waitForNextRevision();
  for(uint i = 0; i < V; i++) {
    auto colorGet = RS.color.get();
    windows[i].resize(colorGet()[i].d1, colorGet()[i].d0);
  }


  CycleTimer tim;
  for(;;) {
    RS.color.waitForNextRevision();

    tim.cycleStart();
    int key=0;
    for(uint i = 0; i < V; i++) {
      auto colorGet = RS.color.get();
      key = windows[i].watchImage(colorGet()[i], false, 1.);
    }
    tim.cycleDone();

    if(key=='q') break;
  }

  cout <<"DISPLAY timer:   " <<tim.report() <<endl;
  cout <<"RealSense timer: " <<RS.timer.report() <<endl;

  LOG(0) <<"bye bye";

  return EXIT_SUCCESS;
}
