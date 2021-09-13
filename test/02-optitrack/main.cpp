#include <OptiTrack/optitrack.h>

//#include "orgTest.cxx"

int main(int argc, char** argv) {
  rai::initCmdLine(argc, argv);

//  return rawTest("optitrack", rai::getParameter<rai::String>("optitrack/host"));

  rai::Configuration C;
  rai::OptiTrack OT;
  for(;;) OT.step(C);

  return 0;
}
