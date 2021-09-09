#include <OptiTrack/optitrack.h>

//#include "orgTest.cxx"

void test(){
  rai::Configuration C;

  rai::OptiTrack OT;

  for(;;) OT.step(C);
}

int main(int argc, char** argv) {
  rai::initCmdLine(argc, argv);

//  return rawTest("optitrack", rai::getParameter<rai::String>("optitrack/host"));

  test();

  return 0;
}
