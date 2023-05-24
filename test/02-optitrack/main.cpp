#include <OptiTrack/optitrack.h>
#include <Core/graph.h>

const char *USAGE =
    "\nTest of low-level (without bot interface) OptiTrack interface"
    "\n";

int main(int argc,char **argv){
  rai::initCmdLine(argc, argv);

  cout <<USAGE <<endl;

//  return rawTest("optitrack", rai::getParameter<rai::String>("optitrack/host"));

  rai::Configuration C;
  rai::OptiTrack OT;

  for(;;){
    OT.pull(C);
    if(C.view(false)=='q') break;
  }

  LOG(0) <<" === bye bye ===\n used parameters:\n" <<rai::params() <<'\n';

  return 0;
}
