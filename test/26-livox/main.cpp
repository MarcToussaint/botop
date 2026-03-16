#include <Livox/livox.h>
#include <Core/graph.h>

const char *USAGE =
    "\nTest of low-level (without bot interface) Livox interface"
    "\n";

int main(int argc,char **argv){
  rai::initCmdLine(argc, argv);

  cout <<USAGE <<endl;

  rai::Configuration C;
  rai::Livox lidar;

  for(;;){
    lidar.pull(C);
    if(C.view(false)=='q') break;
  }

  LOG(0) <<" === bye bye ===\n used parameters:\n" <<rai::params() <<'\n';

  return 0;
}
