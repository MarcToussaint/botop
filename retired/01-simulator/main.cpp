#include <Sim/simulator.h>
#include <Sim/simulationIO.h>

//=============================================================================

int main(int argc,char **argv){
  rai::initCmdLine(argc, argv);

#if 0
  Simulator sim("../model/LGP-kuka.g");
#else
  SimulationIO sim("../model/LGP-kuka.g");
#endif

  sim.loop();

  return 0;
}
