#pragma once

#include "simulation.h"

struct SimulationIO : Thread{
  struct SimulationIO_self *self=0;

  SimulationIO(bool pubSubToROS, const char* modelFile, double dt=.01);
  ~SimulationIO();

  void step();
  void close(){}
  void open(){}

  void loop();
};
