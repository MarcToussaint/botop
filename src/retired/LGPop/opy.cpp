#ifdef RAI_PYBIND

#include "opy.h"
#include "lgpop.h"

void RyLGPop::runRobotControllers(LGPop::OpMode opMode){
  self->runRobotControllers(opMode);
}

void RyLGPop::runTaskController(int verbose){
  self->runTaskController(verbose);
}

void RyLGPop::runCamera(int verbose){
  self->runCamera(verbose);
}

void RyLGPop::runPerception(int verbose){
  self->runPerception(verbose);
}

void RyLGPop::runCalibration(){
  self->runCalibration();
}

void RyLGPop::reportCycleTimes(){
  self->reportCycleTimes();
}

void RyLGPop::updateArmPoseCalibInModel(){
  self->updateArmPoseCalibInModel();
}

void RyLGPop::sim_addRandomBox(const char* name){
  self->sim_addRandomBox(name);
}


#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>

namespace py = pybind11;


#define METHOD(method) .def(#method, &RyLGPop::method)
#define METHOD_set(method) .def(#method, [](ry::Config& self) { self.set()->method(); } )
#define METHOD_set1(method, arg1) .def(#method, [](ry::Config& self) { self.set()->method(arg1); } )


PYBIND11_MODULE(libLGPop, m) {

  py::class_<RyLGPop>(m, "LGPop")
      .def(py::init<LGPop::OpMode>())
      METHOD(runRobotControllers)
      METHOD(runTaskController)
      METHOD(runCamera)
      METHOD(runPerception)
      METHOD(runCalibration)
      METHOD(reportCycleTimes)
      METHOD(updateArmPoseCalibInModel)
      METHOD(sim_addRandomBox)
      ;

}

#endif


