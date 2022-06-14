/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#ifdef RAI_PYBIND

#include "pyBot.h"

#include <ry/types.h>

#include "bot.h"

#include <KOMO/pathTools.h>

template<> pybind11::array_t<double> arr2numpy(const rai::Array<double>& x){
  //default!
  if(!x.isSparse()) return pybind11::array_t<double>(x.dim(), x.p);
  //sparse!
  arr triplets = x.sparse().getTriplets();
  return pybind11::array_t<double>(triplets.dim(), triplets.p);
}

PYBIND11_MODULE(libpybot, m) {
  m.doc() = "bot bindings";

  init_PyBot(m);
}

void init_PyBot(pybind11::module& m) {
  
  pybind11::class_<BotOp, shared_ptr<BotOp>>(m, "BotOp", "")

  .def(pybind11::init<rai::Configuration&, bool>(), "")

  .def("get_t", &BotOp::get_t)
  .def("get_qHome", &BotOp::get_qHome)
  .def("get_q", &BotOp::get_q)
  .def("get_qDot", &BotOp::get_qDot)
  .def("getTimeToEnd", &BotOp::getTimeToEnd)

    .def("move", pybind11::overload_cast<const arr&, const arr&, const arr&, bool, double>(&BotOp::move))
    .def("move", pybind11::overload_cast<const arr&, const arr&, bool, double>(&BotOp::move))
  .def("moveAutoTimed", &BotOp::moveAutoTimed)
  .def("moveLeap", &BotOp::moveLeap)
  .def("setControllerWriteData", &BotOp::setControllerWriteData)

  .def("gripperOpen", &BotOp::gripperOpen)
  .def("gripperClose", &BotOp::gripperClose)
  .def("gripperPos", &BotOp::gripperPos)

  .def("step", &BotOp::step)

  .def("home", &BotOp::home)
  .def("hold", &BotOp::hold)

  .def("isDone", &BotOp::isDone)

  ;

  //===========================

  m.def("getStartGoalPath", [](rai::Configuration& C, const arr& qTarget, const arr& qHome){
      return getStartGoalPath(C, qTarget, qHome);
  } );

}

#endif
