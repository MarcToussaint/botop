/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

#ifdef RAI_PYBIND

#include "py-BotOp.h"

#include <ry/types.h>

#include "bot.h"

#include <KOMO/pathTools.h>

//PYBIND11_MODULE(libpybot, m) {
//  m.doc() = "bot bindings";

//  init_PyBot(m);
//}

void init_BotOp(pybind11::module& m) {
  
  pybind11::class_<BotOp, shared_ptr<BotOp>>(m, "BotOp", "Robot Operation interface -- see https://marctoussaint.github.io/robotics-course/tutorials/1b-botop.html")

  .def(pybind11::init<rai::Configuration&, bool>(),
       "constructor",
       pybind11::arg("C"),
       pybind11::arg("useRealRobot")
       )

  .def("get_t", &BotOp::get_t,
       "returns the control time (absolute time managed by the high freq tracking controller)")

  .def("get_qHome", &BotOp::get_qHome,
       "returns home joint vector (defined as the configuration C when you created BotOp)")

  .def("get_q", &BotOp::get_q,
       "get the current (real) robot joint vector")

  .def("get_qDot", &BotOp::get_qDot,
       "get the current (real) robot joint velocities")

  .def("get_tauExternal", &BotOp::get_tauExternal,
       "get the current (real) robot joint torques (external: gravity & acceleration removed) -- each call averages from last call; first call might return nonsense!")

  .def("getTimeToEnd", &BotOp::getTimeToEnd,
       "get time-to-go of the current spline reference that is tracked (use getTimeToEnd()<=0. to check if motion execution is done)")

   .def("getKeyPressed", &BotOp::getKeyPressed, "",
        "get key pressed in window at last sync")

  //.def("move", pybind11::overload_cast<const arr&, const arr&, const arr&, bool, double>(&BotOp::move))
  .def("move", pybind11::overload_cast<const arr&, const arr&, bool, double>(&BotOp::move),
       "core motion command: set a spline motion reference; if only a single time [T] is given for multiple waypoints, it assumes equal time spacing with TOTAL time T"
       "\n\nBy default, the given spline is APPENDED to the current reference spline. The user can also enforce the given spline to overwrite the current reference "
       "starting at the given absolute ctrlTime. This allows implementation of reactive (e.g. MPC-style) control. However, the user needs to take care that "
       "overwriting is done in a smooth way, i.e., that the given spline starts with a pos/vel that is close to the pos/vel of the current reference at the given ctrlTime.",
       pybind11::arg("path"),
       pybind11::arg("times"),
       pybind11::arg("overwrite") = false,
       pybind11::arg("overwriteCtrlTime") = -1.)

  .def("moveAutoTimed", &BotOp::moveAutoTimed,
       "helper to execute a path (typically fine resolution, from KOMO or RRT) with equal time spacing chosen for given max vel/acc",
       pybind11::arg("path"),
       pybind11::arg("maxVel") =  1.,
       pybind11::arg("maxAcc") =  1.)

  .def("moveTo", &BotOp::moveTo,
       "helper to move to a single joint vector target, where timing is chosen optimally based on the given timing cost"
       "\n\nWhen using overwrite, this immediately steers to the target -- use this as a well-timed reactive q_target controller",
       pybind11::arg("q_target"),
       pybind11::arg("timeCost") = 1.,
       pybind11::arg("overwrite") = false)

  .def("setCompliance", &BotOp::setCompliance,
       "set a task space compliant, where J defines the task space Jacobian, and compliance goes from 0 (no compliance) to 1 (full compliance, but still some damping)",
       pybind11::arg("J"),
       pybind11::arg("compliance") = .5)

  .def("setControllerWriteData", &BotOp::setControllerWriteData,
       "[for internal debugging only] triggers writing control data into a file")

  .def("gripperMove", &BotOp::gripperMove,
       "move the gripper to width (default: open)",
      pybind11::arg("leftRight"),
      pybind11::arg("width") = .075,
      pybind11::arg("speed") = .2)

  .def("gripperClose", &BotOp::gripperClose,
       "close gripper",
       pybind11::arg("leftRight"),
       pybind11::arg("force") = 10.,
       pybind11::arg("width") = .05,
       pybind11::arg("speed") = .1)

  .def("gripperCloseGrasp", &BotOp::gripperCloseGrasp,
        "close gripper and indicate what should be grasped -- makes no difference in real, but helps simulation to mimic grasping more reliably",
        pybind11::arg("leftRight"),
        pybind11::arg("objName"),
        pybind11::arg("force") = 10.,
        pybind11::arg("width") = .05,
        pybind11::arg("speed") = .1)

  .def("getGripperPos", &BotOp::getGripperPos,
       "returns the gripper pos",
       pybind11::arg("leftRight"))

  .def("gripperDone", &BotOp::gripperDone,
       "returns if gripper is done",
       pybind11::arg("leftRight"))

  .def("getCameraFxycxy", &BotOp::getCameraFxycxy,
       "returns camera intrinsics",
       pybind11::arg("sensorName"))

  .def("getImageAndDepth",  [](std::shared_ptr<BotOp>& self, const char* sensorName) {
      byteA img;
      floatA depth;
      self->getImageAndDepth(img, depth, sensorName);
      return pybind11::make_tuple(Array2numpy<byte>(img),
                                  Array2numpy<float>(depth)); },
       "returns image and depth from a camera sensor",
       pybind11::arg("sensorName"))

  .def("getImageDepthPcl",  [](std::shared_ptr<BotOp>& self, const char* sensorName, bool globalCoordinates) {
         byteA img;
         floatA depth;
         arr pts;
         self->getImageDepthPcl(img, depth, pts, sensorName, globalCoordinates);
         return pybind11::make_tuple(Array2numpy<byte>(img),
                                     Array2numpy<float>(depth),
                                     Array2numpy<double>(pts)); },
       "returns image, depth and point cloud (assuming sensor knows intrinsics) from a camera sensor, optionally in global instead of camera-frame-relative coordinates",
       pybind11::arg("sensorName"),
       pybind11::arg("globalCoordinates") = false)

  .def("sync", &BotOp::sync,
       "sync your workspace configuration C with the robot state",
       pybind11::arg("C"),
       pybind11::arg("waitTime") = .1,
       pybind11::arg("viewMsg") = rai::String{})

  .def("wait", &BotOp::wait,
       "repeatedly sync your workspace C until a key is pressed or motion ends (optionally)",
       pybind11::arg("C"),
       pybind11::arg("forKeyPressed") = true,
       pybind11::arg("forTimeToEnd") = true,
       pybind11::arg("forGripper") = false)

  .def("home", &BotOp::home,
       "immediately drive the robot home (see get_qHome); keeps argument C synced; same as moveTo(qHome, 1., True); wait(C);",
       pybind11::arg("C"))

  .def("stop", &BotOp::stop,
       "immediately stop the robot; keeps argument C synced; same as moveTo(get_q(), 1., True); wait(C);",
       pybind11::arg("C"))

  .def("hold", &BotOp::hold,
       "hold the robot with a trivial PD controller, floating means reference = real, without damping the robot is free floating",
       pybind11::arg("floating") = false,
       pybind11::arg("damping") = true)
  ;

}

#endif
