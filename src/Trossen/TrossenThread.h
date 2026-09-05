#include <Core/thread.h>
#include <Control/CtrlMsgs.h>

namespace trossen_arm{
class TrossenArmDriver;
}

struct TrossenThread : rai::RobotAbstraction, rai::Thread {
  std::shared_ptr<trossen_arm::TrossenArmDriver> driver;
  str ipAddress;
  arr Kp, Kd;
  double ctrlTime=0.;

  ofstream fil;

  TrossenThread(rai::Var<rai::CtrlCmdMsg>& cmd, rai::Var<rai::CtrlStateMsg>& state, const char* ipAddress="192.168.1.3");
  ~TrossenThread(){
    LOG(0) <<"shutting down Trossen -- " <<timer.report();
    threadClose();
  }

  void open();
  void step();
  void close();
};
