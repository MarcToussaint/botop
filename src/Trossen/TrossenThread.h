#include <Core/thread.h>
#include <Control/CtrlMsgs.h>

namespace trossen_arm{
class TrossenArmDriver;
}

struct TrossenThread : rai::RobotAbstraction, rai::Thread {
  std::shared_ptr<trossen_arm::TrossenArmDriver> driver;
  strA ipAddresses;
  arr Kp, Kd;
  double ctrlTime=0.;
  enum { torque_mode, position_mode } mode;

  ofstream dataFile;

  TrossenThread(rai::Var<rai::CtrlCmdMsg>& cmd, rai::Var<rai::CtrlStateMsg>& state, const strA& ids);
  ~TrossenThread(){
    LOG(0) <<"shutting down Trossen -- " <<timer.report();
    threadClose();
  }

  void open();
  void step();
  void close();
};
