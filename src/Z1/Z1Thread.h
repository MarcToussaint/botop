#include <Core/thread.h>
#include <Control/CtrlMsgs.h>

namespace UNITREE_ARM{
class unitreeArm;
}

struct Z1Thread : rai::RobotAbstraction, rai::Thread {
  std::shared_ptr<UNITREE_ARM::unitreeArm> driver;
  strA ipAddresses;
  arr Kp, Kd;
  double ctrlTime=0.;
  enum { torque_mode, position_mode } mode;

  ofstream dataFile;

  Z1Thread(rai::Var<rai::CtrlCmdMsg>& cmd, rai::Var<rai::CtrlStateMsg>& state, const strA& ids);
  ~Z1Thread(){
    LOG(0) <<"shutting down Trossen -- " <<timer.report();
    threadClose();
  }

  void open();
  void step();
  void close();
};
