#include <Franka/gripper.h>
#include <Franka/controlEmulator.h>

int main(int argc, char** argv) {

  {
    auto G_ri = make_shared<FrankaGripper>(0);
//    auto G_le = make_shared<FrankaGripper>(1);

    G_ri->close(20, .035);
    rai::wait(2.);
    G_ri->open();
    rai::wait(2.);

//    rai::wait(1.);
//    G_le->open();
//    rai::wait(1.);
//    G_le->close(20, .035);

    rai::wait(.1);
  }

  return 0;
}
