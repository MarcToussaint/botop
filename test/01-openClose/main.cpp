#include <Franka/gripper.h>
#include <Franka/controlEmulator.h>

int main(int argc, char** argv) {

  {
    std::shared_ptr<FrankaGripper> G_ri = make_shared<FrankaGripper>(0);
//    auto G_le = make_shared<FrankaGripper>(1);
    cout <<"gripper pos:" <<G_ri->pos() <<" isGrasped:" <<G_ri->isGrasped() <<endl;

    cout <<"=========== fast close (is not faster) ..." <<endl;
    G_ri->close(20, .05, .2);
    for(uint t=0;t<10;t++){
      cout <<"gripper pos while running:" <<G_ri->pos() <<" isGrasped:" <<G_ri->isGrasped() <<endl;
    }
    G_ri->waitForIdle();
    cout <<"done" <<endl;
    rai::wait();
    G_ri->open();
    G_ri->waitForIdle();
    rai::wait();

    cout <<"=========== slow weak close (slow, but not weaker)..." <<flush;
    G_ri->close(2, .05, .02);
    G_ri->waitForIdle();
    cout <<"done" <<endl;
    rai::wait();
    G_ri->open();
    G_ri->waitForIdle();
    rai::wait();

    cout <<"=========== strong close (becomes strong late) ..." <<flush;
    G_ri->close(100.);
    G_ri->waitForIdle();
    cout <<"done" <<endl;
    rai::wait();
    G_ri->open();
    G_ri->waitForIdle();
    rai::wait();

    cout <<"=========== homing (measures the close/open positions) ..." <<flush;
    G_ri->homing();
    G_ri->waitForIdle();
    cout <<"done" <<endl;
  }

  return 0;
}
