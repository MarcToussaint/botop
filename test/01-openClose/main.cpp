#include <Franka/gripper.h>
#include <Franka/controlEmulator.h>

int main(int argc, char** argv) {

  {
    auto G_ri = make_shared<FrankaGripper>(0);
//    auto G_le = make_shared<FrankaGripper>(1);
    cout <<"gripper pos:" <<G_ri->pos() <<" isGrasped:" <<G_ri->isGrasped() <<endl;

    G_ri->close(20, .035, .5);

    //    cout <<"gripper pos:" <<G_ri->pos() <<" isGrasped:" <<G_ri->isGrasped() <<endl;
    for(uint t=0;t<20;t++){
      cout <<"gripper pos:" <<G_ri->pos() <<" isGrasped:" <<G_ri->isGrasped() <<endl;
    }

//    rai::wait();
    G_ri->waitForIdle();

    G_ri->open();

    G_ri->waitForIdle();

    G_ri->close(20, .05, .2);

    G_ri->waitForIdle();

    G_ri->homing();

    G_ri->waitForIdle();
  }

  return 0;
}
