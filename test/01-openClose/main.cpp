#include <Franka/gripper.h>
#include <Franka/controlEmulator.h>

int main(int argc, char** argv) {

  {
    auto G_ri = make_shared<FrankaGripper>(0);
//    auto G_le = make_shared<FrankaGripper>(1);
    cout <<"gripper pos:" <<G_ri->pos() <<" isGrasped:" <<G_ri->isGrasped() <<endl;

    G_ri->close(20, .035, .5);
    cout <<"gripper pos:" <<G_ri->pos() <<" isGrasped:" <<G_ri->isGrasped() <<endl;

    G_ri->open();
    cout <<"gripper pos:" <<G_ri->pos() <<" isGrasped:" <<G_ri->isGrasped() <<endl;

    G_ri->close(20, .05, .2);
    cout <<"gripper pos:" <<G_ri->pos() <<" isGrasped:" <<G_ri->isGrasped() <<endl;

    G_ri->homing();
//    G_ri->open();
    cout <<"gripper pos:" <<G_ri->pos() <<" isGrasped:" <<G_ri->isGrasped() <<endl;
  }

  return 0;
}
