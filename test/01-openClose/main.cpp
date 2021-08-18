#include <Franka/gripper.h>
#include <Robotiq/RobotiqGripper.h>
#include <Franka/controlEmulator.h>

int main(int argc, char** argv) {
  rai::initCmdLine(argc, argv);

  {
    std::cout<<"hello"<<std::endl;

    std::shared_ptr<rai::GripperAbstraction> G_ri;
    if(rai::getParameter<bool>("robotiq", true)){
      G_ri = make_shared<RobotiqGripper>(0);
    }else{
      G_ri = make_shared<FrankaGripper>(1);
    }
    std::cout <<"gripper pos:" <<G_ri->pos() /*<<" isGrasped:" <<G_ri->isGrasped()*/ <<std::endl;
    rai::wait();

    std::cout <<"=========== fast close (is not faster) ..." <<std::endl;
    G_ri->close(2, .01, .1);
    for(uint t=0;t<10;t++){
        std::cout <<"gripper pos while running:" <<G_ri->pos() /*<<" isGrasped:" <<G_ri->isGrasped()*/ <<std::endl;
    }
    G_ri->waitForIdle();
    cout <<"done" <<endl;
    rai::wait();

    G_ri->open();
    rai::wait();

    G_ri->close();
    rai::wait();
    G_ri->open();
    rai::wait();

    G_ri->close();
    rai::wait();
    G_ri->open();
    rai::wait();

#if 0
    rai::wait();
    cout <<"=========== Normal open..." <<std::endl;
    G_ri->open();
    G_ri->waitForIdle();
    rai::wait();

    cout <<"=========== slow weak close (slow, but not weaker)..." <<flush;
    G_ri->close(2, .05, .02);
    G_ri->waitForIdle();
    cout <<"done" <<endl;
    rai::wait();
    cout <<"=========== Normal open..." <<std::endl;
    G_ri->open();
    G_ri->waitForIdle();
    rai::wait();

    cout <<"=========== strong close (becomes strong late) ..." <<flush;
    G_ri->close(100.);
    G_ri->waitForIdle();
    cout <<"done" <<endl;
    rai::wait();
    cout <<"=========== Normal open..." <<std::endl;
    G_ri->open();
    G_ri->waitForIdle();
    rai::wait();

//    cout <<"=========== homing (measures the close/open positions) ..." <<flush;
//    G_ri->homing();
//    G_ri->waitForIdle();
//    cout <<"done" <<endl;
#endif
  }

  return 0;
}
