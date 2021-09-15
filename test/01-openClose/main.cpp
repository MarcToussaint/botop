#include <Franka/gripper.h>
#include <Robotiq/RobotiqGripper.h>

const char *USAGE =
    "\nTest of low-level (without bot interface) RobotiqGripper and FrankaGripper interfaces"
    "\n";

int main(int argc,char **argv){
  rai::initCmdLine(argc, argv);

  cout <<USAGE <<endl;

  {
    std::shared_ptr<rai::GripperAbstraction> G_ri;
    if(rai::getParameter<bool>("bot/useRobotiq", true)){
      G_ri = make_shared<RobotiqGripper>(0);
    }else{
      G_ri = make_shared<FrankaGripper>(0);
    }

    std::cout <<"=========== standard close ..." <<std::endl;
    G_ri->close();
    while(!G_ri->isDone()){
      std::cout <<"gripper pos while running:" <<G_ri->pos() /*<<" isGrasped:" <<G_ri->isGrasped()*/ <<std::endl;
      rai::wait(.1);
    }
    cout <<"...done" <<endl;

    std::cout <<"=========== intermediate steps ..." <<std::endl;
    for(double w=.3;w<=1.;w+=.1){
      G_ri->close(0., w);
      cout <<"w: " <<w <<"pos: " <<G_ri->pos() <<endl;;
      rai::wait(.3);
    }

    cout <<"=========== Normal open..." <<std::endl;
    G_ri->open();
    while(!G_ri->isDone()) rai::wait(.1);

    cout <<"=========== slow & weak ..." <<flush;
    G_ri->close(0, .2, .02);
    while(!G_ri->isDone()) rai::wait(.1);
    cout <<"done" <<endl;
    rai::wait();
    G_ri->open();
    while(!G_ri->isDone()) rai::wait(.1);

    cout <<"=========== fast & strong..." <<flush;
    G_ri->close(1., .2, 1.);
    while(!G_ri->isDone()) rai::wait(.1);
    cout <<"done" <<endl;
    rai::wait();
    G_ri->open();
    while(!G_ri->isDone()) rai::wait(.1);

//    cout <<"=========== homing (measures the close/open positions) ..." <<flush;
//    G_ri->homing();
//    G_ri->waitForIdle();
//    cout <<"done" <<endl;

  }

  LOG(0) <<" === bye bye ===\n used parameters:\n" <<rai::getParameters()() <<'\n';

  return 0;
}
