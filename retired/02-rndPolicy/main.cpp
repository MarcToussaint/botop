#include <KOMO/komo.h>
#include <Kin/taskMaps.h>
#include <Core/graph.h>
#include <Kin/frame.h>

#include <RosCom/roscom.h>
#include <RosCom/spinner.h>

#include <rai_msgs/MotionReference.h>
#include <rai_msgs/arr.h>


//=============================================================================

void rndPolicy(){
  RosCom ROS("rndPolicy");

  rai::KinematicWorld K("../model/LGP-kuka.g");
  arr q0 = K.getJointState();
  StringA joints = K.getJointNames();
  cout <<"JOINTS: " <<joints <<endl;

  double speed = .1;

  Var<rai_msgs::MotionReference> ref("MotionReference");

  auto pubRef = ROS.publish(ref);

  for(uint i=0;;i++){
    rai::wait(1e-3*rnd(10,1000)/speed);

    uint T = rnd(1,10);
    arr x = repmat(~q0,T,1);
    rndUniform(x, -3., 3., true);

    arr t = zeros(T);
    rndUniform(t, .5/speed, 2./speed);

    ref.writeAccess();
    ref->x = conv_arr2arr(x);
    ref->t = conv_arr2arr(t);
    ref->append = rnd(0,1);
    ref->revision = i;
    ref.deAccess(); //publishes automatically
  }
}

//=============================================================================

int main(int argc,char **argv){
  rai::initCmdLine(argc, argv);

  rndPolicy();

  return 0;
}
