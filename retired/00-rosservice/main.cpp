#include <Gui/opengl.h>
#include <LGP/optLGP.h>
#include <Kin/TM_qItself.h>
#include <trajectory_msgs/JointTrajectory.h>

#include <RosCom/roscom.h>
#include <RosCom/spinner.h>

#include <ros/ros.h>
#include <RosCom/rai_msgs/StringString.h>
#include <Core/util.h>

//======================================================================

bool echo(rai_msgs::StringStringRequest &req,
                 rai_msgs::StringStringResponse &res){
    res.str = req.str;
    return true;
}

//======================================================================

int main(int argc,char **argv){
  ros::init(argc, argv, "echo_service");

  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("echo", echo);
  ROS_INFO("Ready to echo.");
  ros::spin();

  return 0;
}

//===========================================================================

/**int main(int argc, char** argv) {
  setLogLevels(0,0);
  rai::initCmdLine(argc, argv);
  ros::init(rai::argc, rai::argv, "RAPshell");

  if(argc<2) return query("getState");
  if(!strcmp(argv[1],"st")) return query("getState");
  if(!strcmp(argv[1],"sy")) return query("getSymbols");

  //-- send a fact
  rai::String fact;
  fact <<"( ";
  for(int i=1;i<argc;i++) fact <<argv[i] <<' ';
  fact <<")";
  return query(fact);

  return 0;
}**/
