#include <ros/ros.h>
#include <rai_srv/StringString.h>
#include <Core/util.h>


//===========================================================================

int query(const char* cmd){
  rai_srv::StringString com;
  com.request.str = cmd;
  if(ros::service::call("/RAP/service", com)){
    cout <<cmd <<": " <<com.response.str <<endl;
  }else cout <<"getState failed" <<endl;

  return 0;
}

//===========================================================================

int main(int argc, char** argv) {
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
}
