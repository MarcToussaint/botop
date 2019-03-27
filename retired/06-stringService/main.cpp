#include <RosCom/roscom.h>
#include <RosCom/rai_msgs/StringString.h>

struct StringService {
  ros::NodeHandle nh;
  ros::ServiceServer service;
  bool cb_service(rai_msgs::StringString::Request& _request, rai_msgs::StringString::Response& _response);

  StringService();
  ~StringService(){}
};


StringService::StringService() {
  LOG(1) <<"*** Starting String Service" <<endl;
  service = nh.advertiseService("/StringService", &StringService::cb_service, this);
}

bool StringService::cb_service(rai_msgs::StringString::Request& _request, rai_msgs::StringString::Response& _response) {
  rai::String request = _request.str.c_str();

  rai::String response = "HELLO";

  cout <<"---\nreceived new string '" <<request <<"'" <<endl;
  cout <<"returning string '"<<response <<"'" <<endl;

  _response.str = response.p;
  return true;
}

//===========================================================================

int main(int argc, char** argv) {
  ros::init(argc, argv, "RelationalMachineNode");

  StringService S;
  ros::spin();

  return 0;
}

