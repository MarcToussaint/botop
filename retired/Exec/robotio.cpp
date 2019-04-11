#include "robotio.h"
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <RosCom/roscom.h>
#include <ros/ros.h>

#include <trajectory_msgs/JointTrajectory.h>
#include <RosCom/rai_msgs/SendJointTrajectory.h>
#include <RosCom/rai_msgs/StringString.h>

RobotIO::RobotIO(bool _pubSubToROS):
  pubSubToROS(_pubSubToROS),
  jointState("/joint_states"),
  tfMessages("/tf"),
  gripperCommand("/schunk_driver/schunk_wsg_command"){
  if(pubSubToROS){
    sub_jointState = ROS.subscribe(jointState);
    sub_tfMessages = ROS.subscribe(tfMessages);
    pub_gripperCommand = ROS.publish(gripperCommand);
  }else{
    LOG(1) <<"test mode -- not publishing/subscribing to ROS!";
  }
}

bool RobotIO::executeMotion(const StringA &joints, const arr &path, const arr &times, double timeScale){
  trajectory_msgs::JointTrajectory msg;

  //msg.header.stamp = ros::Time::now();

  for(const rai::String& str:joints){
    msg.joint_names.push_back(str.p);
  }

  CHECK_EQ(path.d1, joints.N, "");
  CHECK_EQ(path.d0, times.N, "");
  CHECK_GE(timeScale, 1., "need a timeScale >=1");

  for(uint t=0;t<path.d0;t++){
    trajectory_msgs::JointTrajectoryPoint point;
    point.positions = conv_arr2stdvec(path[t]);
    point.velocities = conv_arr2stdvec(zeros(path.d1));
    point.accelerations = conv_arr2stdvec(zeros(path.d1));
    point.effort = conv_arr2stdvec(zeros(path.d1));
    point.time_from_start = ros::Duration(times(t)*timeScale);
    msg.points.push_back(point);
  }

  if(pubSubToROS){
    rai_msgs::SendJointTrajectory com;
    com.request.trajectory = msg;
    LOG(0) <<"SERVICE CALL: send trajectory";
    if(ros::service::call("/robot_control/SendJointTrajectory", com)){
      LOG(0) <<"SERVICE CALL: return = " <<(com.response.success?"TRUE":"FALSE");
    }else{
      LOG(0) <<"SERVICE CALL: FAILED!";
      return false;
    }
  }else{
    LOG(1) <<"test mode -- not publishing/subscribing to ROS!";
  }
  return true;

}

void RobotIO::execGripper(double position, double force){
  if(position<0 || position>100.){
    LOG(-1) <<"commanded gripper position '" <<position <<"' out of range [0, 100]";
    return;
  }

  rai_msgs::WSG_50_command cmd;
  cmd.position_mm = position;
  cmd.force = 60.0;

  gripperCommand.set() = cmd;
}

arr RobotIO::getJointPositions(const StringA &joints){
  jointState.waitForRevisionGreaterThan(10);
  arr X(joints.N);
  jointState.readAccess();
  for(uint i=0;i<joints.N;i++){
    for(uint j=0;j<jointState->name.size();j++){
      if(joints(i)==jointState->name[j].c_str()){
        X(i) = jointState->position[j];
      }
    }
  }
  jointState.deAccess();
  return X;
}

arr RobotIO::getObjectPoses(const StringA& objs){

    rai::Transformation OPTI=0;
    OPTI.addRelativeRotationDeg(-90, 1,0,0);
    OPTI.addRelativeRotationDeg(180, 0,0,1);
    arr poses(objs.N,7); // point x,y,z quaternion x,y,z,w
    for(uint i=0; i<objs.N; i++){
        auto obName = objs(i);
        if(objectStates.count(obName)){
            objectStates[obName]->waitForRevisionGreaterThan(10);
            objectStates[obName]->readAccess();
            auto pose = objectStates[obName]->pose;
            poses(i,0) = pose.position.x;
            poses(i,1) = pose.position.y;
            poses(i,2) = pose.position.z;
            poses(i,3) = pose.orientation.w;
            poses(i,4) = pose.orientation.x;
            poses(i,5) = pose.orientation.y;
            poses(i,6) = pose.orientation.z;

            rai::Transformation tmp;
            tmp.set(poses[i]);
            poses[i] = ((-OPTI)*tmp*OPTI).getArr7d();

            objectStates[obName]->deAccess();
        }
    }

    return poses;
}

StringA RobotIO::getJointNames() {
  jointState.waitForRevisionGreaterThan(10);
  StringA joints;
  jointState.readAccess();
  joints.resize(jointState->name.size());
  for(uint j=0;j<jointState->name.size();j++){
    joints(j) = jointState->name[j];
  }
  jointState.deAccess();
  return joints;
}

StringA RobotIO::getObjectNames() {

    StringA objects;
    ros::master::V_TopicInfo master_topics;
    ros::master::getTopics(master_topics);
    std::string objectTopicPrefix = "/vrpn_client_node/";
    uint numObjects = 0;

    for (ros::master::V_TopicInfo::iterator it = master_topics.begin() ; it != master_topics.end(); it++) {
      const ros::master::TopicInfo& info = *it;
      //cout << "ROSTOPIC: " << info.name << endl;
       if(std::equal(
             info.name.begin(),
             info.name.begin() + std::min( info.name.size(), objectTopicPrefix.size() ),
             objectTopicPrefix.begin() )){
         numObjects++;
       }
    }

    objects.resize(numObjects);
    numObjects = 0;

    for (ros::master::V_TopicInfo::iterator it = master_topics.begin() ; it != master_topics.end(); it++) {
      const ros::master::TopicInfo& info = *it;
       if(std::equal(
             info.name.begin(),
             info.name.begin() + std::min( info.name.size(), objectTopicPrefix.size() ),
             objectTopicPrefix.begin() )){
         objects(numObjects) = info.name.substr(objectTopicPrefix.size(),info.name.size() - objectTopicPrefix.size() - 5);
         numObjects++;
       }
    }

    // add new topics to subscriber map
    for (uint i= 0; i < objects.size(); i++){
        auto obName = objects(i);
        if (objectStates.count(obName)){
            // already added
        } else {
            // new topic
            auto obTopic = STRING(objectTopicPrefix << obName << "/pose");
            auto obState = new Var<geometry_msgs::PoseStamped>(obTopic);
            objectStates[obName] = obState;
            sub_objectStates[obName] = ROS.subscribe(objectStates[obName]);
        }
    }

    return objects;
}


