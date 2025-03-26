#include "slam.h"

#define RAI_SLAM
#ifdef RAI_SLAM

#include <Kin/frame.h>


const char* slam_base_frame_name = "slam_graph_base";
const char* mobile_base_frame = "ranger_coll";

void set_new_node_frame(rai::Configuration& C, int node_id)
{
  const std::string intials = "pgo_node";
  const std::string new_frame_name = intials + std::to_string(node_id);

  rai::Frame *mobile_base = C.getFrame(mobile_base_frame);
  arr new_pose = mobile_base->getPose();
  
  rai::Frame *new_node_frame = C.addFrame(new_frame_name.c_str(), slam_base_frame_name, nullptr, false);
  new_node_frame->setShape(rai::ST_marker, {.2});
  new_node_frame->setColor({.0, .0, .7});
  new_pose.elem(2) += .75;
  new_node_frame->setPose(new_pose);
}

namespace rai{

  SLAM::SLAM(rai::Configuration& C)
  {
    if (C.getJointState().N > 3) {
      LOG(-1) <<"BotOp SLAM only works for SE(2) for now!";
    }

    new_node_thresh = rai::getParameter<arr>("SLAM/newNodeThresh", arr{.3, .3, .75});
    
    rai::Frame *base = C.getFrame(slam_base_frame_name, false);
    if(!base){
      LOG(0) <<"creating new frame 'slam_graph_base'";
      base = C.addFrame(slam_base_frame_name);
      base->setShape(rai::ST_marker, {.3});
      base->setColor({.8, .8, .2});
    }
    
    q_last = C.getJointState();
    graph_nodes.push_back(q_last);
    
    node_counter = 1;
    set_new_node_frame(C, node_counter);
  }

  SLAM::~SLAM() { }

  void SLAM::update(rai::Configuration& C)
  {
    arr q = C.getJointState();

    // Create new graph node if q state changed enough
    bool over_thresh = false;
    for (int i = 0; i < 3; i++) {
      if (abs(q.elem(i) - q_last.elem(i)) > new_node_thresh.elem(i)) {
        over_thresh = true;
        break;
      }
    }
    
    if (over_thresh)
    {
      graph_nodes.push_back(q);
      
      node_counter++;
      set_new_node_frame(C, node_counter);
      
      q_last = q;
    }
  }  
} //namespace

#else

rai::SLAM::SLAM(rai::Configuration& C) { NICO }
rai::SLAM::~SLAM(){ NICO }
void rai::SLAM::update(rai::Configuration& C){ NICO }

#endif
