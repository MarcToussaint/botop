#pragma once

#include <Kin/kin.h>
#include <Core/thread.h>


namespace rai{

struct SLAM {

  SLAM(rai::Configuration& C);
  ~SLAM();

  void update(rai::Configuration& C);

private:
  std::vector<arr> graph_nodes;
  arr new_node_thresh;
  arr q_last;
  int node_counter;
};

} //namespace
