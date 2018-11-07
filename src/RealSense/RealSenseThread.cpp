#include "RealSenseThread.h"

#include <librealsense2/rs.hpp>

RealSenseThread::RealSenseThread()
  : Thread("RealSense"){
  threadLoop();
}

RealSenseThread::~RealSenseThread(){
  threadClose();
}

void RealSenseThread::open(){
  rs2::log_to_console(RS2_LOG_SEVERITY_ERROR);
  pipe = std::make_shared<rs2::pipeline>();
  pipe->start();
}

void RealSenseThread::close(){
  pipe->stop();
}

void RealSenseThread::step(){
  rs2::frameset data = pipe->wait_for_frames(); // Wait for next set of frames from the camera

  rs2::depth_frame rs_depth = data.get_depth_frame(); // Find and colorize the depth data
  rs2::video_frame rs_color = data.get_color_frame();            // Find the color data

  {
    auto depthSet = depth.set();
    depthSet->resize(rs_depth.get_height(), rs_depth.get_width());
    for(uint y=0;y<depthSet->d0;y++) for(uint x=0;x<depthSet->d1;x++){
      double d = rs_depth.get_distance(x,y);
      if(d>1.) d=1.;
      depthSet->operator()(y,x) = d;
    }
  }
  {
    auto colorSet = color.set();
    colorSet->resize(rs_color.get_height(), rs_color.get_width(), 3);
    CHECK(rs_color.get_bytes_per_pixel()==3,"");
    memmove(colorSet->p, rs_color.get_data(), colorSet->N);
  }
}
