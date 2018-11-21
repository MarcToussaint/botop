#include "RealSenseThread.h"

#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>

float get_depth_scale(rs2::device dev);
rs2_stream find_stream_to_align(const std::vector<rs2::stream_profile>& streams);

struct sRealSenseThread{
  std::shared_ptr<rs2::pipeline> pipe;
  std::shared_ptr<rs2::align> align;
  float depth_scale;
  rs2_intrinsics depth_intrinsics;
};

RealSenseThread::RealSenseThread()
  : Thread("RealSense"){
  threadLoop();
}

RealSenseThread::~RealSenseThread(){
  threadClose();
}

void RealSenseThread::open(){
  s = new sRealSenseThread;
  rs2::log_to_console(RS2_LOG_SEVERITY_ERROR);
  s->pipe = std::make_shared<rs2::pipeline>();
  s->pipe->start();

  rs2::pipeline_profile profile = s->pipe->get_active_profile();

  s->depth_scale = get_depth_scale(profile.get_device());

  for (rs2::stream_profile sp : profile.get_streams()){

    LOG(1) <<"stream '" <<sp.stream_name() <<"' idx:" <<sp.stream_index() <<" type:" <<sp.stream_type() <<" format:" <<sp.format() <<" fps" <<sp.fps() <<" id:" <<sp.unique_id();

    rs2_intrinsics intrinsics;
    rs2_get_video_stream_intrinsics(sp.get(), &intrinsics, NULL);
    LOG(1) <<"intrinsics: " <<intrinsics.width <<' ' <<intrinsics.height <<' ' <<intrinsics.ppx <<' ' <<intrinsics.ppy <<' ' <<intrinsics.fx <<' ' <<intrinsics.fy;
    if(sp.stream_type()==RS2_STREAM_DEPTH) rs2_get_video_stream_intrinsics(sp.get(), &s->depth_intrinsics, NULL);
  }
  LOG(1) <<"depth scale: " <<s->depth_scale;
  s->align = std::make_shared<rs2::align>(RS2_STREAM_DEPTH); //RS2_STREAM_COLOR
}

void RealSenseThread::close(){
  s->pipe->stop();
  delete s;
}

void RealSenseThread::step(){
  rs2::frameset data = s->pipe->wait_for_frames(); // Wait for next set of frames from the camera

  rs2::frameset processed = s->align->process(data);

  rs2::depth_frame rs_depth = processed.get_depth_frame(); // Find and colorize the depth data
  rs2::video_frame rs_color = processed.get_color_frame();            // Find the color data

//  // Trying to get both color and aligned depth frames
//  rs2::video_frame other_frame = processed.first_or_default(align_to);
//  rs2::depth_frame aligned_depth_frame = processed.get_depth_frame();


  float pixel[2];
  float point[3];
  {
    auto depthSet = depth.set();
    auto pointsSet = points.set();
    depthSet->resize(rs_depth.get_height(), rs_depth.get_width());
    pointsSet->resize(rs_depth.get_height(), rs_depth.get_width(), 3);
    for(uint y=0;y<depthSet->d0;y++) for(uint x=0;x<depthSet->d1;x++){
      float d = rs_depth.get_distance(x,y);
      if(d>1.) d=1.;
      depthSet->operator()(y,x) = d;
      pixel[0]=x;
      pixel[1]=y;
      rs2_deproject_pixel_to_point(point, &s->depth_intrinsics, pixel, d);
      for(uint i=0;i<3;i++) pointsSet()(y,x,i) = point[i];
    }
  }
  {
    auto colorSet = color.set();
    colorSet->resize(rs_color.get_height(), rs_color.get_width(), 3);
    CHECK(rs_color.get_bytes_per_pixel()==3,"");
    memmove(colorSet->p, rs_color.get_data(), colorSet->N);
  }
}

float get_depth_scale(rs2::device dev){
    // Go over the device's sensors
    for (rs2::sensor& sensor : dev.query_sensors())
    {
        // Check if the sensor if a depth sensor
        if (rs2::depth_sensor dpt = sensor.as<rs2::depth_sensor>())
        {
            return dpt.get_depth_scale();
        }
    }
    throw std::runtime_error("Device does not have a depth sensor");
}


void rs2_get_motion_intrinsics(const rs2_stream_profile* mode, rs2_motion_device_intrinsic * intrinsics, rs2_error ** error);
