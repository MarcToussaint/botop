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

  rs2::temporal_filter temp_filter;
  rs2::hole_filling_filter hole_filter;
};

RealSenseThread::RealSenseThread()
  : Thread("RealSenseThread"){
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

  //-- info on all sensors of the device
  rs2::device dev = profile.get_device();
  for (rs2::sensor& sensor:dev.query_sensors()){
    LOG(1) <<"sensor " <<sensor.get_info(RS2_CAMERA_INFO_NAME);
    for (int i = 0; i < static_cast<int>(RS2_OPTION_COUNT); i++)  {
      rs2_option option_type = static_cast<rs2_option>(i);
      if (sensor.supports(option_type)){
        LOG(1) <<"  option " <<option_type <<'=' <<sensor.get_option(option_type) <<"  (" <<sensor.get_option_description(option_type) <<")  [" <<sensor.get_option_range(option_type).min <<',' <<sensor.get_option_range(option_type).max<<']';
      }
    }
    if(!strcmp(sensor.get_info(RS2_CAMERA_INFO_NAME),"RGB Camera")){
      if(sensor.supports(RS2_OPTION_ENABLE_AUTO_EXPOSURE)){
        sensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 0);
        sensor.set_option(RS2_OPTION_EXPOSURE, 500.);
        LOG(1) <<"  I disabled auto exposure";
      }
      if(sensor.supports(RS2_OPTION_ENABLE_AUTO_WHITE_BALANCE)){
        sensor.set_option(RS2_OPTION_ENABLE_AUTO_WHITE_BALANCE, 0);
        sensor.set_option(RS2_OPTION_WHITE_BALANCE, 4000.);
        LOG(1) <<"  I disabled auto white balance";
      }
    }
    if(!strcmp(sensor.get_info(RS2_CAMERA_INFO_NAME),"Stereo Module")){
      if(sensor.supports(RS2_OPTION_ENABLE_AUTO_EXPOSURE)){
        sensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 0);
        sensor.set_option(RS2_OPTION_EXPOSURE, 6000.);
        LOG(1) <<"  I disabled auto exposure";
      }
    }
  }

  //-- info on all streams
  for (rs2::stream_profile sp : profile.get_streams()){
    LOG(1) <<"stream '" <<sp.stream_name() <<"' idx:" <<sp.stream_index() <<" type:" <<sp.stream_type() <<" format:" <<sp.format() <<" fps" <<sp.fps() <<" id:" <<sp.unique_id();
    rs2::video_stream_profile vsp = sp.as<rs2::video_stream_profile>();
    if(vsp){
      rs2_intrinsics intrinsics = vsp.get_intrinsics();
      LOG(1) <<"  is video: w=" <<intrinsics.width <<" h=" <<intrinsics.height <<" px=" <<intrinsics.ppx << " py=" <<intrinsics.ppy <<" fx=" <<intrinsics.fx <<" fy=" <<intrinsics.fy <<" distorsion=" <<intrinsics.model <<floatA(intrinsics.coeffs, 5);
      if(sp.stream_type()==RS2_STREAM_DEPTH) depth_fxypxy = ARR(intrinsics.fx, intrinsics.fy, intrinsics.ppx, intrinsics.ppy);
      if(sp.stream_type()==RS2_STREAM_COLOR) color_fxypxy = ARR(intrinsics.fx, intrinsics.fy, intrinsics.ppx, intrinsics.ppy);
    }

    if(sp.stream_type()==RS2_STREAM_COLOR) rs2_get_video_stream_intrinsics(sp.get(), &s->depth_intrinsics, NULL);
  }

  //-- depth scale of the camera (
  s->depth_scale = get_depth_scale(profile.get_device());
  LOG(1) <<"depth scale: " <<s->depth_scale;

  //-- align with depth or color?
  s->align = std::make_shared<rs2::align>(RS2_STREAM_DEPTH); //RS2_STREAM_COLOR
}

void RealSenseThread::close(){
  s->pipe->stop();
  delete s;
}

void RealSenseThread::step(){
  rs2::frameset data;
  try {
    data = s->pipe->wait_for_frames(); // Wait for next set of frames from the camera
  } catch (rs2::error& err) {
    LOG(-1) <<"Can't get frames from RealSense: " <<err.what();
    return;
  }

  rs2::frameset processed = s->align->process(data);

  rs2::depth_frame rs_depth = processed.get_depth_frame(); // Find and colorize the depth data
  rs2::video_frame rs_color = processed.get_color_frame();            // Find the color data


  rs2::depth_frame rs_depth2 = s->temp_filter.process(rs_depth);
  rs_depth = s->hole_filter.process(rs_depth2);

#if 1
  {
    auto depthSet = depth.set();
    depthSet->resize(rs_depth.get_height(), rs_depth.get_width());
    for(uint y=0;y<depthSet->d0;y++) for(uint x=0;x<depthSet->d1;x++){
      float d = rs_depth.get_distance(x,y);
      if(d>1.) d=1.;
      depthSet->operator()(y,x) = d;
    }
  }
#else //also compute point cloud
  float pixel[2];
  float point[3];
  {
    auto depthSet = depth.set();
    auto pointsSet = points.set();
    depthSet->resize(rs_depth.get_height(), rs_depth.get_width());
    pointsSet->resize(rs_depth.get_height(), rs_depth.get_width(), 3);
    depth2
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
#endif
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
