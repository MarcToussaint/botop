#include "RealSenseThread.h"

#ifdef RAI_REALSENSE

#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
#include "utils.h"

rs2_stream find_stream_to_align(const std::vector<rs2::stream_profile>& streams);

struct sRealSenseThread{
  std::shared_ptr<rs2::config> cfg;
  std::shared_ptr<rs2::pipeline> pipe;
  std::shared_ptr<rs2::align> align;
  float depth_scale;
  rs2_intrinsics depth_intrinsics;

  rs2::temporal_filter temp_filter;
  rs2::hole_filling_filter hole_filter;
};

RealSenseThread::RealSenseThread(const char *_name)
  : Thread("RealSenseThread"),
    image(this),
    depth(this){
  if(_name) CameraAbstraction::name=_name;
  threadOpen(true);
  threadLoop();
}

RealSenseThread::~RealSenseThread(){
  LOG(0) <<"DTOR";
  threadClose();
}

void RealSenseThread::getImageAndDepth(byteA& _image, floatA& _depth){
  uint n=60;
  if(image.getRevision()<n){
    LOG(0) <<"waiting to get " <<n <<" images from RealSense for autoexposure settling";
    image.waitForRevisionGreaterThan(n); //need many starting images for autoexposure to get settled!!
    depth.waitForRevisionGreaterThan(n);
  }
  _image = image.get();
  _depth = depth.get();
}

void RealSenseThread::open(){
  bool longCable = rai::getParameter<bool>("RealSense/longCable", false);
  int resolution = rai::getParameter<int>("RealSense/resolution", 640);
  bool alignToDepth = rai::getParameter<bool>("RealSense/alignToDepth", true);
  bool autoExposure = rai::getParameter<bool>("RealSense/autoExposure", true);
  double exposure = rai::getParameter<double>("RealSense/exposure", 500);
  double white = rai::getParameter<double>("RealSense/white", 4000);

  s = new sRealSenseThread;

  s->cfg = std::make_shared<rs2::config>();
  if(resolution==480){
    s->cfg->enable_stream(RS2_STREAM_COLOR, -1, 424, 240, rs2_format::RS2_FORMAT_RGB8, 0);
    s->cfg->enable_stream(RS2_STREAM_DEPTH, -1, 480, 270, rs2_format::RS2_FORMAT_Z16, 15);
  }else if(resolution==640){
    s->cfg->enable_stream(RS2_STREAM_COLOR, -1, 640, 360, rs2_format::RS2_FORMAT_RGB8, 30);
    s->cfg->enable_stream(RS2_STREAM_DEPTH, -1, 640, 360, rs2_format::RS2_FORMAT_Z16, 30);
  }else if(resolution==1280){
    s->cfg->enable_stream(RS2_STREAM_COLOR, -1, 1280, 720, rs2_format::RS2_FORMAT_RGB8, 30);
    s->cfg->enable_stream(RS2_STREAM_DEPTH, -1, 1280, 720, rs2_format::RS2_FORMAT_Z16, 30);
  }else{
    LOG(-2) <<"RealSense Driver: resolution=" <<resolution <<" option not available (avaiable: 480, 640, 1280)";
  }

  rs2::log_to_console(RS2_LOG_SEVERITY_ERROR);
  s->pipe = std::make_shared<rs2::pipeline>();
  s->pipe->start(*s->cfg);
  rs2::pipeline_profile profile = s->pipe->get_active_profile();

  //-- info on all sensors of the device
  rs2::device dev = profile.get_device();
  for (rs2::sensor& sensor:dev.query_sensors()){
    LOG(1) <<"sensor " <<sensor.get_info(RS2_CAMERA_INFO_NAME);
    if(!longCable){ //crashes with long cable
      for (int i = 0; i < static_cast<int>(RS2_OPTION_COUNT); i++)  {
        rs2_option option_type = static_cast<rs2_option>(i);
        if (sensor.supports(option_type)){
          LOG(1) <<"  option " <<option_type <<'=' <<sensor.get_option(option_type) <<"  (" <<sensor.get_option_description(option_type) <<")  [" <<sensor.get_option_range(option_type).min <<',' <<sensor.get_option_range(option_type).max<<']';
        }
      }
    }
    if(!autoExposure){
      if(!strcmp(sensor.get_info(RS2_CAMERA_INFO_NAME),"RGB Camera")){
        if(sensor.supports(RS2_OPTION_ENABLE_AUTO_EXPOSURE)){
          sensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 0);
          sensor.set_option(RS2_OPTION_EXPOSURE, exposure);
          LOG(1) <<"  I disabled auto exposure";
        }
        if(sensor.supports(RS2_OPTION_ENABLE_AUTO_WHITE_BALANCE)){
          sensor.set_option(RS2_OPTION_ENABLE_AUTO_WHITE_BALANCE, 0);
          sensor.set_option(RS2_OPTION_WHITE_BALANCE, white);
          LOG(1) <<"  I disabled auto white balance";
        }
      }
      if(false && !longCable){ //auto exposure works pretty bad for depth module //crashes with long cable
        if(!strcmp(sensor.get_info(RS2_CAMERA_INFO_NAME),"Stereo Module")){
          if(sensor.supports(RS2_OPTION_ENABLE_AUTO_EXPOSURE)){
            sensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 0);
            sensor.set_option(RS2_OPTION_EXPOSURE, 5000.);
            LOG(1) <<"  I disabled auto depth exposure";
          }
        }
      }
    }else{
      if(!strcmp(sensor.get_info(RS2_CAMERA_INFO_NAME),"RGB Camera")){
        if(sensor.supports(RS2_OPTION_ENABLE_AUTO_EXPOSURE)){
          sensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 1);
          LOG(1) <<"  I enabled auto exposure";
        }
        if(sensor.supports(RS2_OPTION_ENABLE_AUTO_WHITE_BALANCE)){
          sensor.set_option(RS2_OPTION_ENABLE_AUTO_WHITE_BALANCE, 1);
          LOG(1) <<"  I enabled auto white balance";
        }
      }
      if(!longCable){ //crashes with long cable
        if(!strcmp(sensor.get_info(RS2_CAMERA_INFO_NAME),"Stereo Module")){
          if(sensor.supports(RS2_OPTION_ENABLE_AUTO_EXPOSURE)){
            sensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 1);
            LOG(1) <<"  I enabled auto exposure";
          }
        }
      }
    }
  }

  //-- info on all streams
  for (rs2::stream_profile sp : profile.get_streams()){
    LOG(1) <<"stream '" <<sp.stream_name() <<"' idx:" <<sp.stream_index() <<" type:" <<sp.stream_type() <<" format:" <<sp.format() <<" fps" <<sp.fps() <<" id:" <<sp.unique_id();
    rs2::video_stream_profile vsp = sp.as<rs2::video_stream_profile>();
    if(vsp){
      rs2_intrinsics intrinsics = vsp.get_intrinsics();
      LOG(1) <<"  is video: w=" <<intrinsics.width <<" h=" <<intrinsics.height <<" px=" <<intrinsics.ppx << " py=" <<intrinsics.ppy <<" fx=" <<intrinsics.fx <<" fy=" <<intrinsics.fy <<" distorsion=" <<intrinsics.model <<floatA().referTo(intrinsics.coeffs, 5);
      if(sp.stream_type()==RS2_STREAM_DEPTH) depth_fxycxy = arr{intrinsics.fx, intrinsics.fy, intrinsics.ppx, intrinsics.ppy};
      if(sp.stream_type()==RS2_STREAM_COLOR) color_fxycxy = arr{intrinsics.fx, intrinsics.fy, intrinsics.ppx, intrinsics.ppy};
    }

    if(sp.stream_type()==RS2_STREAM_COLOR) rs2_get_video_stream_intrinsics(sp.get(), &s->depth_intrinsics, NULL);
  }

  //-- depth scale of the camera (
  s->depth_scale = rai::realsense::get_depth_scale(profile.get_device());
  LOG(1) <<"depth scale: " <<s->depth_scale;

  //-- align with depth or color?
  if(alignToDepth){
    s->align = std::make_shared<rs2::align>(RS2_STREAM_DEPTH);
    fxycxy = depth_fxycxy;
  }else{
    s->align = std::make_shared<rs2::align>(RS2_STREAM_COLOR);
    fxycxy = color_fxycxy;
  }
}

void RealSenseThread::close(){
  LOG(0) <<"STOPPING";
  s->pipe->stop();
  rai::wait(.1);
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
//  rs2::frameset processed = data;

  rs2::depth_frame rs_depth = processed.get_depth_frame();
  rs2::video_frame rs_color = processed.get_color_frame();

//  rs2::depth_frame rs_depth2 = s->temp_filter.process(rs_depth);
//  rs_depth = rs_depth2;
//  rs_depth = s->hole_filter.process(rs_depth2);

  {
    auto depthSet = depth.set();
    depthSet->resize(rs_depth.get_height(), rs_depth.get_width());
#if 0
    for(uint y=0;y<depthSet->d0;y++) for(uint x=0;x<depthSet->d1;x++){
      float d = rs_depth.get_distance(x,y);
      if(d>2.) d=2.;
      depthSet->p[y*depthSet->d1+x] = d;
    }
#else
    CHECK_EQ(rs_depth.get_bits_per_pixel(), 16, "");
    CHECK_EQ(rs_depth.get_stride_in_bytes(), rs_depth.get_width()*2, "");
    const uint16_t *data = reinterpret_cast<const uint16_t*>(rs_depth.get_data());
    for(uint i=0;i<depthSet->N;i++){
      depthSet->p[i] = float(data[i])*s->depth_scale;
    }
#endif
  }

  {
    auto colorSet = image.set();
    colorSet->resize(rs_color.get_height(), rs_color.get_width(), 3);
    CHECK(rs_color.get_bytes_per_pixel()==3,"");
    memmove(colorSet->p, rs_color.get_data(), colorSet->N);
  }
}

void rs2_get_motion_intrinsics(const rs2_stream_profile* mode, rs2_motion_device_intrinsic * intrinsics, rs2_error ** error);

#else //REALSENSE

RealSenseThread::RealSenseThread(const char *_name) : Thread("RealSenseThread") { NICO }
RealSenseThread::~RealSenseThread(){ NICO }
void RealSenseThread::getImageAndDepth(byteA& _image, floatA& _depth){ NICO }
void RealSenseThread::open(){ NICO }
void RealSenseThread::close(){ NICO }
void RealSenseThread::step(){ NICO }

#endif
