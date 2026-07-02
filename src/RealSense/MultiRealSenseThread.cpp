#include "MultiRealSenseThread.h"

#ifdef RAI_REALSENSE

#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>

#include "utils.h"


namespace {

void setSettings(rs2::pipeline_profile& profile, bool autoExposure, double exposure, double white, double gain) {
  rs2::device dev = profile.get_device();
  for(rs2::sensor& sensor : dev.query_sensors()) {
    LOG(1) <<"sensor " <<sensor.get_info(RS2_CAMERA_INFO_NAME);
    for (int i = 0; i < static_cast<int>(RS2_OPTION_COUNT); i++)  {
      rs2_option option_type = static_cast<rs2_option>(i);
      if (sensor.supports(option_type)){
        LOG(1) <<"  option " <<option_type <<'=' <<sensor.get_option(option_type) <<"  (" <<sensor.get_option_description(option_type) <<")  [" <<sensor.get_option_range(option_type).min <<',' <<sensor.get_option_range(option_type).max<<']';
      }
    }
    if(!autoExposure){
      if(!strcmp(sensor.get_info(RS2_CAMERA_INFO_NAME),"RGB Camera")) {
        if(sensor.supports(RS2_OPTION_ENABLE_AUTO_EXPOSURE)) {
          sensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 0);
          sensor.set_option(RS2_OPTION_EXPOSURE, exposure);
          LOG(1) <<"  I disabled auto exposure";
        }
        if(sensor.supports(RS2_OPTION_ENABLE_AUTO_WHITE_BALANCE)) {
          sensor.set_option(RS2_OPTION_ENABLE_AUTO_WHITE_BALANCE, 0);
          sensor.set_option(RS2_OPTION_WHITE_BALANCE, white);
          LOG(1) <<"  I disabled auto white balance";
        }
        if(sensor.supports(RS2_OPTION_GAIN)) {
          sensor.set_option(RS2_OPTION_GAIN, gain);
          LOG(1) <<"  I set gain";
        }
      }
      if(!strcmp(sensor.get_info(RS2_CAMERA_INFO_NAME),"Stereo Module")) {
        if(sensor.supports(RS2_OPTION_ENABLE_AUTO_EXPOSURE)) {
          sensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 0);
          sensor.set_option(RS2_OPTION_EXPOSURE, 5000.);
          LOG(1) <<"  I disabled auto exposure";
        }
      }

    } else {
      if(!strcmp(sensor.get_info(RS2_CAMERA_INFO_NAME),"RGB Camera")) {
        if(sensor.supports(RS2_OPTION_ENABLE_AUTO_EXPOSURE)) {
          sensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 1);
          LOG(1) <<"  I enabled auto exposure";
        }
        if(sensor.supports(RS2_OPTION_ENABLE_AUTO_WHITE_BALANCE)){
          sensor.set_option(RS2_OPTION_ENABLE_AUTO_WHITE_BALANCE, 1);
          LOG(1) <<"  I enabled auto white balance";
        }
      }
      if(!strcmp(sensor.get_info(RS2_CAMERA_INFO_NAME),"Stereo Module")) {
        if(sensor.supports(RS2_OPTION_ENABLE_AUTO_EXPOSURE)) {
          sensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 1);
          LOG(1) <<"  I enabled auto exposure";
        }
      }
    }
  }
}

}

#endif


namespace rai {

#ifdef RAI_REALSENSE

RealSenseCamera::RealSenseCamera(String serialNumber, bool captureColor, bool captureDepth)
  : serialNumber(serialNumber),
    captureColor(captureColor),
    captureDepth(captureDepth)
{

  cfg = std::make_shared<rs2::config>();
  cfg->enable_device(serialNumber.p);

  double wRGB = rai::getParameter<double>(STRING("RealSense/" << "wRGB"), 1920);
  double hRGB = rai::getParameter<double>(STRING("RealSense/" << "hRGB"), 1080);
  double wDepth = rai::getParameter<double>(STRING("RealSense/" << "wDepth"), 424);
  double hDepth = rai::getParameter<double>(STRING("RealSense/" << "hDepth"), 240);
  if(captureColor) cfg->enable_stream(RS2_STREAM_COLOR, -1, wRGB, hRGB, rs2_format::RS2_FORMAT_RGB8, 30);
  if(captureDepth) cfg->enable_stream(RS2_STREAM_DEPTH, -1, wDepth, hDepth, rs2_format::RS2_FORMAT_Z16, 30);

  pipe = std::make_shared<rs2::pipeline>();
  pipe->start(*cfg);

  bool alignToDepth = rai::getParameter<bool>(STRING("RealSense/" << "alignToDepth"), false);
  bool autoExposure = rai::getParameter<bool>(STRING("RealSense/" << "autoExposure"), false);
  double exposure = rai::getParameter<double>(STRING("RealSense/" << "exposure"), 500);
  double white = rai::getParameter<double>(STRING("RealSense/" << "white"), 4000);
  double gain = rai::getParameter<double>(STRING("RealSense/" << "gain"), 50);

  LOG(0) <<"REALSENSE params: " <<alignToDepth <<' ' <<autoExposure <<' ' <<exposure <<' ' <<white <<' ' <<gain;
  rs2::pipeline_profile profile = pipe->get_active_profile();
  setSettings(profile, autoExposure, exposure, white, gain);

  //-- info on all streams
  for(rs2::stream_profile sp : profile.get_streams()) {
    LOG(1) <<"stream '" <<sp.stream_name() <<"' idx:" <<sp.stream_index() <<" type:" <<sp.stream_type() <<" format:" <<sp.format() <<" fps" <<sp.fps() <<" id:" <<sp.unique_id();
    rs2::video_stream_profile vsp = sp.as<rs2::video_stream_profile>();
    if(vsp){
      rs2_intrinsics intrinsics = vsp.get_intrinsics();
      LOG(1) <<"  is video: w=" <<intrinsics.width <<" h=" <<intrinsics.height <<" px=" <<intrinsics.ppx << " py=" <<intrinsics.ppy <<" fx=" <<intrinsics.fx <<" fy=" <<intrinsics.fy <<" distorsion=" <<intrinsics.model <<floatA().referTo(intrinsics.coeffs, 5);
      if(sp.stream_type()==RS2_STREAM_DEPTH) depth_fxycxy = arr{intrinsics.fx, intrinsics.fy, intrinsics.ppx, intrinsics.ppy};
      if(sp.stream_type()==RS2_STREAM_COLOR) color_fxycxy = arr{intrinsics.fx, intrinsics.fy, intrinsics.ppx, intrinsics.ppy};
    }
  }

  //-- depth scale of the camera (
  depth_scale = rai::realsense::get_depth_scale(profile.get_device());
  LOG(1) << "depth scale: " << depth_scale;

  //-- align with depth or color?
  if(captureColor && captureDepth) {
    if(alignToDepth){
      align = std::make_shared<rs2::align>(RS2_STREAM_DEPTH);
      fxycxy = depth_fxycxy;
    }else{
      align = std::make_shared<rs2::align>(RS2_STREAM_COLOR);
      fxycxy = color_fxycxy;
    }
  }
}



///////////////////////////////////////////////////////////////////////////////



MultiRealSenseThread::MultiRealSenseThread(const StringA& cameraNames, bool captureColor, bool captureDepth)
  : Thread("MultiRealSenseThread"),
    serialNumbers(cameraNames),
    color(cameraNames.N),
    depth(cameraNames.N),
    captureColor(captureColor),
    captureDepth(captureDepth)
{
  threadOpen(true);
  threadLoop();
}


MultiRealSenseThread::~MultiRealSenseThread(){
  LOG(0) << "DTOR";
  threadClose();
}

void MultiRealSenseThread::open() {
  rs2::log_to_console(RS2_LOG_SEVERITY_ERROR);

  for(const auto& serialNumber: serialNumbers) {
    cameras.append(new RealSenseCamera(serialNumber, captureColor, captureDepth));
  }
}

void MultiRealSenseThread::close() {
  LOG(0) << "REALSENSE STOPPING";
  for(auto cam : cameras) {
    cam->pipe->stop();
    rai::wait(0.2);
    delete cam;
  }
  cameras.clear();
}

void MultiRealSenseThread::step() {
  for(uint c=0;c<cameras.N;c++){
    auto& camera = cameras.elem(c);

    rs2::frameset data;
    try {
      data = camera->pipe->wait_for_frames(); // Wait for next set of frames from the camera
    } catch(rs2::error& err) {
      LOG(-1) <<"Can't get frames from RealSense: " << err.what();
      return;
    }

    rs2::frameset processed;
    if(captureColor & captureDepth) {
      processed = camera->align->process(data);
    } else {
      processed = data;
    }

    if(captureColor) {
      rs2::video_frame rs_color = processed.get_color_frame();
      auto colorSet = color.elem(c).set();
      colorSet->resize(rs_color.get_height(), rs_color.get_width(), 3);
      CHECK(rs_color.get_bytes_per_pixel()==3,"");
      memmove(colorSet->p, rs_color.get_data(), colorSet->N);
    }

    if(captureDepth) {
      rs2::depth_frame rs_depth = processed.get_depth_frame();

      /*rs2::hole_filling_filter hole_filter(2);
      rs_depth = hole_filter.process(rs_depth);*/

      auto depthSet = depth.elem(c).set();
      depthSet->resize(rs_depth.get_height(), rs_depth.get_width());
      CHECK_EQ(rs_depth.get_bits_per_pixel(), 16, "");
      CHECK_EQ(rs_depth.get_stride_in_bytes(), rs_depth.get_width()*2, "");
      const uint16_t *data = reinterpret_cast<const uint16_t*>(rs_depth.get_data());
      for(uint i=0;i<depthSet->N;i++){
        depthSet->p[i] = float(data[i]) * camera->depth_scale;
      }
    }
  }
}


#else //REALSENSE

RealSenseCamera::RealSenseCamera(std::string serialNumber, bool captureColor, bool captureDepth) { NICO }

MultiRealSenseThread::MultiRealSenseThread(const std::vector<std::string> cameraNames, bool captureColor, bool captureDepth)  : Thread("MultiRealSenseThread"){ NICO }
MultiRealSenseThread::~MultiRealSenseThread() { NICO }
uint MultiRealSenseThread::getNumberOfCameras() { NICO }
void MultiRealSenseThread::open(){ NICO }
void MultiRealSenseThread::close(){ NICO }
void MultiRealSenseThread::step(){ NICO }

#endif

}
