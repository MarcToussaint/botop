#include <Core/array.h>
#include <Gui/opengl.h>

#include <librealsense2/rs.hpp>

int main(int argc, char * argv[]){
  rs2::log_to_console(RS2_LOG_SEVERITY_ERROR);
  rs2::pipeline pipe;
  pipe.start();

  OpenGL glDepth, glColor;

  arr _depth;
  byteA _color;
  int key;

  for(;;){
    rs2::frameset data = pipe.wait_for_frames(); // Wait for next set of frames from the camera

    rs2::depth_frame depth = data.get_depth_frame(); // Find and colorize the depth data
    rs2::video_frame color = data.get_color_frame(); // Find the color data

//#if 0
    _depth.resize(depth.get_height(), depth.get_width());
    for(uint y=0;y<_depth.d0;y++) for(uint x=0;x<_depth.d1;x++){
      double d = depth.get_distance(x,y);
      if(d>2.) d=2.;
      _depth(y,x) = d;
    }

    glDepth.resize(depth.get_width(), depth.get_height());
    key = glDepth.displayGrey(_depth, false, 1.);
    if(key=='q') break;
//#else
    _color.resize(color.get_height(), color.get_width(), 3);
    CHECK(color.get_bytes_per_pixel()==3,"");
    memmove(_color.p, color.get_data(), _color.N);

    glColor.resize(color.get_width(), color.get_height());
    key = glColor.watchImage(_color, false, 1.);
    if(key=='q') break;
//#endif
  }

  return EXIT_SUCCESS;
}
