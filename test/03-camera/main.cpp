#include <Core/array.h>
#include <Gui/opengl.h>
#include <RealSense/RealSenseThread.h>
#include <Gui/viewer.h>
#include <Perception/depth2PointCloud.h>

int main(int argc, char * argv[]){

  RealSenseThread RS({}, {});
  OpenGL gl, gl2;

  {
    RS.depth.waitForNextRevision();
    Depth2PointCloud cvt2pcl(RS.depth, RS.depth_fxypxy(0), RS.depth_fxypxy(1), RS.depth_fxypxy(2), RS.depth_fxypxy(3));
    PointCloudViewer pcview(cvt2pcl.points, RS.color);

  for(;;){

    int key=0;
    {
//      RS.depth.waitForNextRevision();
      rai::wait(.1);
      auto depthGet = RS.depth.get();
      if(depthGet->nd==2){
        gl.resize(depthGet->d1, depthGet->d0);
        key = gl.watchImage(128.f*depthGet(), false, 1.);
      }
    }

    {
      auto colorGet = RS.color.get();
      if(colorGet->nd==3){
        gl2.resize(colorGet->d1, colorGet->d0);
        key = gl2.watchImage(colorGet(), false, 1.);
      }
    }

    if(key=='q') break;
  }

  }

  LOG(0) <<"bye bye";

  return EXIT_SUCCESS;
}
