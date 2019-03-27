#include <Core/array.h>
#include <Gui/opengl.h>
#include <RealSense/RealSenseThread.h>
#include <Gui/viewer.h>
#include <Perception/depth2PointCloud.h>
#include <Core/thread.h>

int main(int argc, char * argv[]){

  RealSenseThread RS({}, {});
  OpenGL gl, gl2;

  {
    RS.depth.waitForNextRevision();
    RS.color.waitForNextRevision();
//    Depth2PointCloud cvt2pcl(RS.depth, RS.fxypxy(0), RS.fxypxy(1), RS.fxypxy(2), RS.fxypxy(3));
//    PointCloudViewer pcview(cvt2pcl.points, RS.color);

    {
      auto depthGet = RS.depth.get();
      gl.resize(depthGet->d1, depthGet->d0);
    }
    {
      auto colorGet = RS.color.get();
      gl2.resize(colorGet->d1, colorGet->d0);
    }

    CycleTimer tim;
    for(;;){
      RS.depth.waitForNextRevision();

      tim.cycleStart();
      int key=0;
      {
        auto depthGet = RS.depth.get();
        key = gl.watchImage(128.f*depthGet(), false, 1.);
      }

      {
        auto colorGet = RS.color.get();
        key = gl2.watchImage(colorGet(), false, 1.);
      }
      tim.cycleDone();

      if(key=='q') break;
    }
    cout <<"DISPLAY timer:   " <<tim.report() <<endl;
    cout <<"RealSense timer: " <<RS.timer.report() <<endl;
  }


  LOG(0) <<"bye bye";

  return EXIT_SUCCESS;
}
