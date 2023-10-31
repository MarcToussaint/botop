#include <Core/array.h>
#include <Gui/opengl.h>
#include <RealSense/RealSenseThread.h>
#include <Gui/viewer.h>
#include <Geo/depth2PointCloud.h>
#include <Core/thread.h>
#include <Kin/frame.h>
#include <Kin/viewer.h>

#include <BotOp/bot.h>

//===========================================================================

int testDirect(){

  RealSenseThread RS("realsense");
  OpenGL gl, gl2;

  {
    RS.depth.waitForNextRevision();
    RS.image.waitForNextRevision();
    Depth2PointCloud cvt2pcl(RS.depth, RS.fxycxy(0), RS.fxycxy(1), RS.fxycxy(2), RS.fxycxy(3));
    PointCloudViewer pcview(cvt2pcl.points, RS.image);

    cout <<"Camera fxycxy: " <<RS.fxycxy <<endl;

    {
      auto depthGet = RS.depth.get();
      gl.resize(depthGet->d1, depthGet->d0);
    }
    {
      auto colorGet = RS.image.get();
      gl2.resize(colorGet->d1, colorGet->d0);
    }

    CycleTimer tim;
    for(;;){
      RS.depth.waitForNextRevision();

      tim.cycleStart();
      int key=0;
      {
        floatA depth = RS.depth.get();
        for(float& d:depth) d *= 128.f;
        key = gl.watchImage(depth, false, 1.);
      }

      {
        auto colorGet = RS.image.get();
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

//===========================================================================

void testBotop(){
  //-- setup a configuration
  rai::Configuration C;
  C.addFile(rai::raiPath("../rai-robotModels/scenarios/pandaSingle.g"));
  C.view(false);

  //-- start a robot thread
  BotOp bot(C, rai::getParameter<bool>("BotOp/real", false));

//  bot.home(C);

  bot.hold(true, false);

  rai::Frame *pcl = C.addFrame("wristPcl", "cameraWrist");
  pcl->setPointCloud({}, {});

  OpenGL gl, gl2;
  byteA image;
  floatA depth;
  arr points;
  for(;;){
    bot.sync(C);
    if(bot.keypressed=='q'){ LOG(0) <<"HERE"; break; }

    bot.getImageDepthPcl(image, depth, points, "cameraWrist", false);
    {
      auto mux = C.gl().dataLock(RAI_HERE);
      pcl->setPointCloud(points, image);
    }

#if 0
    for(float& d:depth) d *= 128.f;
    int key=0;
    key |= gl.watchImage(depth, false, 1.);
    key |= gl2.watchImage(image, false, 1.);
    if(key=='q') break;
#endif
  }
}

//===========================================================================

int main(int argc, char * argv[]){
  rai::initCmdLine(argc, argv);

//  testDirect();
  testBotop();

}
