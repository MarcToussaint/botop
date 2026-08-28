#include <Perception/opencv.h> //always include this first! OpenCV headers define stupid macros
#include <Perception/opencvCamera.h>
#include <Perception/aruco.h>
#include <Perception/MultiViewProblems.h>

#include <Gui/opengl.h>
#include <Kin/kin.h>
#include <Kin/frame.h>
#include <Kin/viewer.h>
#include <Kin/cameraview.h>
#include <Optim/NLP_Solver.h>

#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/objdetect/aruco_detector.hpp>

#include <BotOp/bot.h>
#include <BotOp/simulation.h>
#include <Perception/aruco.h>
#include <Perception/KomoArucoTracker.h>

void testRender(){
    rai::Configuration C;
    C.addFile("/home/mtoussai/git/tests/calibration/station.g");
    rai::Frame *obj = C.getFrame("obj");
    // obj->unLink();

    arr q0 = C.getJointState();
    C.get_viewer()->opt.renderShadow = false;
    C.get_viewer()->opt.flatColors = true;
    C.get_viewer()->opt.renderText = false;
    C.get_viewer()->renderUntil = rai::_solid;
    // RAI_PARAM("Render/", bool, flatColors, false)
    // RAI_PARAM("Render/", bool, renderShadow, true)
    // RAI_PARAM("Render/", bool, renderPolygonLines, false)

    CycleTimer tim;
    for(uint t=0;t<100;t++){
        tim.tic(0);
        C.setJointState(q0 + .1*randn(q0.N));
        C.view(false);
        tim.tic(1);
        rai::wait(.01);
    }
    cout <<tim.report() <<endl;
}

void testKomoTracker(){
    rai::Configuration C;
    C.addFile("/home/mtoussai/git/tests/calibration/station_reduced.g");
    // rai::Frame *obj = C.getFrame("obj");
    // obj->unLink();

    C.get_viewer()->opt.renderShadow = false;
    C.get_viewer()->opt.flatColors = true;
    C.get_viewer()->opt.renderText = false;
    C.get_viewer()->renderUntil = rai::_solid;

    rai::KomoArucoTracker K(C, "obj");
    // cout <<K.CS.report() <<endl;

    // rai::CameraView V(CS.C);
    // byteA rgb;
    // floatA depth;

    rai::setParameter("botsim/verbose", 0);
    BotOp bot(C, false);
    // cout <<bot.state.get()->q <<endl;

    bot.launch_Basler(3);
    bot.launch_arucos();
    bot.launch_arucoObjTracker(C, "obj");

    // cout <<bot.state.get()->q <<endl;

    // rai::ArucoSystemThread Ar(12, ar_outputs, Fxycxy, Pose);

    CycleTimer tim;
    OpenGL gl;
    auto finder = rai::ArucoFinder();

    for(uint t=0;;t++){
        tim.tic(0);
        int key = 0;
        // if(!(t%1)) key = C.view(false);
        key = bot.sync(C, .0);
        if(key=='q') break;

        // if(!(t%1)) key = gl.watchImage(bot.getImage("camera_0"), false, .5);
        if(key=='q') break;

        tim.tic(1);

#if 0
        K.reset();

#if 0
        byteAA imgs(3);
        for(uint c=0;c<imgs.N;c++) imgs(c) = bot.getImage(STRING("camera_" <<c));

        tim.tic(2);

        for(uint c=0;c<imgs.N;c++){
            finder.find(imgs(c));
            K.addMultiPointView(finder.ids, finder.pts, c);
        }
#else
        tim.tic(2);
        rai::Array<rai::ArucoOutput> ao(bot.aruco_threads.N);
        rai::wait(.05);
        // bot.aruco_threads(0)->output.waitForNextRevision();
        for(uint i=0;i<ao.N;i++) ao(i) = bot.aruco_threads(i)->output.get();
        for(auto& o:ao) K.addMultiPointView(o.ids, o.pts, o.cam_id);
#endif
        tim.tic(3);

        K.solve(0);
        // cout <<*K.ret <<K.ret->x <<endl;
        arr q_obj = K.ret->x;
        q_obj = K.filter.q;
        // K.komo->view(false, "komo");
        obj->joint->setDofs(q_obj);
        bot.state.set()->q({obj->joint->qIndex, obj->joint->qIndex+7}) = q_obj;
#else
        arr q_obj = bot.state.get()->q;
#endif
        // cout <<q_obj <<' ' <<K.filter.err_filtered <<endl;
        tim.tic(4);

        //somewhat awkward?

    }

    cout <<"TIMING: " <<tim.report() <<endl;

}

void testKomoTracker2(){
    rai::Configuration C;
    C.addFile("/home/mtoussai/git/tests/calibration/station_reduced.g");

    auto bot = BotOp(C, false);

    bot.launch_Basler(3);
    bot.launch_arucos();
    bot.launch_arucoObjTracker(C, "obj");

    for(uint t=0;;t++){
        int key = 0;
        key = bot.sync(C, .0);
        if(key=='q') break;
    }
}

void test(){
  rai::Configuration C;
  C.addFile("table.yml");

  // rai::ConfigurationViewer& V = *C.get_viewer();
  // V.setWindow("bla", 800, 600);

  rai::CameraView V(C);
  byteA rgb;
  floatA depth;

  FrameL arucos(12);
  for(uint i=0;i<arucos.N;i++) arucos(i) = C.getFrame(STRING("ar" <<i));

  FrameL tests(12);
  for(uint i=0;i<arucos.N;i++){
    tests(i) = C.addFrame(STRING("test_"<<i));
    tests(i)-> setShape(rai::ST_marker, {.2});
  }

  FrameL cams(5);
  for(uint i=0;i<cams.N;i++) cams(i) = C.getFrame(STRING("cam" <<i));

  for(uint k=0;k<20;k++){
    C.setRandom();
    C.view(true);
    // auto rgb = V.getRgb();

    MultiViewSolver D(12, 4);

    V.updateConfiguration(C);
    for(uint k=0;k<cams.N;k++){
      V.selectSensor(cams(k));
      V.computeImageAndDepth(rgb, depth);
      D.setCamera(k, V.getFxycxy(), V.currentCamera->cam.X);

      auto finder = rai::ArucoFinder();
      finder.verbose=2;
      finder.find(rgb);
      C.get_viewer()->setQuad(k, finder.rgb_annotated, 0., k*.25, .25);

      for(uint j=0;j<finder.ids.N;j++){
        D.addDataPoint(finder.ids(j), k, finder.pts[j]);
      }
    }

    D.subSelectObservedPoints();
    D.solveColinearityForPoints();
    cout <<D.X <<endl;
    for(uint i=0;i<D.J;i++) tests(i)->setPosition(D.X[i]);

    int key = C.view(true);
    if(key=='q') break;
  }
}

void testBotop(){
  rai::Configuration C;
  C.addFile("table.yml");

  rai::setParameter("botsim/verbose", 0);
  BotOp bot(C, false);


  FrameL f_cams;
  for(uint k=0;k<1;k++)  f_cams.append( C.getFrame(STRING("cam"<<k)) );
  // bot.launch_MultiRealSense(f_cams, true, false);
  bot.launch_arucos();

  // rai::Array<Var<PointViewA>*> ar_outputs;
  // arrA Fxycxy;
  // rai::Array<rai::Transformation> Pose;
  // for(uint k=0;k<4;k++){
  //   ar_outputs.append(&bot.aruco_threads(k)->output);
  //   Fxycxy.append(bot.cameras(k)->getFxycxy());
  //   Pose.append(bot.cameras(k)->getPose());
  // }
  // rai::ArucoSystemThread Ar(12, ar_outputs, Fxycxy, Pose);

  for(uint t=0;;t++){
    int key = bot.sync(C);

    // Ar.pull(C);
    PointViewData data;
    NIY; //for(auto& ar:bot.aruco_threads) data.append(ar->output.get());
    for(auto& pv:data){
      cout <<"aruco marker id " <<pv.j <<", camera id " <<pv.k <<", point " <<pv.p <<endl;
    }


    if(!(t%20)){
      arr q = randn(4); q /= length(q);
      arr p = .1*randn(3); p(1) += .2; p(2) += 1.;
      bot.cheat_setFramePose("obj", (p,q));
    }

    if(key=='q') break;
  }
}

int main(int argc,char **argv){
  rai::initCmdLine(argc,argv);

  // testKomoTracker();
  testKomoTracker2();

  // test();

  // testBotop();
  
  return 0;
}
