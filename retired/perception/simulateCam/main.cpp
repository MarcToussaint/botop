#include <Perception/opencv.h>

#include <Kin/kin.h>

#include <KOMO/komo.h>
#include <KOMO/komo-ext.h>

#include <Operate/path.h>
#include <Operate/robotio.h>
#include <Operate/simulationThread.h>
#include <Operate/op.h>

#include <Perception/depth2PointCloud.h>
#include <Perception/perceptViewer.h>
#include <Perception/perceptSyncer.h>

#include <Kin/cameraview.h>
#include <Kin/kinViewer.h>
#include <Gui/viewer.h>

#include <BackgroundSubtraction/cv_depth_backgroundSubstraction.h>

#include <Control/TaskControlThread.h>
#include <Franka/controlEmulator.h>


struct Pipeline{
  RobotIO &R;
  Pipeline(RobotIO& R) : R(R){}

  int step(){
    arr depth;
    R.getSensor(SEN_depth, depth);

    if(depth.nd!=2) return 0;
    floatA __depth;
    copy(__depth, depth);

    cv::Mat input = CV(__depth);
    cv::imshow("depth", input);
    cv::waitKey(1);

//      cv::Mat filtered;
//      cv::bilateralFilter(input, filtered, 10, .5, 10);
//      cv::imshow("filtered", filtered);
//      cv::waitKey(1);
    return 1;
  }
};

//===========================================================================

void cv_color_pipeline(byteA &img, const byteA& _input){
  if(_input.nd!=3) return;

  //-- grey
  cv::Mat input = CV(_input);
  cv::Mat grey;
  cv::cvtColor(input, grey, CV_BGR2GRAY);
  cv::imshow("grey", grey);
  cv::waitKey(1);

  //-- Canny
  float th=rai::getParameter<double>("cannyTh", 50.);
  cv::Mat edges;
  cv::Canny(grey, edges, th, 4.f*th, 3);
  cv::blur(edges, edges, cv::Size(5,5));
  cv::imshow("canny", edges);
  cv::waitKey(1);

  img = _input;
}

#include <opencv2/opencv.hpp>


//===========================================================================

void test_pickAndPlace(){
  //-- that's the "working" config,
  rai::KinematicWorld K;
  K.addFile("../../model/pandaStation.g");
  K.optimizeTree();
  Var<arr> cameraPose;
  cameraPose.set() = K["camera"]->X.getArr7d();
  Var<arr> cameraFxypxy;


  Var<CtrlMsg> ctrl;
  Var<CtrlMsg> state;

//  FrankaThread F(ctrl, state);
//  FrankaGripper G;
  ControlEmulator F(ctrl, state, K);
  GripperEmulator G;

  //-- robot simulation

  TaskControlThread C(K, ctrl, state);


//    SimulationThread R(K);
  Var<rai::KinematicWorld> RK;
  RK.set() = K;



  //-- camera simulation
  rai::Sim_CameraView CV(RK, .05, "camera");
  auto sen = CV.C.currentSensor;
  cameraFxypxy.set() = ARR(sen->cam.focalLength*sen->height, sen->cam.focalLength*sen->height, .5*sen->width, .5*sen->height);

  //-- pipeline: depth -> point cloud
//  Depth2PointCloud depth2pc(CV.depth, CV.C.currentSensor->cam.focalLength*300);
//  depth2pc.pose.set() = CV.C.currentSensor->cam.X;

  //displays: image & point cloud
//  ImageViewer CVview(CV.color);
//  PointCloudViewer pcview(depth2pc.pts, CV.color);

  //-- camera model! This would run also on the real robot; the simulations above not!!
  byteA segmentationRemap(K.frames.N);
  segmentationRemap.setZero();
  for(rai::Frame *f:K.frames){
    if(f->shape && f->shape->visual && f->name!="table"){
      segmentationRemap(f->ID)=0xff;
      cout <<f->ID <<' ' <<f->name <<endl;
    }
  }
  rai::Sim_CameraView robotMask(RK, .05, "camera", true, segmentationRemap);

  ImageViewer CVview(robotMask.color);



  //==============
  //
  // now the perception pipeline
  //

  ::Mutex cvLock;

//  Var<byteA> cameraImage(CV.color);
//  auto cvpipeline = loop([&cameraImage, &cvLock](){
//    cameraImage.waitForNextRevision();
//    byteA img = cameraImage.get();
//    byteA output;
//    auto lock = cvLock();
//    cv_color_pipeline(output, img);
//    return 0;
//  });

  CV_BackgroundSubstraction_Thread BS(CV.color, CV.depth, robotMask.color, cameraPose, cameraFxypxy, 1);

  cout <<"WAITING FOR BACKGROUND..." <<flush;
  CV.depth.waitForRevisionGreaterThan(20);
  cout <<"...DONE" <<endl;


  //==============
  //
  // Perception Viewer
  //

//  RK.addCallback([](Var_base* data,int){
//    Var_data<rai::KinematicWorld>* var = dynamic_cast<Var_data<rai::KinematicWorld>*>(data);
//    cout <<var->revision <<" #frames=" <<var->data.frames.N <<endl;
//  });

  PerceptViewer PV(BS.CVpercepts, RK);

  SyncFiltered PS(BS.CVpercepts, RK);
  KinViewer RKview(RK);


  //==============
  //
  // add the hook now
  //

  RK.set()->addFile("../../model/hook.g", "table", rai::Transformation({.4,.1,.05}, 0));


  //cheating! in real I could not copy the real world!
  RK.waitForNextRevision(1);
  K = RK.get();


//  rai::wait();
  FILE("z.ALL") <<RK.get()() <<endl;


  //==============
  //
  // what follows is a little script to make the robot do things
  //

  const char* endeff="endeffL";
  const char* object="stickTip";

  arr x0 = K.getFrameState();
  arr q0 = K.getJointState();
  StringA joints = K.getJointNames();
  K.watch(true);

  //compute a grasp
  chooseBoxGrasp(K, endeff, object);
  arr grasp = K.getJointState();
  K.watch(true);

  //compute a path from x0 to grasp
  K.setFrameState(x0);
  auto path = computePath(K, grasp, joints, endeff, .0, .8);

#if 0
  //open the gripper
  R.execGripper("pandaL", .1);
  R.waitForCompletion();

  //execute the path
  R.executeMotion(joints, path.first, path.second);
  R.waitForCompletion();

  //close gripper
  R.execGripper("pandaL", .0);
  R.waitForCompletion();

  //attach
  R.attach(endeff, object);

  arr q_now = R.getJointPositions(joints);
  K.setJointState(q_now);
  K.watch(true);

  path = computePath(K, q0, joints, endeff, .2, .8);
  R.executeMotion(joints, path.first, path.second);
  R.waitForCompletion();
#endif

  rai::wait();
}

//===========================================================================

int main(int argc,char** argv){
  rai::initCmdLine(argc,argv);

  test_pickAndPlace();

  return 0;
}


