#include <Perception/opencv.h>
#include <opencv2/opencv.hpp>
#include <NewControl/TaskControlThread.h>
#include <LGPop/lgpop.h>
#include <FlatVision/registrationCalibration.h>

#include <Gui/viewer.h>

//===========================================================================

void online(){
  LGPop OP(LGPop::SimulationMode);

  //-- that's the "working" config,
  rai::KinematicWorld K(OP.rawModel);
  StringA joints = K.getJointNames();

  OP.runRobotControllers();
  OP.runTaskController(1);
  OP.runCamera(1);
  //OP.runPerception(1);

  /*
  rai::wait();
  cv::Mat rgb = CV(OP.cam_color.get()).clone();
  cv::imshow("rgb", rgb);
  cv::waitKey();
  rai::wait();
*/

  TaskControlInterface tci(OP.ctrl_config, OP.ctrl_tasks);
  tci.ctrl_tasks.waitForRevisionGreaterThan(20);

  rai::wait();

  //-- bring arms in calib pose
  {
    arr q = {.0, .0, 1., 1.,  -0.1,  0.1, -1.3,  -1.3,  -1.6,   1.6,   1.7,   1.7,   -0., -1.6,    0.05,  0.05};
    auto t = tci.addCtrlTaskSineMP("calib", FS_qItself, {}, 5.0, 300.0, 30.0);
    t->setTarget( q );
    wait(+t);
  }


#if 0

  for(uint k=0;k<3;k++){
    rai::wait();
    for(uint i=0;i<30;i++){
      OP.updateArmPoseCalibInModel();
      rai::wait(.1);
    }
  }
#endif

#if 0
  //save relevant images
  {
    ofstream fil("z.calibImages");
    OP.cam_color.get()->writeTagged(fil, "cam_color", true);
    OP.cam_depth.get()->writeTagged(fil, "cam_depth", true);
    OP.model_segments.get()->writeTagged(fil, "model_segments", true);
    OP.model_depth.get()->writeTagged(fil, "model_depth", true);
  }

  rai::wait();

  for(uint k=0;k<0;k++){
    byteA cam_color = OP.cam_color.get();
    byteA model_segments = OP.model_segments.get();
    floatA cam_depth = OP.cam_depth.get();
    floatA model_depth = OP.model_depth.get();

    uint cL=50,cR=10,cT=10,cB=10;
    cam_color = cam_color.sub(cT,-cB,cL,-cR,0,-1);
    model_segments = model_segments.sub(cT,-cB,cL,-cR);
    model_depth = model_depth.sub(cT,-cB,cL,-cR);
    cam_depth = cam_depth.sub(cT,-cB,cL,-cR);

    //left arm:
    arr calib = registrationCalibration(cam_color, cam_depth, model_segments, model_depth, 0x80);
    cout <<"CALIB:" <<calib <<endl;

    {
      arr fxycxy = OP.cam_fxycxy.get();
      auto armCalib = OP.armPoseCalib.set();
      armCalib->setZero();
      armCalib->operator()(0,0) = calib(1)/fxycxy(1);
      armCalib->operator()(0,1) = calib(0)/fxycxy(0);
//      armCalib->operator()(0,2) = .2* calib(2);
      armCalib->operator()(0,5) = calib(2);
    }

    OP.updateArmPoseCalibInModel();
  }
#endif

  rai::wait();

  //-- bring arms in home pose

  {
    auto t = tci.addCtrlTaskSineMP("calib", FS_qItself, {}, 5.0, 300.0, 30.0);
    t->setTarget( OP.q_home );
    rai::wait();
    //wait(+ t);
  }

  rai::wait();

  return;

  //cheating! in real I could not copy the real world!
  OP.ctrl_config.set()->addFile("../../model/stick.g", "table", rai::Transformation({.0,-.5,.05}, 0));

  rai::wait();
  return;

  //-- bring arms in home pose
  /*
  {
    auto t = addCtrlTask(OP.ctrl_tasks, OP.ctrl_config, "calib", FS_qItself, {}, 4.);
    t->setTarget( OP.q_home );
    wait(+ t);
  }
  */
  rai::wait();


#if 0
  //-- camera simulation
#if 1
  Var<rai::KinematicWorld> Cfake;
  Cfake.set() = TC.ctrl_config.get();
  //  Cfake.set()->getFrameByName("camera")->X.pos.z = 1.87;
  rai::Sim_CameraView doCamera(Cfake, .05, "camera");
#else
  RealSenseThread doCamera;
#endif
  doCamera.depth.waitForNextRevision(1);

  //-- some knowns:
  double tableHeight = C.get()->getFrameByName("table")->X.pos.z;
  tableHeight += 0.5*C.get()->getFrameByName("table")->shape->size(2);
  double cameraHeight = cameraPose.get()->elem(2);
  //-- measured depth:
  double d=sum(doCamera.depth.get()());
  d /= doCamera.depth.get()->N;
  cout <<"TABLE=" <<tableHeight
      <<"  CAMERA=" <<cameraHeight
     <<"  mean depth=" <<d
    <<"  error="<<cameraHeight - (tableHeight+d) <<endl;

  Depth2PointCloud doPcl(doCamera.depth, doCamera.getFxycxy()); //(0), doCamera.depth_fxycxy(1), doCamera.depth_fxycxy(2), doCamera.depth_fxycxy(3));
  PointCloudViewer pcview(doPcl.points, doCamera.color);
  doPcl.points.waitForNextRevision(10);

  //-- robot mask
  byteA frameIDmap = franka_getFrameMaskMap(C.get());
  rai::Sim_CameraView doMask(TC.ctrl_config, .05, "camera", true, frameIDmap);
  ImageViewer CVview(doMask.color);
  doMask.color.waitForNextRevision(10);


  /* Concept: use OpenCV affine registration to decide on xy-translation and rotation;
 * then use linear regression to decide on z-translation and xy-tilt */

  //linear regression

  rai::Frame *pcl_frame = C.set()->addObject("pcl", rai::ST_mesh);
  rai::Frame *camera_frame = C.set()->getFrameByName("camera");
  rai::Mesh& mesh = pcl_frame->shape->mesh();
  mesh.C = {1., 1., .5};
  //  rai::Frame *table = C.set()->addObject("table", rai::ST_ssBox, {1.,1.,.1,.01});

  for(;;){
    arr X = doPcl.points.get();
    byteA mask = doMask.color.get();
    X = X.sub(70,-1,50,-50,0,-1);
    mask = mask.sub(70,-1,50,-50);
    X.reshape(X.N/3,3);
    mask.reshape(mask.N);
    camera_frame->X.applyOnPointArray(X);
    mesh.V = X;

    arr y = X.col(2);
    y.reshape(y.N);
    for(uint i=0;i<X.d0;i++) X(i,2) = 1.;
    arr weights = ones(y.N);
    for(uint i=0;i<mask.N;i++) if(mask(i)) weights(i) = 0.;
    arr beta = ridgeRegression(X, y, 1e-10, NoArr, weights);
    //    cout <<"beta:" <<~beta <<endl;

    C.set()->setJointState(state.get()->q);
    int key = C.set()->watch();
    if(key=='q') break;

    //adapt camera height:
    double height = beta.elem(2);
    //    camera_frame->X.pos.z += .778 - height;

    cout
        <<"real camera height: " <<camera_frame->X.pos.z <<' ' <<beta.elem(2)
       <<"     camera x-tilt: " <<beta.elem(0)
      <<"     camera y-tilt: " <<beta.elem(1)
     <<endl;

  }
#endif
}

//===========================================================================

void offline(){
  ifstream fil("data.calibImages");
  byteA cam_color, model_segments;
  floatA cam_depth, model_depth;
  cam_color.readTagged(fil, "cam_color");
  cam_depth.readTagged(fil, "cam_depth");
  model_segments.readTagged(fil, "model_segments");
  model_depth.readTagged(fil, "model_depth");

  uint cL=50,cR=10,cT=10,cB=10;
  cam_color = cam_color.sub(cT,-cB,cL,-cR,0,-1);
  model_segments = model_segments.sub(cT,-cB,cL,-cR);
  model_depth = model_depth.sub(cT,-cB,cL,-cR);
  cam_depth = cam_depth.sub(cT,-cB,cL,-cR);

//  registrationCalibration(cam_color, cam_depth, model_segments, model_depth, 0x80);
//  registrationCalibration(cam_color, cam_depth, model_segments, model_depth, 0x8f);

  rai::wait();

}

//===========================================================================

int main(int argc, char * argv[]){

  online();
//  offline();

  return 0;
}
