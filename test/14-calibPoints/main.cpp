#include <Kin/kin.h>
#include <Kin/frame.h>
#include <KOMO/komo.h>
#include <KOMO/pathTools.h>

#include <BotOp/bot.h>
#include <BotOp/motionHelpers.h>
#include <BotOp/tools.h>

#include <MarkerVision/cvTools.h>
#include <Gui/opengl.h>
#include <Geo/depth2PointCloud.h>

#include <Optim/NLP_Solver.h>

//===========================================================================

void setupConfig(rai::Configuration& C){
  C.addFile(rai::raiPath("../rai-robotModels/scenarios/pandaSingle.g"));

  C.addFrame("base_dots", "table") ->setRelativePosition({0.,0.,.05});

  if(rai::checkParameter<arr>("QcameraWrist")){
    rai::Transformation Q = rai::getParameter<arr>("QcameraWrist");
    Q.rot.normalize();
    rai::Frame* cam = C["cameraWrist"];
    cam->setRelativePose(Q);
  }

  arr dots = rai::getParameter<arr>("dots");
  dots.reshape(-1, 2);

  cout <<"-- adding " <<dots.d0 <<" dots" <<endl;
  for(uint i=0;i<dots.d0;i++){
    C.addFrame(STRING("dot" <<i), "base_dots")
        ->setRelativePosition((dots[i],arr{0.}))
        .setShape(rai::ST_cylinder, {.001, .02})
        .setColor({.2, .3, 1});
  }
}
void collectData(){
  //-- setup a configuration
  rai::Configuration C;
  setupConfig(C);

  arr dots = rai::getParameter<arr>("dots");
  uint views = rai::getParameter<uint>("views");
  dots.reshape(-1, 2);

  OpenGL imgGl;

  rai::Frame* cam = C["cameraWrist"];
  rai::Frame* mount = C["l_panda_joint7"];

  BotOp bot(C, rai::getParameter<bool>("real", false));

#if 0
  bot.gripperClose(rai::_left);
  bot.hold(true, false);
  while(bot.sync(C)) cout <<C["l_gripper"]->getPosition() <<endl;
  bot.gripperOpen(rai::_left);
  return;
#endif

  //initialize, also camera
  bot.home(C);
  byteA img;
  floatA depth;
  bot.getImageAndDepth(img, depth, cam->name);

  rai::Graph data;

  for(uint d=0;d<dots.d0;d++){
    for(uint k=0;k<views; k++){
      rai::Frame* dot = C[STRING("dot" <<d)];

      arr dotPos = arr{.2, .15, .2}%(rand(3)-.5);
      dotPos(2) += .3;
      double wristAngle = rnd.uni(-2.3, 2.3);

      {
        Move_IK move(bot, C, false);
        move().add_collision();
        move().addObjective({}, FS_positionRel, {dot->name, cam->name}, OT_eq, {1e0}, dotPos);
        move().addObjective({}, FS_positionRel, {"l_gripper", dot->name}, OT_ineq, {{1,3},{0.,0.,-1.}}, {0.,0.,.1});
        move().addObjective({}, FS_positionRel, {"l_gripper", dot->name}, OT_eq, {{2,3},{1.,0.,0.,0.,1.,0.}}, {});
        move().addObjective({}, FS_qItself, {mount->name}, OT_eq, {1e0}, {wristAngle});
        if(!move.go()){
          k--;
          LOG(0) <<"RRRRRRRRRRRRRRRRRRRRRRRRRRREPEATING";
          continue;
        }
      }

      for(uint t=0;t<3;t++){ bot.sync(C); rai::wait(.1); }

      rai::Graph& dat = data.addSubgraph(STRING(dot->name<<k));
      bot.getImageAndDepth(img, depth, cam->name);
      dat.add("dot", dot->ID);
      dat.add("q", bot.get_q());
      dat.add("mountPose", mount->getPose());
      dat.add("camPose", cam->getPose());
      dat.add("camFxycxy", bot.getCameraFxycxy(cam->name));
      dat.add("dotPosition", dot->getPosition());
      dat.add("img", img);
      dat.add("depth", depth);
      imgGl.watchImage(img, false);
    }
  }

  data.write(FILE("dat.g"), ",", "{}", 0);

  bot.home(C);
}

//===========================================================================

void selectHSV(){
  rai::Configuration C;
  C.addFile("station.g");
  rai::Frame* cam = C["cameraWrist"];
  BotOp bot(C, rai::getParameter<bool>("real"));

  arr hsvFilter = rai::getParameter<arr>("hsvFilter");

  CameraCalibrationHSVGui gui;
  OpenGL disp;
  byteA img;
  floatA depth;
  for(;;){
    bot.getImageAndDepth(img, depth, cam->name);
    hsvFilter = gui.getHSV();
    arr u = getHsvBlobImageCoords(img, depth, hsvFilter);
    disp.watchImage(img, false);
    bot.sync(C);
    if(bot.getKeyPressed()=='q') break;
  }

  cout <<"SELECTED:" <<hsvFilter <<endl;
}

//===========================================================================

void computeCalibration(){
  rai::Graph data("dat.g");

  // set hsv filter parameters
  arr hsvFilter = rai::getParameter<arr>("hsvFilter");
  int checks = rai::getParameter<int>("checks", 1);

  arr U(0,4), X(0,4);
  arr histogram;

  OpenGL disp;
  for(rai::Node *n:data){
    LOG(0) <<"===========" <<n->key;
    rai::Graph& dat = n->graph();

    //get stuff
    byteA img(dat.get<byteA>("img"));
    floatA depth(dat.get<floatA>("depth"));
    rai::Transformation mountPose(dat.get<arr>("mountPose"));
    rai::Transformation camPose(dat.get<arr>("camPose"));
    rai::Vector dotWorld(dat.get<arr>("dotPosition"));
    arr fxycxy = dat.get<arr>("camFxycxy");

    if(img.d0 != depth.d0) continue;

    // blob image coordinates
    arr u = getHsvBlobImageCoords(img, depth, hsvFilter, 0, histogram);
    if(!u.N){ rai::wait(); continue; }

    // blob homogeneous coordinates
    arr uHom = u;
    makeHomogeneousImageCoordinate(uHom);

    // blob camera coordinates,  assuming given intrinsics
    arr xCam = u;
    depthData2point(xCam, fxycxy); //transforms the point to camera xyz coordinates

    // dot coordinates in cam/mount frame
    arr x = (dotWorld / mountPose).getArr(); //camPose or mountPose - both work...

    cout <<"blob im image: " <<u
        <<"\nin homogen:    " <<uHom
       <<"\ndot-rel-cam:   " <<xCam
      <<"\nblob-from-hom: " <<x
     <<"\ncalib err:     " <<x-xCam <<endl;

    disp.watchImage(img, checks>1, 1.);

    //collect data
    U.append(uHom);
    x.append(1.); //works equally with or without appending 1...
    X.append(x);
  }
  X.reshape(U.d0, -1);

  //-- multiple iterations
  arr Pinv;
  for(uint k=0;k<5;k++){
    Pinv = ~X * U * inverse_SymPosDef(~U*U);
    for(uint i=0;i<X.d0;i++){
      double ei = sqrt(sumOfSqr(X[i] - Pinv*U[i]));
      cout <<"   error on data " <<i <<": " <<ei;
      if(ei>.007){ X[i]=0.; U[i]=0.; cout <<" -- removed"; }
      cout <<endl;
    }
    double err = sqrt(sumOfSqr(U*~Pinv - X)/double(X.d0));
    cout <<"total ERROR = " <<err <<endl;
    if(err<.01) break;
  }

  //-- output
  arr K, R, t;
  decomposeInvProjectionMatrix(K, R, t, Pinv);
  rai::Transformation T;
  T.rot.setMatrix(R);
  T.pos=t;
  arr fxycxy = {K(0,0), K(1,1), K(0,2), K(1,2)};
  cout <<"*** total Pinv:\n" <<Pinv <<endl;
  cout <<"*** R*K^-1:  \n" <<R*inverse(K) <<endl;
  cout <<"*** camera intrinsics K:\n" <<K <<endl;
  cout <<"*** camera fxycxy :\n" <<fxycxy <<endl;
  cout <<"*** camera world pose: " <<T <<endl;
  cout <<"*** camera world pos: " <<T.pos <<endl;
  cout <<"*** camera world rot: " <<T.rot <<endl;

  FILE("z.hsv") <<(~histogram).modRaw() <<endl;
  gnuplot("set title 'HSV histogram'; plot 'z.hsv' us 0:1 t 'H', '' us 0:2 t 'S', '' us 0:3 t 'V'");
  rai::wait();
}

//===========================================================================

void komoCalibrate(){
  arr hsvFilter = rai::getParameter<arr>("hsvFilter");
  int checks = rai::getParameter<int>("checks", 1);

  rai::Configuration C;
  setupConfig(C);

  rai::Graph data("dat.g");
  data.checkConsistency();

  //-- collect calib frames and initial states
  FrameL calibs;
  uintA calib_Id;
  arr calib_q;

  //make camera mount stable free joint
  rai::Frame* cam = C["cameraWrist"];
  calibs.append(cam);
  cout <<" -- making stable dof: " <<cam->name <<endl;
  cam->setJoint(rai::JT_free);
  cam->joint->isStable = true;
  calib_q.append(cam->joint->getDofState());
  calib_Id.append(cam->ID);

  //make all dots translational joints
  for(rai::Frame *dot:C.frames){
    if(!dot->name.startsWith("dot")) continue;
    calibs.append(dot);
    cout <<" -- making stable dof: " <<dot->name <<endl;
    dot->setJoint(rai::JT_transXY);
    dot->joint->isStable = true;
    calib_q.append(dot->joint->getDofState());
    calib_Id.append(dot->ID);
  }

  //-- report positions
//  for(rai::Frame *c:calibs){
//    cout <<c->name <<c->getRelativePosition() <<c->getPosition() <<endl;
//  }

  //add a 'viewPoint' marker to the camera
  rai::Frame* viewPoint = C.addFrame("viewPoint", cam->name);
  viewPoint->setShape(rai::ST_marker, {.1});

  //setup KOMO, one slice for each datapoint
  KOMO komo(C, data.N, 1, 0, false);
  komo.setupPathConfig();
  komo.addQuaternionNorms();

  //add objectives for each data point
  arr histogram;
  for(uint t=0;t<komo.T;t++){
    rai::Node *n = data(t);

    rai::Graph& dat = n->graph();

    //set the configuration
    arr q = dat.get<arr>("q");
    if(!t){
      komo.setConfiguration_qOrg(t, (q, calib_q));
    }else{
      komo.setConfiguration_qOrg(t, q);
    }

    //set the viewPoint
    byteA img(dat.get<byteA>("img"));
    floatA depth(dat.get<floatA>("depth"));
    arr fxycxy = dat.get<arr>("camFxycxy");
    arr u = getHsvBlobImageCoords(img, depth, hsvFilter, checks, histogram);
    if(!u.N) continue;
    arr xCam = u;
    depthData2point(xCam, fxycxy); //transforms the point to camera xyz coordinates
    rai::Frame *viewP = komo.timeSlices(t, viewPoint->ID);
    viewP->setRelativePosition(xCam);

    //    cout <<t <<' ' <<q <<viewP->name <<viewP->getPosition() <<endl;

    n->key.resize(4, true);

    //add calibration objective
    komo.addObjective({double(t+1)}, FS_positionDiff, {"viewPoint", n->key}, OT_sos, {1e2});

    //if(checks>1) komo.view(true, STRING("init t=" <<t));
  }

  //select only calib dofs:
  {
    DofL dofs;
    for(uint id:calib_Id) dofs.append(komo.timeSlices(0, id)->joint);
    komo.pathConfig.selectJoints(dofs);
//    dofs = komo.pathConfig.getDofs(komo.pathConfig.frames, true, false, false);
//    for(auto* d: dofs) cout <<*d <<endl;
//    komo.pathConfig.report();
  }

  komo.run_prepare(0.);
  cout <<"== initial parameters (camera, dots): " <<komo.x <<endl;
  komo.view(true, "before optim");
//  komo.pathConfig.animate();

  NLP_Solver sol;
  sol.setProblem(komo.nlp());
  sol.opt.set_stopTolerance(1e-6);
  sol.opt.set_verbose(0);
  auto ret = sol.solve();
  cout <<komo.report(false) <<endl; //reports match per feature..
  cout <<"-- result: " <<*ret <<endl;
  cout <<"== optimized parameters (camera, dots): " <<ret->x <<endl;

  //-- report positions
//  for(uint t=0;t<komo.T;t++){
//    rai::Frame *f = komo.timeSlices(t, viewPoint->ID);
//    cout <<t <<' ' <<f->name <<f->getPosition() <<endl;
//    for(rai::Frame *c:calibs){
//      rai::Frame *f = komo.timeSlices(t, c->ID);
//      cout <<t <<' ' <<f->name <<f->getPosition() <<endl;
//    }
//  }

  komo.view(true, "after optim");

  FILE("z.hsv") <<(~histogram).modRaw() <<endl;
  gnuplot("set title 'HSV histogram'; plot 'z.hsv' us 0:1 t 'H', '' us 0:2 t 'S', '' us 0:3 t 'V'");
  rai::wait();
}

//===========================================================================

void demoCalibration(){
  //-- setup a configuration
  rai::Configuration C;
  setupConfig(C);

  rai::Frame* target = C.addFrame("target", "table");
  target->setShape(rai::ST_marker, {.1});
  target->setColor({1.,.5,0.});

  C.view(true);

  rai::Frame* cam = C["cameraWrist"];
  arr qCam = rai::getParameter<arr>("QcameraWrist");
  cam->setRelativePose(qCam);

  arr hsvFilter = rai::getParameter<arr>("hsvFilter");
  arr Pinv = rai::getParameter<arr>("Pinv");
  int checks = rai::getParameter<int>("checks", 1);

  BotOp bot(C, rai::getParameter<bool>("real", false));
  bot.home(C);

  for(uint i=0;;i++){
    rai::Frame *dot = C.getFrame(STRING("dot"<<i), false);
    if(!dot) break;
    //pre motion
    //bot.gripperMove(rai::_left);
    {
      Move_IK move(bot, C, checks);
      move().addObjective({}, FS_positionRel, {dot->name, cam->name}, OT_eq, {1e0}, {0.,0.,.25});
      if(!move.go()) return;
    }

    //fine motion
    bot.gripperMove(rai::_left, 0);
#if 1
    for(uint t=0;t<5;t++) bot.sync(C);
    if(!sense_HsvBlob(bot, C, cam->name, "target", hsvFilter, Pinv, checks)) return;
//    target->setRelativePosition(dot->getRelativePosition());
#endif
    {
      Move_IK move(bot, C, checks);
      move().addObjective({}, FS_vectorZ, {"l_gripper"}, OT_eq, {1e0}, {0.,0.,1.});
      move().addObjective({}, FS_positionDiff, {"l_gripper", "target" /*dot->name*/}, OT_eq, {1e0}, {0.,0., .01});
      if(!move.go()) return;
    }

    bot.wait(C, true, false);
    if(bot.keypressed=='q') break;
  }

  bot.gripperMove(rai::_left);
  bot.home(C);
}

//===========================================================================

void checkTip(){
  //-- setup a configuration
  rai::Configuration C;
  C.addFile("station.g");

  BotOp bot(C, rai::getParameter<bool>("real", false));

  rai::Frame* tip = C["l_gripper"];

  //pre motion
  bot.gripperClose(rai::_left);
  bot.hold(true, false);
  for(;;){
    bot.sync(C, .1);
    cout <<"tip: " <<tip->getPosition() <<endl;
    if(bot.getKeyPressed()) break;
  }

  if(!bot.wait(C)) return;
  bot.gripperMove(rai::_left);
  bot.home(C);

}

//===========================================================================

int main(int argc, char * argv[]){
  rai::initCmdLine(argc, argv);

//  collectData();

  computeCalibration();
//  komoCalibrate();

//  selectHSV();

//  demoCalibration();

//  checkTip();

  LOG(0) <<" === bye bye ===\n";

  return 0;
}
