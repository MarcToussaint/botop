#include <Kin/kin.h>
#include <Kin/frame.h>
#include <KOMO/komo.h>
#include <KOMO/pathTools.h>

#include <BotOp/bot.h>
#include <BotOp/motionHelpers.h>

#include <Optim/NLP_Solver.h>

#include <Gui/opengl.h>

#include <Geo/depth2PointCloud.h>

#include <MarkerVision/cvTools.h>

//===========================================================================

void decomposeInvProjectionMatrix(arr& K, arr& R, arr& t, const arr& P){
  arr KR = P.sub(0,2,0,2);
  t = ~P.col(3);
  KR = inverse(KR);
  lapack_RQ(K, R, KR);
}

//===========================================================================

void collectData(){
  //-- setup a configuration
  rai::Configuration C;
  C.addFile("station.g");

  C["bellybutton"]->name = "dot0";
  rai::Frame* cam = C["cameraWrist"];
  rai::Frame* mount = C["l_panda_joint7"];

  BotOp bot(C, rai::getParameter<bool>("real"));

  //initialize, also camera
  bot.home(C);
  byteA img;
  floatA depth;
  bot.getImageAndDepth(img, depth, cam->name);


  rai::Graph data;

  uint nDots=1;
  for(uint d=0;d<nDots;d++){
    for(uint k=0;k<50; k++){
      rai::Frame* dot = C[STRING("dot" <<d)];

      KOMO komo;
      komo.setConfig(C, false);
      komo.setTiming(1., 1, 1., 0);
      komo.addControlObjective({}, 0, 1e-1);

      arr dotPos = .15*(rand(3)-.5);
      dotPos(2) -= .35;
      komo.addObjective({}, FS_positionRel, {dot->name, cam->name}, OT_eq, {1e0}, dotPos);
      komo.addObjective({}, FS_positionRel, {"l_gripper", dot->name}, OT_ineq, {{1,3},{0.,0.,-1.}}, {0.,0.,.1});
      komo.addObjective({}, FS_positionRel, {"l_gripper", dot->name}, OT_eq, {{2,3},{1.,0.,0.,0.,1.,0.}}, {});

      double wristAngle = rnd.uni(-2.3, 2.3);
      komo.addObjective({}, FS_qItself, {mount->name}, OT_eq, {1e0}, {wristAngle});

      auto ret = NLP_Solver().setProblem(komo.nlp()).solve();
      cout <<dot->name <<' ' <<k <<' ' <<dotPos <<' ' <<wristAngle <<' ' <<*ret <<endl;
      if(!ret->feasible){
        k--;
        LOG(0) <<"RRRRRRRRRRRRRRRRRRRRRRRRRRREPEATING";
        continue;
      }

      bot.moveTo(komo.getPath_qOrg()[-1], 2.);
      for(;bot.getTimeToEnd()>0.;) bot.sync(C);

      bot.hold(false, true);
      for(uint t=0;t<2;t++) bot.sync(C);

      rai::Graph& dat = data.addSubgraph(STRING(dot->name<<k));
      bot.getImageAndDepth(img, depth, cam->name);
      dat.add("mountPose", mount->getPose());
      dat.add("camPose", cam->getPose());
      dat.add("camFxypxy", bot.getCameraFxypxy(cam->name));
      dat.add("dotPosition", dot->getPosition());
      dat.add("img", img);
      dat.add("depth", depth);
    }
  }

  data.write(FILE("dat.g"), "\n", 0, -1, false, true);

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

  arr U(0,4), X(0,3);

  OpenGL disp;
  for(rai::Node *n:data){
    LOG(0) <<"===========" <<n->key;
    rai::Graph& dat = n->graph();

    // grap copies of rgb and depth

    byteA img(dat.get<byteA>("img"));
    floatA depth(dat.get<floatA>("depth"));
    rai::Transformation mountPose(dat.get<arr>("mountPose"));
    rai::Transformation camPose(dat.get<arr>("camPose"));
    rai::Vector dotWorld(dat.get<arr>("dotPosition"));
    arr Fxypxy = dat.get<arr>("camFxypxy");

    if(img.d0 != depth.d0) continue;

    // blur
    arr u = getHsvBlobImageCoords(img, depth, hsvFilter);
    if(!u.N){ rai::wait(); continue; }


    // camera coordinates
    arr xCam=u;
    depthData2point(xCam, Fxypxy); //transforms the point to camera xyz coordinates

    arr dotCam = (dotWorld / camPose).getArr();
    //camPose.applyOnPoint(pos);

    arr x = (dotWorld / mountPose).getArr();

    cout <<"blob position: " <<xCam <<' ' <<dotCam <<' ' <<xCam-dotCam <<' ' <<Fxypxy <<endl;

    disp.watchImage(img, true, 1.);

    //collect data
    makeHomogeneousImageCoordinate(u, img.d0);
    U.append(u);
    X.append(x);
  }

  //-- multiple iterations
  arr Pinv, K, R, t;
  for(uint k=0;k<1;k++){
    Pinv = ~X * U * inverse_SymPosDef(~U*U);
    decomposeInvProjectionMatrix(K, R, t, Pinv);
    for(uint i=0;i<X.d0;i++){
      double ei = sqrt(sumOfSqr(X[i] - Pinv*U[i]));
      cout <<"   error on data " <<i <<": " <<ei;
      if(ei>.05){ X[i]=0.; U[i]=0.; cout <<" -- removed"; }
      cout <<endl;
    }
    double err = sqrt(sumOfSqr(U*~Pinv - X)/double(X.d0));
    cout <<"total ERROR = " <<err <<endl;
    if(err<.01) break;
  }

  //-- output
  rai::Transformation T;
  T.rot.setMatrix(~R);
  T.pos=t;
  arr Fxypxy = {K(0,0), K(1,1), K(0,2), K(1,2)};
  cout <<"*** total Pinv:\n" <<Pinv <<endl;
  cout <<"*** camera intrinsics K:\n" <<K <<endl;
  cout <<"*** camera Fxypxy :\n" <<Fxypxy <<endl;
  cout <<"*** camera world pose: " <<T <<endl;
  cout <<"*** camera world pos: " <<T.pos <<endl;
  cout <<"*** camera world rot: " <<T.rot <<endl;
}

//===========================================================================

void demoCalibration(){
  //-- setup a configuration
  rai::Configuration C;
  C.addFile("station.g");
  rai::Frame* target = C.addFrame("target");
  target->setShape(rai::ST_marker, {.1});
  target->setColor({1.,.5,0.});

  C["bellybutton"]->name = "dot0";
  rai::Frame* cam = C["cameraWrist"];
  rai::Frame* mount = C["l_panda_joint7"];
  arr hsvFilter = rai::getParameter<arr>("hsvFilter");
  arr Pinv = rai::getParameter<arr>("Pinv");

  BotOp bot(C, rai::getParameter<bool>("real"));

  bot.gripperOpen(rai::_left);

  //initialize camera
  OpenGL disp;
  byteA img;
  floatA depth;
  for(uint t=0;t<30;t++){
    bot.getImageAndDepth(img, depth, cam->name);
    disp.watchImage(img, false);
    bot.sync(C);
  }

  bot.getImageAndDepth(img, depth, cam->name);
//  disp.watchImage(img, false);
  arr u = getHsvBlobImageCoords(img, depth, hsvFilter);
  cout <<"dot in image coords: " <<u <<endl;

  if(!u.N) return;

#if 1
  makeHomogeneousImageCoordinate(u, img.d0);
  arr x = Pinv*u;
  cout <<"dot in wrist coords: " <<x <<endl;
  mount->get_X().applyOnPoint(x);
#else
  depthData2point(u, bot.getCameraFxypxy(cam->name));
  arr x = u;
  cout <<"dot in wrist coords: " <<x <<endl;
  cam->get_X().applyOnPoint(x);
#endif
  cout <<"dot in world coords: " <<x <<endl;
  target->setPosition(x);

  arr qT;
  {
    KOMO komo;
    komo.setConfig(C, false);
    komo.setTiming(1., 1, 1., 0);
    komo.addControlObjective({}, 0, 1e-1);
    komo.addObjective({}, FS_positionRel, {"target", cam->name}, OT_eq, {1e0}, {0.,0.,-.2});
    auto ret = NLP_Solver().setProblem(komo.nlp()).solve();
    cout <<*ret <<endl;
    komo.view();
    if(!ret->feasible) return;
    qT = komo.getPath_qOrg()[-1];
  }

  for(;;){
    bot.sync(C);
    if(bot.getKeyPressed()=='q') return;
    if(bot.getKeyPressed()==' ') break;
  }

  bot.moveTo(qT, 5.);
  for(;bot.getTimeToEnd()>0.;) bot.sync(C);

  bot.hold(false, true);
  for(uint t=0;t<2;t++) bot.sync(C);

  for(;;){
    bot.sync(C);
    if(bot.getKeyPressed()=='q') return;
    if(bot.getKeyPressed()==' ') break;
  }

  bot.gripperClose(rai::_left);

  bot.getImageAndDepth(img, depth, cam->name);
//  disp.watchImage(img, false);
  u = getHsvBlobImageCoords(img, depth, hsvFilter);
  cout <<"dot in image coords: " <<u <<endl;

  if(!u.N) return;

#if 1
  makeHomogeneousImageCoordinate(u, img.d0);
  x = Pinv*u;
  cout <<"dot in wrist coords: " <<x <<endl;
  mount->get_X().applyOnPoint(x);
#else
  depthData2point(u, bot.getCameraFxypxy(cam->name));
  x = u;
  cout <<"dot in wrist coords: " <<x <<endl;
  cam->get_X().applyOnPoint(x);
#endif
  cout <<"dot in world coords: " <<x <<endl;
  target->setPosition(x);

  {
    KOMO komo;
    komo.setConfig(C, false);
    komo.setTiming(1., 1, 1., 0);
    komo.addControlObjective({}, 0, 1e-1);
    komo.addObjective({}, FS_vectorZ, {"l_gripper"}, OT_eq, {1e0}, {0.,0.,1.});
    komo.addObjective({}, FS_positionRel, {"target", "l_gripper"}, OT_eq, {1e0}, {0.,0.,-.01});
    auto ret = NLP_Solver().setProblem(komo.nlp()).solve();
    cout <<*ret <<endl;
    komo.view();
    if(!ret->feasible) return;
    qT = komo.getPath_qOrg()[-1];
  }

  for(;;){
    bot.sync(C);
    if(bot.getKeyPressed()=='q') return;
    if(bot.getKeyPressed()==' ') break;
  }

  bot.moveTo(qT, 5.);
  for(;bot.getTimeToEnd()>0.;) bot.sync(C);
}

//===========================================================================

int main(int argc, char * argv[]){
  rai::initCmdLine(argc, argv);

//  collectData();

//  computeCalibration();

//    selectHSV();

  demoCalibration();

  LOG(0) <<" === bye bye ===\n";

  return 0;
}
