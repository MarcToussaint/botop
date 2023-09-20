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

//===========================================================================

void collectData(){
  //-- setup a configuration
  rai::Configuration C;
  C.addFile("station.g");

//  C["bellybutton"]->name = "dot0";
  rai::Frame* cam = C["cameraWrist"];
  rai::Frame* mount = C["l_panda_joint7"];

  BotOp bot(C, rai::getParameter<bool>("real"));

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

  uint nDots=2;
  for(uint d=0;d<nDots;d++){
    for(uint k=0;k<20; k++){
      rai::Frame* dot = C[STRING("dot" <<d)];

      arr dotPos = arr{.2, .15, .2}%(rand(3)-.5);
      dotPos(2) -= .35;
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

    //get stuff
    byteA img(dat.get<byteA>("img"));
    floatA depth(dat.get<floatA>("depth"));
    rai::Transformation mountPose(dat.get<arr>("mountPose"));
    rai::Transformation camPose(dat.get<arr>("camPose"));
    rai::Vector dotWorld(dat.get<arr>("dotPosition"));
    arr Fxypxy = dat.get<arr>("camFxypxy");

    if(img.d0 != depth.d0) continue;

    // image coordinates
    arr u = getHsvBlobImageCoords(img, depth, hsvFilter);
    if(!u.N){ rai::wait(); continue; }

    // homogeneous
    arr uHom = u;
    makeHomogeneousImageCoordinate(uHom, img.d0);

    // camera coordinates assuming given intrinsics
    arr xCam=u;
    depthData2point(xCam, Fxypxy); //transforms the point to camera xyz coordinates

    // camera coordinates from ground truth world coordinates
    arr x = (dotWorld / camPose).getArr();

    cout <<"blob: " <<u <<' ' <<uHom <<' ' <<xCam <<' ' <<x <<' ' <<xCam-x <<' ' <<Fxypxy <<endl;

    disp.watchImage(img, true, 1.);

    //collect data
    U.append(uHom);
    X.append(x);
  }

  //-- multiple iterations
  arr Pinv, K, R, t;
  for(uint k=0;k<5;k++){
    Pinv = ~X * U * inverse_SymPosDef(~U*U);
    decomposeInvProjectionMatrix(K, R, t, Pinv);
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

  rai::Frame* cam = C["cameraWrist"];
  arr hsvFilter = rai::getParameter<arr>("hsvFilter");
  arr Pinv = rai::getParameter<arr>("Pinv");
  int checks = rai::getParameter<int>("checks", 1);

  BotOp bot(C, rai::getParameter<bool>("real"));

  //pre motion
  bot.gripperMove(rai::_left);
  {
    Move_IK move(bot, C, checks);
    move().addObjective({}, FS_positionRel, {"dot0", cam->name}, OT_eq, {1e0}, {0.,0.,-.3});
    if(!move.go()) return;
  }

  //fine motion
  bot.gripperCloseGrasp(rai::_left, 0);
  for(uint t=0;t<5;t++) bot.sync(C);
  if(!sense_HsvBlob(bot, C, cam->name, "target", hsvFilter, Pinv, checks)) return;
  {
    Move_IK move(bot, C, checks);
    move().addObjective({}, FS_vectorZ, {"l_gripper"}, OT_eq, {1e0}, {0.,0.,1.});
    move().addObjective({}, FS_positionRel, {"target", "l_gripper"}, OT_eq, {1e0}, {0.,0.,-.01});
    if(!move.go()) return;
  }

  //pre motion
  if(!bot.wait(C)) return;
  bot.gripperMove(rai::_left);
  {
    Move_IK move(bot, C, checks);
    move().addObjective({}, FS_positionRel, {"dot1", cam->name}, OT_eq, {1e0}, {0.,0.,-.3});
    if(!move.go()) return;
  }

  //fine motion
  bot.gripperCloseGrasp(rai::_left, 0);
  for(uint t=0;t<5;t++) bot.sync(C);
  if(!sense_HsvBlob(bot, C, cam->name, "target", hsvFilter, Pinv, checks)) return;
  {
    Move_IK move(bot, C, checks);
    move().addObjective({}, FS_vectorZ, {"l_gripper"}, OT_eq, {1e0}, {0.,0.,1.});
    move().addObjective({}, FS_positionRel, {"target", "l_gripper"}, OT_eq, {1e0}, {0.,0.,-.01});
    if(!move.go()) return;
  }

  if(!bot.wait(C)) return;
  bot.gripperMove(rai::_left);
  bot.home(C);
}

//===========================================================================

void checkTip(){
  //-- setup a configuration
  rai::Configuration C;
  C.addFile("station.g");

  BotOp bot(C, rai::getParameter<bool>("real"));

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

//  computeCalibration();

//  selectHSV();

  demoCalibration();

//  checkTip();

  LOG(0) <<" === bye bye ===\n";

  return 0;
}
