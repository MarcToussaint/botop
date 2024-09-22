#include "bot.h"

#include <Franka/help.h>
#include <Kin/F_qFeatures.h>
#include <Kin/viewer.h>
#include <KOMO/pathTools.h>
#include <Control/timingOpt.h>
#include <Optim/NLP_Solver.h>
#include <Gui/opengl.h>
#include <Geo/depth2PointCloud.h>

#include <Franka/franka.h>
#include <Franka/FrankaGripper.h>
#include "simulation.h"
#include <Omnibase/omnibase.h>
#include <Robotiq/RobotiqGripper.h>
#include <OptiTrack/optitrack.h>
#include <RealSense/RealSenseThread.h>
#ifdef RAI_VIVE
#  include <ViveController/vivecontroller.h>
#endif

#include <Audio/audio.h>

//===========================================================================

BotOp::BotOp(rai::Configuration& C, bool useRealRobot){
  //-- launch arm(s) & gripper(s)
  bool useGripper = rai::getParameter<bool>("bot/useGripper", true);
  bool blockRealRobot = rai::getParameter<bool>("bot/blockRealRobot", false);

  C.ensure_indexedJoints();
  qHome = C.getJointState();
  state.set()->initZero(qHome.N);

  if(blockRealRobot && useRealRobot){
    LOG(0) <<"-- blocking useRealRobot -- ";
    useRealRobot=false;
  }

  //-- launch robots & grippers
  if(useRealRobot && useGripper){
    LOG(0) <<"CONNECTING TO GRIPPERS";
    try{
      if(C.getFrame("l_panda_hand", false) && C.getFrame("r_panda_hand", false)){
        gripperL = make_shared<FrankaGripper>(0);
        gripperR = make_shared<FrankaGripper>(1);
      }else if(C.getFrame("l_panda_hand", false)){
        gripperL = make_shared<FrankaGripper>(0);
      }else if(C.getFrame("r_panda_hand", false)){
        gripperR = make_shared<FrankaGripper>(1);

      }else if(C.getFrame("l_robotiq_base", false) && C.getFrame("r_robotiq_base", false)){
        gripperL = make_shared<RobotiqGripper>(0);
        gripperR = make_shared<RobotiqGripper>(1);
      }else if(C.getFrame("l_robotiq_base", false)){
        gripperL = make_shared<RobotiqGripper>(0);
      }else if(C.getFrame("r_robotiq_base", false)){
        gripperR = make_shared<RobotiqGripper>(1);
      }
    } catch(const std::exception& ex) {
      LOG(-1) <<"Starting the gripper(s) failed! Error msg: " <<ex.what();
    }
  }

  if(useRealRobot){
    uint robotID=0;
    LOG(0) <<"CONNECTING TO FRANKAS";
    try{
      if(C.getFrame("l_panda_base", false) && C.getFrame("r_panda_base", false)){
        robotL = make_shared<FrankaThread>(robotID++, franka_getJointIndices(C,'l'), cmd, state);
        robotR = make_shared<FrankaThread>(robotID++, franka_getJointIndices(C,'r'), cmd, state);
      } else if(C.getFrame("l_panda_base", false)){
        robotL = make_shared<FrankaThread>(robotID++, franka_getJointIndices(C,'l'), cmd, state);
      } else if(C.getFrame("r_panda_base", false)){
        robotR = make_shared<FrankaThread>(robotID++, franka_getJointIndices(C,'r'), cmd, state);
      }else{
        LOG(0) <<"starting botop without franka robots (no frames l_panda_base or r_panda_base defined)";
      }
      C.setJointState(get_q());
    } catch(const std::exception& ex) {
      LOG(-1) <<"Starting the franka robot(s) failed! Error msg: " <<ex.what();
    } catch(...) {
      LOG(-1) <<"Starting the franka robot(s) failed! Error msg: " <<rai::errString();
    }

    try{
      if(C.getFrame("omnibase_world", false)){
        LOG(0) <<"CONNECTING TO OMNIBASE";
        robotL = make_shared<OmnibaseThread>(robotID++, uintA{0,1,2}, cmd, state);
      }
    } catch(const std::exception& ex) {
      LOG(-1) <<"Starting the omnibase failed! Error msg: " <<ex.what();
    }

  }else{
    simthread = make_shared<BotThreadedSim>(C, cmd, state);
    robotL = simthread;
    if(useGripper) gripperL = make_shared<GripperSim>(simthread, "l_gripper");
  }

  startRealTime = rai::realTime();

  //-- initialize the control reference
  hold(false, true);

  //-- launch OptiTrack
  if(rai::getParameter<bool>("bot/useOptitrack", false)){
    LOG(0) <<"OPENING OPTITRACK";
    if(!useRealRobot) LOG(-1) <<"useOptitrack with real:false -- that's usually wrong!";
    optitrack = make_shared<rai::OptiTrack>();
    optitrack->pull(C);
  }

#ifdef RAI_VIVE
  //-- launch ViveController
  if(rai::getParameter<bool>("bot/useViveController", false)){
    LOG(0) <<"OPENING ViveController";
    vivecontroller = make_shared<rai::ViveController>();
    vivecontroller->pull(C);
  }
#endif

  //-- launch Audio/Sound
  if(rai::getParameter<bool>("bot/useAudio", false)){
    LOG(0) <<"OPENING SOUND";
    audio = make_shared<rai::Sound>();
  }

  C.gl().setTitle("BotOp associated Configuration");
  C.view(false, STRING("time: 0"));
}

BotOp::~BotOp(){
  LOG(0) <<"shutting down BotOp...";
  if(simthread) simthread.reset();
  gripperL.reset();
  gripperR.reset();
  robotL.reset();
  robotR.reset();
}

double BotOp::get_t(){
  return state.get()->ctrlTime;
}

void BotOp::getState(arr& q_real, arr& qDot_real, double& ctrlTime){
  auto stateGet = state.get();
  q_real = stateGet->q;
  qDot_real = stateGet->qDot;
  ctrlTime = stateGet->ctrlTime;
}

void BotOp::getReference(arr& q_ref, arr& qDot_ref, arr& qDDot_ref, const arr& q_real, const arr& qDot_real, double ctrlTime){
  auto cmdGet = cmd.get();
  CHECK(cmdGet->ref, "reference not initialized yet!");
  cmdGet->ref->getReference(q_ref, qDot_ref, qDDot_ref, q_real, qDot_real, ctrlTime);
}

arr BotOp::get_q() {
  return state.get()->q;
}

arr BotOp::get_qDot() {
  return state.get()->qDot;
}

double BotOp::getTimeToEnd(){
  auto sp = std::dynamic_pointer_cast<rai::BSplineCtrlReference>(ref);
  if(!sp){
    LOG(-1) <<"can't get timeToEnd for non-spline mode";
    return 0.;
  }
  double ctrlTime = get_t();
  return sp->getEndTime() - ctrlTime;
}

arr BotOp::getEndPoint(){
  auto sp = std::dynamic_pointer_cast<rai::BSplineCtrlReference>(ref);
  if(!sp) return get_q();
  return sp->getEndPoint();
}

arr BotOp::get_tauExternal(){
  arr tau;
  {
    auto stateSet = state.set();
    tau = stateSet->tauExternalIntegral;
    tau /= double(stateSet->tauExternalCount);
    stateSet->tauExternalIntegral.setZero();
    stateSet->tauExternalCount=0;
  }
  return tau;
}

int BotOp::sync(rai::Configuration& C, double waitTime, rai::String viewMsg){
  //update q state
  C.setJointState(state.get()->q);

  //update optitrack state
  if(optitrack) optitrack->pull(C);

#ifdef RAI_VIVE
  //update vivecontroller state
  if(vivecontroller) vivecontroller->pull(C);
#endif

  //update sim state
  if(simthread) simthread->pullDynamicStates(C);

  //gui
  if(rai::getParameter<bool>("bot/raiseWindow",false)) C.get_viewer()->raiseWindow();
  double ctrlTime = get_t();
  keypressed = C.view(false, STRING("BotOp sync ctrl time: "<<ctrlTime <<" (=" <<int(100.*ctrlTime/(rai::realTime()-startRealTime)) <<"% real time)\n" <<viewMsg));
  if(keypressed) C.get_viewer()->_resetPressedKey();
//  if(keypressed==13) return false;
//  if(keypressed=='q' || keypressed==27) return false;
//  auto sp = std::dynamic_pointer_cast<rai::SplineCtrlReference>(ref);
//  if(sp && ctrlTime>sp->getEndTime()) return false;
  if(!keypressed && waitTime>0.) rai::wait(waitTime);
  return keypressed;
}

int BotOp::wait(rai::Configuration& C, bool forKeyPressed, bool forTimeToEnd, bool forGripper){
  C.get_viewer()->raiseWindow();
  C.get_viewer()->_resetPressedKey();
  for(;;){
    sync(C, .1);
    //if(keypressed=='q') return keypressed;
    if(forKeyPressed && keypressed) return keypressed;
    if(forTimeToEnd && getTimeToEnd()<=0.) return keypressed;
    if(forGripper && gripperDone(rai::_left)) return 'g';
    if(!rai::getInteractivity() && !forTimeToEnd && forKeyPressed) return ' ';
  }
}

std::shared_ptr<rai::BSplineCtrlReference> BotOp::getSplineRef(){
  auto sp = std::dynamic_pointer_cast<rai::BSplineCtrlReference>(ref);
  if(!sp){
    setReference<rai::BSplineCtrlReference>();
    sp = std::dynamic_pointer_cast<rai::BSplineCtrlReference>(ref);
    CHECK(sp, "this is not a spline reference!")
  }
  return sp;
}

void BotOp::move(const arr& path, const arr& times, bool overwrite, double overwriteCtrlTime){
  arr _times=times;

  CHECK(_times.N, "move needs some times specified - use moveAutoTimed as alternative");

  //-- if times.N != path.d0, fill in times
  if(_times.N==1 && path.d0>1){ //divide total time in grid
    _times = range(0., times.scalar(), path.d0-1);
    _times += _times(1);
  }
  if(_times.N){ //times are fully specified
    CHECK_EQ(_times.N, path.d0, "");
  }

  if(overwrite){
    CHECK(overwriteCtrlTime>0., "overwrite -> need to give a cut-time (e.g. start or MPC cycle, or just get_t())");
    //LOG(1) <<"overwrite: " <<ctrlTime <<" - " <<_times;
    if(times.first()>0.){
      getSplineRef()->overwriteSmooth(path, /*vels,*/ _times, overwriteCtrlTime);
    }else{
      getSplineRef()->overwriteHard(path, /*vels,*/ _times, overwriteCtrlTime);
    }
  }else{
    //LOG(1) <<"append: " <<ctrlTime <<" - " <<_times;
    getSplineRef()->append(path, /*vels,*/ _times, get_t());
  }
}

void BotOp::move_oldCubic(const arr& path, const arr& times, bool overwrite, double overwriteCtrlTime){
  arr _times=times;


  if(std::dynamic_pointer_cast<rai::BSplineCtrlReference>(ref)){
    return move(path, _times, overwrite, overwriteCtrlTime);
  }

  arr vels;
  if(path.d0==1){
    vels = zeros(1, path.d1);
  }else{ //use timing opt to decide on vels and, optionally, on timing
    arr q, qDot;
    if(!overwrite){
      getSplineRef()->eval(q, qDot, NoArr, getSplineRef()->getEndTime());
      q = path[0];
      qDot = zeros(q.N);
    }else{ //THIS IS STILL BUGGY - need overwriteCtrlTime!!
      CHECK(overwriteCtrlTime>0., "overwrite -> need to give a cut-time (e.g. start og MPC cycle, or just get_t())");
      getSplineRef()->eval(q, qDot, NoArr, overwriteCtrlTime);
    }

    bool optTau = (times.N==0);
    arr tauInitial = {};
    if(!optTau) tauInitial = differencing(_times);
    TimingProblem timingProblem(path, {}, q, qDot, 1., 1., optTau, false, {}, tauInitial);
    NLP_Solver solver;
    solver
        .setProblem(timingProblem.ptr())
        .setSolver(NLPS_newton);
    solver.opt
        .set_stopTolerance(1e-4)
        .set_maxStep(1e0)
        .set_damping(1e-2);
    auto ret = solver.solve();
    //LOG(1) <<"timing f: " <<ret->f;
    timingProblem.getVels(vels);
    if(!_times.N) _times = integral(timingProblem.tau);
  }

  NIY; //move(path, vels, _times, overwrite, overwriteCtrlTime);
}

void BotOp::moveAutoTimed(const arr& path, double maxVel, double maxAcc){
  CHECK_GE(path.d0, 16, "this only works for smooth paths!");
  double D = getMinDuration(path, maxVel, maxAcc);
  arr times = range(0., D, path.d0-1);
  times += times(1);
  move(path, times);
}

void BotOp::moveTo(const arr& q_target, double timeCost, bool overwrite){
  arr q, qDot;
  double ctrlTime=0.;
  if(overwrite){
    getState(q, qDot, ctrlTime);
  }else{
    q = getEndPoint();
    qDot.resize(q.N).setZero();
  }
  double dist = length(q-q_target) +1e-4; //plus eps for numerical robustness
  double vel = scalarProduct(qDot, q_target-q)/dist;
  double T = (sqrt(6.*timeCost*dist+vel*vel) - vel)/timeCost;
  //cout <<q <<qDot <<q_target <<dist <<' ' <<vel <<' ' <<T <<' ' <<ctrlTime <<endl;
  if(dist<1e-4 || T<.1) T=.1;
  if(overwrite){
    move(~q_target, {T}, true, ctrlTime);
  }else{
    move(~q_target, {T}, false);
  }
}

void BotOp::setControllerWriteData(int _writeData){
  if(robotL) robotL->writeData=_writeData;
  if(robotR) robotR->writeData=_writeData;
}

void BotOp::setCompliance(const arr& J, double compliance){
  CHECK_LE(compliance, 1., "");
  CHECK_GE(compliance, 0., "");
  if(!J.N || !compliance){
    LOG(0) <<"clearing compliance";
    cmd.set()->P_compliance.clear();
    return;
  }


  arr U, d, V;
  svd(U, d, V, J, false);
  CHECK_EQ(d.N, J.d0, "");
  for(uint i=0;i<d.N;i++) CHECK_GE(fabs(d(i)), 1e-3, "singular Jacobian?");

  arr P = eye(J.d1);
  P -= compliance * (V*~V);

  cmd.set()->P_compliance = P;
}

void BotOp::gripperMove(rai::ArgWord leftRight, double width, double speed){
  if(leftRight==rai::_left){ if(!gripperL) LOG(-1) <<"gripper disabled"; else gripperL->open(width, speed); }
  if(leftRight==rai::_right){ if(!gripperR) LOG(-1) <<"gripper disabled"; else gripperR->open(width, speed); }
}

void BotOp::gripperClose(rai::ArgWord leftRight, double force, double width, double speed){
  if(leftRight==rai::_left){ if(!gripperL) LOG(-1) <<"gripper disabled"; else gripperL->close(force, width, speed); }
  if(leftRight==rai::_right){ if(!gripperR) LOG(-1) <<"gripper disabled"; else gripperR->close(force, width, speed); }
}

void BotOp::gripperCloseGrasp(rai::ArgWord leftRight, const char* objName, double force, double width, double speed){
  if(leftRight==rai::_left){ if(!gripperL) LOG(-1) <<"gripper disabled"; else gripperL->closeGrasp(objName, force, width, speed); }
  if(leftRight==rai::_right){ if(!gripperR) LOG(-1) <<"gripper disabled"; else gripperR->closeGrasp(objName, force, width, speed); }
}

double BotOp::getGripperPos(rai::ArgWord leftRight){
  if(leftRight==rai::_left){ if(!gripperL) LOG(-1) <<"gripper disabled"; else return gripperL->pos(); }
  if(leftRight==rai::_right){ if(!gripperR) LOG(-1) <<"gripper disabled"; else return gripperR->pos(); }
  return 0;
}

bool BotOp::gripperDone(rai::ArgWord leftRight){
  if(leftRight==rai::_left){ if(!gripperL) LOG(-1) <<"gripper disabled"; else return gripperL->isDone(); }
  if(leftRight==rai::_right){ if(!gripperR) LOG(-1) <<"gripper disabled"; else return gripperR->isDone(); }
  return true;
}

std::shared_ptr<rai::CameraAbstraction>& BotOp::getCamera(const char* sensor){
  for(std::shared_ptr<rai::CameraAbstraction>& cam:cameras){
    if(cam->name==sensor) return cam;
  }
  if(simthread){
    cameras.append( make_shared<CameraSim>(simthread, sensor) );
  }else{
    cameras.append( make_shared<RealSenseThread>(sensor) );
  }
  return cameras(-1);
}

void BotOp::getImageAndDepth(byteA& image, floatA& depth, const char* sensor){
  auto cam = getCamera(sensor);
  cam->getImageAndDepth(image, depth);
}

arr BotOp::getCameraFxycxy(const char* sensor){
  auto cam = getCamera(sensor);
  return cam->getFxycxy();
}

void BotOp::getImageDepthPcl(byteA& image, floatA& depth, arr& points, const char* sensor, bool globalCoordinates){
  auto cam = getCamera(sensor);
  cam->getImageAndDepth(image, depth);
  depthData2pointCloud(points, depth, cam->getFxycxy());
  if(globalCoordinates){
    rai::Transformation pose=cam->getPose();
    if(!pose.isZero()) pose.applyOnPointArray(points);
  }
}

void BotOp::home(rai::Configuration& C){
  C.get_viewer()->raiseWindow();
  moveTo(qHome, 1., true);
  wait(C);
}

void BotOp::stop(rai::Configuration& C){
  C.get_viewer()->raiseWindow();
  moveTo(get_q(), .01, true);
  wait(C);
}

void BotOp::hold(bool floating, bool damping){
  auto zref = std::dynamic_pointer_cast<ZeroReference>(ref);
  if(!zref){
    setReference<ZeroReference>();
    zref = std::dynamic_pointer_cast<ZeroReference>(ref);
    CHECK(zref, "this is not a spline reference!")
  }
  if(floating){
    zref->setPositionReference({});
    if(damping){
      zref->setVelocityReference({0.}); //{0.}: have a Kd with zero vel ref;
    }else{
      zref->setVelocityReference({}); //{}: have no Kd term at all; {1.} have a Kd term with velRef=velTrue (and friction compensation!)
    }
  }else{
    arr q = get_q();
    zref->setPositionReference(q);
    zref->setVelocityReference({0.});
  }
}

void BotOp::sound(int noteRelToC, float a, float decay){
  if(audio){
    audio->addNote(noteRelToC, a, decay);
  }
}

//===========================================================================

void ZeroReference::getReference(arr& q_ref, arr& qDot_ref, arr& qDDot_ref, const arr& q_real, const arr& qDot_real, double ctrlTime){
  {
    arr pos = position_ref.get()();
    if(pos.N) q_ref = pos;
    else q_ref.clear(); // = q_real;  //->no position gains at all
  }
  {
    arr vel = velocity_ref.get()();
    if(vel.N==1){
      double a = vel.scalar();
      CHECK(a>=0. && a<=1., "");
      qDot_ref = a * qDot_real; //[0] -> zero vel reference -> damping
    }
    else if(vel.N) qDot_ref = vel;
    else qDot_ref.clear(); //.clear();  //[] -> no damping at all! (and also no friction compensation based on reference qDot)
  }
  qDDot_ref.clear(); //[] -> no acc at all
}

