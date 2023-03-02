#include "SecMPC_Stepper.h"

#include <GL/glew.h>
#include <GL/glut.h>

#include <OptiTrack/optitrack.h>
#include <Gui/opengl.h>
#include <Kin/viewer.h>


bool SecMPC_Stepper::step(rai::Configuration& C, BotOp& bot, SecMPC& mpc){
  stepCount++;

  //-- iterate
  tic.waitForTic();

  //-- get optitrack
  if(bot.optitrack) bot.optitrack->pull(C);

  //-- get current state (time,q,qDot)
  arr q,qDot, q_ref, qDot_ref;
  double ctrlTime = 0.;
  bot.getState(q, qDot, ctrlTime);
  bot.getReference(q_ref, qDot_ref, NoArr, q, qDot, ctrlTime);

  //-- iterate MPC
  mpc.cycle(C, q_ref, qDot_ref, q, qDot, ctrlTime);
  mpc.report(C);
  if(mpc.phaseSwitch) bot.sound(7 * mpc.timingMPC.phase);

  if(fil.is_open()){
    fil <<stepCount <<' ' <<ctrlTime <<' ' <<mpc.timingMPC.phase <<endl;
    q.writeTagged(fil, "q_real", true);
    mpc.waypointMPC.path.writeTagged(fil, "waypoints", true);
    mpc.timingMPC.tau.writeTagged(fil, "tau", true);
    mpc.shortMPC.path.writeTagged(fil, "shortPath", true);
    C.getFrameState(logPoses).writeTagged(fil, "poses", true);
  }

  //-- send spline update
  ctrlTime = bot.get_t();
  auto sp = mpc.getShortPath(ctrlTime);
  if(sp.pts.d0){
//    if(sp.times.first()<0.) bot.sound(2*(stepCount%12));
    if(sp.vels.N) HALT("") //bot.move(sp.pts, sp.vels, sp.times, true, ctrlTime);
    else bot.move(sp.pts, sp.times, true, ctrlTime);
  }

  //-- update C
  bot.step(C, .0);
  if(bot.keypressed=='q' || bot.keypressed==27) return false;

  return true;
}

//===========================================================================

void randomWalkPosition(rai::Frame* f, arr& centerPos, arr& velocity, double rate){
  arr pos = f->getPosition();
  if(!centerPos.N) centerPos = pos;
  if(!velocity.N) velocity = zeros(pos.N);
  rndGauss(velocity, rate, true);
  velocity *= .99;
  pos += velocity;
  pos = centerPos + .9 * (pos - centerPos);
  f->setPosition(pos);
}

//===========================================================================

void setCamera(OpenGL& gl, rai::Frame* camF){
  rai::Camera& cam = gl.camera;
  {
    auto _dataLock = gl.dataLock(RAI_HERE);
    cam.X = camF->ensure_X();

    rai::Node *at=0;
    if((at=camF->ats->getNode("focalLength"))) cam.setFocalLength(at->as<double>());
    if((at=camF->ats->getNode("orthoAbsHeight"))) cam.setHeightAbs(at->as<double>());
    if((at=camF->ats->getNode("zRange"))){ arr z=at->as<arr>(); cam.setZRange(z(0), z(1)); }
    if((at=camF->ats->getNode("width"))) gl.width=at->as<double>();
    if((at=camF->ats->getNode("height"))) gl.height=at->as<double>();
    //    cam.setWHRatio((double)gl.width/gl.height);
  }
  gl.resize(gl.width, gl.height);
}

void playLog(const rai::String& logfile){
  rai::Configuration C;
  C.addFile("pushScenario.g");

  FrameL logPoses = C.getFrames({"puck", "target"});


  uint stepCount, phase;
  double ctrlTime;
  arr q_real, waypoints, tau, shortPath, poses;

  OpenGL gl;
  gl.add(glStandardScene);

  if(C["camera_gl"]) setCamera(gl, C["camera_gl"]);

  gl.add([&](OpenGL& gl){
    //C itself
    gl.drawOptions.drawVisualsOnly=true;
    gl.drawOptions.drawColors=true;
    C.setJointState(q_real);
    C.setFrameState(poses, logPoses);
    C.glDraw(gl);

    //waypoints
    glColor(1., 1., .5, .3);
    gl.drawOptions.drawColors=false;
    for(uint t=0;t<waypoints.d0;t++){
      C.setJointState(waypoints[t]);
      C.glDraw(gl);
    }

    //shortPath
    glColor(.5, 1., 1., .3);
    gl.drawOptions.drawColors=false;
    for(uint t=0;t<shortPath.d0;t++){
      C.setJointState(shortPath[t]);
      C.glDraw(gl);
    }

    //text
    rai::String text;
    text <<"phase: " <<phase <<" ctrlTime:" <<ctrlTime <<"\ntau: " <<tau;

    glColor(.2,.2,.2,.5);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glMatrixMode(GL_MODELVIEW);
    glOrtho(0., (double)gl.width, (double)gl.height, .0, -1., 1.);
    glDrawText(text, 20, +20, 0, true);
  });

  ifstream fil(logfile);
  for(uint k=1;;k++){

    fil >>stepCount >>ctrlTime >>phase;
    if(!fil.good()) break;
    CHECK_EQ(stepCount, k, "");

    q_real.readTagged(fil, "q_real");
    waypoints.readTagged(fil, "waypoints");
    tau.readTagged(fil, "tau");
    shortPath.readTagged(fil, "shortPath");
    poses.readTagged(fil, "poses");


    gl.update();
    rai::wait(.1);
  }

};
