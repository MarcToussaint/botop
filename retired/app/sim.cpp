#include "sim.h"
#include "filter.h"

#include <Gui/opengl.h>
#include <Kin/frame.h>

KinSim::KinSim(double dt) : Thread("KinSim", dt), dt(dt),
    ref(this, "MotionReference"),
    switches(this, "switches"),
    gripper(this, "gripperReference"),
    currentQ(this, "currentQ"),
    currentGripper(this, "currentGripper"),
    jointNames(this, "jointNames"),
    robotBase(this, "robotBase"),
//    nextQ(this, "nextQ"),
//    world(this, "world"),
    timeToGo(this, "timeToGo"),
    percepts_input(this, "percepts_input"){

    K = Var<rai::KinematicWorld>("world").get();

    arr q = K.getJointState(jointNames.get());
    reference.points = q;
    reference.points.append( q );
    reference.points.append( q );
    reference.points.reshape(3, q.N );
    reference.degree = 2;
    reference.setUniformNonperiodicBasis();
    log.open("z.KinSim");
    K.gl().title = "KinSim";

    //add random noise to perceptual objects:
    for(rai::Frame *a:K.frames){
      if(a->ats["percept"]){
        a->X.pos.y += rnd.uni(-.5, .5);
      }
    }

    currentQ.set() = q;
}

void KinSim::step(){
  //-- publish robot base
  robotBase.set() = K.getFrameByName("base")->X;

  //-- reference tracking

  //check for path update
  if(ref.hasNewRevision()){
    ref.readAccess();
    reference.points = K.getJointState(jointNames.get());
    reference.points.append(ref().path);
    reference.points.reshape(ref().path.d0+1, ref().path.d1);
    reference.setUniformNonperiodicBasis();
    phase=0.;
    CHECK(ref().tau.N==1,"");
    planDuration = ref().tau.scalar() * ref().path.d0;
    timeToGo.set() = planDuration;
    ref.deAccess();
  }

  //check for switches update
  uint rev2=switches.readAccess();
  if(switchesRev != rev2){
    switchesRev = rev2;
    StringA cmd = switches.get();
    cout <<"CMD = " <<cmd <<endl;
    if(cmd(0)=="attach"){
      rai::Frame *a = K.getFrameByName(cmd(1));
      rai::Frame *b = K.getFrameByName(cmd(2));

      if(b->parent) b->unLink();
      b->linkFrom(a, true);
      (new rai::Joint(*b)) -> type=rai::JT_rigid;
      K.calc_q();
    }

  }
  switches.deAccess();

  //check for a gripper command
  if(gripper.hasNewRevision()){
    K["wsg_50_base_joint_gripper_left"]->joint->getQ() = gripper.get();
  }

  //set the simulation's joint state
  arr q = reference.eval(phase);
  K.setJointState(q, jointNames.get());
  K.gl().update();
  log <<q <<endl;
  //progress the reference tracking
  if(phase!=1.){
    phase += dt/planDuration;
    if(phase>1.) phase=1.;
    timeToGo.set() = planDuration*(1.-phase);
  }

  //== outputs

  //provide the q perception
  currentQ.set() = K.getJointState(jointNames.get());
  currentGripper.set() = K["wsg_50_base_joint_gripper_left"]->joint->getQ();

  //-- percept simulation
  PerceptSimpleL P;
  for(rai::Frame *a:K.frames){
    if(a->ats["percept"]){
      P.append(new PerceptSimple(*a->shape->geom, a->X));
    }
  }
  if(P.N) percepts_input.set()->append(P);

}
