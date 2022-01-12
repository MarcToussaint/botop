#include "SequenceController.h"

#include <Optim/MP_Solver.h>

WaypointMPC::WaypointMPC(rai::Configuration& C, rai::Array<ObjectiveL> phiflag, rai::Array<ObjectiveL> phirun, const arr& qHome){
  komo.clearObjectives();
  CHECK_EQ(phiflag.N, phirun.N, "");
  uint K=phiflag.N;

  uint subSamples=0; //#subframes

  komo.setModel(C, false);
  komo.setTiming(K, subSamples+1, 2., 1);
  komo.setupPathConfig();

  komo.add_qControlObjective({}, komo.k_order, 1e0);
  if(qHome.N) komo.addObjective({}, FS_qItself, {}, OT_sos, {.1}, qHome);

  for(uint k=0;k<K;k++){
    for(auto& o: phiflag(k))  komo.addObjective({1.+double(k)}, o->feat, {}, o->type);
    for(auto& o: phirun(k))  komo.addObjective({double(k), 1.+double(k)}, o->feat, {}, o->type);
  }

  komo.reset();
  komo.initWithConstant(qHome);
  path = komo.getPath_qOrg();
  this->tau = komo.getPath_tau();
}

WaypointMPC::WaypointMPC(rai::Configuration& C, ObjectiveL phi, const arr& qHome){
  komo.clearObjectives();

  double maxPhase=0.;
  for(auto& o:phi) {
    if(o->times.N) maxPhase = rai::MAX(maxPhase, o->times.max());
  }

  uint K=maxPhase;

  uint subSamples=0; //#subframes

  komo.setModel(C, false);
  komo.setTiming(K, subSamples+1, 2., 1);
  komo.setupPathConfig();

  komo.add_qControlObjective({}, komo.k_order, 1e0);
  if(qHome.N) komo.addObjective({}, FS_qItself, {}, OT_sos, {.1}, qHome);

  for(auto& o:phi) komo.addObjective(o->times, o->feat, {}, o->type);

  komo.reset();
  komo.initWithConstant(qHome);
  path = komo.getPath_qOrg();
  this->tau = komo.getPath_tau();
}

void WaypointMPC::reinit(const rai::Configuration& C){
  komo.updateRootObjects(C);
}

void WaypointMPC::solve(){
  //re-run KOMO
  rai::OptOptions opt;
  opt.stopTolerance = 1e-4;
  opt.stopGTolerance = 1e-4;
  komo.opt.verbose=0;
  komo.timeTotal=0.;
  komo.pathConfig.setJointStateCount=0;
  //komo.reportProblem();
  komo.optimize(0., opt);
  //komo.checkGradients();

  //is feasible?
  feasible=komo.sos<50. && komo.ineq<.1 && komo.eq<.1;
  rai::String msg;
  msg <<"it " <<steps <<" feasible: " <<(feasible?" good":" FAIL") <<" -- queries: " <<komo.pathConfig.setJointStateCount <<" time:" <<komo.timeTotal <<"\t sos:" <<komo.sos <<"\t ineq:" <<komo.ineq <<"\t eq:" <<komo.eq <<endl;

  komo.view(false, msg);

  path = komo.getPath_qOrg();
  this->tau = komo.getPath_tau();

  //  while(komo.view_play(true, 1.));

  //  arr path = komo.getPath_qOrg();

  //  bot.move(path, {2., 2.5});
  //  while(bot.step(C, .5));
  //  if(bot.keypressed=='q' || bot.keypressed==27) return;

  //  rai::wait();



  //store as output result
  //  x1 = komo.getConfiguration_qOrg(0);
  //  xT = komo.getConfiguration_qOrg(komo.T-1); //the last one
  //  tau = komo.getPath_tau();
}

//===========================================================================

SequenceController::SequenceController(rai::Configuration& C, rai::Array<ObjectiveL> _phiflag, rai::Array<ObjectiveL> _phirun, const arr& qHome)
  : pathMPC(C, _phiflag, _phirun, qHome),
    timingMPC(pathMPC.path, 1e0) {
  pathMPC.solve();

  uint K = _phiflag.N;

  timingMPC.tangents = zeros(K-1, qHome.N);
  for(uint k=1; k<K; k++){
    timingMPC.tangents[k-1] = pathMPC.path[k] - pathMPC.path[k-1];
    op_normalize(timingMPC.tangents[k-1]());
  }
}

SequenceController::SequenceController(rai::Configuration& C, ObjectiveL phi, const arr& qHome)
  : pathMPC(C, phi, qHome),
    timingMPC(pathMPC.path, 1e0) {

  uint K = pathMPC.komo.T;

  timingMPC.tangents = zeros(K-1, qHome.N);
  for(uint k=1; k<K; k++){
    timingMPC.tangents[k-1] = pathMPC.path[k] - pathMPC.path[k-1];
    op_normalize(timingMPC.tangents[k-1]());
  }
}

void SequenceController::updateWaypoints(const rai::Configuration& C){
  pathMPC.reinit(C); //adopt all frames in C as prefix (also positions of objects)
  //      pathProblem.reinit(q, qDot);
  pathMPC.solve();

  msg <<" (path) #:" <<pathMPC.komo.pathConfig.setJointStateCount;
  //      msg <<" T:" <<pathProblem.komo.timeTotal <<" f:" <<pathProblem.komo.sos <<" eq:" <<pathProblem.komo.eq <<" iq:" <<pathProblem.komo.ineq;
}

void SequenceController::updateTiming(const rai::Configuration& C, const ObjectiveL& phi, double ctrlTime, const arr& q_real, const arr& qDot_real){
  //-- adopt the new path
  timingMPC.update_flags(pathMPC.path);

  //-- progress time (potentially phase)
  if(!timingMPC.done() && ctrlTimeLast>0.) timingMPC.update_progressTime(ctrlTime - ctrlTimeLast);
  ctrlTimeLast = ctrlTime;

  //-- phase backtracking
  if(timingMPC.done()){
    if(phi.maxError(C, timingMPC.phase) > 0.1){
      timingMPC.update_backtrack();
    }
  }
  if(!timingMPC.done()){
    while(phi.maxError(C, 0.5+timingMPC.phase) > 0.1){
      phi.maxError(C, 0.5+timingMPC.phase, 1); //verbose
      timingMPC.update_backtrack();
    }
  }

  //-- solve the timing problem
  if(!timingMPC.done()){
    if(timingMPC.tau(timingMPC.phase)>.2){
      auto ret = timingMPC.solve(q_real, qDot_real, 0);
      msg <<" (timing) ph:" <<timingMPC.phase <<" #:" <<ret->evals;
      //      msg <<" T:" <<ret->time <<" f:" <<ret->f;
    }else{
      LOG(0) <<"skipping timing opt, as too close ahead: " <<timingMPC.tau;
    }
  }

  msg <<" tau: " <<timingMPC.tau << ctrlTime + timingMPC.getTimes(); // <<' ' <<F.vels;
}

void SequenceController::cycle(const rai::Configuration& C, const ObjectiveL& phi, const arr& q_real, const arr& qDot_real, double ctrlTime){
  msg.clear();
  msg <<"CYCLE ctrlTime:" <<ctrlTime; //<<' ' <<q;

  updateWaypoints(C);
  updateTiming(C, phi, ctrlTime, q_real, qDot_real);
}

rai::CubicSplineCtor SequenceController::getSpline(double realtime){
  if(timingMPC.done() || !pathMPC.feasible) return {arr{}, arr{}, arr{}};
  arr pts = timingMPC.getFlags();
  arr vels = timingMPC.getVels();
  arr times = timingMPC.getTimes();
  times -= realtime - ctrlTimeLast;
  return {pts, vels, times};
}

void SequenceController::report(const rai::Configuration& C, const rai::Array<ObjectiveL>& phiflag, const rai::Array<ObjectiveL>& phirun){
  msg <<" (fea) "
     <<phirun.elem(0).maxError(C, 0) <<' '
    <<phiflag.elem(0).maxError(C, 0) <<' '
   <<phirun.elem(1).maxError(C, 0) <<' '
  <<phiflag.elem(1).maxError(C, 0);
  cout <<msg <<endl;
}

void SequenceController::report(const rai::Configuration& C, const ObjectiveL& phi) {
  msg <<" (fea) " <<phi.maxError(C, 0.5+timingMPC.phase)
     <<' ' <<phi.maxError(C, 1.+timingMPC.phase)
    <<' ' <<phi.maxError(C, 1.5+timingMPC.phase)
   <<' ' <<phi.maxError(C, 2.+timingMPC.phase);
  cout <<msg <<endl;
}
