#include "SecMPC.h"

#include <Optim/MP_Solver.h>

WaypointMPC::WaypointMPC(KOMO& _komo, const arr& _qHome)
  : komo(_komo), qHome(_qHome){

  if(!qHome.N) qHome=_komo.world.getJointState();

//  komo.reset();
//  komo.initWithConstant(qHome);
  path = komo.getPath_qOrg();
  this->tau = komo.getPath_tau();
}

void WaypointMPC::reinit(const rai::Configuration& C){
  komo.updateRootObjects(C);
}

void WaypointMPC::solve(){
  steps++;

  //re-run KOMO
  rai::OptOptions opt;
  opt.stopTolerance = 1e-3;
  opt.stopGTolerance = 1e-3;
  komo.opt.verbose=0;
  komo.timeTotal=0.;
  komo.pathConfig.setJointStateCount=0;
  //komo.reportProblem();
//  komo.initWithConstant(qHome);
  komo.optimize(.0, opt);
  //komo.checkGradients();

  //is feasible?
  feasible=komo.sos<50. && komo.ineq<.1 && komo.eq<.1;
  rai::String msg;
  msg <<"it " <<steps <<" feasible: " <<(feasible?" good":" FAIL") <<" -- queries: " <<komo.pathConfig.setJointStateCount <<" time:" <<komo.timeTotal <<"\t sos:" <<komo.sos <<"\t ineq:" <<komo.ineq <<"\t eq:" <<komo.eq <<endl;

  komo.view(false, msg);

  path = komo.getPath_qOrg();
  tau = komo.getPath_tau();

  if(!feasible){ // || komo.pathConfig.setJointStateCount>50){
    komo.reset();
    komo.initWithConstant(qHome);
  }

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

SecMPC::SecMPC(KOMO& komo, int subSeqStart, int subSeqStop, double timeCost, double ctrlCost)
  : pathMPC(komo),
    timingMPC(pathMPC.path({subSeqStart,subSeqStop}), timeCost, ctrlCost),
    subSeqStart(subSeqStart), subSeqStop(subSeqStop){

  uint K = timingMPC.waypoints.d0;

  timingMPC.tangents = zeros(K-1, timingMPC.waypoints.d1);
  for(uint k=1; k<K; k++){
    timingMPC.tangents[k-1] = timingMPC.waypoints[k] - timingMPC.waypoints[k-1];
    op_normalize(timingMPC.tangents[k-1]());
  }
}

void SecMPC::updateWaypoints(const rai::Configuration& C){
  pathMPC.reinit(C); //adopt all frames in C as prefix (also positions of objects)
  pathMPC.solve();

  msg <<" (path) #:" <<pathMPC.komo.pathConfig.setJointStateCount;
  //      msg <<" T:" <<pathProblem.komo.timeTotal <<" f:" <<pathProblem.komo.sos <<" eq:" <<pathProblem.komo.eq <<" iq:" <<pathProblem.komo.ineq;
}

void SecMPC::updateTiming(const rai::Configuration& C, const ObjectiveL& phi, double ctrlTime, const arr& q_real, const arr& qDot_real, const arr& q_ref, const arr& qDot_ref){
  //-- adopt the new path
  timingMPC.update_waypoints(pathMPC.path({subSeqStart, subSeqStop}));

  //-- progress time (potentially phase)
  if(!timingMPC.done() && ctrlTimeLast>0.) timingMPC.update_progressTime(ctrlTime - ctrlTimeLast);
  ctrlTimeLast = ctrlTime;

  //-- phase backtracking
  if(timingMPC.done()){
    if(phi.maxError(C, timingMPC.phase) > precision){
      phi.maxError(C, timingMPC.phase, 1); //verbose
      timingMPC.update_backtrack();
    }
  }
  if(!timingMPC.done()){
    while(phi.maxError(C, 0.5+timingMPC.phase) > precision){ //OR while?
      phi.maxError(C, 0.5+timingMPC.phase, 1); //verbose
      timingMPC.update_backtrack();
    }
  }

  //-- solve the timing problem
  if(!timingMPC.done()){
    if(timingMPC.tau(timingMPC.phase) > tauCutoff){
      //auto ret = timingMPC.solve(q_real, qDot_real, 0);
      auto ret = timingMPC.solve(q_ref, qDot_ref, 0);
      msg <<" (timing) ph:" <<timingMPC.phase <<" #:" <<ret->evals;
      //      msg <<" T:" <<ret->time <<" f:" <<ret->f;
    }else{
      LOG(0) <<"skipping timing opt, as too close ahead: " <<timingMPC.tau;
    }
  }

  msg <<" tau: " <<timingMPC.tau << ctrlTime + timingMPC.getTimes(); // <<' ' <<F.vels;
}

void SecMPC::cycle(const rai::Configuration& C, const arr& q_ref, const arr& qDot_ref, const arr& q_real, const arr& qDot_real, double ctrlTime){
  msg.clear();
  msg <<"CYCLE ctrlTime:" <<ctrlTime; //<<' ' <<q;

  updateWaypoints(C);
  updateTiming(C, pathMPC.komo.objectives, ctrlTime, q_real, qDot_real, q_ref, qDot_ref);
}

rai::CubicSplineCtor SecMPC::getSpline(double realtime){
  if(timingMPC.done() || !pathMPC.feasible) return {};
  arr pts = timingMPC.getWaypoints();
  arr vels = timingMPC.getVels();
  arr times = timingMPC.getTimes();
  times -= realtime - ctrlTimeLast;
  if(times.first()<tauCutoff) return {};
  return {pts, vels, times};
}

void SecMPC::report(const rai::Configuration& C) {
  const ObjectiveL& phi = pathMPC.komo.objectives;
  msg <<" (fea) " <<phi.maxError(C, 0.5+timingMPC.phase)
     <<' ' <<phi.maxError(C, 1.+timingMPC.phase)
    <<' ' <<phi.maxError(C, 1.5+timingMPC.phase)
   <<' ' <<phi.maxError(C, 2.+timingMPC.phase);
  cout <<msg <<endl;
}
