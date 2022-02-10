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
//  komo.setConfiguration_qOrg(-1, C.getJointState());
}

void WaypointMPC::solve(){
  steps++;

  //re-run KOMO
  rai::OptOptions opt;
  opt.stopTolerance = 1e-3;
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

  if(!feasible){ // || komo.pathConfig.setJointStateCount>50
    cout <<komo.getReport(false);
    komo.reset();
    komo.initWithConstant(qHome);
  }
}

//===========================================================================

SecMPC::SecMPC(KOMO& komo, int subSeqStart, int subSeqStop, double timeCost, double ctrlCost, bool _setNextWaypointTangent)
  : pathMPC(komo),
    timingMPC(pathMPC.path({subSeqStart,subSeqStop}), timeCost, ctrlCost),
    subSeqStart(subSeqStart), subSeqStop(subSeqStop), setNextWaypointTangent(_setNextWaypointTangent){

  if(setNextWaypointTangent) timingMPC.update_waypoints(timingMPC.waypoints, true);
}

void SecMPC::updateWaypoints(const rai::Configuration& C){
  pathMPC.reinit(C); //adopt all frames in C as prefix (also positions of objects)
  pathMPC.solve();

  msg <<" (path) #:" <<pathMPC.komo.pathConfig.setJointStateCount;
  //      msg <<" T:" <<pathProblem.komo.timeTotal <<" f:" <<pathProblem.komo.sos <<" eq:" <<pathProblem.komo.eq <<" iq:" <<pathProblem.komo.ineq;
}

void SecMPC::updateTiming(const rai::Configuration& C, const ObjectiveL& phi, double ctrlTime, const arr& q_real, const arr& qDot_real, const arr& q_ref, const arr& qDot_ref){
  //-- adopt the new path
  timingMPC.update_waypoints(pathMPC.path({subSeqStart, subSeqStop}), setNextWaypointTangent);

  //-- progress time (potentially phase)
  if(!timingMPC.done() && ctrlTimeLastUpdate>0.){
    phaseSwitch = timingMPC.update_progressTime(ctrlTime - ctrlTimeLastUpdate);
  }else{
    phaseSwitch = false;
  }
  ctrlTimeLastUpdate = ctrlTime;

  //-- phase backtracking
  if(timingMPC.done()){
    if(phi.maxError(C, timingMPC.phase+subSeqStart) > precision){
      phi.maxError(C, timingMPC.phase+subSeqStart, 1); //verbose
      timingMPC.update_backtrack();
      phaseSwitch=true;
    }
  }
  if(!timingMPC.done()){
    while(timingMPC.phase>0 && phi.maxError(C, 0.5+timingMPC.phase+subSeqStart) > precision){ //OR while?
      phi.maxError(C, 0.5+timingMPC.phase+subSeqStart, 1); //verbose
      timingMPC.update_backtrack();
      phaseSwitch=true;
    }
  }

  //-- re-optimize the timing
  if(!timingMPC.done()){
    if(timingMPC.tau(timingMPC.phase) > tauCutoff){
      shared_ptr<SolverReturn> ret;
      double ctrlErr = length(q_real-q_ref);
      double thresh = .02;
      //cout <<"err: " <<err <<"  \t" <<flush;
      if(ctrlErr>thresh){ //LOG(0) <<"ERROR MODE" <<endl;
        q_refAdapted = q_ref + ((ctrlErr-thresh)/ctrlErr) * (q_real-q_ref);
        qDot_refAdapted = qDot_ref;
        ret = timingMPC.solve(q_refAdapted, qDot_ref, 0);
      }else{
        q_refAdapted.clear();
        qDot_refAdapted.clear();
        ret = timingMPC.solve(q_ref, qDot_ref, 0);
      }
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
  times -= realtime - ctrlTimeLastUpdate; //ctrlTimeLast=when the timing was optimized; realtime=time now; -> shift spline to stich it at realtime
  if(times.first()<tauCutoff) return {};
  if(q_refAdapted.N){ //this will overrideHard the spline, as first time is negative;
    pts.prepend(q_refAdapted);
    vels.prepend(qDot_refAdapted);
    times.prepend(0. - (realtime - ctrlTimeLastUpdate));
  }
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
