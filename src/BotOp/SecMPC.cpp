#include "SecMPC.h"

#include <Optim/NLP_Solver.h>
#include <KOMO/pathTools.h>
#include <Kin/F_qFeatures.h>

#include <iomanip>

//===========================================================================

SecMPC::SecMPC(KOMO& komo, int subSeqStart, int subSeqStop, double timeCost, double ctrlCost, bool _setNextWaypointTangent)
  : waypointMPC(komo),
    timingMPC(waypointMPC.path({subSeqStart,subSeqStop}), timeCost, ctrlCost),
    shortMPC(komo.world, 5, .1),
    subSeqStart(subSeqStart), subSeqStop(subSeqStop), setNextWaypointTangent(_setNextWaypointTangent){

  StringA colls = {"l_palm", "l_finger1", "l_finger2", "l_panda_coll7b", "l_panda_coll7", "l_panda_coll6", "l_panda_coll5", "l_panda_coll4", "l_panda_coll3"};
  if(komo.world["stick"]) colls.append("stick");
  for(auto& s:colls){
    shortMPC.komo.addObjective({}, FS_distance, {"obst", s}, OT_ineqP, {5.}, {-.1});
  }

  for(uint t=0;t<shortMPC.komo.T;t++){
    shortMPC.komo.addObjective({0.}, FS_qItself, {}, OT_sos, {1e1}, waypointMPC.qHome, 0, t+1, t+1);
  }

  if(setNextWaypointTangent) timingMPC.update_waypoints(timingMPC.waypoints, true);
}

void SecMPC::updateWaypoints(const rai::Configuration& C){
  waypointMPC.reinit(C); //adopt all frames in C as prefix (also positions of objects)
  waypointMPC.solve(verbose-2);

  msg <<" WAY #:" <<waypointMPC.komo.pathConfig.setJointStateCount;
  msg <<' ' <<waypointMPC.komo.sos <<'|' <<waypointMPC.komo.ineq <<'|' <<waypointMPC.komo.eq;
}

void SecMPC::updateTiming(const rai::Configuration& C, const ObjectiveL& phi, const arr& q_real){
  //-- adopt the new path
  timingMPC.update_waypoints(waypointMPC.path({subSeqStart, subSeqStop}), setNextWaypointTangent);

  //-- progress time (potentially phase)
  if(!timingMPC.done() && ctrlTimeDelta>0.){
    phaseSwitch = timingMPC.update_progressTime(ctrlTimeDelta);
  }else{
    phaseSwitch = false;
  }

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
      double ctrlErr = length(q_real-q_ref_atLastUpdate);
      double thresh = .02;
      //cout <<"err: " <<err <<"  \t" <<flush;
      if(ctrlErr>thresh){
        //LOG(0) <<"ERROR MODE: " <<ctrlErr <<endl;
        q_refAdapted = q_ref_atLastUpdate + ((ctrlErr-thresh)/ctrlErr) * (q_real-q_ref_atLastUpdate);
        ret = timingMPC.solve(q_refAdapted, qDot_ref_atLastUpdate, verbose-2);
      }else{
        q_refAdapted.clear();
        q_refAdapted = q_ref_atLastUpdate;
        ret = timingMPC.solve(q_ref_atLastUpdate, qDot_ref_atLastUpdate, verbose-2);
      }
      msg <<" \tTIMING #:" <<ret->evals <<" phase: " <<timingMPC.phase ;
      //      msg <<" T:" <<ret->time <<" f:" <<ret->f;
    }else{
      msg <<" \tTIMING #skipped" <<" phase: " <<timingMPC.phase ;
//      LOG(0) <<"skipping timing opt, as too close ahead: " <<timingMPC.tau;
    }
  }

  msg <<" tau: " <<timingMPC.tau;
//  msg <<ctrlTime_atLastUpdate + timingMPC.getTimes(); // <<' ' <<F.vels;
}

void SecMPC::updateShortPath(const rai::Configuration& C){
  shortMPC.reinit(C); //adopt all frames in C as prefix (also positions of objects)
  shortMPC.reinit(q_refAdapted, qDot_ref_atLastUpdate);
  rai::CubicSpline S;
#if 0
  timingMPC.getCubicSpline(S, q_ref, qDot_ref);
  if(!S.pieces.N) return;
#else
//  timingMPC.getCubicSpline(S, q_ref_atLastUpdate, qDot_ref_atLastUpdate);
  auto sp = getSpline(ctrlTime_atLastUpdate, true);
  if(!sp.pts.N){ shortMPC.feasible=false; return; }
  S.set(sp.pts, sp.vels, sp.times);
#endif
  arr times = shortMPC.komo.getPath_times();
  arr pts = S.eval(times);
  CHECK_EQ(times.N, shortMPC.komo.T, "");
  CHECK_EQ(pts.d0, shortMPC.komo.T, "");
  for(int t=0;t<(int)pts.d0;t++){
    shortMPC.komo.setConfiguration_qOrg(t, q_refAdapted); //pts[t]);
    std::shared_ptr<GroundedObjective> ob = shortMPC.komo.objs.elem(t - (int)pts.d0);
    ob->feat->setTarget(pts[t]);
//    cout <<off <<' ' <<t <<' ' <<ob->feat->shortTag(C) <<ob->feat->scale <<ob->feat->target <<ob->timeSlices <<endl;
  }
  shortMPC.komo.run_prepare(0.);
  //shortMPC.reinit_taus(times(0));

  shortMPC.solve(false, verbose-2);

//  shortMPC.komo.view(false, "SHORT");
//  shortMPC.feasible = true;
//  shortMPC.times = shortMPC.komo.getPath_times(); //grid(1, .0, .5, 10).reshape(-1);
//  shortMPC.times.prepend(0);
//  shortMPC.path = S.eval(shortMPC.times);
//  shortMPC.vels = S.eval(shortMPC.times, 1);

//  cout <<init - shortMPC.path <<endl;
//  cout <<shortMPC.komo.getConfiguration_qOrg(-2) <<endl;
//  cout <<shortMPC.komo.getConfiguration_qOrg(-1) <<endl;
//  cout <<shortMPC.komo.getConfiguration_qOrg(-0) <<endl;
//  cout <<q_ref <<endl <<qDot_ref <<endl;
//  cout <<init <<endl <<shortMPC.path <<endl;
//  rai::wait();
//  shortMPC.komo.reportProblem();

  msg <<" \tPATH #:" <<shortMPC.komo.pathConfig.setJointStateCount;
  msg <<' ' <<shortMPC.komo.sos <<'|' <<shortMPC.komo.ineq <<'|' <<shortMPC.komo.eq;
}

void SecMPC::cycle(const rai::Configuration& C, const arr& q_ref, const arr& qDot_ref, const arr& q_real, const arr& qDot_real, double ctrlTime){
  msg.clear();
  msg <<std::setprecision(6);
  msg <<"CYCLE ctrlTime:" <<ctrlTime;
  msg <<std::setprecision(3);

  //-- store ctrl state at start of this cycle
  if(ctrlTime_atLastUpdate>0.){
    ctrlTimeDelta = ctrlTime - ctrlTime_atLastUpdate;
  }
  ctrlTime_atLastUpdate = ctrlTime;
  q_ref_atLastUpdate = q_ref;
  qDot_ref_atLastUpdate = qDot_ref;

  updateWaypoints(C);
  updateTiming(C, waypointMPC.komo.objectives, q_real);
  updateShortPath(C);
}

rai::CubicSplineCtor SecMPC::getSpline(double realtime, bool prependRef){
  if(timingMPC.done() || !waypointMPC.feasible) return {};
  arr pts = timingMPC.getWaypoints();
  arr vels = timingMPC.getVels();
  arr times = timingMPC.getTimes();
  times -= realtime - ctrlTime_atLastUpdate; //ctrlTimeLast=when the timing was optimized; realtime=time now; -> shift spline to stich it at realtime
  if(times.first()<tauCutoff) return {};
  if(q_refAdapted.N){ //this will overrideHard the spline, as first time is negative;
    pts.prepend(q_refAdapted);
    vels.prepend(qDot_ref_atLastUpdate);
    times.prepend(0. - (realtime - ctrlTime_atLastUpdate));
  }else if(prependRef){
    pts.prepend(q_ref_atLastUpdate);
    vels.prepend(qDot_ref_atLastUpdate);
    times.prepend(0. - (realtime - ctrlTime_atLastUpdate));
  }
  return {pts, vels, times};
}

rai::CubicSplineCtor SecMPC::getShortPath_debug(double realtime){
  if(timingMPC.done() || !waypointMPC.feasible) return {};

  rai::CubicSpline S;
//  timingMPC.getCubicSpline(S, q_ref_atLastUpdate, qDot_ref_atLastUpdate);
  auto sp = getSpline(ctrlTime_atLastUpdate, true);
  if(!sp.pts.N) return {};
  S.set(sp.pts, sp.vels, sp.times);

#if 1
  arr times = grid(1, .0, .5, 10).reshape(-1);
  arr pts = S.eval(times);
  arr vels = S.eval(times, 1);
  vels.clear();
#else
  arr times = sp.times, pts=sp.pts, vels=sp.vels;
#endif

  times -= realtime - ctrlTime_atLastUpdate; //ctrlTimeLast=when the timing was optimized; realtime=time now; -> shift spline to stich it at realtime
//  while(times.N && times.first()<tauCutoff){
//    times.remove(0);
//    pts.delRows(0);
//    vels.delRows(0);
//  }
  return {pts, vels, times};
}

rai::CubicSplineCtor SecMPC::getShortPath(double realtime){
  if(timingMPC.done() || !waypointMPC.feasible || !shortMPC.feasible){ return {}; }
  arr times = shortMPC.times; //komo.getPath_times();
  arr pts = shortMPC.path;
  arr vels = shortMPC.vels;
  if(!pts.N) return {};

  times -= realtime - ctrlTime_atLastUpdate; //ctrlTimeLast=when the timing was optimized; realtime=time now; -> shift spline to stich it at realtime
//  while(times.N && times.first()<tauCutoff){
//    times.remove(0);
//    pts.delRows(0);
//    if(vels.N) vels.delRows(0);
//  }
  return {pts, vels, times};
}


void SecMPC::report(const rai::Configuration& C) {
  const ObjectiveL& phi = waypointMPC.komo.objectives;
  msg <<" \tFEA " <<phi.maxError(C, 0.5+timingMPC.phase)
     <<' ' <<phi.maxError(C, 1.+timingMPC.phase)
    <<' ' <<phi.maxError(C, 1.5+timingMPC.phase)
   <<' ' <<phi.maxError(C, 2.+timingMPC.phase);
  cout <<msg <<endl;
}
