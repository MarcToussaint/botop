#include "WaypointMPC.h"

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
  msg.clear() <<"WAY it " <<steps <<" feasible: " <<(feasible?" good":" FAIL") <<" -- queries: " <<komo.pathConfig.setJointStateCount <<" time:" <<komo.timeTotal <<"\t sos:" <<komo.sos <<"\t ineq:" <<komo.ineq <<"\t eq:" <<komo.eq <<endl;

  komo.view(false, msg);

  path = komo.getPath_qOrg();
  tau = komo.getPath_tau();

  if(!feasible){ // || komo.pathConfig.setJointStateCount>50
    cout <<komo.getReport(false);
    komo.reset();
    komo.initWithConstant(qHome);
  }
}
