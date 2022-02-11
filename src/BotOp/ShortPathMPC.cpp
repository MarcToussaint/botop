#include "ShortPathMPC.h"

ShortPathMPC::ShortPathMPC(rai::Configuration& C, double timingScale){
  qHome = C.getJointState();

  komo.setModel(C, true);
#if 0
  komo.setTiming(2., 1, .1, 2);

  //control costs at short horizon
  komo.add_qControlObjective({1}, 2, 1e0);

  //leap costs for the leap
  komo.addObjective({2}, make_shared<CubicSplineLeapCost>(C.getCtrlFramesAndScale()), {}, OT_sos, {1.}, NoArr, 2);

  //add a time joint for just the last slice
  {
    rai::Joint* jt = new rai::Joint(*komo.timeSlices(-1,0), rai::JT_tau);
    jt->H = 0.;
    //timing cost
    komo.addObjective({2}, make_shared<F_qTime>(), {"world"}, OT_f, {timingScale}, {});
    komo.addObjective({2}, make_shared<F_qTime>(), {"world"}, OT_ineq, {-1e1}, {.1}); //lower bound on timing
  }
  komo.reportProblem();

  komo.timeSlices(-1,0)->setJointState({100.}); //this should be the tau joint!
#else
  komo.setTiming(1., 5, 1., 2);

  //control costs at short horizon
  //komo.add_qControlObjective({}, 1, 1e-1);
  komo.add_jointLimits();
  komo.add_qControlObjective({}, 2, 1);
  komo.add_qControlObjective({}, 0, 1e0);
  //komo.add_collision();
  komo.reportProblem();
#endif


  //MISSING: THE TASK COSTS..
}

void ShortPathMPC::reinit(const arr& x, const arr& v){
  //set the prefix to init:
  komo.setConfiguration_qOrg(-1, x);
  komo.setConfiguration_qOrg(-2, x - komo.tau*v);
  //initialize x(0) also to current
  komo.setConfiguration_qOrg(0, x);
  //leave the leap configuration as is...
//  komo.timeSlices(-1,0)->setJointState({100.}); //this should be the tau joint!
}

void ShortPathMPC::reinit(const rai::Configuration& C){
  //shifts only prefix, not the whole trajectory! (would not make sense for x(H) \gets x(T) )
//  komo.updateAndShiftPrefix(C);
  komo.updateRootObjects(C);
}

void ShortPathMPC::solve(){
  steps++;

  //re-run KOMO
  rai::OptOptions opt;
  opt.stopTolerance = 1e-4;
  opt.stopGTolerance = 1e-4;
  komo.opt.verbose=0;
  komo.timeTotal=0.;
  komo.pathConfig.setJointStateCount=0;
  komo.initWithConstant(qHome);
  komo.optimize(0., opt);
  //komo.checkGradients();

  //is feasible?
  feasible=komo.sos<50. && komo.ineq<.1 && komo.eq<.1;
  msg.clear() <<"it " <<steps <<" feasible: " <<(feasible?" good":" FAIL") <<" -- queries: " <<komo.pathConfig.setJointStateCount <<" time:" <<komo.timeTotal <<"\t sos:" <<komo.sos <<"\t ineq:" <<komo.ineq <<"\t eq:" <<komo.eq <<endl;

  komo.view(false, msg);

  //store as output result
  if(feasible){
    path = komo.getPath_qOrg();
    tau = komo.getPath_tau();
  }else{
    cout <<komo.getReport(false);
    komo.reset();
    komo.initWithConstant(qHome);
  }
}
