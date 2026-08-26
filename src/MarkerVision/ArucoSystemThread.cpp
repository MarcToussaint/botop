#include "ArucoSystemThread.h"
#include <Kin/frame.h>

namespace rai{

ArucoSystemThread::ArucoSystemThread(uint J_numMarkers,
                     const rai::Array<Var<PointViewData>*>& ar_outputs,
				     arrA Fxycxy,
				     rai::Array<rai::Transformation> Pose)
    : Thread(STRING("aruco_system thread"), .05),
    inputs(ar_outputs),
    solver(J_numMarkers, inputs.N){
  X.set() = zeros(J_numMarkers, 3);
  uint K = inputs.N;
  for(uint k=0;k<K;k++){
    solver.setCamera(k, Fxycxy(k), Pose(k));
  }

  LOG(0) <<"launching aruco system thread";
  threadLoop();
}

ArucoSystemThread::~ArucoSystemThread(){
  LOG(0) <<"shutting down aruco system thread - " <<timer.report();
  threadClose();
}

void ArucoSystemThread::step() {
  uint K = inputs.N;
  solver.data.clear();
  solver.X.setZero();
  for(uint k=0;k<K;k++){
    solver.data.append(inputs(k)->get());
  }

  solver.subSelectObservedPoints();
  solver.solveColinearityForPoints();
  X.set() = solver.X;
}

void ArucoSystemThread::pull(Configuration& C){
  //-- update configuration
  if(!frames.N){
    frames.resize(solver.J);
    for(uint j=0;j<solver.J;j++){
      str name = STRING("ar_" <<j);
      rai::Frame *f = C.getFrame(name, false);
      if(!f){//create a new marker frame!
        f = C.addFrame(name);
        f->setShape(rai::ST_sphere, {.01});
        f->setColor({1., 1., 0., .9});
      }
      frames(j) = f;
    }
  }

  arr Xcopy = X.get();
  for(uint j=0;j<solver.J;j++){
    frames(j)->setPosition(Xcopy[j]);
  }
}

} //rai
