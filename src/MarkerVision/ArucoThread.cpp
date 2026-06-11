#include "ArucoThread.h"

namespace rai{

ArucoThread::ArucoThread(uint k_id, Var<byteA>& _input)
    : Thread(STRING("aruco_thread_" <<k_id), -1.), input(_input), k_id(k_id) {
  finder.verbose=0;
  LOG(0) <<"launching aruco thread " <<k_id;
  threadLoop();
}

ArucoThread::~ArucoThread(){
  LOG(0) <<"shutting down aruco thread " <<k_id;
  threadClose();
}

void ArucoThread::step() {
  input_revision = input.waitForRevisionGreaterThan(input_revision);
  rgb = input.get();
  //do things...
  finder.find(rgb);
  // LOG(0) <<"aruco: found #" <<finder.ids.N <<" rev " <<input_revision <<" timer " <<timer.report();
  {
    auto set = output.set();
    uint N = finder.ids.N;
    set->resize(N);
    for(uint i=0;i<N;i++){
      PointView& p = set->elem(i);
      p.j = finder.ids(i);
      p.d = -1.;
      p.k = k_id;
      p.p = finder.pts(i,0,{});
    }
  }
}

} //rai
