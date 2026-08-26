#include "ArucoThread.h"

namespace rai{

ArucoThread::ArucoThread(uint k_id, Var<byteA>& _input, double beatIntervalSec)
    : Thread(STRING("aruco_thread_" <<k_id), beatIntervalSec), input(_input), cam_id(k_id) {
  finder.verbose=0;
  LOG(0) <<"launching aruco thread " <<k_id;
  // status.listenTo(input);
  // threadOpen();
  threadLoop();
}

ArucoThread::~ArucoThread(){
  LOG(0) <<"shutting down aruco thread " <<cam_id <<" - " <<timer.report();
  threadClose();
}

void ArucoThread::step() {
  rgb = input.get()();
  if(!rgb.N) return;
  finder.find(rgb);
  // LOG(0) <<"aruco " <<cam_id <<" found #" <<finder.ids.N <<" points in image rev " <<input_revision;
  {
    auto set = output.set();
    set->cam_id = cam_id;
    set->ids = finder.ids;
    set->pts = finder.pts;
  }
  timer.tic(1);
}

} //rai
