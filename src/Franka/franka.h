#pragma once

#include <Core/array.h>
#include <Core/thread.h>

struct FrankaControlMessage{
  arr qref;

};
struct FrankaFeedbackMessage{
  arr qreal, qdotreal;
};

struct FrankaThread : Thread{
  Var<FrankaControlMessage> ctrl;
  Var<FrankaFeedbackMessage> state;
  bool stop=false;

  FrankaThread(Var<FrankaControlMessage>& _ctrl, Var<FrankaFeedbackMessage>& _state);
  ~FrankaThread();

  void open(){}
  void step();
  void close(){}
};
