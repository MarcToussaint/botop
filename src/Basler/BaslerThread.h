#pragma once

#include <Core/array.h>
#include <Core/thread.h>

namespace rai {

struct BaslerThread : Thread {
  struct sBaslerThread *s=0;

  rai::Array<Var<byteA>> color;

  BaslerThread(uint nCams);
  ~BaslerThread();

  void open();
  void close();
  void step();
};

}
