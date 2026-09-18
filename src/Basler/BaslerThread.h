#pragma once

#include <Core/array.h>
#include <Core/thread.h>

namespace rai {

struct BaslerThread : Thread {
  struct sBaslerThread *s=0;

  rai::Array<Var<byteA>> color;

  BaslerThread(const strA& ids, const rai::Array<std::shared_ptr<Graph>>& ats = {});
  ~BaslerThread();

  void open();
  void close();
  void step();
};

}
