#pragma once

#include <Core/array.h>
#include <Core/util.h>

namespace rai {
//===========================================================================

struct SineSound{
  float sampleRate;
  floatA notes; //four entries per note: (sin-buffer-step-size, amplitude, time, decay)
  floatA SIN;
  Mutex mutex;

  SineSound(float _sampleRate=16000);

  void addNote(int noteRelToC, float a=.1, float decay=0.0007);
  void addFreq(float freq, float a=.1, float decay=0.0007);
  void changeFreq(uint i, float freq);
  void changeAmp(uint i, float amp);
  void reset();
  void clean();
  float get();
};

//===========================================================================

struct Audio{
  void *stream;
  Audio(SineSound& S);
  ~Audio();
};

//===========================================================================

struct Sound : SineSound{
  Audio A;
  Sound() : A(*this){}
};

extern Singleton<Sound> sound;

} //namespace
