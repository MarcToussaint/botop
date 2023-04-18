#include "audio.h"

namespace rai {

Singleton<Sound> sound;

SineSound::SineSound(float _sampleRate):sampleRate(_sampleRate){
  SIN.resize(1024);
  for(uint i=0;i<SIN.N;i++) SIN(i) = ::sin((RAI_2PI*i)/SIN.N);
}

void SineSound::addNote(int noteRelToC, float a, float decay){
  addFreq(440.*pow(2.,double(noteRelToC)/12.), a, decay);
}

void SineSound::addFreq(float freq, float a, float decay){
  floatA note = { float(SIN.N*freq/sampleRate), a, 0., decay };
  mutex.lock(RAI_HERE);
  notes.append( note );
  notes.reshape(notes.N/4, 4);
  mutex.unlock();
}

void SineSound::changeFreq(uint i, float freq){
  mutex.lock(RAI_HERE);
  notes(i,0) = float(SIN.N*freq/sampleRate);
  mutex.unlock();
}

void SineSound::changeAmp(uint i, float amp){
  mutex.lock(RAI_HERE);
  notes(i,1) = amp;
  mutex.unlock();
}

void SineSound::reset(){
  mutex.lock(RAI_HERE);
  notes.clear();
  mutex.unlock();
}

void SineSound::clean(){
  mutex.lock(RAI_HERE);
  for(uint i=notes.d0;i--;){
    if(notes(i,1)<1e-4) notes.delRows(i);
  }
  mutex.unlock();
}

float SineSound::get(){
  double x=0.;
  mutex.lock(RAI_HERE);
  for(uint i=0;i<notes.d0; i++){
    float &a=notes(i, 1);
    float &t=notes(i, 2);
    float &dt=notes(i, 0);
    float decay=notes(i,3);
    x += a * SIN(int(t)&0x3ff); //sin(t);
    t += dt;
    //if(a>0.05) a *= 1.-10.*decay; else
    a *= 1.-decay;
  }
  mutex.unlock();
  return x;
}

//===========================================================================

#ifdef RAI_PORTAUDIO

#include <portaudio.h>


void err(PaError e){
  if(!e) return;
  Pa_Terminate();
  HALT("PortAudio error" <<e <<": " <<Pa_GetErrorText( e ) );
}

static int PortAudioCallback( const void *inputBuffer, void *outputBuffer,
                            unsigned long framesPerBuffer,
                            const PaStreamCallbackTimeInfo* timeInfo,
                            PaStreamCallbackFlags statusFlags,
                            void *userData ){
  SineSound &S = *((SineSound*)userData);
  float *out = (float*)outputBuffer;
  unsigned long i;

  (void) timeInfo; /* Prevent unused variable warnings. */
  (void) statusFlags;
  (void) inputBuffer;

  S.clean();
  for( i=0; i<framesPerBuffer; i++ ) *out++ = S.get();

  return paContinue;
}



Audio::Audio(SineSound& S){
  err( Pa_Initialize() );

  PaStreamParameters outputParameters;
  outputParameters.device = Pa_GetDefaultOutputDevice(); /* default output device */
  if (outputParameters.device == paNoDevice) err(paNoDevice);
  outputParameters.channelCount = 1; /* stereo output */
  outputParameters.sampleFormat = paFloat32; /* 32 bit floating point output */
  outputParameters.suggestedLatency = Pa_GetDeviceInfo( outputParameters.device )->defaultLowOutputLatency;
  outputParameters.hostApiSpecificStreamInfo = NULL;

  err( Pa_OpenStream(
         &stream,
         NULL, /* no input */
         &outputParameters,
         S.sampleRate,
         64, //frames per buffer
         0,
         PortAudioCallback,
         &S ) );

  err( Pa_StartStream( stream ) );
}

Audio::~Audio(){
  err( Pa_StopStream( stream ) );
  err( Pa_CloseStream( stream ) );
  err( Pa_Terminate() );
}

#else //PORTAUDIO

Audio::Audio(SineSound& S){ NICO }
Audio::~Audio(){}

#endif

} //namespace
