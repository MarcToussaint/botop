#include <Gamepad/gamepad.h>

void threadedRun() {

  GamepadInterface G;

  for(;;){
    G.gamepadState.waitForNextRevision();
    cout <<"\r" <<G.gamepadState.get()() <<std::flush;
    if(G.quitSignal.get()) break;
  }

  cout <<"bye bye" <<endl;
}

//void rawTest(){
//  KinectThread kin;
//  kin.open();
//  kin.close();
//}

int main(int argc,char **argv){
  //  rawTest();
  threadedRun();
  return 0;
};
