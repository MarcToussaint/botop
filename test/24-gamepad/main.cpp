#include <Gamepad/gamepad.h>

void threadedRun() {

  GamepadInterface G;

  std::cout << "Gamepad initialized\n";
  for(;;){
    for (int i = 0; i < G.count; i++) {
      G.gamepadState[i].waitForNextRevision();
      std::cout << "______ Gamepad " << i << " ______" << std::endl;
      std::cout << G.getButtonPressed(i) <<std::endl;
    }
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
