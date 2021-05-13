#include <Core/thread.h>
#include <IPC/ipc.h>



struct SomeMsg : CompoundSerializable{
  arr x;
  arr y;
  SomeMsg() : CompoundSerializable({&x, &y}) {}
};

void sender(){
  Var<SomeMsg> x;
  lcm_publish<SomeMsg>(x, "X");

  for(uint k=0;k<10;k++){
    if(rnd.uni()<.5)
      x.set()->x = consts((double)k, k);
    else
      x.set()->y = consts((double)k, k, k/2);

    cout <<"published: x=" <<x.get()->x <<" y=" <<x.get()->y <<endl;
    rai::wait(1.);
    if(k==9) k=1;
  }
}

void listener(){
  Var<SomeMsg> x;
  lcm_subscribe(x, "X");

  while(true){
    x.waitForNextRevision();
    cout <<"received: x=" <<x.get()->x <<" y=" <<x.get()->y <<endl;
  }
}

int main(int argc, char **argv){
  if(argc<=1) listener();
  else sender();

  return 0;
}
