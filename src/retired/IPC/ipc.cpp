#include "ipc.h"

#include "lcm/lcm-cpp.hpp"
#include "lcm/lcm.h"

struct LCM_Interface : Thread{
  lcm::LCM lcm;

  LCM_Interface() : Thread("LCM_Spinner", -1.) { threadLoop(); }
  ~LCM_Interface() { threadClose(); }

  void step(){
      int ret = lcm.handle();
      CHECK(!ret, "LCM step failed");
  }

};

Singleton<LCM_Interface> LCM;

void __lcm_publish(const char* message_name, char* data, uint32_t data_size){
  int ret = LCM()->lcm.publish(message_name, data, data_size);
  if(ret) LOG(-1) <<"publishing '" <<message_name <<"' failed";
}

void __lcm_subscribe(const char* message_name,
                   void (*callback)(const lcm::ReceiveBuffer *rbuf, const std::string& channel, Var_base* var),
                   Var_base* var){
    LCM()->lcm.subscribeFunction(message_name, callback, var);
}
