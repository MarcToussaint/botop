#include <Core/util.h>
#include <Core/thread.h>
#include "lcm/lcm-cpp.hpp"

void __lcm_publish(const char *message_name, char* data, uint size);
void __lcm_subscribe(const char *message_name, void (*callback)(const lcm::ReceiveBuffer*, const std::string&, Var_base*), Var_base* var);

template<class T>
void subscribeCallback(const lcm::ReceiveBuffer *rbuf, const std::string &channel, Var_base* var){
  Var_data<T>* x = dynamic_cast<Var_data<T>*>(var);
  CHECK(x, "");
  x->writeAccess();
  x->data.serial_decode((char*)rbuf->data, rbuf->data_size);
  x->deAccess();
}

template<class T>
void publishCallback(Var_base* var){
  Var_data<T>* x = dynamic_cast<Var_data<T>*>(var);
  CHECK(x, "");
  std::vector<char> buf(x->data.serial_size());
  x->data.serial_encode(buf.data(), buf.size());
  __lcm_publish(x->name.p, buf.data(), buf.size());
}

template<class T>
void lcm_publish(Var<T>& x, const char* message_name){
    x.data->name = message_name;
    x.addCallback(publishCallback<T>);
}

template<class T>
void lcm_subscribe(Var<T>& x, const char* message_name){
  __lcm_subscribe(message_name, subscribeCallback<T>, x.data.get());
}


