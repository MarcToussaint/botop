#include <Core/thread.h>

template<class T> void lcm_publish(Var<T>& x, const char* message_name);
template<class T> void lcm_subscribe(Var<T>& x, const char* message_name);

struct CompoundSerializable : Serializable{
  std::vector<Serializable*> objs;
  CompoundSerializable(std::initializer_list<Serializable*> _objs) : objs(_objs){}

  virtual uint serial_size(){
    uint s=0;
    for(auto* o:objs) s += o->serial_size();
    return s;
  }

  virtual uint serial_encode(char* data, uint data_size){
    uint s=0;
    for(auto* o:objs){
      CHECK_GE(data_size, s, "");
      s += o->serial_encode(data + s, data_size - s);
    }
    CHECK_GE(data_size, s, "");
    return s;
  }

  virtual uint serial_decode(char* data, uint data_size){
    uint s=0;
    for(auto* o:objs){
      CHECK_GE(data_size, s, "");
      s += o->serial_decode(data + s, data_size - s);
    }
    CHECK_GE(data_size, s, "");
    return s;
  }
};

#include "ipc.tpp"
