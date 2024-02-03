#include "SimplexMotion-com.h"

#include <cstdint>
#include <iostream>
#include <string.h>

#define S1(x) #x
#define S2(x) S1(x)
#define __FILENAME__ (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)
#define LOG cout <<"# " <<__FILENAME__ <<":" <<S2(__LINE__) <<": "

using std::cerr;
using std::cout;
using std::endl;

void SimplexMotion_Communication::writeRegister(int regNumber, RegType regType, int value){
  int buflen=-1;
  if(regType==uns16){
    uint16_t data = value;
    buf[0] = 0x21; //transfer mode ('report number')
    buf[4] = data; //LOWEST BYTE FIRST!!! &0xff is implicit
    buf[5] = data>>8;
    buflen=6;
  }else if(regType==int16){
    int16_t data = value;
    buf[0] = 0x21;
    buf[4] = data;
    buf[5] = data>>8;
    buflen=6;
  }else if(regType==uns32){
    uint32_t data = value;
    buf[0] = 0x22;
    buf[4] = data;
    buf[5] = data>>8;
    buf[6] = data>>16;
    buf[7] = data>>24;
    buflen=8;
  }else if(regType==int32){
    int32_t data = value;
    buf[0] = 0x22;
    buf[4] = data;
    buf[5] = data>>8;
    buf[6] = data>>16;
    buf[7] = data>>24;
    buflen=8;
  }else{
    cerr <<"can't write string" <<endl; return;
  }
  buf[1] = 1;
  buf[2] = regNumber&0xff;
  buf[3] = regNumber>>8;

  writeBuf(buflen);
}

int SimplexMotion_Communication::readRegister(int regNumber, RegType regType){
  int readlen=-1;
  if(regType==uns16 || regType==int16){
    buf[0] = 0x11; //transfer mode ('report number')
    readlen=2;
  }else if(regType==uns32 || regType==int32){
    buf[0] = 0x12;
    readlen=4;
  }else{
    cerr <<"can't read string" <<endl;
    return 0;
  }
  buf[1] = 1;
  buf[2] = regNumber&0xff;
  buf[3] = regNumber>>8;

  if(!writeBuf(4)) return 0;

  if(!readBuf(readlen)) return 0;

  if(regType==uns16){
    uint16_t data;
    data = buf[1];
    data = (data<<8) | buf[0];
    return data;
  }else if(regType==int16){
    int16_t data;
    data = buf[1];
    data = (data<<8) | buf[0];
    return data;
  }else if(regType==uns32){
    uint32_t data;
    data = buf[3];
    data = (data<<8) | buf[2];
    data = (data<<8) | buf[1];
    data = (data<<8) | buf[0];
    return data;
  }else if(regType==int32){
    int32_t data;
    data = buf[3];
    data = (data<<8) | buf[2];
    data = (data<<8) | buf[1];
    data = (data<<8) | buf[0];
    return data;
  }
  return 0;
}

const char* SimplexMotion_Communication::readString(int regNumber, int n){
  int numReg = n/2 + (n&1);
  buf[0] = 0x11;
  buf[1] = numReg;
  buf[2] = regNumber&0xff;
  buf[3] = regNumber>>8;

  if(!writeBuf(4)) return 0;

  if(!readBuf(2*numReg)) return 0;

  buf[n]=0;
  return (char*)buf;
}

#ifdef USE_HIDAPI

//===========================================================================
//
// hidapi implementation
//

#include <hidapi/hidapi.h>
#include <mutex>

struct HidapiSingleton{
  std::mutex M;

  HidapiSingleton(){
    int res = hid_init();
    if(res<0){
      LOG <<"hid init error" <<std::endl;
      exit(0);
    }
  }

  ~HidapiSingleton(){
    // Finalize the hidapi library
    hid_exit();
  }
};

HidapiSingleton& getHidapiSingleton(){
  static HidapiSingleton hidapi;
  return hidapi;
}


struct SimplexMotion_Communication_Self{
  hid_device *handle;
};

SimplexMotion_Communication::SimplexMotion_Communication(const char* devPath, unsigned short vendor_id, unsigned short product_id){
  self = new SimplexMotion_Communication_Self;

  getHidapiSingleton().M.lock();

  LOG <<"opening device " <<devPath <<"..." <<std::endl;

  // Open the device using the VID, PID,
  // and optionally the Serial number.
  if(devPath){
    self->handle = hid_open_path(devPath);
  }else{
    self->handle = hid_open(vendor_id, product_id, NULL);
  }
  if(!self->handle){
    LOG <<"hid open error" <<std::endl;
    exit(0);
  }

  // Read the Manufacturer String
  wchar_t wstr[64];
  int res = hid_get_manufacturer_string(self->handle, wstr, 64);
  if(res<0) LOG <<"error: " <<hid_error(self->handle) <<std::endl;
  wcstombs((char*)buf, wstr, 64);
  LOG <<"Manufacturer String: '" <<buf <<"'" <<std::endl;

  // Read the Product String
  res = hid_get_product_string(self->handle, wstr, 64);
  if(res<0) LOG <<"error: " <<hid_error(self->handle) <<std::endl;
  wcstombs((char*)buf, wstr, 64);
  LOG <<"Product String: '" <<buf <<"'" <<std::endl;

  // Read the Serial Number String
  res = hid_get_serial_number_string(self->handle, wstr, 64);
  if(res<0) LOG <<"error: " <<hid_error(self->handle) <<std::endl;
  wcstombs((char*)buf, wstr, 64);
  LOG <<"Serial Number: '" <<buf <<"'" <<std::endl;

  // Read Indexed String 1
//  res = hid_get_indexed_string(self->handle, 1, wstr, 64);
//  if(res<0) LOG  <<"error: " <<hid_error(self->handle);
//  LOG <<"Indexed String: '" <<wstr <<"'" <<std::endl;

  LOG <<"...done opening device " <<devPath <<std::endl;

  getHidapiSingleton().M.unlock();

}

SimplexMotion_Communication::~SimplexMotion_Communication(){
  getHidapiSingleton().M.lock();
  hid_close(self->handle);
  getHidapiSingleton().M.unlock();

  delete self;
}

bool SimplexMotion_Communication::writeBuf(int len){
  int res = hid_write(self->handle, buf, len);

  if(res!=len){
    LOG <<"write error: " <<errno <<" written bytes:" <<res <<" wanted bytes:" <<len <<std::endl;
    return false;
  }
  return true;
}

bool SimplexMotion_Communication::readBuf(int len){
//  int res = hid_read(self->handle, buf, len);
  int res = hid_read_timeout(self->handle, buf, len, 100);

  if(res!=len){
    LOG <<"read error: " <<errno <<" read bytes:" <<res <<" wanted bytes:" <<len <<std::endl;
    return false;
  }
  return true;
}

#elif defined USE_HIDRAW

//===========================================================================
//
// hidraw implementation
//

#include <linux/types.h>
#include <linux/input.h>
#include <linux/hidraw.h>

#ifndef HIDIOCSFEATURE
#warning Please have your distro update the userspace kernel headers
#define HIDIOCSFEATURE(len)    _IOC(_IOC_WRITE|_IOC_READ, 'H', 0x06, len)
#define HIDIOCGFEATURE(len)    _IOC(_IOC_WRITE|_IOC_READ, 'H', 0x07, len)
#endif

#include <fcntl.h>
#include <unistd.h>

#include <string.h>
#include <errno.h>
#include <iostream>

#define S1(x) #x
#define S2(x) S1(x)
#define __FILENAME__ (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)
#define LOG cout <<"# " <<__FILENAME__ <<":" <<S2(__LINE__) <<": "

using std::cerr;
using std::cout;
using std::endl;

struct SimplexMotion_Communication_Self{
  int fd;
};

SimplexMotion_Communication::SimplexMotion_Communication(const char* devPath,unsigned short vendor_id, unsigned short product_id){
  self = new SimplexMotion_Communication_Self;

  self->fd = open(devPath, O_RDWR); //|O_NONBLOCK);

  if(self->fd<0) {
    cerr <<"Unable to open device " <<devPath <<endl;
    return;
  }

  hidraw_report_descriptor rpt_desc;
  hidraw_devinfo info;
  memset(&rpt_desc, 0x0, sizeof(rpt_desc));
  memset(&info, 0x0, sizeof(info));
  memset(buf, 0x0, sizeof(buf));

#if 0
  /* Get Report Descriptor Size */
  int desc_size = 0;
  int res = ioctl(self->fd, HIDIOCGRDESCSIZE, &desc_size);
  if(res<0) cerr <<"HIDIOCGRDESCSIZE error" <<endl;
  else cout <<"Report Descriptor Size: " <<desc_size <<endl;

  /* Get Report Descriptor */
  self->rpt_desc.size = desc_size;
  res = ioctl(self->fd, HIDIOCGRDESC, &self->rpt_desc);
  if(res<0) cerr <<"HIDIOCGRDESC error" <<endl;
  else{
    cout <<"Report Descriptor: ";
    for(int i=0; i<desc_size; i++) cout <<"0x" <<std::hex <<self->rpt_desc.value[i];
    cout <<endl;
  }

  /* Get Physical Location */
  res = ioctl(self->fd, HIDIOCGRAWPHYS(256), buf);
  if(res<0) cerr <<"HIDIOCGRAWPHYS error" <<endl;
  else cout <<"Raw Phys: " <<buf <<endl;
#endif

  /* Get Raw Name */
  int res = ioctl(self->fd, HIDIOCGRAWNAME(256), buf);
  if(res<0) cerr <<"HIDIOCGRAWNAME error" <<endl;
  else LOG <<"Raw Name: " <<buf <<endl;

  /* Get Raw Info */
  res = ioctl(self->fd, HIDIOCGRAWINFO, &info);
  if(res<0) cerr <<"HIDIOCGRAWINFO error" <<endl;
  else{
    LOG <<"Raw Info: bustype: " <<info.bustype <<" (";
    if(info.bustype==BUS_USB) cout <<"USB";
    else if(info.bustype==BUS_HIL) cout <<"HIL";
    else if(info.bustype==BUS_BLUETOOTH) cout <<"Bluetooth";
    else if(info.bustype==BUS_VIRTUAL) cout <<"Virtual";
    else cout <<"Other";
    cout <<") vendor: 0x" <<std::hex <<info.vendor;
    cout <<" product: 0x" <<std::hex <<info.product <<endl;
  }
}

SimplexMotion_Communication::~SimplexMotion_Communication(){
  close(self->fd);
  delete self;
}

bool SimplexMotion_Communication::writeBuf(int len){
  int res = ::write(self->fd, buf, len);
  if(res!=len){
    cerr <<"write error: " <<errno <<" written bytes:" <<res <<" wanted bytes:" <<len <<endl;
    return false;
  }
  return true;
}

bool SimplexMotion_Communication::readBuf(int len){
  int res = ::read(self->fd, buf, len);
  if(res!=len){
    cerr <<"read error: " <<errno <<" read bytes:" <<res <<" wanted bytes:" <<len <<endl;
    return false;
  }
  return true;
}

#else 

//===========================================================================
//
// fake (NICO) implementation
//

#define NICO throw std::runtime_error("not implemented with this compiler options: usually this means that the implementation needs an external library and a corresponding compiler option - see the source code");

SimplexMotion_Communication::SimplexMotion_Communication(const char* devPath, unsigned short vendor_id, unsigned short product_id){ NICO }

SimplexMotion_Communication::~SimplexMotion_Communication(){ NICO }
bool SimplexMotion_Communication::writeBuf(int len){ NICO }
bool SimplexMotion_Communication::readBuf(int len){ NICO }

#endif
