#include "livox.h"

#define RAI_LIVOX
#ifdef RAI_LIVOX

#include <Kin/frame.h>

#include "livox_lidar_def.h"
#include "livox_lidar_api.h"

#include <unistd.h>
#include <arpa/inet.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <thread>
#include <chrono>
#include <iostream>


void PointCloudCallback(uint32_t handle, const uint8_t dev_type, LivoxLidarEthernetPacket* data, void* client_data) {
  if (data == nullptr) {
    return;
  }
  // printf("point cloud handle: %u, data_num: %d, data_type: %d, length: %d, frame_counter: %d\n",
  //     handle, data->dot_num, data->data_type, data->length, data->frame_cnt);
  
  PointCloudData* pcData = static_cast<PointCloudData*>(client_data);
  
  if (data->data_type == kLivoxLidarCartesianCoordinateHighData) {
    LivoxLidarCartesianHighRawPoint *p_point_data = (LivoxLidarCartesianHighRawPoint *)data->data;
    for (uint32_t i = 0; i < data->dot_num; i++)
    {
      Point point;
      point.x = p_point_data[i].x * 0.001;
      point.y = p_point_data[i].y * 0.001;
      point.z = p_point_data[i].z * 0.001;
      
      float l1_norm = abs(point.x) + abs(point.y) + abs(point.z);
      if (l1_norm != 0)
      {
        // If the point cloud has more than MAX_POINTS, remove the first point
        if (pcData->points.size() >= pcData->max_points) {
            pcData->points.erase(pcData->points.begin());  // Remove the first point
        }
  
        // Add the point to the vector in the struct
        pcData->points.push_back(point);
      }

    }
  }
  else if (data->data_type == kLivoxLidarCartesianCoordinateLowData) {
    LivoxLidarCartesianLowRawPoint *p_point_data = (LivoxLidarCartesianLowRawPoint *)data->data;
  } else if (data->data_type == kLivoxLidarSphericalCoordinateData) {
    LivoxLidarSpherPoint* p_point_data = (LivoxLidarSpherPoint *)data->data;
  }
}

void ImuDataCallback(uint32_t handle, const uint8_t dev_type,  LivoxLidarEthernetPacket* data, void* client_data) {
  if (data == nullptr) {
    return;
  } 
  printf("Imu data callback handle:%u, data_num:%u, data_type:%u, length:%u, frame_counter:%u.\n",
      handle, data->dot_num, data->data_type, data->length, data->frame_cnt);
}

void WorkModeCallback(livox_status status, uint32_t handle,LivoxLidarAsyncControlResponse *response, void *client_data) {
  if (response == nullptr) {
    return;
  }
  printf("WorkModeCallack, status:%u, handle:%u, ret_code:%u, error_key:%u",
      status, handle, response->ret_code, response->error_key);

}

void RebootCallback(livox_status status, uint32_t handle, LivoxLidarRebootResponse* response, void* client_data) {
  if (response == nullptr) {
    return;
  }
  printf("RebootCallback, status:%u, handle:%u, ret_code:%u",
      status, handle, response->ret_code);
}

void SetIpInfoCallback(livox_status status, uint32_t handle, LivoxLidarAsyncControlResponse *response, void *client_data) {
  if (response == nullptr) {
    return;
  }
  printf("LivoxLidarIpInfoCallback, status:%u, handle:%u, ret_code:%u, error_key:%u",
      status, handle, response->ret_code, response->error_key);

  if (response->ret_code == 0 && response->error_key == 0) {
    LivoxLidarRequestReboot(handle, RebootCallback, nullptr);
  }
}

void QueryInternalInfoCallback(livox_status status, uint32_t handle, 
    LivoxLidarDiagInternalInfoResponse* response, void* client_data) {
  if (status != kLivoxLidarStatusSuccess) {
    printf("Query lidar internal info failed.\n");
    QueryLivoxLidarInternalInfo(handle, QueryInternalInfoCallback, nullptr);
    return;
  }

  if (response == nullptr) {
    return;
  }

  uint8_t host_point_ipaddr[4] {0};
  uint16_t host_point_port = 0;
  uint16_t lidar_point_port = 0;

  uint8_t host_imu_ipaddr[4] {0};
  uint16_t host_imu_data_port = 0;
  uint16_t lidar_imu_data_port = 0;

  uint16_t off = 0;
  for (uint8_t i = 0; i < response->param_num; ++i) {
    LivoxLidarKeyValueParam* kv = (LivoxLidarKeyValueParam*)&response->data[off];
    if (kv->key == kKeyLidarPointDataHostIpCfg) {
      memcpy(host_point_ipaddr, &(kv->value[0]), sizeof(uint8_t) * 4);
      memcpy(&(host_point_port), &(kv->value[4]), sizeof(uint16_t));
      memcpy(&(lidar_point_port), &(kv->value[6]), sizeof(uint16_t));
    } else if (kv->key == kKeyLidarImuHostIpCfg) {
      memcpy(host_imu_ipaddr, &(kv->value[0]), sizeof(uint8_t) * 4);
      memcpy(&(host_imu_data_port), &(kv->value[4]), sizeof(uint16_t));
      memcpy(&(lidar_imu_data_port), &(kv->value[6]), sizeof(uint16_t));
    }
    off += sizeof(uint16_t) * 2;
    off += kv->length;
  }

  printf("Host point cloud ip addr:%u.%u.%u.%u, host point cloud port:%u, lidar point cloud port:%u.\n",
      host_point_ipaddr[0], host_point_ipaddr[1], host_point_ipaddr[2], host_point_ipaddr[3], host_point_port, lidar_point_port);

  printf("Host imu ip addr:%u.%u.%u.%u, host imu port:%u, lidar imu port:%u.\n",
    host_imu_ipaddr[0], host_imu_ipaddr[1], host_imu_ipaddr[2], host_imu_ipaddr[3], host_imu_data_port, lidar_imu_data_port);

}

void LidarInfoChangeCallback(const uint32_t handle, const LivoxLidarInfo* info, void* client_data) {
  if (info == nullptr) {
    printf("lidar info change callback failed, the info is nullptr.\n");
    return;
  } 
  printf("LidarInfoChangeCallback Lidar handle: %u SN: %s\n", handle, info->sn);
  
  // set the work mode to kLivoxLidarNormal, namely start the lidar
  SetLivoxLidarWorkMode(handle, kLivoxLidarNormal, WorkModeCallback, nullptr);

  QueryLivoxLidarInternalInfo(handle, QueryInternalInfoCallback, nullptr);

  // LivoxLidarIpInfo lidar_ip_info;
  // strcpy(lidar_ip_info.ip_addr, "192.168.1.10");
  // strcpy(lidar_ip_info.net_mask, "255.255.255.0");
  // strcpy(lidar_ip_info.gw_addr, "192.168.1.1");
  // SetLivoxLidarLidarIp(handle, &lidar_ip_info, SetIpInfoCallback, nullptr);
}

void LivoxLidarPushMsgCallback(const uint32_t handle, const uint8_t dev_type, const char* info, void* client_data) {
  struct in_addr tmp_addr;
  tmp_addr.s_addr = handle;  
  std::cout << "handle: " << handle << ", ip: " << inet_ntoa(tmp_addr) << ", push msg info: " << std::endl;
  std::cout << info << std::endl;
  return;
}

namespace rai{

  Livox::Livox() : Thread("LivoxThread", 0.){
    max_points = rai::getParameter<int>("livox/max_points", 30000);
    points = zeros(max_points*3);
    points.reshape(max_points, 3);

    points_message.max_points = max_points;

    const std::string path = "./mid360_config.json";

    if (!LivoxLidarSdkInit(path.c_str())) {
      printf("Livox Init Failed\n");
      LivoxLidarSdkUninit();
    }

    // REQUIRED, to get point cloud data via 'PointCloudCallback'
    SetLivoxLidarPointCloudCallBack(PointCloudCallback, &points_message);
    
    // OPTIONAL, to get imu data via 'ImuDataCallback'
    // some lidar types DO NOT contain an imu component
    // SetLivoxLidarImuDataCallback(ImuDataCallback, nullptr);
      
    // SetLivoxLidarInfoCallback(LivoxLidarPushMsgCallback, nullptr);
    
    // REQUIRED, to get a handle to targeted lidar and set its work mode to NORMAL
    // SetLivoxLidarInfoChangeCallback(LidarInfoChangeCallback, nullptr);

    threadLoop();
  }

  Livox::~Livox(){
    LivoxLidarSdkUninit();
    threadClose();
  }

  void Livox::pull(rai::Configuration& C){
    //-- Update configuration
    const char* name = "livox_point_cloud";
    rai::Frame *base = C.getFrame(name, false);
    if(!base){
      LOG(0) << "Creating new frame 'livox_point_cloud'";
      base = C.addFrame(name);
      base->setColor({0., 1., 0.});
    }

    std::lock_guard<std::mutex> lock(mux);

    base->setPointCloud(points);
  }

  void Livox::step(){
    points = zeros(max_points*3);
    int counter = 0;
    for (const auto& point : points_message.points) {
      points.elem(counter) = point.x;
      points.elem(counter+1) = point.y;
      points.elem(counter+2) = point.z;
      counter += 3;
    }
    points.reshape(max_points, 3);
  }
} //namespace

#else

rai::Livox::Livox() : Thread("LivoxThread") { NICO }
rai::Livox::~Livox(){ NICO }
void rai::Livox::pull(rai::Configuration& C){ NICO }
void rai::Livox::step(){ NICO }

#endif
