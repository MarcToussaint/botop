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
        // If the point cloud has more than max_points, remove the first point
        if (pcData->points.size() >= pcData->max_points) {
            pcData->points.erase(pcData->points.begin());
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

namespace rai{

  Livox::Livox() : Thread("LivoxThread", 0.){
    max_points = rai::getParameter<int>("livox/max_points", 30000);
    points = zeros(max_points*3);
    points.reshape(max_points, 3);

    points_message.max_points = max_points;

    const std::string path = "./mid360_config.json";

    if (!LivoxLidarSdkInit(path.c_str()))
    {
      printf("Livox Init Failed\n");
      LivoxLidarSdkUninit();
    }
    else
    {
      SetLivoxLidarPointCloudCallBack(PointCloudCallback, &points_message);
    }

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
