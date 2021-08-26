/***
 * TAKEN FROM https://github.com/USC-ACTLab/libmotioncapture
 * Aug 26, 2021
 */

#pragma once
#include "motioncapture.h"

namespace libmotioncapture {
  class MotionCaptureOptitrackImpl;

  class MotionCaptureOptitrack : public MotionCapture{
  public:
    MotionCaptureOptitrack(
      const std::string& hostname);

    virtual ~MotionCaptureOptitrack();

    const std::string& version() const;

    virtual void waitForNextFrame();
    virtual void getObjects(
      std::vector<Object>& result) const;

//    virtual void getPointCloud(
//      pcl::PointCloud<pcl::PointXYZ>::Ptr result) const;
    virtual void getLatency(
      std::vector<LatencyInfo>& result) const;
    virtual uint64_t getTimeStamp() const;

    virtual bool supportsObjectTracking() const;
    virtual bool supportsLatencyEstimate() const;
    virtual bool supportsPointCloud() const;
    virtual bool supportsTimeStamp() const;

  private:
    MotionCaptureOptitrackImpl * pImpl;
  };
}

