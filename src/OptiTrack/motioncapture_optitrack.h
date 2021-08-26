/***
 * TAKEN FROM https://github.com/IMRCLab/libmotioncapture
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

    // implementations for MotionCapture interface
    virtual void waitForNextFrame();
    virtual const std::map<std::string, RigidBody>& rigidBodies() const;
    virtual const PointCloud& pointCloud() const;

    virtual bool supportsRigidBodyTracking() const
    {
      return true;
    }
    virtual bool supportsPointCloud() const
    {
      return true;
    }

  private:
    MotionCaptureOptitrackImpl * pImpl;
  };
}

