/***
 * TAKEN FROM https://github.com/IMRCLab/libmotioncapture
 * Aug 26, 2021
 */

#include "motioncapture.h"

#define ENABLE_OPTITRACK

#ifdef ENABLE_VICON
#include "libmotioncapture/vicon.h"
#endif
#ifdef ENABLE_OPTITRACK
#include "motioncapture_optitrack.h"
#endif
#ifdef ENABLE_QUALISYS
#include "libmotioncapture/qualisys.h"
#endif
#ifdef ENABLE_VRPN
#include "libmotioncapture/vrpn.h"
#endif

namespace libmotioncapture {

  const char *version_string = "1.0a1";

  const char *version()
  {
    return version_string;
  }

  RigidBody MotionCapture::rigidBodyByName(
      const std::string& name) const
  {
    const auto& obj = rigidBodies();
    const auto iter = obj.find(name);
    if (iter != obj.end()) {
      return iter->second;
    }
    throw std::runtime_error("Rigid body not found!");
  }

  MotionCapture* MotionCapture::connect(
    const std::string& type,
    const std::string& hostname)
  {
    MotionCapture* mocap = nullptr;

    if (false)
    {
    }
#ifdef ENABLE_VICON
    else if (type == "vicon")
    {
      mocap = new libmotioncapture::MotionCaptureVicon(
        hostname,
        /*enable_objects*/ true,
        /*enable_pointclout*/ true);
    }
#endif
#ifdef ENABLE_OPTITRACK
    else if (type == "optitrack")
    {
      mocap = new libmotioncapture::MotionCaptureOptitrack(
        hostname);
    }
#endif
#ifdef ENABLE_QUALISYS
    else if (type == "qualisys")
    {
      mocap = new libmotioncapture::MotionCaptureQualisys(
        hostname,
        /*port*/ 22222,
        /*enable_objects*/ true,
        /*enable_pointclout*/ true);  
    }
#endif
#ifdef ENABLE_VRPN
    else if (type == "vrpn")
    {
      mocap = new libmotioncapture::MotionCaptureVrpn(
        hostname);
    }
#endif
    else
    {
      throw std::runtime_error("Unknown motion capture type!");
    }

    return mocap;
  }

}
