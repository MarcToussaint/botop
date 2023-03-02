#pragma once

#ifdef RAI_REALSENSE

#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>

namespace rai {
namespace realsense {

inline float get_depth_scale(rs2::device dev) {
  // Go over the device's sensors
  for(rs2::sensor& sensor : dev.query_sensors()) {
    // Check if the sensor if a depth sensor
    if(rs2::depth_sensor dpt = sensor.as<rs2::depth_sensor>()) {
      return dpt.get_depth_scale();
    }
  }
  throw std::runtime_error("Device does not have a depth sensor");
}

}
}


#endif // REALSENSE_UTILS_H
