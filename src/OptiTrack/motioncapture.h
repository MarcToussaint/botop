/***
 * TAKEN FROM https://github.com/USC-ACTLab/libmotioncapture
 * Aug 26, 2021
 */

#pragma once
#include <cstddef>
#include <stdint.h>
#include <string>
#include <vector>

#include <Eigen/Dense>

// PCL
//#include <pcl/point_cloud.h>
//#include <pcl/point_types.h>

namespace libmotioncapture {

  class Object
  {
  public:
    Object(
      const std::string& name,
      const Eigen::Vector3f& position,
      Eigen::Quaternionf& rotation)
      : m_name(name)
      , m_position(position)
      , m_rotation(rotation)
      , m_occluded(false)
    {
    }

    Object(
      const std::string& name)
      : m_name(name)
      , m_position()
      , m_rotation()
      , m_occluded(true)
    {
    }

    Object()
      : m_name()
      , m_position()
      , m_rotation()
      , m_occluded(true)
    {
    }

    const std::string& name() const {
      return m_name;
    }

    const Eigen::Vector3f& position() const {
      return m_position;
    }

    const Eigen::Quaternionf& rotation() const {
      return m_rotation;
    }

    bool occluded() const {
      return m_occluded;
    }

  private:
    std::string m_name;
    Eigen::Vector3f m_position;
    Eigen::Quaternionf m_rotation;
    bool m_occluded;
  };

  class LatencyInfo
  {
  public:
    LatencyInfo(
      std::string& name,
      double value)
      : m_name(name)
      , m_value(value)
    {
    }

    const std::string& name() const {
      return m_name;
    }

    double value() const {
      return m_value;
    }
  private:
    std::string m_name;
    double m_value;
  };

  class MotionCapture
  {
  public:
    virtual ~MotionCapture()
    {
    }

    // waits until a new frame is available
    virtual void waitForNextFrame() = 0;

    // Query data

    // returns reference to objects available in the current frame
    virtual void getObjects(std::vector<Object>& result) const = 0;

    // returns an object with a specified name
    virtual void getObjectByName(
      const std::string& name,
      Object& result) const;

    // returns pointer to point cloud (all unlabled markers)
//    virtual void getPointCloud(
//      pcl::PointCloud<pcl::PointXYZ>::Ptr result) const = 0;

    // return latency information
    virtual void getLatency(
      std::vector<LatencyInfo>& result) const = 0;

    // returns timestamp in microseconds
    virtual uint64_t getTimeStamp() const = 0;

    // Query API capabilities

    // return true, if tracking of objects is supported
    virtual bool supportsObjectTracking() const = 0;
    // returns true, if latency can be estimated
    virtual bool supportsLatencyEstimate() const = 0;
    // returns true if raw point cloud is available
    virtual bool supportsPointCloud() const = 0;
    // returns true if timestamp is available
    virtual bool supportsTimeStamp() const = 0;
  };

} // namespace libobjecttracker


