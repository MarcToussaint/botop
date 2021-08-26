/***
 * TAKEN FROM https://github.com/IMRCLab/libmotioncapture
 * Aug 26, 2021
 */

#pragma once
#include <cstddef>
#include <stdint.h>
#include <string>
#include <vector>
#include <map>

// Eigen
#include <Eigen/Geometry>

namespace libmotioncapture {

  typedef Eigen::Matrix<float, Eigen::Dynamic, 3> PointCloud;

  const char* version();

  class RigidBody
  {
  public:
    RigidBody(
      const std::string& name,
      const Eigen::Vector3f& position,
      Eigen::Quaternionf& rotation)
      : m_name(name)
      , m_position(position)
      , m_rotation(rotation)
      , m_occluded(false)
    {
    }

    RigidBody(
      const std::string& name)
      : m_name(name)
      , m_position()
      , m_rotation()
      , m_occluded(true)
    {
    }

    RigidBody()
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
    static MotionCapture* connect(
      const std::string& type,
      const std::string& hostname);

    virtual ~MotionCapture()
    {
    }

    // waits until a new frame is available
    virtual void waitForNextFrame() = 0;

    // Query data

    // returns reference to rigid bodies available in the current frame
    virtual const std::map<std::string, RigidBody>& rigidBodies() const
    {
      rigidBodies_.clear();
      return rigidBodies_;
    }

    // returns copy of rigid body with a specified name
    virtual RigidBody rigidBodyByName(
      const std::string& name) const;

    // returns pointer to point cloud (all unlabled markers)
    virtual const PointCloud& pointCloud() const
    {
      pointcloud_.resize(0, Eigen::NoChange);
      return pointcloud_;
    }

    // return latency information
    virtual const std::vector<LatencyInfo>& latency() const
    {
      latencies_.clear();
      return latencies_;
    }

    // returns timestamp in microseconds
    virtual uint64_t timeStamp() const
    {
      return 0;
    }

    // Query API capabilities

    // return true, if tracking of objects is supported
    virtual bool supportsRigidBodyTracking() const
    {
      return false;
    }
    // returns true, if latency can be estimated
    virtual bool supportsLatencyEstimate() const
    {
      return false;
    }
    // returns true if raw point cloud is available
    virtual bool supportsPointCloud() const
    {
      return false;
    }
    // returns true if timestamp is available
    virtual bool supportsTimeStamp() const
    {
      return false;
    }

  protected:
    mutable std::map<std::string, RigidBody> rigidBodies_;
    mutable PointCloud pointcloud_;
    mutable std::vector<LatencyInfo> latencies_;
  };

} // namespace libobjecttracker


