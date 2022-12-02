//
// Created by luke on 1-12-22.
//

#ifndef ROBOTV2_APP_SLAM_SCANNING_LIDAR_H
#define ROBOTV2_APP_SLAM_SCANNING_LIDAR_H

#include <algorithm>
#include <boost/cast.hpp>
#include <boost/container/vector.hpp>
#include <boost/geometry/core/cs.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/log/trivial.hpp>
#include <boost/move/move.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/smart_ptr.hpp>
#include <numbers>
#include <rplidar.h>
#include <sstream>
#include <vector>

namespace app::slam::scanning {
class lidar;

class lidar_serial_builder {
protected:
  std::string m_serial_port;
  int m_baud;

public:
  lidar_serial_builder(void) noexcept;

public:
  inline lidar_serial_builder &
  set_serial_port(const std::string &new_serial_port) noexcept {
    this->m_serial_port = new_serial_port;
    return *this;
  }

  inline lidar_serial_builder &set_serial_baud(const int new_baud) noexcept {
    this->m_baud = new_baud;
    return *this;
  }

public:
  lidar build(void) noexcept;
};

class lidar {
public:
  typedef boost::container::vector<
      boost::geometry::model::point<double, 2, boost::geometry::cs::cartesian>>
      scan_result_t;

public:
  class mode {};

  class health {
  public:
    enum class status { Ok, Warning, Error };

  protected:
    status m_status;
    uint16_t m_code;

  public:
    health(const status initial_status, const uint16_t initial_code) noexcept;

  public:
    inline status get_status(void) const noexcept { return this->m_status; }

    inline uint16_t get_code(void) const noexcept { return this->m_code; }
  };

protected:
  sl::IChannel *m_channel;
  sl::ILidarDriver *m_driver;

public:
  lidar(sl::IChannel *channel,
        sl::ILidarDriver *driver);

  ~lidar(void);
public:
  /// @brief sets the speed of the motor.
  /// @param speed the speed to set the motor to.
  /// @return the curent instance.
  lidar &set_motor_speed(const uint16_t speed);

  /// @brief gets a vector containing all the lidar scan modes.
  /// @return the vector of scan modes.
  boost::container::vector<sl::LidarScanMode> get_scan_modes(void);

  /// @brief gets the id of the typical scan mode.
  /// @return the id of the typical scan mode.
  uint16_t get_typical_scan_mode(void);

  /// @brief gets the health of the lidar.
  /// @return the lidar health.
  health get_health(void);

  /// @brief performs a scan and returns the cartesian coordinates.
  /// @param scan_mode_id the id of the scan mode to use.
  /// @return the vector containing all the scanned points.
  scan_result_t scan(const uint16_t scan_mode_id);

  /// @brief performs a scan and returns the cartesian coordinates.
  /// @return the vector containing all the scanned points.
  scan_result_t scan(void);

public:
  static lidar_serial_builder builder(void) noexcept {
    return lidar_serial_builder();
  }
};
} // namespace app::slam::scanning

#endif // ROBOTV2_LIDAR_H
