//
// Created by luke on 1-12-22.
//

#ifndef ROBOTV2_APP_SLAM_SCANNING_LIDAR_H
#define ROBOTV2_APP_SLAM_SCANNING_LIDAR_H

#include <boost/move/move.hpp>
#include <boost/container/vector.hpp>
#include <boost/log/trivial.hpp>
#include <boost/shared_ptr.hpp>
#include <rplidar.h>
#include <sstream>
#include <vector>
#include <algorithm>

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
  class mode
  {

  };

  class health
  {
  public:
    enum class status
    {
      Ok,
      Warning,
      Error
    };
  protected:
    status m_status;
    uint16_t m_code;
  public:
    health(const status initial_status, const uint16_t initial_code) noexcept;
  public:
    inline status get_status(void) const noexcept
    {
      return this->m_status;
    }

    inline uint16_t get_code(void) const noexcept
    {
      return this->m_code;
    }
  };
protected:
  boost::shared_ptr<sl::IChannel> m_channel;
  boost::shared_ptr<sl::ILidarDriver> m_driver;

public:
  lidar(boost::shared_ptr<sl::IChannel> channel,
        boost::shared_ptr<sl::ILidarDriver> driver);

public:
  /// @brief gets a vector containing all the lidar scan modes.
  /// @return the vector of scan modes.
  boost::container::vector<sl::LidarScanMode> get_scan_modes(void);

  /// @brief gets the health of the lidar.
  /// @return the lidar health.
  health get_health(void);
public:
  static lidar_serial_builder builder(void) noexcept
  {
    return lidar_serial_builder();
  }
};
} // namespace app::slam::scanning

#endif // ROBOTV2_LIDAR_H
