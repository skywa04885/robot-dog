//
// Created by luke on 1-12-22.
//

#include "lidar.h"

namespace app::slam::scanning {
#define HANDLE_SL_RESULT(EXPR) __handle_sl_result(#EXPR, (EXPR))

static void __handle_sl_result(const char *const expr, const sl_result result) {
  if (SL_IS_OK(result))
    return;

  BOOST_LOG_TRIVIAL(fatal) << "Expression (" << expr << ") failed: " << result;
  throw std::runtime_error("");
}

lidar_serial_builder::lidar_serial_builder(void) noexcept
    : m_serial_port("/dev/ttyUSB0"), m_baud(115200) {}

lidar lidar_serial_builder::build(void) noexcept {
  boost::shared_ptr<sl::IChannel> channel(
      sl::createSerialPortChannel(this->m_serial_port, this->m_baud).value);

  boost::shared_ptr<sl::ILidarDriver> driver(sl::createLidarDriver().value);

  HANDLE_SL_RESULT(driver->connect(channel.get()));

  return lidar(channel, driver);
}


lidar::lidar::health::health(const status initial_status, const uint16_t initial_code) noexcept:
  m_status(initial_status), m_code(initial_code)
{}

lidar::lidar(boost::shared_ptr<sl::IChannel> channel,
             boost::shared_ptr<sl::ILidarDriver> driver)
    : m_channel(channel), m_driver(driver) {}

/// @brief gets a vector containing all the lidar scan modes.
/// @return the vector of scan modes.
boost::container::vector<sl::LidarScanMode> lidar::get_scan_modes(void) {

  std::vector<sl::LidarScanMode> tempScanModes;
  HANDLE_SL_RESULT(this->m_driver->getAllSupportedScanModes(tempScanModes));

  boost::container::vector<sl::LidarScanMode> scanModes;
  scanModes.reserve(tempScanModes.size());
  std::copy(tempScanModes.begin(), tempScanModes.end(),
            std::back_inserter(scanModes));

  return boost::move(scanModes);
}

/// @brief gets the health of the lidar.
/// @return the lidar health.
lidar::health lidar::get_health(void)
{
  sl_lidar_response_device_health_t health;

  HANDLE_SL_RESULT(this->m_driver->getHealth(health));

  lidar::health::status s;
  switch (health.status)
  {
    case SL_LIDAR_STATUS_WARNING:
      s = lidar::health::status::Warning;
      break;
    case SL_LIDAR_STATUS_OK:
      s = lidar::health::status::Ok;
      break;
    case SL_LIDAR_STATUS_ERROR:
      s = lidar::health::status::Error;
      break;
    default:
      throw std::runtime_error("Invalid received status code.");
  }

  return lidar::health(s, health.error_code);
}
} // namespace app::slam::scanning