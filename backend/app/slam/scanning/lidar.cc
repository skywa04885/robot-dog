//
// Created by luke on 1-12-22.
//

#include "lidar.h"

namespace app::slam::scanning
{
#define HANDLE_SL_RESULT(EXPR) __handle_sl_result(#EXPR, (EXPR))

  /// @brief performs error handling for the given expression based on the given result.
  /// @param expr the expression we're handling it for.
  /// @param result the result we're checking.
  static void __handle_sl_result(const char *const expr, const sl_result result)
  {
    if (SL_IS_OK(result))
      return;

    std::ostringstream stream;
    stream << "Expression (" << expr << ") failed: ";

    switch (result)
    {
      case SL_RESULT_OPERATION_TIMEOUT:
        stream << "Operation timeout.";
        break;
      case SL_RESULT_OPERATION_FAIL:
        stream << "Operation failure.";
        break;
      default:
        stream << result;
        break;
    }

    throw std::runtime_error(stream.str());
  }

  lidar_serial_builder::lidar_serial_builder(void) noexcept
    : m_serial_port("/dev/ttyUSB0"), m_baud(115200)
  {}

  lidar lidar_serial_builder::build(void) noexcept
  {
    sl::ILidarDriver *driver = *sl::createLidarDriver();
    BOOST_ASSERT(driver != nullptr);

    sl::IChannel *channel =
      *sl::createSerialPortChannel(this->m_serial_port, this->m_baud);
    BOOST_ASSERT(channel != nullptr);

    HANDLE_SL_RESULT(driver->connect(channel));

    return {channel, driver};
  }

  lidar::lidar::health::health(const status initial_status,
                               const uint16_t initial_code) noexcept
    : m_status(initial_status), m_code(initial_code)
  {}

  lidar::lidar(sl::IChannel *channel, sl::ILidarDriver *driver)
    : m_channel(channel), m_driver(driver)
  {}

  lidar::~lidar(void)
  {
    delete this->m_driver;
    delete this->m_channel;
  }

  /// @brief sets the speed of the motor.
  /// @param speed the speed to set the motor to.
  /// @return the curent instance.
  lidar &lidar::set_motor_speed(const uint16_t speed)
  {
    HANDLE_SL_RESULT(this->m_driver->setMotorSpeed(speed));

    return *this;
  }

  /// @brief gets a vector containing all the lidar scan modes.
  /// @return the vector of scan modes.
  boost::container::vector<sl::LidarScanMode> lidar::get_scan_modes(void)
  {

    std::vector<sl::LidarScanMode> tempScanModes;
    HANDLE_SL_RESULT(this->m_driver->getAllSupportedScanModes(tempScanModes));

    boost::container::vector<sl::LidarScanMode> scanModes;
    scanModes.reserve(tempScanModes.size());
    std::copy(tempScanModes.begin(), tempScanModes.end(),
              std::back_inserter(scanModes));

    return boost::move(scanModes);
  }

  /// @brief gets the id of the typical scan mode.
  /// @return the id of the typical scan mode.
  uint16_t lidar::get_typical_scan_mode(void)
  {
    uint16_t result;
    HANDLE_SL_RESULT(this->m_driver->getTypicalScanMode(result));
    return result;
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

    return {s, health.error_code};
  }

  /// @brief performs a scan and returns the cartesian coordinates.
  /// @param scan_mode_id the id of the scan mode to use.
  /// @return the vector containing all the scanned points.
  lidar::scan_result_t lidar::scan(const uint16_t scan_mode_id)
  {
    HANDLE_SL_RESULT(this->m_driver->startScanExpress(false, scan_mode_id, 0));

    size_t buffer_size = 4096;
    boost::scoped_array<sl_lidar_response_measurement_node_hq_t> buffer(
      new sl_lidar_response_measurement_node_hq_t[buffer_size]);

    HANDLE_SL_RESULT(this->m_driver->grabScanDataHq(buffer.get(), buffer_size));
    HANDLE_SL_RESULT(this->m_driver->ascendScanData(buffer.get(), buffer_size));

    scan_result_t result;
    result.reserve(buffer_size);

    for (size_t i = 0; i < buffer_size; ++i)
    {
      const sl_lidar_response_measurement_node_hq_t *node = &buffer[i];

      if (node->dist_mm_q2 == 0)
        continue;

      const double distance = boost::numeric_cast<double>(node->dist_mm_q2) / 4.0;
      const double angle =
        (boost::numeric_cast<double>(node->angle_z_q14) / 32768.0) *
        std::numbers::pi_v<double>;

      result.emplace_back(distance * std::cos(angle), distance * std::sin(angle));
    }

    return boost::move(result);
  }

  /// @brief performs a scan and returns the cartesian coordinates.
  /// @return the vector containing all the scanned points.
  lidar::scan_result_t lidar::scan(void)
  {
    const uint16_t scan_mode_id = this->get_typical_scan_mode();
    return this->scan(scan_mode_id);
  }

} // namespace app::slam::scanning