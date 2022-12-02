//
// Created by luke on 2-12-22.
//

#include "process.h"

namespace app::slam
{
  boost::optional<process> process::s_instance = boost::none;

  process::process()
  {
    this->m_thread = boost::thread(boost::bind(&process::thread_runner, this));
  }

  void process::thread_runner() {
    app::slam::scanning::lidar primary_lidar = app::slam::scanning::lidar::builder()
      .set_serial_port("/dev/ttyUSB0")
      .set_serial_baud(115200).build();

    BOOST_LOG_TRIVIAL(info) << "Setting LiDAR motor speed to: " << DEFAULT_MOTOR_SPEED;
    primary_lidar.set_motor_speed(DEFAULT_MOTOR_SPEED);

    while (true)
    {
      app::slam::scanning::lidar::scan_result_t scan_result = primary_lidar.scan(0);

      {
        boost::lock_guard lock(m_environment_point_cloud_mutex);

        if (m_environment_point_cloud.empty())
        {
          for (const boost::geometry::model::point<double, 2, boost::geometry::cs::cartesian> &point: scan_result)
          {
            m_environment_point_cloud.insert(std::make_pair(point, 1));
          }
        }
        else
        {
          auto [offsetX, offsetY, angle] = app::slam::point_cloud::registrator::builder().build().execute(scan_result, m_environment_point_cloud);
        }
      }
    }
  }
}