//
// Created by luke on 2-12-22.
//

#ifndef BACKEND_PROCESS_H
#define BACKEND_PROCESS_H

#include <boost/optional.hpp>
#include <boost/thread.hpp>
#include <boost/geometry.hpp>

#include "point_cloud/registrator.h"
#include "scanning/lidar.h"

namespace app::slam
{
  class process
  {
  protected:
    static boost::optional<process> s_instance;
  public:
    static inline process &get_instance(void)
    {
      if (!s_instance.has_value())
        s_instance.emplace();

      return s_instance.value();
    }

  protected:
    boost::mutex m_environment_point_cloud_mutex;
    boost::geometry::index::rtree<std::pair<boost::geometry::model::point<double, 2, boost::geometry::cs::cartesian>,
      unsigned int>, boost::geometry::index::quadratic<16>> m_environment_point_cloud;

    boost::thread m_thread;

  public:
    process();

  protected:
    void thread_runner(void);

  public:
    inline boost::mutex &get_environment_point_cloud_mutex()
    {
      return m_environment_point_cloud_mutex;
    }

    inline boost::geometry::index::rtree<std::pair<boost::geometry::model::point<double, 2, boost::geometry::cs::cartesian>,
      unsigned int>, boost::geometry::index::quadratic<16>> &get_environment_point_cloud()
    {
      return m_environment_point_cloud;
    }
  };
}


#endif //BACKEND_PROCESS_H
