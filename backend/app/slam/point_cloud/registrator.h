#ifndef _APP_SLAM_POINT_CLOUD_REGISTRATOR
#define _APP_SLAM_POINT_CLOUD_REGISTRATOR

#include <algorithm>
#include <cmath>
#include <iostream>
#include <valarray>

#include <boost/log/trivial.hpp>
#include <boost/compute.hpp>
#include <boost/compute/core.hpp>
#include <boost/container/vector.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/index/rtree.hpp>
#include <boost/compute/algorithm/accumulate.hpp>
#include <boost/compute/container/valarray.hpp>
#include <boost/foreach.hpp>

#include "../../tools/round.h"

namespace app::slam::point_cloud
{
  class registrator_builder
  {
  public:
    registrator_builder();

  public:
  };

  class registrator
  {
  public:
    typedef boost::geometry::model::point<double, 2,
      boost::geometry::cs::cartesian>
      point;
    typedef std::pair<point, unsigned> value;
    typedef boost::geometry::index::rtree<value,
      boost::geometry::index::quadratic<15>> tree;

    struct execute_result
    {
    public:
      boost::array<double, 2> offsets;
      double angle;
    };
  public:
    std::valarray<double> execute_compute_partial_derivatives(
      std::valarray<double> &parameters,
      boost::container::vector<point> &source,
      tree &target
    );

    execute_result execute(boost::container::vector<point> &source,
                           tree &target);

  public:
    inline static registrator_builder builder(void) noexcept
    {
      return registrator_builder();
    }
  };
} // namespace app::slam::point_cloud

#endif
