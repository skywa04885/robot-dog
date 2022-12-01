//
// Created by luke on 1-12-22.
//

#include "leg.h"

namespace app::kinematics
{
  leg_builder::leg_builder() noexcept:
    length1(10.0), length2(10.0)
  {}

  leg leg_builder::build(void) const noexcept
  {
    return leg(this->length1, this->length2);
  }

  leg::leg(const double length1, const double length2) noexcept:
    length1(length1), length2(length2)
  {}

  boost::tuple<double, double, double>
  leg::inverse(const boost::tuple<double, double> &end_position,
          const boost::tuple<double, double> &delta_end_position) const
  {
    boost::tuple<double, double, double> result;

    return boost::move(result);
  }

  boost::tuple<double, double>
  leg::forward(const boost::tuple<double, double, double> &angles) const
  {
    boost::tuple<double, double> result;

    return boost::move(result);
  }
}