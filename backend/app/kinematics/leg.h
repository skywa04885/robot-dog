//
// Created by luke on 1-12-22.
//

#ifndef ROBOTV2_APP_KINEMATICS_LEG_H
#define ROBOTV2_APP_KINEMATICS_LEG_H

#include <boost/move/move.hpp>
#include <boost/tuple/tuple.hpp>

namespace app::kinematics
{
  class leg;

  class leg_builder
  {
  protected:
    double length1, length2;
  public:
    leg_builder(void) noexcept;

  public:
    inline leg_builder &set_length1(const double length1)
    {
      this->length1 = length1;
      return *this;
    }

    inline leg_builder &set_length2(const double length2)
    {
      this->length2 = length2;
      return *this;
    }

  public:
    leg build(void) const noexcept;
  };

  class leg
  {
  protected:
    const double length1, length2;

  public:
    leg(const double length1, const double length2) noexcept;

    boost::tuple<double, double, double>
    inverse(const boost::tuple<double, double> &end_position,
            const boost::tuple<double, double> &delta_end_position) const;

    boost::tuple<double, double>
    forward(const boost::tuple<double, double, double> &angles) const;

  public:
    static inline leg_builder builder(void) noexcept
    {
      return leg_builder();
    }
  };
}

#endif //ROBOTV2_LEG_H
