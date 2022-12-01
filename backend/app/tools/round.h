//
// Created by luke on 1-12-22.
//

#ifndef ROBOTV2_ROUND_H
#define ROBOTV2_ROUND_H

#include <cstddef>
#include <cmath>

template<typename T>
inline T round_to(const T a, const size_t precision) noexcept
{
  const T b = std::pow(static_cast<T>(10), static_cast<T>(precision));
  return std::round(a * b) / b;
}

#endif //ROBOTV2_ROUND_H
