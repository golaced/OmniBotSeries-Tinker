/*! @file Interpolation.h
 *  @brief Utility functions to interpolate between two values
 *
 */

#ifndef PROJECT_INTERPOLATION_H
#define PROJECT_INTERPOLATION_H

#include <assert.h>
#include <type_traits>

//namespace Interpolate {

/*!
 * Linear interpolation between y0 and yf.  x is between 0 and 1
 */
//template <typename float, typename float>
float lerp(float y0, float yf, float x) {
  static_assert(std::is_floating_point<float>::value,
                "must use floating point value");
  assert(x >= 0 && x <= 1);
  return y0 + (yf - y0) * x;
}

/*!
 * Cubic bezier interpolation between y0 and yf.  x is between 0 and 1
 */
//template <typename float, typename float>
float cubicBezier(float y0, float yf, float x) {
  static_assert(std::is_floating_point<float>::value,
                "must use floating point value");
  assert(x >= 0 && x <= 1);
  float yDiff = yf - y0;
  float bezier = x * x * x + float(3) * (x * x * (float(1) - x));
  return y0 + bezier * yDiff;
}

/*!
 * Cubic bezier interpolation derivative between y0 and yf.  x is between 0 and
 * 1
 */
//template <typename float, typename float>
float cubicBezierFirstDerivative(float y0, float yf, float x) {
  static_assert(std::is_floating_point<float>::value,
                "must use floating point value");
  assert(x >= 0 && x <= 1);
  float yDiff = yf - y0;
  float bezier = float(6) * x * (float(1) - x);
  return bezier * yDiff;
}

/*!
 * Cubic bezier interpolation derivative between y0 and yf.  x is between 0 and
 * 1
 */
//template <typename float, typename float>
float cubicBezierSecondDerivative(float y0, float yf, float x) {
  static_assert(std::is_floating_point<float>::value,
                "must use floating point value");
  assert(x >= 0 && x <= 1);
  float yDiff = yf - y0;
  float bezier = float(6) - float(12) * x;
  return bezier * yDiff;
}

//}  // namespace Interpolate

#endif  // PROJECT_INTERPOLATION_H
