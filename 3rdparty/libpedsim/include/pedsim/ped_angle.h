//
// pedsim - A microscopic pedestrian simulation system.
// Copyright (c) 2003 - 2012 by Christian Gloor
//

#ifndef _ped_angle_h_
#define _ped_angle_h_ 1

#ifdef WIN32
#define LIBEXPORT __declspec(dllexport)
#else
#define LIBEXPORT
#endif

#include <cmath>

namespace Ped {
class LIBEXPORT Tangle {
 public:
  // Type Definitions
  typedef enum {
    // [0, 2*pi) or [0°, 360°)
    PositiveOnlyRange,
    // (-pi, +pi] or (-180°, 180°]
    PositiveNegativeRange
  } AngleRange;

  // Constructor
 public:
  Tangle() { value = 0; }

 protected:
  Tangle(double valueIn) {
    value = valueIn;
    normalize();
  }

  // Static Methods
 public:
  static Tangle fromRadian(double valueIn) { return Tangle(valueIn); }
  static Tangle fromDegree(double valueIn) {
    return Tangle(valueIn / 180 * M_PI);
  }

  // Methods
 public:
  double toRadian(AngleRange range = PositiveNegativeRange) const {
    if (range == PositiveNegativeRange)
      return value;
    else
      return (value >= 0) ? value : (value + 2 * M_PI);
  }
  double toDegree(AngleRange range = PositiveNegativeRange) const {
    double degreeValue = value * 180 / M_PI;

    if (range == PositiveNegativeRange)
      return degreeValue;
    else
      return (degreeValue >= 0) ? degreeValue : (degreeValue + 360);
  }
  void setRadian(double valueIn) {
    value = valueIn;
    normalize();
  }
  void setDegree(double valueIn) {
    value = valueIn / 180 * M_PI;
    normalize();
  }

  int sign() const {
    if (value == 0)
      return 0;
    else if (value > 0)
      return 1;
    else
      return -1;
  }

 protected:
  void normalize() {
    while (value <= -M_PI) value += 2 * M_PI;
    while (value > M_PI) value -= 2 * M_PI;
  }

  // Operators
 public:
  Tangle operator+(const Tangle& angleIn) const {
    return Tangle(value + angleIn.value);
  }
  Tangle operator-(const Tangle& angleIn) const {
    return Tangle(value - angleIn.value);
  }
  Tangle& operator+=(const Tangle& angleIn) {
    value += angleIn.value;
    normalize();
    return *this;
  }
  Tangle& operator-=(const Tangle& angleIn) {
    value -= angleIn.value;
    normalize();
    return *this;
  }

  // Operators
  bool operator==(const Ped::Tangle& otherIn) const;
  bool operator!=(const Ped::Tangle& otherIn) const;
  bool operator<(const Ped::Tangle& otherIn) const;
  bool operator<=(const Ped::Tangle& otherIn) const;
  bool operator>(const Ped::Tangle& otherIn) const;
  bool operator>=(const Ped::Tangle& otherIn) const;

  // Attributes
 protected:
  double value;
};
}

#endif
