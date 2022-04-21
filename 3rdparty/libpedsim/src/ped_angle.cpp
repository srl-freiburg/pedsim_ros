//
// pedsim - A microscopic pedestrian simulation system.
// Copyright (c) 2003 - 2012 by Christian Gloor
//

#include "pedsim/ped_angle.h"

bool Ped::Tangle::operator==(const Ped::Tangle& otherIn) const {
  return value == otherIn.value;
}

bool Ped::Tangle::operator!=(const Ped::Tangle& otherIn) const {
  return value != otherIn.value;
}

bool Ped::Tangle::operator<(const Ped::Tangle& otherIn) const {
  return value < otherIn.value;
}

bool Ped::Tangle::operator<=(const Ped::Tangle& otherIn) const {
  return value <= otherIn.value;
}

bool Ped::Tangle::operator>(const Ped::Tangle& otherIn) const {
  return value > otherIn.value;
}

bool Ped::Tangle::operator>=(const Ped::Tangle& otherIn) const {
  return value >= otherIn.value;
}
