
#include "ped_angle.h"

bool Ped::Tangle::operator==(const Ped::Tangle& otherIn) const 
{
    return value == otherIn.value;
}

bool Ped::Tangle::operator!=(const Ped::Tangle& otherIn) const 
{
    return value != otherIn.value;
}

bool Ped::Tangle::operator<(const Ped::Tangle& otherIn) const 
{
    return value < otherIn.value;
}

bool Ped::Tangle::operator<=(const Ped::Tangle& otherIn) const 
{
    return value <= otherIn.value;
}

bool Ped::Tangle::operator>(const Ped::Tangle& otherIn) const 
{
    return value > otherIn.value;
}

bool Ped::Tangle::operator>=(const Ped::Tangle& otherIn) const 
{
    return value >= otherIn.value;
}
