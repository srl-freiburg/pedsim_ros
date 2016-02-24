/**
* Copyright 2014 Social Robotics Lab, University of Freiburg
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
*    # Redistributions of source code must retain the above copyright
*       notice, this list of conditions and the following disclaimer.
*    # Redistributions in binary form must reproduce the above copyright
*       notice, this list of conditions and the following disclaimer in the
*       documentation and/or other materials provided with the distribution.
*    # Neither the name of the University of Freiburg nor the names of its
*       contributors may be used to endorse or promote products derived from
*       this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
* \author Billy Okal <okal@cs.uni-freiburg.de>
*/

#ifndef UTILITIES_H
#define UTILITIES_H

#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>
#include <boost/random/uniform_real.hpp>
#include <boost/random/exponential_distribution.hpp>

#include <boost/shared_ptr.hpp>
#include <boost/foreach.hpp>

#include <ctime>

static boost::mt19937 generator;

/// --------------------------------------
/// \brief Generate a random number in a
/// \details range over uniform distribution
/// \param[in] min - minimum value
/// \param[in] max - maximum value
/// \return random number
/// --------------------------------------
inline double randRange(double min, double max)
{
    boost::uniform_real<> uni_dist(min, max);
    boost::variate_generator<boost::mt19937&, boost::uniform_real<> > uni(generator, uni_dist);

    return uni();
}

inline double coinFlip()
{
    return randRange(0, 1);
}

inline double randService(double service_rate)
{
    boost::exponential_distribution<> exp_dist(service_rate);
    boost::variate_generator<boost::mt19937&, boost::exponential_distribution<> > expd(generator, exp_dist);

    return expd();
}

/// --------------------------------------
/// \brief Euclidean distance between two points
/// --------------------------------------
inline double distance(double x1, double y1, double x2, double y2)
{
    return sqrt(((x1 - x2) * (x1 - x2)) + ((y1 - y2) * (y1 - y2)));
}

/// --------------------------------------
/// \enum RobotMode
/// \brief Robot control mode
/// --------------------------------------
enum class RobotMode {
    CONTROLLED = 0,
    TELEOPERATION = 1,
    SOCIAL_DRIVE = 2
};

/// --------------------------------------
/// \struct ForceWeight
/// \brief Scaling factors for forces
/// --------------------------------------
struct ForceWeight {
    float desired;
    float social;
    float obstacle;
    float group_gaze;
    float group_cohesion;
    float group_repulsion;

    ForceWeight()
    {
        desired = 1.0;
        obstacle = 10.0;
        social = 2.1;
        group_cohesion = 1.0;
        group_gaze = 1.0;
        group_repulsion = 1.0;
    }
};

typedef boost::shared_ptr<ForceWeight> ForceWeightPtr;
typedef boost::shared_ptr<ForceWeight const> ForceWeightConstPtr;

/// --------------------------------------
/// \struct Location
/// \brief 2D location/cell
/// --------------------------------------
struct Location {
    float x;
    float y;

    Location(float xx, float yy)
        : x(xx)
        , y(yy)
    {
    }

    bool operator==(Location a)
    {
        if (x == a.x && y == a.y)
            return true;
        else
            return false;
    }
};

#endif
