#ifndef UTILITIES_H
#define UTILITIES_H

#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>
#include <boost/random/uniform_real.hpp>
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
    boost::variate_generator<boost::mt19937 &, boost::uniform_real<> > uni(generator, uni_dist);

    return uni();
}


inline double coinFlip()
{
	return randRange(0, 1);
}


/// --------------------------------------
/// \brief Euclidean distance between two points
/// --------------------------------------
inline double distance(double x1, double x2, double y1, double y2)
{
    return sqrt((x1 - y1) * (x1 - y1) + (x2 - y2) * (x2 - y2));
}



/// --------------------------------------
/// \enum RobotMode
/// \brief Robot control mode
/// --------------------------------------
enum RobotMode
{
	CONTROLLED = 0,
	TELEOPERATION = 1,
	SOCIAL_DRIVE = 2
};


/// --------------------------------------
/// \struct ForceWeight
/// \brief Scaling factors for forces
/// --------------------------------------
struct ForceWeight
{
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
struct Location 
{
    float x;
    float y;

    Location(float xx, float yy) : x(xx), y(yy) {}

    bool operator ==(Location a) 
    {
        if (x == a.x && y == a.y) return true;
        else return false;
    }
};

#endif