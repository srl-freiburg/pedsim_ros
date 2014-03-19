#ifndef UTILITIES_H
#define UTILITIES_H

#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>
#include <boost/random/uniform_real.hpp>
#include <ctime>


static boost::mt19937 generator;

inline double randHeight()
{
    // generator.seed(static_cast<unsigned int>(42));
    boost::normal_distribution<> nd(1.70, 0.30);

    boost::variate_generator<boost::mt19937 &, boost::normal_distribution<> >
var_nor(generator, nd);

    return var_nor();
}

inline double coinFlip()
{
    boost::uniform_real<> uni_dist(0, 1);
    boost::variate_generator<boost::mt19937 &, boost::uniform_real<> >
uni(generator, uni_dist);

    return uni();
}

inline double randRange(double min, double max)
{
	boost::uniform_real<> uni_dist(min, max);
    boost::variate_generator<boost::mt19937 &, boost::uniform_real<> >
uni(generator, uni_dist);

    return uni();
}



/// \brief Euclidean distance between two points
inline double distance(double x1, double x2, double y1, double y2)
{
    return sqrt((x1 - y1) * (x1 - y1) + (x2 - y2) * (x2 - y2));
}


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