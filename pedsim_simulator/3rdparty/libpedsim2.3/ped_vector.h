//
// pedsim - A microscopic pedestrian simulation system.
// Copyright (c) 2003 - 2014 by Christian Gloor
//

#ifndef _ped_vector_h_
#define _ped_vector_h_ 1

#ifdef WIN32
#define LIBEXPORT __declspec(dllexport)
#else
#define LIBEXPORT
#endif

#include <cmath>

namespace Ped {
    /// Vector helper class. This is basically a struct with some related functions attached.
    /// x, y, and z are public, so that they can be accessed easily.
    /// \author  chgloor
    /// \date    2010-02-12
    class LIBEXPORT Tvector {
    public:
        // Default constructor
        Tvector();

        // Initializing constructor
        Tvector(double px, double py, double pz = 0) : x(px), y(py), z(pz) {};


        // Methods
        double length() const;
        double lengthSquared() const;
        void normalize();
        Tvector normalized() const;
        void scale(double factor);
        Tvector scaled(double factor) const;

        Tvector leftNormalVector() const;
        Tvector rightNormalVector() const;

        double polarRadius() const;
        double polarAngle() const;

        double angleTo(const Tvector &other) const;

        static double scalar(const Tvector &a, const Tvector &b);
        static double dotProduct(const Tvector &a, const Tvector &b);
        static Tvector crossProduct(const Tvector &a, const Tvector &b);


        // Operators
        Tvector operator+(const Tvector& other) const;
        Tvector operator-(const Tvector& other) const;
        Tvector operator*(double factor) const;
        Tvector operator/(double divisor) const;
        Tvector& operator+=(const Tvector& vectorIn);
        Tvector& operator-=(const Tvector& vectorIn);
        Tvector& operator*=(double factor);
        Tvector& operator*=(const Tvector& vectorIn);
        Tvector& operator/=(double divisor);


        // Attributes
        double x;
        double y;
        double z;
    };
}

bool operator==(const Ped::Tvector& vector1In, const Ped::Tvector& vector2In);
bool operator!=(const Ped::Tvector& vector1In, const Ped::Tvector& vector2In);
Ped::Tvector operator-(const Ped::Tvector& vectorIn);
Ped::Tvector operator*(double factor, const Ped::Tvector& vector);



// Puts angle alpha into the interval [min..min+2*pi[
inline double normAngle(double alpha, double min) 
{
    while (alpha >= min+2.0*M_PI) {
        alpha -= 2.0*M_PI;
    }
    while (alpha < min) {
        alpha += 2.0*M_PI;
    }
    return alpha;
};

inline double diffAngle(double alpha1, double alpha2) 
{
    double delta;

    // normalize angles alpha1 and alpha2
    alpha1 = normAngle(alpha1, 0);
    alpha2 = normAngle(alpha2, 0);

    // take difference and unwrap
    delta = alpha1 - alpha2;
    if (alpha1 > alpha2) {
        while (delta > M_PI) {
            delta -= 2.0*M_PI;
        }
    }
    else if (alpha2 > alpha1) {
        while (delta < -M_PI) {
            delta += 2.0*M_PI;
        }
    }
    return delta;
};

// angle utils
inline double angleBetween(Ped::Tvector a, Ped::Tvector b)
{
    a.normalize();
    b.normalize();

    double dt = (a.x * b.x) + (a.y * b.y);
    double angle_dot =  acos(dt);
    angle_dot = normAngle(angle_dot, 0);

    double ac1, ac2;
    ac1 = atan2(a.y, a.x);
    ac2 = atan2(b.y, b.x);

    double dangle = diffAngle(ac1, ac2);
    dangle = normAngle(dangle, 0);

    return dangle;
}




#endif
