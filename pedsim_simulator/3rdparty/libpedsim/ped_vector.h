//
// pedsim - A microscopic pedestrian simulation system. 
// Copyright (c) 2003 - 2012 by Christian Gloor
//                              

#ifndef _ped_vector_h_ 
#define _ped_vector_h_ 1

#ifdef WIN32
#define LIBEXPORT __declspec(dllexport)
#else
#define LIBEXPORT
#endif

namespace Ped {

/// Vector helper class. This is basically a struct with some related functions attached.
/// x, y, and z are public, so that they can be accessed easily.
/// \author  chgloor
/// \date    2010-02-12
class LIBEXPORT Tvector {
public:

    // Defaul constructor
    Tvector();

    // Initializing constructor
    Tvector(double px, double py, double pz) : x(px), y(py), z(pz) {};

    // Copy constructor
    Tvector(const Tvector& source) : x(source.x), y(source.y), z(source.z) {};

    // Assignment operator
    Tvector& operator= (const Tvector& source) {
        if (this == &source) return *this;
        x = source.x;
        y = source.y;
        z = source.z;
        return *this;
    }

    Tvector& operator+ (const Tvector& source) {
        x += source.x;
        y += source.y;
        z += source.z;
        return *this;
    }

    Tvector& operator- (const Tvector& source) {
        x -= source.x;
        y -= source.y;
        z -= source.z;
        return *this;
    }

    virtual ~Tvector();


    double x;
    double y;
    double z;
    void cross(const Tvector &a, const Tvector &b);
    friend double scalar(const Tvector &a, const Tvector &b);
    void normalize();
};


}


#endif
