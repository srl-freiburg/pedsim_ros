// Simulating Group Dynamics in Crowds of Pedestrians (SGDiCoP)
// Author: Sven Wehner <mail@svenwehner.de>
//
// Forked from: PedSim's demo application (version 2.2).
// (http://pedsim.silmaril.org/)
// Copyright text:
//      pedsim - A microscopic pedestrian simulation system.
//      Copyright (c) 2011 by Christian Gloor

#ifndef _config_h_
#define _config_h_

// Includes
// → Qt
#include <map>
#include <vector>
#include <cmath>
#include <cstring>


typedef struct Location 
{
    float x;
    float y;

    Location(float xx, float yy) : x(xx), y(yy) {}

    bool operator ==(Location a) 
    {
        if (x == a.x && y == a.y) return true;
        else return false;
    }

} TLoc;


class Config
{
protected:
    Config();

    // Singleton Design Pattern
#define CONFIG Config::getInstance()
protected:
    static Config* instance;
public:
    static Config& getInstance();


    // Attributes
public:
    double simWallForce;
    double simPedForce;
    int simSpeed;
    bool mlLookAhead;
    double simh;
    double width;
    double height;
};


#endif