// Simulating Group Dynamics in Crowds of Pedestrians (SGDiCoP)
// Author: Sven Wehner <mail@svenwehner.de>
//
// Forked from: PedSim's demo application (version 2.2).
// (http://pedsim.silmaril.org/)
// Copyright text:
// 		pedsim - A microscopic pedestrian simulation system.
// 		Copyright (c) 2011 by Christian Gloor

#ifndef _config_h_
#define _config_h_

// Includes
// → Qt
#include <QObject>
#include <map>
#include <vector>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>


typedef struct Location {
    float x;
    float y;

    Location(float xx, float yy) : x(xx), y(yy) {}

    bool operator ==(Location a) {
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



//    // Slots
//public slots:
//    void setGuiShowWaypoints(bool value);
//    void setSimWallForce(double value);
//    void setSimPedForce(double value);
//    void setSimSpeed(int value);


    // Attributes
public:
    bool guiShowWaypoints;
    double simWallForce;
    double simPedForce;
    int simSpeed;
    bool mlLookAhead;
    bool showForces;
    bool showDirection;
    double simh;
    double width;
    double height;

    std::vector<TLoc> obstacle_positions;
    std::string config_file;

    void readParameters(std::string filename);


    /// feature thresholds
    double *angles;
    double *densities;
    double *velocities;


};


#endif
