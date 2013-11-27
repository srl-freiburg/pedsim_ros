

// Includes
#include "config.h"


// initialize static value
Config* Config::Config::instance = NULL;


Config::Config() 
{
    simWallForce = 10;
    simPedForce = 10;
    simSpeed = 0.05;
    mlLookAhead = true;
    simh = 0.1;
}

Config& Config::getInstance() {
    if(instance == NULL)
        instance = new Config();

    return *instance;
}
