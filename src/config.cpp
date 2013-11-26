

// Includes
#include "config.h"


// initialize static value
Config* Config::Config::instance = NULL;


Config::Config() 
{
    simWallForce = 10;
    simPedForce = 5;
    simSpeed = 1000.0/30;
    mlLookAhead = true;
    simh = 0.1;
}

Config& Config::getInstance() {
    if(instance == NULL)
        instance = new Config();

    return *instance;
}
