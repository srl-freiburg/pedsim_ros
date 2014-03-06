//
// pedsim - A microscopic pedestrian simulation system.
// Copyright (c) 2003 - 2013 by Christian Gloor
//
// Use somethin like this to compile:
// g++ examples/example.cpp -o example -I. -lpedsim -L. -g
//
// Check for memory leaks e.g. like this:
// valgrind --leak-check=yes ./example

#include "ped_includes.h"

#include <iostream>
#include <cstdlib> // rand

using namespace std;

int main(int argc, char *argv[]) {

    cout << "PedSim Example using libpedsim version " << Ped::LIBPEDSIM_VERSION << endl;

    // setup
    Ped::Tscene *pedscene = new Ped::Tscene(-200, -200, 400, 400);

    Ped::Twaypoint *w1 = new Ped::Twaypoint(-100, 0, 24);
    Ped::Twaypoint *w2 = new Ped::Twaypoint(+100, 0, 12);


    Ped::Tobstacle *o = new Ped::Tobstacle(0, -50,  0, +50);
    pedscene->addObstacle(o);

    for (int i = 0; i<10; i++) {
        Ped::Tagent *a = new Ped::Tagent();

        a->addWaypoint(w1);
        a->addWaypoint(w2);

        a->setPosition(-50 + rand()/(RAND_MAX/80)-40, 0 + rand()/(RAND_MAX/20) -10, 0);

        pedscene->addAgent(a);
    }

    // move all agents for 10 steps (and print their position)
    for (int i=0; i<10; ++i) {
        pedscene->moveAgents(0.2);

        const vector<Ped::Tagent*>& myagents = pedscene->getAllAgents();
        for (vector<Ped::Tagent*>::const_iterator iter = myagents.begin(); iter != myagents.end(); ++iter) {
            cout << (*iter)->getx() << "/" << (*iter)->gety() << endl;
        }
    }

    // cleanup
    const vector<Ped::Tagent*>& myagents = pedscene->getAllAgents();
    for (vector<Ped::Tagent*>::const_iterator iter = myagents.begin(); iter != myagents.end(); ++iter) {
        delete *iter;
    }
    delete pedscene;
    delete w1;
    delete w2;
    delete o;

}
