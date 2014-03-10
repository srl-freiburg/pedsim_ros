
#include "waitingqueue.h"
#include "config.h"



WaitingQueue::WaitingQueue(const double x, const double y)
    : x_(x), y_(y)
{
    people_.clear();
}

WaitingQueue::~WaitingQueue()
{
    people_.clear();
}


void WaitingQueue::enqueueAgent(Ped::Tagent* a)
{
    // manipulate the forces on the agent
    a->setfactorsocialforce(0.0);
    a->setfactorobstacleforce(0.0);
    a->setfactordesiredforce(0.0);
    a->setfactorlookaheadforce(0.0);
    a->setVmax(0.0);

    // set position to the end of the queue
    // ... TODO
}

void WaitingQueue::serveAgent(size_t agent_id)
{

}


 void WaitingQueue::updateQueue()
 {

 }

void WaitingQueue::releaseAgent(Ped::Tagent* a)
{
    // reset the factors for the social forces
    a->setfactorsocialforce(CONFIG.simPedForce);
    a->setfactorobstacleforce(CONFIG.simWallForce);
    a->setfactordesiredforce(1.0);
    a->setfactorlookaheadforce(1.0);

    a->setVmax(randSpeed());

    a->setPosition(x_+a->getRadius(), y_+a->getRadius(), 0);
}