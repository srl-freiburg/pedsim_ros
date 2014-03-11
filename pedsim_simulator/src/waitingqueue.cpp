
#include "waitingqueue.h"
#include "config.h"



WaitingQueue::WaitingQueue(const double x, const double y)
    : x_(x), y_(y)
{
    people_.clear();
    wait_time_ = 5;
    time_passed_ = 0;
    theta_ = M_PI;
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
    Ped::Tvector qend = getQueueEnd();
    a->setPosition(qend.x, qend.y, qend.z);
}

void WaitingQueue::serveAgent(size_t agent_id)
{
    if (time_passed_ >= wait_time_ && people_.size() > 0)
    {
        // remove top agent from queue
        Ped::Tagent* lucky_one = people_.front();
        releaseAgent(lucky_one);

        // update queue
        updateQueue(lucky_one->getx(), lucky_one->gety());
    }
    
    time_passed_++;
}


 void WaitingQueue::updateQueue(double px, double py)
 {
    double prevx = px;
    double prevy = py;

    BOOST_FOREACH(Ped::Tagent* a, people_)
    {
        double ax = a->getx();
        double ay = a->gety();
        
        a->setPosition(prevx, prevy, a->getz());

        prevx = ax;
        prevy = ay;  
    }
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


Ped::Tvector WaitingQueue::getQueueEnd()
{
    if (people_.size() == 0)
    {
        return Ped::Tvector(x_+(0.5*cos(theta_)), y_+(0.5*sin(theta_)), 0.0);
    }
    else
    {
        Ped::Tagent* last_one = people_.back();
        return Ped::Tvector(last_one->getx()+(0.5*cos(theta_)), last_one->gety()+(0.5*sin(theta_)), 0.0);
    }
}