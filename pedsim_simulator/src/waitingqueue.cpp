
#include "waitingqueue.h"
#include "config.h"



WaitingQueue::WaitingQueue(const double x, const double y)
    : x_(x), y_(y)
{
    people_.clear();
    wait_time_ = 30;
    time_passed_ = 0;
    theta_ = -M_PI/6;
}

WaitingQueue::~WaitingQueue()
{
    people_.clear();
}


void WaitingQueue::enqueueAgent(Ped::Tagent* a)
{
    if (people_.size() == 15)   //TEMP limit queue size
        return;

    // make the agent stop
    a->setStationary();

    // set position to the end of the queue
    Ped::Tvector qend = getQueueEnd();
    a->setPosition(qend.x, qend.y, qend.z);

    // change its color/type
    if (a->gettype() == 0) 
        a->setType(1);
    else 
        a->setType(0);

    people_.push_back(a);
}   

void WaitingQueue::serveAgent()
{
    if (time_passed_ >= wait_time_ && people_.size() > 0)
    {
        std::cout << "Serving an agent " << std::endl;

        // remove top agent from queue
        Ped::Tagent* lucky_one = people_.front();
        releaseAgent(lucky_one);

        // update queue
        // updateQueue(lucky_one->getx(), lucky_one->gety());

        time_passed_ = 0;
    }
    else if (time_passed_ < wait_time_ && people_.size() > 0)
    {
        time_passed_++;
    }
}


bool WaitingQueue::agentInQueue(Ped::Tagent* a)
{
    bool inqueue = false;
    BOOST_FOREACH(Ped::Tagent* p, people_)
    {
        if (a->getid() == p->getid())
            inqueue = true;
    }
    return inqueue;
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
    a->setMobile();

    // a->setVmax(randSpeed());
    a->setPosition(x_+a->getRadius()+10, y_+a->getRadius()+10, 0);

    // restore its color/type
    if (a->gettype() == 0) 
        a->setType(1);
    else 
        a->setType(0);
}


Ped::Tvector WaitingQueue::getQueueEnd()
{
    double buffer = 0.5;
    if (people_.size() == 0)
    {
        return Ped::Tvector(
            x_+(buffer*cos(theta_)), 
            y_+(buffer*sin(theta_)), 
            0.0);

        // return Ped::Tvector(
        //     x_+(buffer), 
        //     y_+(buffer), 
        //     0.0);
    }
    else
    {
        Ped::Tagent* last_one = people_.back();
        return Ped::Tvector(
            last_one->getx()+(buffer*cos(theta_)), 
            last_one->gety()+(buffer*sin(theta_)), 
            0.0);

        // return Ped::Tvector(
        //     last_one->getx()+(buffer), 
        //     last_one->gety()+(buffer), 
        //     0.0);
    }
}