
#include "waitingqueue.h"
#include "config.h"


WaitingQueue::WaitingQueue()
{
    queueing_agents_.clear();
    wait_time_ = 30;
    time_passed_ = 0;
    theta_ = -M_PI/6;
    id_ = "_";
}

WaitingQueue::WaitingQueue(const double x, const double y)
    : x_(x), y_(y)
{
    queueing_agents_.clear();
    wait_time_ = 30;
    time_passed_ = 0;
    theta_ = -M_PI/6;
    id_ = "_";
}

WaitingQueue::WaitingQueue(const double x, const double y, double theta, std::string id)
    : x_(x), y_(y), theta_(theta), id_(id)
{
    queueing_agents_.clear();
    wait_time_ = 30;
    time_passed_ = 0;
}

WaitingQueue::~WaitingQueue()
{
    queueing_agents_.clear();
}


void WaitingQueue::enqueueAgent(Ped::Tagent* a)
{
    if (queueing_agents_.size() == 15)   //TEMP limit queue size
        return;

    // make the agent stop
    a->setStationary();

    // set position to the end of the queue
    Ped::Tvector qend = getQueueEnd();
    a->setPosition(qend.x, qend.y, qend.z);

    queueing_agents_.push_back(a);
}   

void WaitingQueue::serveAgent()
{
    if (time_passed_ >= wait_time_ && queueing_agents_.size() > 0)
    {
        // std::cout << "Serving an agent " << std::endl;

        // remove top agent from queue
        Ped::Tagent* lucky_one = queueing_agents_.front();
        releaseAgent(lucky_one);

        // update queue
        // updateQueue(lucky_one->getx(), lucky_one->gety());

        time_passed_ = 0;
    }
    else if (time_passed_ < wait_time_ && queueing_agents_.size() > 0)
    {
        time_passed_++;
    }
}


bool WaitingQueue::agentInQueue(Ped::Tagent* a)
{
    bool inqueue = false;
    BOOST_FOREACH(Ped::Tagent* p, queueing_agents_)
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

    BOOST_FOREACH(Ped::Tagent* a, queueing_agents_)
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
    a->setPosition(x_+a->getRadius()+10, y_+a->getRadius()+10, 0);
}


Ped::Tvector WaitingQueue::getQueueEnd()
{
    double buffer = 0.5;
    if (queueing_agents_.size() == 0)
    {
        return Ped::Tvector(
            x_+(buffer*cos(theta_)), 
            y_+(buffer*sin(theta_)), 
            0.0);
    }
    else
    {
        Ped::Tagent* last_one = queueing_agents_.back();
        return Ped::Tvector(
            last_one->getx()+(buffer*cos(theta_)), 
            last_one->gety()+(buffer*sin(theta_)), 
            0.0);
    }
}