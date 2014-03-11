#ifndef WAITING_QUEUE_H
#define WAITING_QUEUE_H 

#include "agent.h"
#include <deque>
#include <boost/foreach.hpp>


class WaitingQueue
{
public:
    WaitingQueue(const double x, const double y);
    ~WaitingQueue();

    void enqueueAgent(Ped::Tagent* a);
    void serveAgent(size_t agent_id); 

    void setWaitTime(int wtime) { wait_time_ = wtime; }

private:
    // queue service location
    double x_, y_;

    // queue direction
    double theta_;

    // average wait time per person
    int wait_time_;

    // time passed since last service
    int time_passed_;

    std::deque<Ped::Tagent*> people_;
        

    void updateQueue(double px, double py);
    void releaseAgent(Ped::Tagent* a);
    Ped::Tvector getQueueEnd();
};


#endif