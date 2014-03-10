#ifndef WAITING_QUEUE_H
#define WAITING_QUEUE_H 

#include "agent.h"


class WaitingQueue
{
public:
    WaitingQueue(const double x, const double y);
    ~WaitingQueue();

    void enqueueAgent(Ped::Tagent* a);
    void serveAgent(size_t agent_id); 

private:
    // queue service location
    double x_, y_;

    // average wait time per person
    double wait_time_;

    std::list<Ped::Tagent*> people_;
        

    void updateQueue();
    void releaseAgent(Ped::Tagent* a);
};


#endif