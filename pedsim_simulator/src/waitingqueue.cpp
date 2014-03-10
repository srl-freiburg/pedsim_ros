
#include "waitingqueue.h"

WaitingQueue::WaitingQueue(const double x, const double y)
    : x_(x), y_(y)
{
    people_.clear();
}

WaitingQueue::~WaitingQueue()
{
    people_.clear();
}


void WaitingQueue::enqueueAgent(Agent a)
{
    // manipulate the forces on the agent

    // set position to the end of the queue

}

void WaitingQueue::serveAgent(size_t agent_id)
{

}