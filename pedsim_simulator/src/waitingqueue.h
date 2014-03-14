#ifndef WAITING_QUEUE_H
#define WAITING_QUEUE_H 

#include "agent.h"
#include <deque>
#include <boost/foreach.hpp>
#include <boost/shared_ptr.hpp>


class WaitingQueue
{
public:
    WaitingQueue();
    WaitingQueue(const double x, const double y);
    WaitingQueue(const double x, const double y, double theta, std::string id);
    ~WaitingQueue();

    void enqueueAgent(Ped::Tagent* a);
    void serveAgent(); 
    bool agentInQueue(Ped::Tagent* a);

public:
    // set of quickies
    void setWaitTime(int wtime) { wait_time_ = wtime; }
    void setOrientation(double theta) { theta_ = theta; }
    void setLocation(double x, double y) { x_ = x; y_ = y; }

    int getWaitTime() { return wait_time_; }
    double getOrientation() { return theta_; }
    double getX() { return x_; }
    double getY() { return y_; }
    int getId() { return id_; }

private:
    // queue service location
    double x_, y_;

    std::string name_;
    int id_;

    // queue direction
    double theta_;

    // average wait time per person
    int wait_time_;

    // time passed since last service
    int time_passed_;

    std::deque<Ped::Tagent*> queueing_agents_;
        
    void updateQueue(double px, double py);
    void releaseAgent(Ped::Tagent* a);
    Ped::Tvector getQueueEnd();
};


typedef boost::shared_ptr<WaitingQueue> WaitingQueuePtr;
typedef boost::shared_ptr<WaitingQueue const> WaitingQueueConstPtr;


#endif