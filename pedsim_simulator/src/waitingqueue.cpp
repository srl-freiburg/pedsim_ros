/**
* Copyright 2014 Social Robotics Lab, University of Freiburg
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
*    # Redistributions of source code must retain the above copyright
*       notice, this list of conditions and the following disclaimer.
*    # Redistributions in binary form must reproduce the above copyright
*       notice, this list of conditions and the following disclaimer in the
*       documentation and/or other materials provided with the distribution.
*    # Neither the name of the University of Freiburg nor the names of its
*       contributors may be used to endorse or promote products derived from
*       this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
* \author Billy Okal <okal@cs.uni-freiburg.de>
*/


#include <pedsim_simulator/waitingqueue.h>
#include <pedsim_simulator/config.h>


WaitingQueue::WaitingQueue()
{
    static int staticid = 0;
    id_ = staticid++;

    queueing_agents_.clear();
    wait_time_ = 30;
    time_passed_ = 0;
    theta_ = -M_PI / 6;
    name_ = "_";
}

WaitingQueue::WaitingQueue ( const double x, const double y )
    : x_ ( x ), y_ ( y )
{
    static int staticid = 0;
    id_ = staticid++;

    queueing_agents_.clear();
    wait_time_ = 30;
    time_passed_ = 0;
    theta_ = -M_PI / 6;
    name_ = "_";
}

WaitingQueue::WaitingQueue ( const double x, const double y, const double theta, std::string name )
    : x_ ( x ), y_ ( y ), theta_ ( theta ), name_ ( name )
{
    static int staticid = 0;
    id_ = staticid++;

    queueing_agents_.clear();
    wait_time_ = 30;
    time_passed_ = 0;
}

WaitingQueue::~WaitingQueue()
{
    queueing_agents_.clear();
}


void WaitingQueue::enqueueAgent ( Agent *a )
{
    if ( queueing_agents_.size() == 15 ) //TEMP limit queue size
        return;

    // make the agent stop
    a->setStationary();

    // set position to the end of the queue
    Ped::Tvector qend = getQueueEnd();
    a->setPosition ( qend.x, qend.y );
	
	a->updateState(); /// TODO - add event information here

    queueing_agents_.push_back ( a );
}

void WaitingQueue::serveAgent()
{
    if ( time_passed_ >= wait_time_ && queueing_agents_.size() > 0 )
    {
        // remove top agent from queue
        Agent *lucky_one = queueing_agents_.front();
        releaseAgent ( lucky_one );

        // update queue
        // updateQueue(lucky_one->getx(), lucky_one->gety());

        time_passed_ = 0;
    }
    else if ( time_passed_ < wait_time_ && queueing_agents_.size() > 0 )
    {
        time_passed_++;
    }
}


bool WaitingQueue::agentInQueue ( Agent *a )
{
    bool inqueue = false;
    BOOST_FOREACH ( Agent * p, queueing_agents_ )
    {
        if ( a->getid() == p->getid() )
            inqueue = true;
    }
    return inqueue;
}

void WaitingQueue::updateQueue ( double px, double py )
{
    double prevx = px;
    double prevy = py;

    BOOST_FOREACH ( Agent * a, queueing_agents_ )
    {
        double ax = a->getx();
        double ay = a->gety();

        a->setPosition ( prevx, prevy );

        prevx = ax;
        prevy = ay;
    }
}

void WaitingQueue::releaseAgent ( Agent *a )
{
    // reset the factors for the social forces
    a->setMobile();
    a->setPosition ( x_ + a->getRadius() + 10, y_ + a->getRadius() + 10 );
	a->updateState(); /// TODO - add event information here
}


Ped::Tvector WaitingQueue::getQueueEnd()
{
    double buffer = 0.5;
    if ( queueing_agents_.size() == 0 )
    {
        return Ped::Tvector (
                   x_ + ( buffer * cos ( theta_ ) ),
                   y_ + ( buffer * sin ( theta_ ) ),
                   0.0 );
    }
    else
    {
        Ped::Tagent *last_one = queueing_agents_.back();
        return Ped::Tvector (
                   last_one->getx() + ( buffer * cos ( theta_ ) ),
                   last_one->gety() + ( buffer * sin ( theta_ ) ),
                   0.0 );
    }
}