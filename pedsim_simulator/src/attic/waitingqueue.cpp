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
	
	// TODO - change these to use QueueParameters struct
    wait_time_ = 30;
    time_passed_ = 0;
    theta_ = -M_PI / 6;
    name_ = "_";
	radius_ = 1.5;
	threshold_ = 0.5;
	alpha_ = M_PI / 2.0;
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
	radius_ = 1.5;
	threshold_ = 0.5;
	alpha_ = M_PI / 2.0;
}

WaitingQueue::WaitingQueue ( const double x, const double y, const double theta, std::string name )
    : x_ ( x ), y_ ( y ), theta_ ( theta ), name_ ( name )
{
    static int staticid = 0;
    id_ = staticid++;

    queueing_agents_.clear();
    wait_time_ = 30;
    time_passed_ = 0;
	radius_ = 1.5;
	threshold_ = 0.5;
	alpha_ = M_PI / 2.0;
}

WaitingQueue::~WaitingQueue()
{
    queueing_agents_.clear();
}


/// -----------------------------------------------------------------
/// \brief Pick an agent in the neighborhood
/// \details flip a coin and add an agent to queue in the range given
/// and direction specified \f$ U(a,b) \f$
/// \param[in] neighbors Agents near the last member in the queue
/// \return pointer to selected agent
/// -----------------------------------------------------------------
Agent* WaitingQueue::pickAgent( std::list<Agent*> neighbors , double x, double y )
{
	BOOST_FOREACH( Agent* agent, neighbors )
	{
		if ( agent->gettype() == Ped::Tagent::ROBOT || agentInQueue ( agent ) == true )
			continue;
		
		double d = distance( x, y,  agent->getx(), agent->gety() );
		double theta_p = atan2( agent->gety()-y, agent->getx()-x );
		
		double base = theta_ - (alpha_/2.0);
		
		if ( d < getRadius() && coinFlip() >= getThreshold() )
		{
			if ( theta_p >= base && theta_p <= ( base + alpha_ ) )
			{
				return agent;
			}
		}
	}
	
	return NULL;
}


/// -----------------------------------------------------------------
/// \brief Add agent to the queue
/// \details Adds agent to the back of the queue and updates its state 
/// machine to reflect new status
/// \param[in] a agent pointer
/// -----------------------------------------------------------------
void WaitingQueue::enqueueAgent ( Agent *a )
{
    if ( queueing_agents_.size() == CONFIG.queue_max_length ) 
        return;
	
	// NOTE - new way with follow id
    Ped::Tvector qend = getQueueEnd();
	
	a->setPosition ( qend.x, qend.y );

//     Waypoint *w = new Waypoint ( "-", qend.x, qend.y, 0.1 );
// //     w->settype ( Ped::Twaypoint::TYPE_QUEUE );
//     a->addWaypoint ( w );
// 	
	a->setStationary();
	a->updateState( StateMachine::JOIN_QUEUE ); 

    queueing_agents_.push_back ( a );
}



/// -----------------------------------------------------------------
/// \brief Serve an agent at the queue counter
/// \details Simulates a counter. Waiting time is distribtued based
/// on a negative exponential distribution
/// -----------------------------------------------------------------
void WaitingQueue::serveAgent()
{
	// TODO - change wait time to have exponential distribution
	int service_time = wait_time_ +  (int)randService(wait_time_);
    if ( queueing_agents_.size() > 0 && time_passed_ >= service_time )
    {
        // remove top agent from queue
        Agent* lucky_one = queueing_agents_.front();

        // update queue
        updateQueue(lucky_one->getx(), lucky_one->gety());

		releaseAgent ( lucky_one );
		
        time_passed_ = 0;
    }
    else if ( queueing_agents_.size() > 0 && time_passed_ < service_time )
    {
        time_passed_++;
    }
}


/// -----------------------------------------------------------------
/// \brief Check if an agent is in the queue
/// \param[in] a agent pointer
/// \return boolean flag indicative agent presence in the queue
/// -----------------------------------------------------------------
bool WaitingQueue::agentInQueue ( Agent *a )
{
    bool inqueue = false;
	// TODO - change to a more efficient std::algorithm::find
    BOOST_FOREACH ( Agent * p, queueing_agents_ )
    {
        if ( a->getid() == p->getid() )
            inqueue = true;
    }
    return inqueue;
}


/// -----------------------------------------------------------------
/// \brief Update the queue by moving agents one queue step ahead
/// \param[in] px X position of the last release agent
/// \param[in] py Y position of the last release agent
/// -----------------------------------------------------------------
void WaitingQueue::updateQueue ( double px, double py )
{
    double prevx = px;
    double prevy = py;
	
//     BOOST_FOREACH ( Agent * a, queueing_agents_ )
//     {
//         double ax = a->getx();
//         double ay = a->gety();
// 
//         a->setPosition ( prevx, prevy ); 
// 		
// // 		Ped::Twaypoint* d1 = a->getDestination();
// // 		a->removeWaypoint(d1);
// 
// // 		Ped::Twaypoint* d2 = a->getDestination();
// // 		a->removeWaypoint(d2);
// 		
// 		
// // 		Waypoint *w = new Waypoint ( "-", prevx, prevy, 1.0 );
// // 		w->settype(Ped::Twaypoint::TYPE_QUEUE);
// // 		a->addWaypoint(w);
// 
//         prevx = ax;
//         prevy = ay;
//     }
//     
	
	
    std::deque<Agent*>::iterator it = queueing_agents_.begin();
    for ( ; it != queueing_agents_.end(); it++ )
	{
		Agent* a = (*it);
		
		double ax = a->getx();
        double ay = a->gety();
		
		a->setPosition ( prevx, prevy );
		
		prevx = ax;
        prevy = ay;
	}
}



/// -----------------------------------------------------------------
/// \brief Release an agent after being served
/// \param[in] a pointer to the agent to release
/// -----------------------------------------------------------------
void WaitingQueue::releaseAgent ( Agent *a )
{
    // trigger event that transits it state
	a->updateState( StateMachine::LEAVE_QUEUE ); 
	a->setMobile();
	
	// remove the agent from the queue
	queueing_agents_.pop_front();
}


/// -----------------------------------------------------------------
/// \brief Get the coordinates of the end of the queue
/// \return Vector containing the coordinates
/// -----------------------------------------------------------------
Ped::Tvector WaitingQueue::getQueueEnd()
{
    double buffer = randRange(0.25, 0.4);
	double theta_diff = randRange(0, M_PI/15.0);
	
    if ( queueing_agents_.size() == 0 )
    {	
		return Ped::Tvector (x_ , y_ , 0.0 );
    }
    else
    {
		double qlen = queueing_agents_.size() * buffer;
		return Ped::Tvector (
                   x_ + ( qlen * cos ( theta_ + theta_diff ) ),
                   y_ + ( qlen * sin ( theta_ + theta_diff ) ),
                   0.0 );
		
    }
}
