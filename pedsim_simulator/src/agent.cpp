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


#include <pedsim_simulator/agent.h>
#include <pedsim_simulator/config.h>
#include <pedsim_simulator/waypoint.h>


Agent::Agent ( double xIn, double yIn )
    : Ped::Tagent()
{
    // initialize Ped::Tagent
    Ped::Tagent::setType ( Ped::Tagent::ADULT );
    state_machine_.reset ( new StateMachine ( StateMachine::WALKING ) );
	time_since_queue_ = 0;
	
	// everyone spawns once
	state_machine_->doStateTransition( StateMachine::SPAWN );
};

Agent::~Agent()
{
}

/// Calculates the social force. Same as in lib, but adds graphical representation
Ped::Tvector Agent::socialForce() const
{
    return Tagent::socialForce();
}

/// Calculates the obstacle force. Same as in lib, but adds graphical representation
Ped::Tvector Agent::obstacleForce() const
{
    return Tagent::obstacleForce();
}

/// Calculates the desired force. Same as in lib, but adds graphical representation
Ped::Tvector Agent::desiredForce()
{
    return Tagent::desiredForce();
}

/// Calculates the look ahead force. Same as in lib, but adds graphical representation
Ped::Tvector Agent::lookaheadForce ( Ped::Tvector desired ) const
{
    Ped::Tvector t;
    if ( CONFIG.look_ahead == true )
    {
        t = Tagent::lookaheadForce ( desired );
    }
    return t;
}

/// \brief Group gaze force 
/// Defined according to Helbing et. al paper
Ped::Tvector Agent::groupGazeForce ()
{
	Ped::Tvector t;

    return t;
}

Ped::Tvector Agent::groupCoherenceForce ()
{
	Ped::Tvector t;

    return t;
}

Ped::Tvector Agent::groupRepulsionForce ()
{
	Ped::Tvector t;

    return t;
}
	

/// \brief Place holder to adding forces for controlling agents
/// Use for group forces and other behaviours
Ped::Tvector Agent::myForce ( Ped::Tvector desired ) const
{
    Ped::Tvector t;

    return t;
}

void Agent::updateState(int event)
{
    if ( getteleop() == true )
        state_machine_->reset();

    // check state
	StateMachine::Event ev = static_cast<StateMachine::Event>(event);
    if ( ev == StateMachine::LEAVE_QUEUE )
	{
		// reset the re-join count
		time_since_queue_ = 0;
	}
	
	state_machine_->doStateTransition ( ev );
	
	if (time_since_queue_ > CONFIG.queue_break)
	{
		state_machine_->doStateTransition( StateMachine::START_WALKING );
	}
	
	
	/// NOTE - temporary check for standing agents
	if ( gettype() == Ped::Tagent::ELDERLY )
	{
		state_machine_->doStateTransition( StateMachine::STOP_WALKING );
	}
}

/// \brief Compute the forces that drive the agent
/// Overload in the internal method to allow to add 
/// additional forces into the control game
void Agent::computeForces ()
{
	// compute the basic social forces
	Ped::Tagent::computeForces();
	
	// add additionan group and flock forces here
	myForce (Ped::Tvector (0,0,0) );
}

/// \brief Move the agents in one time step
void Agent::move ( double h )
{
	computeForces();
	
    // use SFM as local controller for the robot
    if ( Tagent::gettype() == Ped::Tagent::ROBOT )
    {
        Ped::Tagent::setfactorsocialforce ( CONFIG.force_weights->social );
        Ped::Tagent::setfactorobstacleforce ( 350 );
        Ped::Tagent::setfactordesiredforce ( 1.5 );
    }

    // those in queues behave differently
    if ( state_machine_->getCurrentState() == StateMachine::QUEUEING )
	{
// 		Ped::Tagent::setfactorsocialforce ( CONFIG.force_weights->social / 10.0 );
//         Ped::Tagent::setfactorobstacleforce ( CONFIG.force_weights->obstacle );
//         Ped::Tagent::setfactordesiredforce ( CONFIG.force_weights->desired );
// 		Ped::Tagent::setVmax( 0.05 );
		
// 		Ped::Tagent::move ( h );
	}
	
	// standing ones remain standing
    if ( state_machine_->getCurrentState() == StateMachine::STANDING )
	{
		Ped::Tagent::setfactorsocialforce ( 0.0 );
        Ped::Tagent::setfactorobstacleforce ( 0.0 );
        Ped::Tagent::setfactordesiredforce ( 0.0 );
		Ped::Tagent::setVmax( 0.01 );
	}
	
    
    if ( state_machine_->getCurrentState() == StateMachine::WALKING )
	{
		// we only work with non-lazy people
        Ped::Tagent::move ( h );
	}
	
	time_since_queue_++;

}

void Agent::addWaypoint ( Waypoint* waypointIn )
{
    // call the original method
    Ped::Tagent::addWaypoint ( waypointIn );
}

void Agent::setPosition ( double px, double py )
{
    // call super class' method
    Ped::Tagent::setPosition ( px, py, 0 );
}


void Agent::setX ( double xIn )
{
    setPosition ( xIn, gety() );
}

void Agent::setY ( double yIn )
{
    setPosition ( getx(), yIn );
}

void Agent::setType ( Ped::Tagent::AgentType t )
{
    // call super class' method
    Ped::Tagent::setType ( t );
}


std::list<Agent*> Agent::getNeighbors () const 
{
	// upcast neighbors
	std::list<Agent*> output;
	BOOST_FOREACH ( const Ped::Tagent* neighbor, neighbors ) 
	{
		Agent* upNeighbor = const_cast<Agent*>( dynamic_cast<const Agent*>( neighbor ) );
		if ( upNeighbor != NULL )
			output.push_back( upNeighbor );
	}

	return output;
}