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

#ifndef STATEMACHINE_H
#define STATEMACHINE_H

#include <boost/shared_ptr.hpp>



/// -----------------------------------------------------------------
/// \class AgentState
/// \brief Simple agent state machine
/// \details Agents transition between states based on events triggered 
/// -----------------------------------------------------------------
class StateMachine
{
public:

    /// \enum State
    /// \brief Agent state
    enum State
    {
        IDLE = 0,
        WALKING = 1,
        QUEUEING = 2,
		STANDING = 3,
		GROUPING = 4
    };


    /// \enum Event
    /// \brief Agent events to transition between states
    enum Event
    {
		NO_EVENT = 0,
        START_WALKING = 1,
        STOP_WALKING = 2,
        JOIN_QUEUE = 3,
        LEAVE_QUEUE = 4,
		SPAWN = 5
    };


    StateMachine()
	{
		current_state_ = IDLE;
		previous_state_ = IDLE;
	}

	StateMachine ( State st )
		: current_state_ ( st )
	{
		previous_state_ = IDLE;
	}

	virtual ~StateMachine() {}
	
	
    inline void doStateTransition ( Event triggered_event );
    inline State getCurrentState();
    inline State getPreviousState();
    inline void reset();


private:

    State current_state_;
    State previous_state_;
};



typedef boost::shared_ptr<StateMachine> StateMachinePtr;
typedef boost::shared_ptr<StateMachine const> StateMachineConstPtr;








/// -----------------------------------------------------------------
/// \brief State Transitions
/// \details Change the agent state based on a triggered event
/// \param[in] triggered_event event for changing state
/// -----------------------------------------------------------------
void StateMachine::doStateTransition ( Event triggered_event )
{
    /// Spawning transitions
    if ( triggered_event == SPAWN )
    {
        current_state_ = WALKING;
        previous_state_ = IDLE;
    }

    /// Idle transitions
    if ( current_state_ == IDLE && triggered_event == START_WALKING )
    {
        previous_state_ = IDLE;
        current_state_ = WALKING;
        return;
    }

    if ( current_state_ == IDLE && triggered_event == STOP_WALKING )
    {
        previous_state_ = IDLE;
        current_state_ = STANDING;
        return;
    }

    /// Walking transitions
    if ( current_state_ == WALKING )
    {
        // avoid immediate re-joining of queues
        if ( previous_state_ == QUEUEING )
            return;

        if ( triggered_event == JOIN_QUEUE )
        {
            previous_state_ = WALKING;
            current_state_ = QUEUEING;
        }
        else if ( triggered_event == STOP_WALKING )
        {
            previous_state_ = WALKING;
            current_state_ = STANDING;
        }
        // to allow rejoining of queues
        else if ( triggered_event == START_WALKING )
        {
            previous_state_ = WALKING;
            current_state_ = WALKING;
        }

        return;
    }

    /// Queueing transitions
    if ( current_state_ == QUEUEING && triggered_event == LEAVE_QUEUE )
    {
        previous_state_ = QUEUEING;
        current_state_ = WALKING;
        return;
    }

    /// Standing transitions
    if ( current_state_ == STANDING && triggered_event == START_WALKING )
    {
        previous_state_ = STANDING;
        current_state_ = WALKING;
        return;
    }

    return;
}

StateMachine::State StateMachine::getCurrentState()
{
    return current_state_;
}

StateMachine::State StateMachine::getPreviousState()
{
    return previous_state_;
}

void StateMachine::reset()
{
    current_state_ = IDLE;
    previous_state_ = IDLE;
}


#endif // STATEMACHINE_H
