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



/// \class AgentState
/// \brief Simple agent state machine
class StateMachine
{
public:

    /// \enum State
    /// \brief Agent state
    enum State
    {
        IDLE = 0,
        WALKING = 1,
        QUEUEING = 2
    };


    /// \enum Event
    /// \brief Agent events to transition between states
    enum Event
    {
        START_WALKING = 0,
        STOP_WALKING = 1,
        JOIN_QUEUE = 2,
        LEAVE_QUEUE = 3,
        NO_EVENT = 4
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

    void doStateTransition ( Event triggered_event )
    {
        /// Idle transitions
        if ( current_state_ == IDLE && triggered_event == START_WALKING )
        {
            previous_state_ = IDLE;
            current_state_ = WALKING;
            return;
        }

        /// Walking transitions
        if ( current_state_ == WALKING )
        {
            if ( triggered_event == JOIN_QUEUE )
            {
                previous_state_ = WALKING;
                current_state_ = QUEUEING;
            }
            else if ( triggered_event == START_WALKING )
            {
                previous_state_ = WALKING;
                current_state_ = IDLE;
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

        return;
    }

    State getCurrentState()
    {
        return current_state_;
    }

    void reset()
    {
        current_state_ = IDLE;
        previous_state_ = IDLE;
    }


private:

    State current_state_;
    State previous_state_;
};



typedef boost::shared_ptr<StateMachine> StateMachinePtr;
typedef boost::shared_ptr<StateMachine const> StateMachineConstPtr;



// #include <boost/statechart/event.hpp>
// #include <boost/statechart/state_machine.hpp>
// #include <boost/statechart/simple_state.hpp>
// #include <boost/statechart/transition.hpp>
//
// #include <boost/config.hpp>
// #include <boost/mpl/list.hpp>
//
// #ifdef BOOST_NO_STDC_NAMESPACE
// namespace std
// {
//   using ::time;
//   using ::difftime;
//   using ::time_t;
// }
// #endif
//
// #ifdef BOOST_INTEL
// #  pragma warning( disable: 304 ) // access control not specified
// #  pragma warning( disable: 444 ) // destructor for base is not virtual
// #  pragma warning( disable: 981 ) // operands are evaluated in unspecified order
// #endif
//
// namespace sc = boost::statechart;
//
// /// Events across states
// struct StartWalking : sc::event<StartWalking> {};
// struct StopWalking : sc::event<StopWalking> {};
// struct JoinQueue : sc::event<JoinQueue> {};
// struct LeaveQueue : sc::event<LeaveQueue> {};
//
//
// /// States the agents transition over
// struct Stationary;
// struct Walking;
// struct Alive;
// // struct Dead;
// struct Queuing;
//
//
// /// state machine
// struct AgentMachine : sc::state_machine< AgentMachine, Alive > {};
//
//
// /// Ask Schroedinger???
// // struct Dead : sc::simple_state<Dead, Alive>
// // {};
//
//
// /// Agent start as being alive and occasionally die (no joke)
// struct Alive: sc::simple_state<Alive, AgentMachine, Alive >
// {};
//
// /// When alive and idle, one can only start walking
// struct Stationary : sc::simple_state<Stationary, Alive>
// {
// public:
// 	typedef sc::transition<StartWalking, Walking> wake_up;
// };
//
// /// When walking, one can either queue of stop walking
// struct Walking : sc::simple_state<Walking, Alive>
// {
// public:
// 	// can either transition into idle or queuing
// 	typedef boost::mpl::list<sc::transition<JoinQueue, Queuing>,
// sc::transition<StopWalking, Stationary> > make_life_choices;
// };
//
// /// When queuing, one can only switch to walking
// struct Queuing : sc::simple_state<Queuing, Alive>
// {
// public:
// 	// can only leave queue to go back to walking
// 	typedef sc::transition<LeaveQueue, Walking> reaction;
// };
//
//




#endif // STATEMACHINE_H
