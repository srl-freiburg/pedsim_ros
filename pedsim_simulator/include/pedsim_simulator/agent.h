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


#ifndef _agent_h_
#define _agent_h_

#include <libpedsim/ped_agent.h>
#include <libpedsim/ped_vector.h>

#include <QList>

#include <pedsim_simulator/statemachine.h>

// Forward Declarations
class Scene;
class Waypoint;


class Agent : public Ped::Tagent 
{
public:
    Agent(double xIn = 0, double yIn = 0);
    virtual ~Agent();

    /// Methods
    void move(double h);
    Ped::Tvector socialForce() const;
    Ped::Tvector obstacleForce() const;
    Ped::Tvector desiredForce();
    Ped::Tvector lookaheadForce(Ped::Tvector desired) const;
    Ped::Tvector myForce(Ped::Tvector desired) const;

    /// Ped::Tagent Overrides/Overloads
public:
    void addWaypoint(Waypoint* waypointIn);
    void setPosition(double px, double py);
    void setX(double xIn);
    void setY(double yIn);
    void setType(Ped::Tagent::AgentType t);
	void updateState(int event = 0);
	
	StateMachine::State status() { return state_machine_->getCurrentState(); }
	StateMachine::State prevStatus() { return state_machine_->getPreviousState(); }

private:
	StateMachinePtr state_machine_;
};

#endif
