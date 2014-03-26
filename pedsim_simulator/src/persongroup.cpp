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

#include <pedsim_simulator/persongroup.h>



PersonGroup::PersonGroup ()
{
    members_.clear();
}

PersonGroup::PersonGroup ( const std::list<Agent*>& agents )
{
    members_.clear();

    BOOST_FOREACH ( Agent* a, agents )
	{
		a->enableGroupFlag();
		members_.push_back ( a );
	}
}

PersonGroup::~PersonGroup()
{
    members_.clear();
}


/// -----------------------------------------------------------------
/// \brief getMembers
/// \details Return the members of a group
/// \returns list<Agent*> group members
/// -----------------------------------------------------------------
std::list<Agent*>& PersonGroup::getMembers ()
{
    return members_;
}


/// -----------------------------------------------------------------
/// \brief getMembers
/// \details Return the members of a group
/// \returns list<const Agent*> group members (const)
/// -----------------------------------------------------------------
const std::list<Agent*>& PersonGroup::getMembers () const
{
    return members_;
}


/// -----------------------------------------------------------------
/// \brief addMember
/// \details Add an agent to a group
/// \param[in] agent - new agent to be added to the group
/// \returns flag - True if succeeded, false if agent is already in
/// -----------------------------------------------------------------
bool PersonGroup::addMember ( Agent* agent )
{
    if ( isMember ( agent ) )
        return false;
    else
    {
		agent->enableGroupFlag();
        members_.push_back ( agent );
        return true;
    }
}

/// -----------------------------------------------------------------
/// \brief removeMember
/// \details Remove an agent from the group
/// \param[in] agent - agent to be removed
/// \returns flag - True if succeeded, false if agent is not in group
/// -----------------------------------------------------------------
bool PersonGroup::removeMember ( Agent* agent )
{
    // NOTE - naive approach
    // TODO - switch to using agent id instead of pointer
    std::list<Agent*>::iterator it = members_.begin();
    while ( it != members_.end() )
    {
        if ( ( *it )->getid() == agent->getid() )
            break;

        it++;
    }

    if ( it != members_.end() )
    {
        members_.erase ( it );
        return true;
    }
    else
        return false;
}

/// -----------------------------------------------------------------
/// \brief setMembers
/// \details Set the group members effectively remving any existing
/// ones (if any) as the container is cleared.
/// \param[in] agents - list of agents to be added
/// \returns flag - True if succeeded, false is given list is emtpy
/// -----------------------------------------------------------------
bool PersonGroup::setMembers ( const std::list<Agent*>& agents )
{
    if ( agents.size() == 0 )
        return false;
    else
    {
        members_.clear();

        BOOST_FOREACH ( Agent* a, agents )
		{
			a->enableGroupFlag();
			members_.push_back ( a );
		}

        return true;
    }
}


/// -----------------------------------------------------------------
/// \brief isMember
/// \details Test for agent's membership of the group
/// \param[in] agent - agent to be tested
/// \returns flag - True if member, false otherwise
/// -----------------------------------------------------------------
bool PersonGroup::isMember ( Agent* agent )
{
    // TODO - change to a more algorithmic approach with 'find'

    std::list<Agent*>::iterator it = members_.begin();
    while ( it != members_.end() )
    {
        if ( ( *it )->getid() == agent->getid() )
            return true;

        it++;
    }

    return false;
}


/// -----------------------------------------------------------------
/// \brief isEmpty
/// \details Test if group has not agents
/// \returns flag - True if member, false otherwise
/// -----------------------------------------------------------------
bool PersonGroup::isEmpty() const
{
    if ( memberCount() == 0 )
        return true;
    else
        return false;
}


/// -----------------------------------------------------------------
/// \brief memberCount
/// \details Count the number of agents in the group
/// \returns uint8 - number of agents in the group \f$ \[0, N \] \f$
/// -----------------------------------------------------------------
size_t PersonGroup::memberCount() const
{
    return members_.size();
}


/// -----------------------------------------------------------------
/// \brief computeGroupForces
/// \details Invoke computation of all the group forces
/// -----------------------------------------------------------------
void  PersonGroup::computeGroupForces ()
{
	// call individual force functions here
	force_gaze_ = gazeForce();
	force_coherence_ = coherenceForce();
	force_repulsion_ = repulsionForce();
	
	// assign forces to members
	BOOST_FOREACH ( Agent* a, members_ )
	{
		a->setGroupGazeForce( force_gaze_ );
		a->setGroupCoherenceForce( force_coherence_ );
		a->setGroupRepulsionForce( force_repulsion_ );
	}
}


/// -----------------------------------------------------------------
/// \brief gazeForce
/// \details Compute the gaze force for the group which corresponds 
/// to \f$ f^{vis}_i = -\beta_1 \alpha_i V_i \f$ in Moussaid et. al.
/// \returns Ped::Tvector - The force in all axes
/// -----------------------------------------------------------------
Ped::Tvector PersonGroup::gazeForce()
{
	Ped::Tvector g;
	
	return g;
}


/// -----------------------------------------------------------------
/// \brief coherenceForce
/// \details Compute the coherence force for the group which is given 
/// by \f$ f^{att}_i = q_A \beta_2 U_i \f$ in Moussaid et. al.
/// \returns Ped::Tvector - The force in all axes
/// -----------------------------------------------------------------
Ped::Tvector PersonGroup::coherenceForce()
{
	Ped::Tvector g;
	
	return g;
}


/// -----------------------------------------------------------------
/// \brief repulsionForce
/// \details Compute the resulsion force for the group which is given 
/// by \f$ f^{rep}_i = \sum_k q_R \beta_3 W_ik \f$ in Moussaid et. al.
/// \returns Ped::Tvector - The force in all axes
/// -----------------------------------------------------------------
Ped::Tvector PersonGroup::repulsionForce()
{
	Ped::Tvector g;
	
	return g;
}
	