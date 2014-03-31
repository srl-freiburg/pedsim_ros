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
	static int staticid = 0;
    id_ = staticid++;
	
    members_.clear();
}

PersonGroup::PersonGroup ( const std::list<Agent*>& agents )
{
	static int staticid = 0;
    id_ = staticid++;
	
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
/// \brief addMember
/// \details Add an agent to a group
/// \param[in] agent - new agent to be added to the group
/// \returns flag - True if succeeded, false if agent is already in
/// -----------------------------------------------------------------
bool PersonGroup::addMember ( Agent* agent )
{
	// NOTE - temporarily limit group sizes
    if ( isMember ( agent ) || agent->inGroup() || memberCount() >= 5 )
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
bool PersonGroup::isEmpty()
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
size_t PersonGroup::memberCount()
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
	gazeForce();
	coherenceForce();
	repulsionForce();
	
	// assign forces to members
// 	BOOST_FOREACH ( Agent* a, members_ )
// 	{
// 		a->setGroupGazeForce( force_gaze_ );
// 		a->setGroupCoherenceForce( force_coherence_ );
// 		a->setGroupRepulsionForce( force_repulsion_ );
// 	}
}


/// -----------------------------------------------------------------
/// \brief gazeForce
/// \details Compute the gaze force for the group which corresponds 
/// to \f$ f^{vis}_i = -\beta_1 \alpha_i V_i \f$ in Moussaid et. al.
/// -----------------------------------------------------------------
void PersonGroup::gazeForce()
{
	// a value of 0 would make groups stick together with no 
	// communication to make 'inverse V shapes'. The value of
	// 4 should create normal V shapes.
	double beta_1 = 4.0;
	
	// compute center of mass c
	Ped::Tvector com = computeCenterOfMass();
	
	/// for each agent
	BOOST_FOREACH ( Agent* a, members_ )
	{
		// compute gazing direction H
		Ped::Tvector H( a->getvx(), a->getvy(), a->getvz());
	
		// compute alpha for rotating H
		Ped::Tvector los( com.x - a->getx(), com.y - a->gety(), 0.0 );
		double theta = angleBetween( H, los );
		double alpha = ( M_PI / 2.0 ) - theta;
		
		// compute force from alpha, V and beta_1
		Ped::Tvector f_i = -beta_1 * alpha * a->getVelocity();
		
		// assign force to agent
		a->setGroupGazeForce( f_i );
	}
}


/// -----------------------------------------------------------------
/// \brief coherenceForce
/// \details Compute the coherence force for the group which is given 
/// by \f$ f^{att}_i = q_A \beta_2 U_i \f$ in Moussaid et. al.
/// -----------------------------------------------------------------
void PersonGroup::coherenceForce ()
{
	double beta_2 = 11.0;
	
	// compute the center of mass of group
	Ped::Tvector com = computeCenterOfMass();
	
	/// for each member
	BOOST_FOREACH ( Agent* a, members_ )
	{
		// compute the unit vector pointing to the center of mass
		Ped::Tvector U( com.x - a->getx(), com.y - a->gety(), 0.0 );
		
		// compute threshold for q and its value
		double q_thresh = ( memberCount() - 1 ) / 2.0;
		
		double q_A = ( distance( com.x, com.y, a->getx(), a->gety() ) > q_thresh ) ? 0.0 : 1.0;
		
		// compute the force from q_A, U and beta_2
		Ped::Tvector f_i = q_A * beta_2 * U;
		
		// assign force to agent
		a->setGroupCoherenceForce( f_i );
	}
}


/// -----------------------------------------------------------------
/// \brief repulsionForce
/// \details Compute the resulsion force for the group which is given 
/// by \f$ f^{rep}_i = \sum_k q_R \beta_3 W_ik \f$ in Moussaid et. al.
/// -----------------------------------------------------------------
void PersonGroup::repulsionForce ()
{
	double beta_3 = 12.0;
	
	/// for all agents
	BOOST_FOREACH ( Agent* a_i, members_ )
	{
		// compute q_R threshold
		double d_0 = ( a_i->getRadius() * 2.5 );
		
		Ped::Tvector f_i( 0.0, 0.0, 0.0 );
		
		// early exit if only single person group
		if ( memberCount() == 1)
		{
			a_i->setGroupRepulsionForce( f_i );
			return;
		}
		
		// for all neighbors within q_R
		BOOST_FOREACH ( Agent* a_k, members_ )
		{
			if ( a_i->getid() == a_k->getid() ) 
				continue;
			else
			{
				// compute unit vector W pointing to neighbor 
				Ped::Tvector W = a_k->getPosition() - a_i->getPosition();
			
				// compute repulsion component
				double q_R = ( distanceBetween( a_i->getPosition(), a_k->getPosition() ) > d_0 ) ? 0.0 : 1.0;
				
				f_i += q_R * beta_3 * W;
			}
		}
		
		// assign force to agent
		a_i->setGroupRepulsionForce( f_i );
	}
}
	

/// -----------------------------------------------------------------
/// \brief computeCenterOfMass
/// \details Compute the group center of mass based on the individual
/// agent postitions
/// \returns Ped::Tvector - center of mass
/// -----------------------------------------------------------------
Ped::Tvector PersonGroup::computeCenterOfMass ()
{
	Ped::Tvector com;
	
	if ( memberCount() > 0 )
	{
		BOOST_FOREACH ( Agent* a, members_ )
		{
			com += a->getPosition();
		}
	
		com /= memberCount();
	}
	
	return com;
}