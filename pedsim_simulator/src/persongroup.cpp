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
    members_.push_back ( a );
}

PersonGroup::~PersonGroup()
{
    members_.clear();
}


std::list<Agent*>& PersonGroup::getMembers ()
{
    return members_;
}

const std::list<Agent*>& PersonGroup::getMembers () const
{
    return members_;
}

bool PersonGroup::addMember ( Agent* agent )
{
    if ( isMember ( agent ) )
        return false;
    else
    {
        members_.push_back ( agent );
        return true;
    }
}

bool PersonGroup::removeMember ( Agent* agent )
{
    // NOTE - naive approach
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

bool PersonGroup::setMembers ( const std::list<Agent*>& agents )
{
    if ( agents.size() == 0 )
        return false;
    else
    {
        // NOTE - should we really clear??
        members_.clear();

        BOOST_FOREACH ( Agent* a, agents )
        members_.push_back ( a );

        return true;
    }
}

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

bool PersonGroup::isEmpty() const
{
    if ( memberCount() == 0 )
        return true;
    else
        return false;
}

size_t PersonGroup::memberCount() const
{
    return members_.size();
}