/******************************************************************************************
*                                                                                        *
*    Yet Another Graph-Search Based Planning Library (YAGSBPL)                           *
*    A template-based C++ library for graph search and planning                          *
*    Version 2.0                                                                         *
*    ----------------------------------------------------------                          *
*    Copyright (C) 2011  Subhrajit Bhattacharya                                          *
*                                                                                        *
*    This program is free software: you can redistribute it and/or modify                *
*    it under the terms of the GNU General Public License as published by                *
*    the Free Software Foundation, either version 3 of the License, or                   *
*    (at your option) any later version.                                                 *
*                                                                                        *
*    This program is distributed in the hope that it will be useful,                     *
*    but WITHOUT ANY WARRANTY; without even the implied warranty of                      *
*    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the                       *
*    GNU General Public License for more details <http://www.gnu.org/licenses/>.         *
*                                                                                        *
*                                                                                        *
*    Contact: subhrajit@gmail.com, http://subhrajit.net/                                 *
*                                                                                        *
*                                                                                        *
******************************************************************************************/
//    For a detailed tutorial and download, visit 
//    http://subhrajit.net/index.php?WPage=yagsbpl


#ifndef __A_STAR_H_
#define __A_STAR_H_

#pragma once

#include <stdio.h>
#include <vector>
#include <cstdlib>
#include <cmath>
#include <ctime>
//#include "yagsbpl_base.h"
#include <3rdparty/yagsbpl/yagsbpl_base.h>

#define _YAGSBPL_A_STAR__VIEW_PROGRESS 0
#define _YAGSBPL_A_STAR__PRINT_VERBOSE 0 // requires 'void print(std::string pre)' as a member function of the node
#define _YAGSBPL_A_STAR__HANDLE_EVENTS 1

template <class CostType>
class A_star_variables
{
public:
    CostType g;
    bool expanded; // Whether in closed list or not
    bool accessible; // Since the environment is assumed to to change, each node has fixed accessibility
    int seedLineage; // stores which seed the node came from

    A_star_variables() { expanded=false; seedLineage=-1; }
};

template <class NodeType, class CostType>
class A_star_planner
{
public:
    // typedef's for convenience:
    typedef  A_star_variables<CostType>  PlannerSpecificVariables;
    typedef  SearchGraphNode< NodeType, CostType, PlannerSpecificVariables >*  GraphNode_p;

    // Instance of generac planner
    GenericPlanner< NodeType, CostType, PlannerSpecificVariables > GenericPlannerInstance;
    // Re-mapping of generic planner variables for ease of use (coding convenience)
    GenericSearchGraphDescriptor<NodeType,CostType>* GraphDescriptor;
    HashTableContainer<NodeType,CostType,PlannerSpecificVariables>* hash;
    HeapContainer<NodeType,CostType,PlannerSpecificVariables>* heap;

    // Member variables
    double subopEps;
    int heapKeyCount;
    int ProgressShowInterval;
    std::vector< GraphNode_p > bookmarkGraphNodes;

    // Optional event handlers - Pointers to function that get called when an event take place
#if _YAGSBPL_A_STAR__HANDLE_EVENTS
    // Node 'n' is expanded. This can also be handeled by 'stopSearch'.
    void (*event_NodeExpanded_g)(NodeType n, CostType gVal, CostType fVal, int seedLineage);
    void (NodeType::*event_NodeExpanded_nm)(CostType gVal, CostType fVal, int seedLineage);
    // Successor 'nn' is in open list and is just initiated or is updated
    void (*event_SuccUpdated_g)(NodeType n, NodeType nn, CostType edgeCost, CostType gVal, CostType fVal, int seedLineage);
    void (NodeType::*event_SuccUpdated_nm)(NodeType nn, CostType edgeCost, CostType gVal, CostType fVal, int seedLineage);
#endif

    // Helper functions for specialized applications
    // Computes the f-value:
    CostType (*heapFun_fp)(NodeType& n, CostType g, CostType h, int s);
    CostType _heapFun(NodeType& n, CostType g, CostType h, int s);

    // Initializer and planner
    A_star_planner()
    { hash = NULL;
        subopEps = 1.0; heapKeyCount = 20; ProgressShowInterval = 10000;
        event_NodeExpanded_g=NULL; event_NodeExpanded_nm=NULL; event_SuccUpdated_g=NULL; event_SuccUpdated_nm=NULL;
        heapFun_fp = NULL; }
    void setParams( double eps=1.0 , int heapKeyCt=20 , int progressDispInterval=10000 ) // call to this is optional.
    { subopEps = eps; heapKeyCount = heapKeyCt; ProgressShowInterval = progressDispInterval; }
    void init( GenericSearchGraphDescriptor<NodeType,CostType>* theEnv_p=NULL , bool createHashAndHeap=true );
    void init( GenericSearchGraphDescriptor<NodeType,CostType> theEnv ) { init( &theEnv); } // for version compatability
    void plan(void);

    // Clear the last plan, but not the hash table. Must be called after at least one initialization.
    //   Use this if you intend to re-plan with different start/goal,
    //   and/or if isAccessible is changed such that the new planning is being done in a sub-graph of the previous graph,
    //   Won't give correct result if cost function has changed or the new graph has new edges attached to nodes explored in old graph.
    void clearLastPlanAndInit( GenericSearchGraphDescriptor<NodeType,CostType>* theEnv_p=NULL );

    // Planner output access: ( to be called after plan(), and before destruction of planner )
    std::vector< NodeType > getGoalNodes(void);
    std::vector< GraphNode_p > getGoalGraphNodePointers(void);
    std::vector< std::vector< NodeType > > getPlannedPaths(void);
    std::vector< CostType > getPlannedPathCosts(void);
    A_star_variables<CostType> getNodeInfo(NodeType n);

    // Other variables for querying progress of planning process from other functions
#if  _YAGSBPL_A_STAR__VIEW_PROGRESS
    clock_t  startclock;
    time_t startsecond;
    int expandcount;
#endif
};

// ------------------------------------------------------------------------------
// ------------------------------------------------------------------------------
// Since we use templates, the definitions need to be included in the header file as well.

//#include "A_star.cpp"











template <class NodeType, class CostType>
void A_star_planner<NodeType,CostType>::init( GenericSearchGraphDescriptor<NodeType,CostType>* theEnv_p, bool createHashAndHeap )
{
    GraphNode_p thisGraphNode;

    if (theEnv_p && createHashAndHeap)
        GenericPlannerInstance.init(*theEnv_p, heapKeyCount);  // This initiates the graph, hash and heap of the generic planner
    else if (theEnv_p)
        *GenericPlannerInstance.GraphDescriptor = *theEnv_p;

    // Remapping for coding convenience
    GraphDescriptor = GenericPlannerInstance.GraphDescriptor;
    hash = GenericPlannerInstance.hash;
    heap = GenericPlannerInstance.heap;

    // Init graph, clear the heap and clear stored paths just in case they not empty due to a previous planning
    GraphDescriptor->init();
    heap->clear();
    bookmarkGraphNodes.clear();

    for (int a=0; a<GraphDescriptor->SeedNodes.size(); a++)
    {
        // Check if node is in hash table - get it if it is, otherwise create it.
        thisGraphNode = hash->getNodeInHash( GraphDescriptor->SeedNodes[a] );

        // If node was created, initialize it
        if ( !thisGraphNode->initiated )
        {
            thisGraphNode->f = _heapFun(thisGraphNode->n, (CostType)0.0, GraphDescriptor->_getHeuristicsToTarget( thisGraphNode->n ), a);
            thisGraphNode->came_from = NULL;
            thisGraphNode->plannerVars.seedLineage = a;
            thisGraphNode->plannerVars.g = (CostType)0.0;
            thisGraphNode->plannerVars.expanded = false;

            if ( !GraphDescriptor->_isAccessible( thisGraphNode->n ) )
            {
//                printf("ERROR (A_star): At least one of the seed nodes is not accessible!" );
//                exit(1);
            }
            else
                thisGraphNode->plannerVars.accessible = true;

            thisGraphNode->initiated = true; // Always set this when other variables have already been set
        }

        // Push in heap
        heap->push( thisGraphNode );
    }
}

// -----------------------------

template <class NodeType, class CostType>
void A_star_planner<NodeType,CostType>::clearLastPlanAndInit( GenericSearchGraphDescriptor<NodeType,CostType>* theEnv_p )
{

    // Set every node in hash to not expanded
    if (hash && hash->HashTable) {
        for (int a=0; a<hash->hashTableSize; a++)
            for (int b=0; b<hash->HashTable[a].size(); b++)
            {
                hash->HashTable[a][b]->plannerVars.expanded = false;
                hash->HashTable[a][b]->initiated = false;
            }
        // Clear the last plan, but not the hash table
        if (theEnv_p)
            init(theEnv_p, false);
        else
            init(GraphDescriptor, false);
    }
    else {
        if (theEnv_p)
            init(theEnv_p);
        else
            init(GraphDescriptor);
    }
}

// ==================================================================================

template <class NodeType, class CostType>
CostType A_star_planner<NodeType,CostType>::_heapFun(NodeType& n, CostType g, CostType h, int s)
{
    if (heapFun_fp)
        return ( heapFun_fp(n, g, h, s) );

    return ( g + subopEps*h );
}

// ==================================================================================

template <class NodeType, class CostType>
void A_star_planner<NodeType,CostType>::plan(void)
{
    GraphNode_p thisGraphNode, thisNeighbourGraphNode;
    CostType this_g_val, thisTransitionCost, test_g_val;
    std::vector< NodeType > thisNeighbours;
    std::vector< CostType > thisTransitionCosts;
    int a;

#if _YAGSBPL_A_STAR__VIEW_PROGRESS
    float timediff = 0.0;
    expandcount = 0;
    startclock = clock();
    startsecond = time(NULL);
#endif
    while ( !heap->empty() )
    {
#if _YAGSBPL_A_STAR__VIEW_PROGRESS
        if(ProgressShowInterval>0) {
            if (expandcount % ProgressShowInterval == 0)
            {
                if (timediff>=0.0)
                    timediff = ((float)(clock()-startclock)) / ((float)CLOCKS_PER_SEC);
//                printf("Number of states expanded: %d. Heap size: %d. Time elapsed: %f s.\n",
//                       expandcount, heap->size(), ((timediff>=0.0) ? timediff : difftime(time(NULL),startsecond)) );
            }
            expandcount++;
        }
#endif

        // Get the node with least f-value
        thisGraphNode = heap->pop();
        thisGraphNode->plannerVars.expanded = true; // Put in closed list
#if _YAGSBPL_A_STAR__PRINT_VERBOSE
        thisGraphNode->n.print("Now expanding: ");
#endif

#if _YAGSBPL_A_STAR__HANDLE_EVENTS
        if (event_NodeExpanded_g)
            event_NodeExpanded_g(thisGraphNode->n, thisGraphNode->plannerVars.g,
                                 thisGraphNode->f, thisGraphNode->plannerVars.seedLineage);
        else if (event_NodeExpanded_nm)
            (thisGraphNode->n.*event_NodeExpanded_nm)(thisGraphNode->plannerVars.g,
                                                      thisGraphNode->f, thisGraphNode->plannerVars.seedLineage);
#endif

        // Check if we need to stop furthur expansion
        if ( GraphDescriptor->_stopSearch( thisGraphNode->n ) )
        {
            bookmarkGraphNodes.push_back(thisGraphNode);
#if _YAGSBPL_A_STAR__VIEW_PROGRESS
            if (ProgressShowInterval>0) {
                if (timediff>=0.0)
                    timediff = ((float)(clock()-startclock)) / ((float)CLOCKS_PER_SEC);
//                printf("Stopping search!! Number of states expanded: %d. Heap size: %d. Time elapsed: %f s.\n",
//                       expandcount, heap->size(), ((timediff>=0.0) ? timediff : difftime(time(NULL),startsecond)) );
            }
#endif
            return;
        }
        // Check if we need to store the path leading to this node
        if ( GraphDescriptor->_storePath( thisGraphNode->n ) )
        {
            bookmarkGraphNodes.push_back(thisGraphNode);
#if _YAGSBPL_A_STAR__VIEW_PROGRESS
            if (timediff>=0.0)
                timediff = ((float)(clock()-startclock)) / ((float)CLOCKS_PER_SEC);
            printf("Stored a path!! Number of states expanded: %d. Heap size: %d. Time elapsed: %f s.\n",
                   expandcount, heap->size(), ((timediff>=0.0) ? timediff : difftime(time(NULL),startsecond)) );
#endif
        }

        // Generate the neighbours if they are already not generated
        thisNeighbours.clear();
        thisTransitionCosts.clear();
        if ( thisGraphNode->successors.empty() ) // Successors were not generated previously
        {
            GraphDescriptor->_getSuccessors( thisGraphNode->n , &thisNeighbours , &thisTransitionCosts );
            thisGraphNode->successors.init( thisNeighbours.size() );
            for (a=0; a<thisNeighbours.size(); a++)
                thisGraphNode->successors.set(a, hash->getNodeInHash(thisNeighbours[a]), thisTransitionCosts[a]);
        }
#if _YAGSBPL_A_STAR__PRINT_VERBOSE
        printf("\tNumber of childeren: %d\n", thisGraphNode->successors.size());
#endif

        // Initiate the neighbours (if required) and update their g & f values
        this_g_val = thisGraphNode->plannerVars.g;
        for (a=0; a<thisGraphNode->successors.size(); a++)
        {
            thisNeighbourGraphNode = thisGraphNode->successors.getLinkSearchGraphNode(a);
            thisTransitionCost = thisGraphNode->successors.getLinkCost(a);
#if _YAGSBPL_A_STAR__PRINT_VERBOSE
            thisNeighbourGraphNode->n.print("\tChild: ");
#endif

            // An uninitiated neighbour node - definitely g & f values not set either.
            if ( !thisNeighbourGraphNode->initiated )
            {
                thisNeighbourGraphNode->plannerVars.accessible = GraphDescriptor->_isAccessible( thisNeighbourGraphNode->n );
                if ( thisNeighbourGraphNode->plannerVars.accessible )
                {
                    thisNeighbourGraphNode->came_from = thisGraphNode;
                    thisNeighbourGraphNode->plannerVars.seedLineage = thisGraphNode->plannerVars.seedLineage;
                    thisNeighbourGraphNode->plannerVars.g = thisGraphNode->plannerVars.g + thisTransitionCost;
                    thisNeighbourGraphNode->f = _heapFun(thisNeighbourGraphNode->n, thisNeighbourGraphNode->plannerVars.g,
                                                         GraphDescriptor->_getHeuristicsToTarget( thisNeighbourGraphNode->n ),
                                                         thisGraphNode->plannerVars.seedLineage);
                    //thisNeighbourGraphNode->plannerVars.g +
                    //subopEps * GraphDescriptor->_getHeuristicsToTarget( thisNeighbourGraphNode->n );
                    thisNeighbourGraphNode->plannerVars.expanded = false;

                    // Put in open list and continue to next neighbour
                    heap->push( thisNeighbourGraphNode );
                }
                thisNeighbourGraphNode->initiated = true; // Always set this when other variables have already been set
#if _YAGSBPL_A_STAR__HANDLE_EVENTS
                if (event_SuccUpdated_g)
                    event_SuccUpdated_g(thisGraphNode->n, thisNeighbourGraphNode->n, thisTransitionCost,
                                        thisNeighbourGraphNode->plannerVars.g, thisNeighbourGraphNode->f,
                                        thisNeighbourGraphNode->plannerVars.seedLineage);
                else if (event_SuccUpdated_nm)
                    (thisGraphNode->n.*event_SuccUpdated_nm)(thisNeighbourGraphNode->n, thisTransitionCost,
                                                             thisNeighbourGraphNode->plannerVars.g, thisNeighbourGraphNode->f,
                                                             thisNeighbourGraphNode->plannerVars.seedLineage);
#endif
                continue;
            }

            // Neighbour that are not accessible or in closed list are to be skipped
            if ( !thisNeighbourGraphNode->plannerVars.accessible || thisNeighbourGraphNode->plannerVars.expanded )
                continue;

            // Update came_from, g and f values if better
            test_g_val = thisGraphNode->plannerVars.g + thisTransitionCost;
            if ( test_g_val < thisNeighbourGraphNode->plannerVars.g )
            {
                thisNeighbourGraphNode->plannerVars.g = test_g_val;
                thisNeighbourGraphNode->f = _heapFun(thisNeighbourGraphNode->n, thisNeighbourGraphNode->plannerVars.g,
                                                     GraphDescriptor->_getHeuristicsToTarget( thisNeighbourGraphNode->n ),
                                                     thisGraphNode->plannerVars.seedLineage);
                //thisNeighbourGraphNode->plannerVars.g +
                //subopEps * GraphDescriptor->_getHeuristicsToTarget( thisNeighbourGraphNode->n );
                thisNeighbourGraphNode->came_from = thisGraphNode;
                thisNeighbourGraphNode->plannerVars.seedLineage = thisGraphNode->plannerVars.seedLineage;

                // Since thisNeighbourGraphNode->f is changed, re-arrange it in heap
                heap->update( thisNeighbourGraphNode );
#if _YAGSBPL_A_STAR__HANDLE_EVENTS
                if (event_SuccUpdated_g)
                    event_SuccUpdated_g(thisGraphNode->n, thisNeighbourGraphNode->n, thisTransitionCost,
                                        thisNeighbourGraphNode->plannerVars.g, thisNeighbourGraphNode->f,
                                        thisNeighbourGraphNode->plannerVars.seedLineage);
                else if (event_SuccUpdated_nm)
                    (thisGraphNode->n.*event_SuccUpdated_nm)(thisNeighbourGraphNode->n, thisTransitionCost,
                                                             thisNeighbourGraphNode->plannerVars.g, thisNeighbourGraphNode->f,
                                                             thisNeighbourGraphNode->plannerVars.seedLineage);
#endif
            }
        }
    }
#if _YAGSBPL_A_STAR__VIEW_PROGRESS
    if (ProgressShowInterval>0 && heap->empty())
        printf("Stopping search!! Heap is empty. Number of states expanded: %d. Heap size: %d. Time elapsed: %f s.\n",
               expandcount, heap->size(), ((timediff>=0.0) ? timediff : difftime(time(NULL),startsecond)) );
#endif
}

// ==================================================================================

template <class NodeType, class CostType>
std::vector< SearchGraphNode< NodeType, CostType, A_star_variables<CostType> >* >
A_star_planner<NodeType,CostType>::getGoalGraphNodePointers(void)
{
    return (bookmarkGraphNodes);
}

template <class NodeType, class CostType>
std::vector<NodeType> A_star_planner<NodeType,CostType>::getGoalNodes(void)
{
    std::vector<NodeType> ret;
    for (int a=0; a<bookmarkGraphNodes.size(); a++)
        ret.push_back(bookmarkGraphNodes[a]->n);
    return (ret);
}

template <class NodeType, class CostType>
std::vector<CostType> A_star_planner<NodeType,CostType>::getPlannedPathCosts(void)
{
    std::vector<CostType> costs;
    for (int a=0; a<bookmarkGraphNodes.size(); a++)
        costs.push_back(bookmarkGraphNodes[a]->plannerVars.g);
    return (costs);
}

template <class NodeType, class CostType>
std::vector< std::vector< NodeType > > A_star_planner<NodeType,CostType>::getPlannedPaths(void)
{
    std::vector< std::vector< NodeType > > paths;
    std::vector< NodeType > thisPath;
    for (int a=0; a<bookmarkGraphNodes.size(); a++)
    {
        thisPath.clear();
        // Reconstruct path
        GraphNode_p thisGraphNode = bookmarkGraphNodes[a];
        while (thisGraphNode)
        {
            thisPath.push_back(thisGraphNode->n);
            thisGraphNode = thisGraphNode->came_from;
        }

        paths.push_back(thisPath);
    }
    return (paths);
}

template <class NodeType, class CostType>
A_star_variables<CostType> A_star_planner<NodeType,CostType>::getNodeInfo(NodeType n)
{
    GraphNode_p thisGraphNode = hash->getNodeInHash(n);
    return thisGraphNode->plannerVars;
}






#endif

//#include "A_star.cpp"
