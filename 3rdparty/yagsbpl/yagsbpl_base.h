/******************************************************************************************
*                                                                                        *
*    Yet Another Graph-Search Based Planning Library (YAGSBPL)                           *
*    A template-based C++ library for graph search and planning                          *
*    Version 2.1                                                                         *
*    ----------------------------------------------------------                          *
*    Copyright (C) 2013  Subhrajit Bhattacharya                                          *
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


#ifndef __YAGSBPL_BASE_H_
#define __YAGSBPL_BASE_H_

#pragma once


#include <stdio.h>
#include <vector>
#include <cstdlib>
#include <limits>

#define _yagsbpl_abs(x) ((x)>0?(x):(-x))
//#define _yagsbpl_display_version { if (!YAGSBPL_vDisplay_done) { printf("\n*** You are using YAGSBPL v 2.1. ***\n"); YAGSBPL_vDisplay_done = true; } }
//bool YAGSBPL_vDisplay_done = false;


// Declaration
template <class NodeType, class CostType, class PlannerSpecificVariables>
class SearchGraphNode;

// ---

// This class stores information about edges emanating from or incident to a node
template <class NodeType, class CostType, class PlannerSpecificVariables>
class NodeLinks
{
public:
    std::vector< SearchGraphNode<NodeType,CostType,PlannerSpecificVariables>* >* targets;
    std::vector<CostType>* costs;

    // -------------------
    NodeLinks() { targets = NULL; costs = NULL; }
    bool empty(void) { return (!targets); }
    int size(void) { if (targets) return targets->size(); else return -1; }
    void init(int count=0) { targets = new std::vector< SearchGraphNode<NodeType,CostType,PlannerSpecificVariables>* >(count);
                             costs = new std::vector<CostType>(count); }

    void push_back( SearchGraphNode<NodeType,CostType,PlannerSpecificVariables>* node_p , CostType cost )
    { targets->push_back(node_p) ; costs->push_back(cost); }
    void set(int a, SearchGraphNode<NodeType,CostType,PlannerSpecificVariables>* node_p , CostType cost )
    { targets->operator[](a) = node_p ; costs->operator[](a) = cost; }

    SearchGraphNode<NodeType,CostType,PlannerSpecificVariables>* getLinkSearchGraphNode (int a)
    { return (targets->operator[](a)); }
    CostType getLinkCost (int a)
    { return (costs->operator[](a)); }

    ~NodeLinks() {  if(targets) delete targets;  if(costs) delete costs;  }
};

// Extension of NodeType for search problem
template <class NodeType, class CostType, class PlannerSpecificVariables>
class SearchGraphNode
{
public:
    NodeType n;

    // ---------------------------------------------------------------
    // TODO for planner: Planner must set each of the following every time a "non-initiated" node is encountered.
    bool initiated; // Used for tracking newly-create nodes by hash table. To be set to "true" by planner.
    NodeLinks<NodeType,CostType,PlannerSpecificVariables> successors;
    NodeLinks<NodeType,CostType,PlannerSpecificVariables> predecessors; // Planner may or may not use this
    SearchGraphNode<NodeType,CostType,PlannerSpecificVariables>* came_from;
    PlannerSpecificVariables plannerVars; // Other variables, if required by planner
    CostType f; // f-value: Used for maintaining the heap

    // ---------------------------------------------------------------
    // These variables are used by heap container
    bool inHeap;
    int heapArrayPos;

    SearchGraphNode() { initiated=false; came_from=NULL; inHeap=false; }
};

// ---------------------------------------------------------------------


template <class NodeType, class CostType, class PlannerSpecificVariables>
class HeapContainer
{
public:
    typedef  SearchGraphNode<NodeType,CostType,PlannerSpecificVariables>*  heapItem_p;

    int heap_size;
    std::vector <heapItem_p> heapArray;

    HeapContainer(int kc=20) { heap_size=0; }

    typedef enum {UNKNOWN_BUBDIR, UPONLY_BUBDIR, DOWNONLY_BUBDIR} BubbleDirection;

    void push(heapItem_p np);
    heapItem_p pop(void);
    void update (heapItem_p np, BubbleDirection bubdir=UNKNOWN_BUBDIR);
    void remove(heapItem_p np);
    void clear(void) { while (heap_size>0) pop(); };
    bool empty(void) { return (heap_size==0); };
    int size(void) { return (heap_size); };
};

// =============================================================================
// Helper classes for 'GenericSearchGraphDescriptor'

template <class NodeType, class CostType>
class SearchGraphDescriptorFunctionContainer
{
public:
    bool func_redefined;
    virtual int getHashBin(NodeType& n) { func_redefined = false; };
    virtual bool isAccessible(NodeType& n) { func_redefined = false; };
    virtual void getSuccessors(NodeType& n, std::vector<NodeType>* s, std::vector<CostType>* c) { func_redefined = false; };
    virtual void getPredecessors(NodeType& n, std::vector<NodeType>* s, std::vector<CostType>* c) { func_redefined = false; };
    virtual bool storePath(NodeType& n) { func_redefined = false; };
    virtual bool stopSearch(NodeType& n) { func_redefined = false; };

    //
    virtual CostType getHeuristics(NodeType& n1, NodeType& n2) { func_redefined = false; };
};

template <class NodeType, class CostType, class ContainerClass>
class SearchGraphDescriptorFunctionPointerContainer : public SearchGraphDescriptorFunctionContainer<NodeType, CostType>
{
public:
    ContainerClass* p;
    int (ContainerClass::*getHashBin_fp)(NodeType& n);
    bool (ContainerClass::*isAccessible_fp)(NodeType& n);
    void (ContainerClass::*getSuccessors_fp)(NodeType& n, std::vector<NodeType>* s, std::vector<CostType>* c);
    void (ContainerClass::*getPredecessors_fp)(NodeType& n, std::vector<NodeType>* s, std::vector<CostType>* c);
    CostType (ContainerClass::*getHeuristics_fp)(NodeType& n1, NodeType& n2);
    bool (ContainerClass::*storePath_fp)(NodeType& n);
    bool (ContainerClass::*stopSearch_fp)(NodeType& n);
    // ----
    int getHashBin(NodeType& n)
    { if(p && getHashBin_fp) return ( (p->*getHashBin_fp)(n) ); else this->func_redefined = false; }
    bool isAccessible(NodeType& n)
    { if(p && isAccessible_fp) return ( (p->*isAccessible_fp)(n) ); else this->func_redefined = false; }
    void getSuccessors(NodeType& n, std::vector<NodeType>* s, std::vector<CostType>* c)
    { if(p && getSuccessors_fp) return ( (p->*getSuccessors_fp)(n, s, c) ); else this->func_redefined = false; }
    void getPredecessors(NodeType& n, std::vector<NodeType>* s, std::vector<CostType>* c)
    { if(p && getPredecessors_fp) return ( (p->*getPredecessors_fp)(n, s, c) ); else this->func_redefined = false; }
    CostType getHeuristics(NodeType& n1, NodeType& n2)
    { if(p && getHeuristics_fp) return ( (p->*getHeuristics_fp)(n1, n2) ); else this->func_redefined = false; }
    bool storePath(NodeType& n)
    { if(p && storePath_fp) return ( (p->*storePath_fp)(n) ); else this->func_redefined = false; }
    bool stopSearch(NodeType& n)
    { if(p && stopSearch_fp) return ( (p->*stopSearch_fp)(n) ); else this->func_redefined = false; }
};

// ----
// TODO for environment: 'GenericSearchGraphDescriptor' is the only class an instance of which needs to be created.
//   That needs to be passed into the "init" function of the "GenericPlanner"

template <class NodeType, class CostType>
class GenericSearchGraphDescriptor
{
public:
    // Primary functions
    int (*getHashBin_fp)(NodeType& n);
    bool (*isAccessible_fp)(NodeType& n); // optional
    void (*getSuccessors_fp)(NodeType& n, std::vector<NodeType>* s, std::vector<CostType>* c); // s: successors, c: tranition costs
    void (*getPredecessors_fp)(NodeType& n, std::vector<NodeType>* s, std::vector<CostType>* c); // May not be used by all planners
    CostType (*getHeuristics_fp)(NodeType& n1, NodeType& n2); // optional
    bool (*storePath_fp)(NodeType& n); // optional
    bool (*stopSearch_fp)(NodeType& n); // optional if "HeuristicsTargetNode" is given.

    // If pointers to primary functions cannot be provided, we can alternatively use the virtual functions in the FunctionContainer
    SearchGraphDescriptorFunctionContainer<NodeType,CostType>* func_container;

    // Primary variables
    int hashTableSize; // Number of hash bins. "getHashBin" must return a value between 0 and hashTableSize
    // An initial set of "start" nodes to be put in heap. At least one of the following two need to be set.
    std::vector<NodeType> SeedNodes;
    NodeType SeedNode;
    // "Goal" used for computing Heuristics. Not required if "getHeuristics" is not provided.
    NodeType TargetNode; // If "stopSearch" is not provided, this is used to determine when to stop.

    // Other variables and functions - constructor chooses a default
    int hashBinSizeIncreaseStep; // optional

    // ---------------------------------------------------------------
    // Constructor and other functions
    GenericSearchGraphDescriptor();
    void init(void); // TODO: Planner is supposed to call this at initialization.
    // Derived functions - planner should use these, and NOT the former functions
    int _getHashBin(NodeType& n);
    bool _isAccessible(NodeType& n);
    void _getSuccessors(NodeType& n, std::vector<NodeType>* s, std::vector<CostType>* c);
    void _getPredecessors(NodeType& n, std::vector<NodeType>* s, std::vector<CostType>* c);
    CostType _getHeuristics(NodeType& n1, NodeType& n2);
    CostType _getHeuristicsToTarget(NodeType& n);
    bool _storePath(NodeType& n);
    bool _stopSearch(NodeType& n);

};

// =============================================================================

template <class NodeType, class CostType, class PlannerSpecificVariables>
class HashTableContainer
{
public:
    GenericSearchGraphDescriptor<NodeType,CostType>* friendGraphDescriptor_p;
    int hashTableSize;
    std::vector< SearchGraphNode<NodeType,CostType,PlannerSpecificVariables>* >* HashTable;
    void init_HastTable(int hash_table_size);
    SearchGraphNode<NodeType,CostType,PlannerSpecificVariables>* getNodeInHash(NodeType n); // Returns pointer to already-existing
    ~HashTableContainer()
    {
        if (HashTable)
        {
            for (int a=0; a<hashTableSize; a++)
                for (int b=0; b<HashTable[a].size(); b++)
                    delete HashTable[a][b];
            delete[] HashTable;
        }
    };
};

// =============================================================================
// TODO for planner: A planner needs to maintain an instance of this.
//   	Inheritance is also possible, but not recommended because of templates.

template <class NodeType, class CostType, class PlannerSpecificVariables>
class GenericPlanner
{
public:
    GenericSearchGraphDescriptor<NodeType,CostType>* GraphDescriptor;
    HashTableContainer<NodeType,CostType,PlannerSpecificVariables>* hash;
    HeapContainer<NodeType,CostType,PlannerSpecificVariables>* heap;

    void init( GenericSearchGraphDescriptor<NodeType,CostType> theEnv , int heapKeyNum=20 ); // A planner must call this explicitly.
    ~GenericPlanner() { delete GraphDescriptor; delete hash; delete heap; }
};


// ------------------------------------------------------------------------------
// ------------------------------------------------------------------------------
// Since we use templates, the definitions need to be included in the header file as well.

//#include "yagsbpl_base.cpp"
























#define f_val(ptr) ( (ptr)->f )

template <class NodeType, class CostType, class PlannerSpecificVariables>
void HeapContainer<NodeType,CostType,PlannerSpecificVariables>::update (heapItem_p np, BubbleDirection bubdir)
{
    // Does both bubble up and down
    if (!(np->inHeap)) return;
    int currentPos = np->heapArrayPos;

    // Bubble up
    if (bubdir != DOWNONLY_BUBDIR) {
        int parentPos = (currentPos - 1) / 2; // position of parent
        if ( f_val(heapArray[parentPos]) > f_val(np) ) {
            heapArray[currentPos] = heapArray[parentPos];
            heapArray[parentPos] = np;
            heapArray[currentPos]->heapArrayPos = currentPos;
            heapArray[parentPos]->heapArrayPos = parentPos;
            update(heapArray[parentPos], UPONLY_BUBDIR); // recursive call. If heap, downward bubbling won't be needed.
            return; // no need to check children, since chilren will have higher values
        }
        if (bubdir == UPONLY_BUBDIR) return;
    }

    // Bubble down
    int leftChildPos = 2*currentPos + 1;
    int rightChildPos = 2*currentPos + 2;
    int exchangeChildPos;
    if (leftChildPos<heapArray.size() && rightChildPos<heapArray.size())
        exchangeChildPos = ( f_val(heapArray[leftChildPos]) > f_val(heapArray[rightChildPos]) ) ? rightChildPos : leftChildPos;
    else if (leftChildPos<heapArray.size())
        exchangeChildPos = leftChildPos;
    else if (rightChildPos<heapArray.size())
        exchangeChildPos = rightChildPos;
    else
        return;

    if ( f_val(np) > f_val(heapArray[exchangeChildPos]) ) {
        heapArray[currentPos] = heapArray[exchangeChildPos];
        heapArray[exchangeChildPos] = np;
        heapArray[currentPos]->heapArrayPos = currentPos;
        heapArray[exchangeChildPos]->heapArrayPos = exchangeChildPos;
        update(heapArray[exchangeChildPos], DOWNONLY_BUBDIR); // recursive call. If heap, upward bubbling won't be needed.
        return;
    }
}

// SB_BINHEAP -------------------------------------------------------


template <class NodeType, class CostType, class PlannerSpecificVariables>
void HeapContainer<NodeType,CostType,PlannerSpecificVariables>::push
                                    ( SearchGraphNode<NodeType,CostType,PlannerSpecificVariables>* np )
{
    // Assumes np is not in heap. No check is done for that.
    // Add as leaf
    heapArray.push_back(np);
    heap_size++; // SB_BINHEAP2
    np->inHeap = true;
    np->heapArrayPos = heapArray.size() - 1;
    // Bubble up
    update(np, UPONLY_BUBDIR);
}


template <class NodeType, class CostType, class PlannerSpecificVariables>
SearchGraphNode<NodeType,CostType,PlannerSpecificVariables>* HeapContainer<NodeType,CostType,PlannerSpecificVariables>::pop(void)
{
    heapItem_p ret = heapArray[0];
    ret->inHeap = false;
    ret->heapArrayPos = -1;
    // Take last leaf and place it at root
    heapArray[0] = heapArray[heapArray.size()-1];
    heapArray[0]->heapArrayPos = 0;
    heapArray.pop_back();
    heap_size--;
    // Bubble down
    update(heapArray[0], DOWNONLY_BUBDIR);

    return (ret);
}


template <class NodeType, class CostType, class PlannerSpecificVariables>
void HeapContainer<NodeType,CostType,PlannerSpecificVariables>::remove (SearchGraphNode<NodeType,CostType,PlannerSpecificVariables>* np)
{
    CostType tmp_h = np->h;
    np->h = -std::numeric_limits<CostType>::infinity();
    update(np);
    pop();
    np->h = tmp_h;
}

// =================================================================================


template <class NodeType, class CostType>
GenericSearchGraphDescriptor<NodeType,CostType>::GenericSearchGraphDescriptor()
{
    hashBinSizeIncreaseStep = 128;
    // Initiating pointers to all functions as NULL - makes easy to check if a function was defined
    getHashBin_fp = NULL;
    isAccessible_fp = NULL;
    getSuccessors_fp = NULL;
    getPredecessors_fp = NULL;
    getHeuristics_fp = NULL;
    storePath_fp = NULL;
    stopSearch_fp = NULL;
    func_container = NULL;
}

template <class NodeType, class CostType>
void GenericSearchGraphDescriptor<NodeType,CostType>::init(void)
{
    if (SeedNodes.size() == 0) {
        SeedNodes.push_back(SeedNode);
    }
    // Other initializations - to be included in future versions
}

// ---------------------------------

template <class NodeType, class CostType>
int GenericSearchGraphDescriptor<NodeType,CostType>::_getHashBin(NodeType& n)
{
    if (getHashBin_fp)
        return ( getHashBin_fp(n) );

    if (func_container)
    {
        func_container->func_redefined = true;
        int ret = func_container->getHashBin(n);
        if (func_container->func_redefined)
            return ret;
    }

    return 0;
}

template <class NodeType, class CostType>
bool GenericSearchGraphDescriptor<NodeType,CostType>::_isAccessible(NodeType& n)
{
    if (isAccessible_fp)
        return ( isAccessible_fp(n) );

    if (func_container)
    {
        func_container->func_redefined = true;
        bool ret = func_container->isAccessible(n);
        if (func_container->func_redefined)
            return ret;
    }

    return true;
}

template <class NodeType, class CostType>
void GenericSearchGraphDescriptor<NodeType,CostType>::_getSuccessors(NodeType& n, std::vector<NodeType>* s, std::vector<CostType>* c)
{
    if (getSuccessors_fp)
        getSuccessors_fp(n, s, c);
    else if (func_container)
        func_container->getSuccessors(n, s, c);
}

template <class NodeType, class CostType>
void GenericSearchGraphDescriptor<NodeType,CostType>::_getPredecessors(NodeType& n, std::vector<NodeType>* s, std::vector<CostType>* c)
{
    if (getPredecessors_fp)
        getPredecessors_fp(n, s, c);
    else if (func_container)
        func_container->getPredecessors(n, s, c);
}

template <class NodeType, class CostType>
CostType GenericSearchGraphDescriptor<NodeType,CostType>::_getHeuristics(NodeType& n1, NodeType& n2)
{
    if (getHeuristics_fp)
        return ( getHeuristics_fp(n1, n2) );

    if (func_container)
    {
        func_container->func_redefined = true;
        CostType ret = func_container->getHeuristics(n1, n2);
        if (func_container->func_redefined)
            return ret;
    }

    return ((CostType)0);
}

template <class NodeType, class CostType>
CostType GenericSearchGraphDescriptor<NodeType,CostType>::_getHeuristicsToTarget(NodeType& n)
{
    return ( _getHeuristics(n, TargetNode) );
}

template <class NodeType, class CostType>
bool GenericSearchGraphDescriptor<NodeType,CostType>::_storePath(NodeType& n)
{
    if (storePath_fp)
        return ( storePath_fp(n) );

    if (func_container)
    {
        func_container->func_redefined = true;
        bool ret = func_container->storePath(n);
        if (func_container->func_redefined)
            return ret;
    }

    return false;
}

template <class NodeType, class CostType>
bool GenericSearchGraphDescriptor<NodeType,CostType>::_stopSearch(NodeType& n)
{
    if (stopSearch_fp)
        return ( stopSearch_fp(n) );

    if (func_container)
    {
        func_container->func_redefined = true;
        bool ret = func_container->stopSearch(n);
        if (func_container->func_redefined)
            return ret;
    }

    if ( n == TargetNode )
        return true;

    return false;
}

// =================================================================================


template <class NodeType, class CostType, class PlannerSpecificVariables>
void HashTableContainer<NodeType,CostType,PlannerSpecificVariables>::init_HastTable(int hash_table_size)
{
    hashTableSize = hash_table_size;
    HashTable = new std::vector< SearchGraphNode<NodeType,CostType,PlannerSpecificVariables>* > [ hashTableSize ];
    for (int a=0; a<hashTableSize; a++)
        if ( HashTable[a].capacity() >= HashTable[a].size()-1 )
            HashTable[a].reserve( HashTable[a].capacity() + friendGraphDescriptor_p->hashBinSizeIncreaseStep );
}

template <class NodeType, class CostType, class PlannerSpecificVariables>
SearchGraphNode<NodeType,CostType,PlannerSpecificVariables>*
    HashTableContainer<NodeType,CostType,PlannerSpecificVariables>::getNodeInHash(NodeType n)
{
    // Search in bin
    int hashBin = friendGraphDescriptor_p->_getHashBin(n);
    // A SIGSEGV signal generated from here most likely 'getHashBin' returned a bin index larger than (hashTableSize-1).
    for (int a=0; a<HashTable[hashBin].size(); a++)
        if ( HashTable[hashBin][a]->n == n )
            return (HashTable[hashBin][a]);

    // If new node, create it!
    SearchGraphNode<NodeType,CostType,PlannerSpecificVariables>* newSearchGraphNode =
                                                            new SearchGraphNode<NodeType,CostType,PlannerSpecificVariables>;
    newSearchGraphNode->n = n; // WARNING: Nothing else is set yet!
    if ( HashTable[hashBin].capacity() <= HashTable[hashBin].size()+1 )
        HashTable[hashBin].reserve( HashTable[hashBin].capacity() + friendGraphDescriptor_p->hashBinSizeIncreaseStep );
    HashTable[hashBin].push_back(newSearchGraphNode);
    return ( newSearchGraphNode );
}

// =================================================================================


template <class NodeType, class CostType, class PlannerSpecificVariables>
void GenericPlanner<NodeType,CostType,PlannerSpecificVariables>::init
                        ( GenericSearchGraphDescriptor<NodeType,CostType> theEnv , int heapKeyNum )
{
//    _yagsbpl_display_version;

    GraphDescriptor = new GenericSearchGraphDescriptor<NodeType,CostType>;
    *GraphDescriptor = theEnv;

    hash = new HashTableContainer<NodeType,CostType,PlannerSpecificVariables>;
    hash->friendGraphDescriptor_p = GraphDescriptor;
    hash->init_HastTable( GraphDescriptor->hashTableSize );

    heap = new HeapContainer<NodeType,CostType,PlannerSpecificVariables>(heapKeyNum);
}







#endif

