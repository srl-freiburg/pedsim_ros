//
// pedsim - A microscopic pedestrian simulation system. 
// Copyright (c) 2003 - 2012 by Christian Gloor
//                              

#ifndef _ped_tree_h_
#define _ped_tree_h_ 1

#ifdef WIN32
#define LIBEXPORT __declspec(dllexport)
#else
#define LIBEXPORT
#endif


#include <set>

using namespace std;

namespace Ped {
	
	class Tscene;
	class Tagent;
	
	class LIBEXPORT Ttree {
		friend class Tscene;

	public:
		Ttree(Ped::Tscene *scene, int depth, double x, double y, double w, double h);
		virtual ~Ttree();

		virtual void addAgent(const Ped::Tagent *a);
		virtual void moveAgent(const Ped::Tagent *a);

		virtual set<const Ped::Tagent*> getAgents() const; 
		
		virtual bool intersects(double px, double py, double pr) const;		

		double getx() const { return x; };                    
		double gety() const { return y; };                    
		double getw() const { return w; };                    
		double geth() const { return h; };                    

		double getdepth() const { return depth; };                    

	protected:
		Ttree *tree1;
		Ttree *tree2;
		Ttree *tree3;
		Ttree *tree4;

		virtual int cut();		
		virtual void addChildren();

	private:
		int agentcount;
		set<const Ped::Tagent*> agents;  // set and not vector, since we need to delete elements from the middle very often
		                                 // set and not list, since deletion is based on pointer (search O(log n) instead of O(n)).
		
		bool isleaf;		
		double x;
		double y;
		double w;
		double h;
		int depth;

		Ped::Tscene *scene;


	};
};

#endif
