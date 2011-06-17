#ifndef STATEREPRESENTATION_H
#define STATEREPRESENTATION_H

#include "Tools/RL/Misc/globals.h"

// STATE AND ACTION REPRESENTATION

struct State{ //implementation in sarepr.cpp
	
	static int dimensionality;// number of variables
	double* x;//array of variables describing a state
	
	State();
	State(int n);
	/* Construcs a "super"-state that combines n states with current dimensionality.
     Used by random MDP generator mainly.
	 */
	
	State(State& s);
	void operator = (const State& s);
	~State();
};

ostream& operator << (ostream& file, const State& s);
/* Overloaded operator for state output to a file or cout.
 */

////////////////////////////////////////////////////////////////////////////////////

struct Subset{//implementation in sarepr.cpp
	/* Hypercube in the state space.
	 */
	double* left;//left bounds on state variables
	double* right;//right bounds on state variables
	
	Subset();
	Subset(int N);
	/* Size of the allocated arrays is (State::dimensionality * N)
	 */
	~Subset();
};
///////////////////////////////////////////////////////////////////////
#endif
