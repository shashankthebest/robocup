#ifndef ENVDATATYPES_H
#define ENVDATATYPES_H

#include "Tools/RL/Misc/globals.h"


////////////////////////////////////////////////////////////////////////////////

struct TransitionSamples{//implemented in environment.cpp
	/* This structurte is used by the function ComputeAttributes() implemented in the base class Environment. 
	 */
	
	int Transitions;//number of transitions made from currentState with action a 
	State currentState;	
	Action a;
	State* nextState;//array of collected samples of next states
	double* reward;//array of collected samples of rewards
	int* binIndexNS;//used to calculate an index of a state in a discretized space
	int B;//number of bins in which the state space is discretized
	double* prob;//estimates of the transition probabilites (for each bin)
	
	TransitionSamples();
	void setTransitionNumber(int T);
	void computeTransitionProbabilities(int b);
	~TransitionSamples();
};

//////////////////////////////////////////////////////////////////




struct Attributes{//implementation in environment.cpp 
	double* Entropy; //array where i-th item is the i-step Entropy
	int n; //up to which step entropy is computed
	double Controllability;
	double RiskFactor;
	double RFconst;
	double RewardVariance;
	double TransitionDistance;
	double TransitionVariability;
	
	Attributes();
	
	Attributes(int N, double c);
	/* N : up to which step the State Transition Entropy should be computed.
     c : (multiplicative) threshold for the risk factor (in [0,1)).
	 */
	
	void setParameters(int N, double c);
	/* Sets parameters for the attributes' calculation:
     N : up to which step the State Transition Entropy should be computed.
     c : (multiplicative) threshold for the risk factor (in [0,1)).
	 */
	
	~Attributes();
};





#endif
