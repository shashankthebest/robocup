#ifndef ENVIRONMENT_H
#define ENVIRONMENT_H

#include "envdata.h"
#include "Tools/RL/Approximator/stateActionFA.h"

//////////////////////////////////////////////////////////////////////////////
class Environment 
{//implementation of some non-virtual functions in environment.cpp
	
protected:
	State CurrentState;//current state
	Action CurrentAction;//last action performed by the agent
	double reward;//reward after transition to state s under action a
	static long idum; //used by a random number generator
	static bool seeded;//indicates if the random number generator has been seeded during this program run
	
public:
	
	Environment();
	/* Seeds and initiates random number generator.
     This constructor is automatically called when objects of the derived
     classes are created.
	 */
	
	
	virtual void startState(State& start, bool& terminal)=0;
	/* Samples a start state. Sets CurrentState data member to that state 
     and also returns it as "start" paramter. Returns an indicatiof if 
     the sampled state is terminal in "terminal" parameter.   
	 */
	
	virtual void setState(const State& s, bool& terminal)=0;
	/* Sets the CurrentState data member to state "s". Returns an indicatiof if 
     the sampled state is terminal in "terminal" parameter. 
	 */
	
	virtual void transition(const Action& action, State& s_new, double& r_new, bool& terminal)=0;
	/* Implements a transtion form CurrentState in responce to the "action" 
     performed by the agent. Updates its internal variables 
     (CurrentAction and rewrd) and returns values to the agent.
     action: action performed by the agent
     s_new : return value - new state
     r_new : return value - new reward
     terminal: indication of whether s_new is a terminal state
	 */
	
	virtual bool applicable(const State& s, const Action& a)=0;
	/* Checks if action a is applicable in state s.
	 */
	
	virtual void bound(int i, bool& bounded, double& left, double& right)=0;
	/* Gives bounds on state variables' values
     i : index of state variable
     bounded: indicates if i^th variable is bounded
     left : left bound
     right: right bound
	 */
	
	void getStateSpaceBounds(double* left, double* right);
	/* Returns bounds on state variables.
     left : array of left bounds
     right : array of right bounds
	 */
	
	virtual void uniformStateSample(State& s)=0;
	/* Implements uniform state space sampling.
	 */
	
	//The following functions empirically measure task attributes 
	
	void computeAttributes(Attributes& att, const State& startState, int Steps, int Transitions, const int* n, const ActionSet& as, StateActionFA* fa);
    /* Computes global values of the attributes for the state distribution
	 as on the trajectory under some policy.
	 att : attributes structureto return computed values;
	 startState : state from which to start a random walk;
	 Steps : maximum number of steps on the trajectory;
	 Transitions: number of sample transitions from each state;
	 n : array indicating into how many intervals each state variable ;
	 should be discretized for the approximate calculation of attributes;
	 as : action set for the current RL system;
	 fa : pointer to the architecture that contains action value functions
	 for each action. According to these value functions, 
	 greedy policy will be exacuted. If it is desired to implement 
	 uniformly random policy, make sure that parameters of the 
	 architectures for all functions are the same (all values are the same).
	 */
	
    void computeAttributes(Attributes& att, int SampleSize, int Transitions, const int* n, const ActionSet& as);
    /* Computes global values of the attributes for the uniform state 
	 distribution. 
	 att : attributes structureto return computed values;
	 SampleSize : number of uniformly distributed samples across the state 
	 space in which attribute values are computed and then averaged;
	 Transitions: number of sample transitions from each state;
	 n : array indicating into how many intervals each state variable;
	 as : action set for the current RL system.
	 */
	
	
    double multiStepEntropy(int N, int sampleSize, int Transitions, const int* n, const ActionSet& as);
    /* Computes multi-step state transition entropy.
	 N : number of steps over which entropy should be computed;
	 sampleSize : number of uniformly distributed samples across the state 
	 space in which attribute values are computed and then averaged;
	 Transitions: number of sample transitions from each state;
	 n : array indicating into how many intervals each state variable;
	 as : action set for the current RL system.
	 */
	
protected:
    void chooseAction(double epsilon, StateActionFA* fa, const ActionSet& actions, const State& s, Action& a);
    /* Implements an epsilon-greedy strategy based on action value 
	 functions in the architecture pointed to by fa.
	 epsilon : parameter for the epsilon-greedy strategy;
	 fa : pointer to the architecture containing action value functions;
	 actions : action set for the current RL system;
	 s : state in which to choose action
	 a : return value - chosen action
	 */
	
	
    void actionSequence(int num, int n, int as_size, int* seq);
    /*	Used my multiStepEntropy() function.
     */
};

///////////////////////////////////////////////////////////////////////////
#endif
