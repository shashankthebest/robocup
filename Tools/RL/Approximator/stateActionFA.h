#ifndef STATEACTIONFA_H
#define STATEACTIONFA_H
////////////////////////////////////////////////////////////////
#include "Tools/RL/Misc/globals.h"
#include "approximator.h"

class StateActionFA 
{
	/* Contains separate approximator for each action.
	 */
	
	int A;//Number of actions (architectures)
	Approximator** fa;/* Array of pointers to approximators.	
					   Each element is a base pointer to a derived approximator object.
					   */
public:
	
	StateActionFA();
	~StateActionFA();
	
	StateActionFA(int n, Approximator** f);
	/* General constructor.
     n : number of actions (architectures)
     f : pointer to the array of pointers to approximator objects
	 */
	
	int getSize();
	/* Return number of parameters in one of the component architectures 
     (assuming that all of them have the same number of parameters). 
	 */
	
	void getMaxParameterChange(double* changes);
	/* Returns an array of MaxParameterChanges for all component architectures */
	
	void getNumberParametersChanged(int* changes);
	/* Returns an array of the number of changed parameters for each component architecture
	 */
	
	void predict(const Action& a, const State& s,  double& output);
	/* Predicts an output value with an approximator corresponding a given action.
     a : reference to an action
     s : reference to the input (state)
     output : returned value of the predicted output
	 */
	
	void learn(const Action& a, const State& s, double target);	
	/* Learns an input-output pair with an approximator corresponding a given action.
     a : reference to an action
     s : reference to the input (state)
     target : target output value
	 */
	
	void computeGradient(const Action& a, const State& s, double* GradientVector);
	/* Compute the gradient w.r.t. architecture parameters at the current parameters' values and input s.
	 */
	
	void updateParameters(const Action& a, double* delta);
	/* Update parameters by amounts in delta array (possibly multiplied with appropriate learning step).
	 */
	
	void clearTraces(const Action& a, const State& s, double replace);
	/* Clears traces for those actions that were not taken in state s.
     a : action for traces should NOT be replaced
     s : input (state)
     replace : value to which traces should be replaced (usually zero)
	 */
	
	void replaceTraces(const Action& a, const State& s, double trace);
	/* Replaces traces of the architecture for action a for parameters activated by input s
	 */
	
	void decayTraces(double factor);
	/* Decay (multiply) traces of all architectures by factor.
	 */
	
	void accumulateTraces(const Action& a, const State& s, double amount);
	/* Increment traces by amount for the architecture of action a for parameters activated by s.
	 */
	
	void setArchitectureParameters(const Action& a, int argc, char *argv[]);
	/* Loads parameters of the architecture corresponding to a given action.
     a : action
     argc : number of supplied arguments
     argv : array of arguments
     What parameters exactly you send in argv depends on the implementation 
     of the class inherited from Approximator class: you send parameters 
     exactly as to the setArchitectureParameters() function of that class.
	 */
	
	void saveArchitectureParameters(const Action& a, int argc, char *argv[]);
	/* Saves parameters of the architecture corresponding to a given action.
     a : action
     argc : number of supplied arguments
     argv : array of arguments
     What parameters exactly you send in argv depends on the implementation 
     of the class inherited from Approximator class: you send parameters 
     exactly as to the saveArchitectureParameters() function of that class.
	 */
	
	void saveAllArchitectureParameters(char** fileNames);
	
	void setLearningParameters(const Action& a, int argc, char *argv[]);
	/* Sets learning parameters of the architecture corresponding to a given action.
     a : action
     argc : number of supplied arguments
     argv : array of arguments
	 */
	
	void setAllLearningParameters(int argc, char* argv[]);
	/* Sets (the same) learning parameters of architectures corresponding to each action.
     argc : number of supplied arguments
     argv : array of arguments
     What parameters exactly you send in argv depends on the implementation 
     of the class inherited from Approximator class: you send parameters 
     exactly as to the setLearningParameters() function of that class. 
	 */
	
};

//////////////////////////////////////////////////////////////////////////////
#endif
