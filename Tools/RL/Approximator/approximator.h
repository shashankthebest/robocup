#ifndef APPROXIMATOR_H
#define APPROXIMATOR_H
#include "Tools/RL/Misc/globals.h"
#include "Tools/RL/State_actions/states.h"
#include "Tools/RL/State_actions/actions.h"

class Approximator{
	/* Abstract class - base for all FA methods.
	 */
	
protected:
	//these data members may be used to trace changes during learning
	double MaxParameterChange;
	int NumberParametersChanged;
	
public:
	
	virtual int getSize()=0;
	/* Return the number of parameters in this architecture
	 */
	
	virtual void predict(const State& s, double& output) =0;
	/* Predicts an output value for a given input.
     s : reference to the input (state)
     output : returned value of the predicted output
	 */
	
	virtual void learn(const State& s, const double target)=0;
	/* Learns an input-output pair.
     s : input (state)
     target: target output value
	 */
	
	virtual void computeGradient(const State& s, double* GradientVector)=0;
	/* Compute the gradient w.r.t. architecture parameters at the current parameters' values and input s
	 */
	
	virtual void updateParameters(double* delta)=0;
	/* Update parameters by amounts in delta array, 
     possible multiplied by appropriate learning steps.
	 */
	
	virtual void replaceTraces(const State& s, double replace)=0;
	/* Replace traces of parameters, activated by input state s to value replace.
	 */
	
	virtual void decayTraces(double factor)=0;
	/* Decay (multiply) traces by factor.
	 */
	
	virtual void accumulateTraces(const State& s, double amount)=0;
	/* Increment traces by amount for parameters activated by input s.
	 */
	
	virtual void setArchitectureParameters(int argc, char *argv[])=0;
	/* Loads parameters of the architecture from a text file.
     argc : number of supplied arguments
     argv : array of arguments
	 */
	
	virtual void saveArchitectureParameters(int argc, char *argv[])=0;
	/* Saves parameters of the architecture into a text file.
     argc : number of supplied arguments
     argv : array of arguments
	 */
	
	virtual void setLearningParameters(int argc, char *argv[])=0;
	/* Sets learning parameters (e.g. learning step).
     argc : number of supplied arguments
     argv : array of arguments
	 */
	
	double getMaxParameterChange(){
		double c=MaxParameterChange;
		MaxParameterChange=0;
		return c;
	}
	
	int getNumberParametersChanged(){
		return NumberParametersChanged;
	}
	
	virtual ~Approximator(){}
	
};

#endif
