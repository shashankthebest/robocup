#ifndef SARSAAGENTRT_H
#define SARSAAGENTRT_H
#include <stdlib.h>
#include <string.h>
#include<stdio.h>
#include<math.h>
#include<sys/types.h>		


#include "Tools/RL/Misc/globals.h"
#include "Tools/RL/rlIncludes.h"
#include "agent.h"

//Declaration of the SarsaAgent class derived from Agent base class


class SarsaAgentRT : public Agent {

  double lambda; //parameter for the SARSA(lambda) learning algorithm
  double epsilon;//parameter for an epsilon greedy policy



 public:
  //SarsaAgentRT(double g, const ActionSet& a_s, StateActionFA* const f, Environment * const e);
  SarsaAgentRT(double g, ActionSet a_s, StateActionFA* const f, Environment * const e);
  /* General constructor.
     Parameters:
     g : discount facor
     a_s : set of available actions
     f : pointer to a collection of approximation architectures to represent
         action value functions (one architecture for each action).
     e : pointer to the Environment object with which the agent will interact
  */
  
  void setLearningParameters(int argc, char *argv[]);
  /* Sets parameters of the RL learning algorithm.
     Parameters are supplied in a command-line argument manner, where
     argc is the number of parameters in argv array. 
     Each string in argv array is one parameter and its value.
     Acceptable parameters and their format:
       lambda=value : value of the lambda parameter in [0,1];
       epsilon=value: value of the epsilon parameter for 
                      epsilon-greedy exploration.
   */

  static void helpLearningParameters();
  /* Print out the format for the command-line specification of the learning parameters.
   */


  ~SarsaAgentRT();
  /* Destructor.
   */
	
	
private:
	State NewState;
	Action NewAction;
	double Qvalue, Qcheck;
	double discount;
	int i;
	int steps;
	int j;
	double TDerror;
	bool startedLearning;
	double rememberEpsilon;
	
	double Qcurrent, Qnext;
	double* Qv;	
	
	
					
 protected:

	
  int actAndLearn(int N, bool SaveTrajectory);
  /* Implements maximum of N successive steps of the trial
     or until a terminal state is entered by the environment.
     Computes return collected on this trial and saves it
     in Return data member of the base class.

     Parameters:
     N : maximal number of steps in the trial
     SaveTrajectory: indicates if trajectory has to be saved.
				
  */
	
	void startLearning(int N, bool SaveTrajectory);
	
	int stepActAndLearn(int N, bool SaveTrajectory);
	
	int stepAct(int N, bool SaveTrajectory, bool ComputeBellmanError);


	
  int act(int N, bool SaveTrajectory, bool ComputeBellmanError);
  /* Implements maximum of N successive steps of the trial
     or until a terminal state is entered by the environment.
     Computes return collected on this trial and saves it
     in Return data member of the base class.

     Parameters:
     N : maximal number of steps in the trial
     SaveTrajectory: indicates if trajectory has to be saved.
     ComputeBellmanError: indicates if Bellman error should be estimated 
           for visited state-action pairs. 			
  */
	

  void chooseAction(const State& s, Action& a, bool islearning);
  /* Chooses and returns with argument "a" an action in state "s" 
     according to epsilon-greedy exploration strategy.
   */

};
#endif
