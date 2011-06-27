#ifndef GOALLINEUPENV_H
#define GOALLINEUPENV_H

#include <stdlib.h>
#include <string.h>
#include<stdio.h>
#include<math.h>
#include<sys/types.h>		

#include "Tools/RL/rlIncludes.h"
#include <vector>
using namespace std;



class GoalLineUp : public Environment{

 protected:
  int Stages;
	double transVel,dirTheta,rotVel;
	vector <float> currPos;
	float compass;
	float measureddistance;
	float balldistance;
	float ballbearing;
	vector<float> kickPosition;
	vector<float> targetPosition;
	vector<float>currVel;
	float mixedSpeed;
	
 public:

  GoalLineUp();//double theta);
  /*	Default constructor.
	Initializes state
  */
	
  void uniformStateSample(State& s);
	
	
  void makeObservation();

  void startState(State& start, bool& terminal);
  /*	Selects start state according to the uniformly random distribution
   */
  void setState(const State& s, bool& terminal);
  /* Sets current state to "s" and returns with argument "terminal" an 
     indication of whether this state is terminal.
  */

  bool checkTerminal();
		
  void transition(const Action& action, State& s_new, double& r_new, bool& terminal);
  /*	Implements a transtion in responce to the action 
	performed by the agent. 
	Parameters:
	action: action performed by the agent
	s_new : return value - new state
	r_new : return value - new reward
	terminal: indication of whether s_new is a terminal state
  */


  bool applicable(const State& s, const Action& a);
  /*	Checks if action a is applicable in state s.
   */

  void bound(int i, bool& bounded, double& left, double& right);
  /*	Gives bounds on state variables' values
	This implementation is for the case where all variables are unbounded.
	This method should be redefined by derived classes in case of bounded variables
	Parameters:
	i : index of state variable
	bounded: indicates if i^th variable is bounded
	left : left bound
	right: right bound
  */
  ~GoalLineUp();
 
};


#endif
