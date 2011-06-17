/*	Declaration of the class for Mountain-Car task, based on the 
	formulation from Sutton & Borto book "Reinforcement Learning: An Introdunction". 
	
	File:		mountain_car.h
	Author:		Bohdana Ratitch
	Version:	December 2000
*/

#include <stdlib.h>
#include <string.h>
#include<stdio.h>
#include<math.h>
#include<sys/types.h>		
#include<iostream.h>

#ifndef INTERFACE_CLASSES	
#define INTERFACE_CLASSES
#include "interface_classes.h"
#endif


//	Declaration of MountainCar class

class MountainCar : public Environment{

 protected:
  int Stages;

 public:

  MountainCar();
  /*	Default constructor.
	Initializes state according to the uniformly random start state distribution.
  */
  void uniformStateSample(State& s);

  void startState(State& start, bool& terminal);
  /*	Selects start state according to the uniformly random distribution
   */
  void setState(const State& s, bool& terminal);
  /* Sets current state to "s" and returns with argument "terminal" an 
     indication of whether this state is terminal.
  */

		
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
  ~MountainCar();
 
};
