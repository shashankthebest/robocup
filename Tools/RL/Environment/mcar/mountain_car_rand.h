/*	Declaration of the randomized version of the mountain-car task,
	based on the formulation from the book "Reinforcement Learning: 
	An Introduction" by R.Sutton and A.Barto.
	
	File:		mountain_car_rand.h
	Author:		Bohdana Ratitch
	Version:	January 2002
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

#ifndef MC
#define MC
#include "mountain_car.h"
#endif


class MountainCarRand : public MountainCar{

  double ActionNoiseVar;
  double PositionNoiseVar;

 public:

  MountainCarRand(double ActVar=0, double PosVar=0);
  /*	Constructor.
	Creates an instance of the Randomized Mountain Car task, where
	actions (-1) and (+1) throttle have zero-mean Gaussian noise with
	variance "ActVar" and position state variable has zero-mean 
	Gaussian noise with variance "PosVar".
	
  */
		
		
  void transition(const Action& action, State& s_new, double& r_new, bool& terminal);
  /*	Reimplements the corresponding function of the base class.
	Implements a transtion in responce to the action 
	performed by the agent. 
	Parameters:
	action: action performed by the agent
	s_new : return value - new state
	r_new : return value - new reward
	terminal: indication of whether s_new is a terminal state
  */

	
	
  ~MountainCarRand();	
};
