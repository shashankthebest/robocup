/*	Implementation of the randomized version of the mountain-car task,
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


#ifndef ENV_MCR	
	#define ENV_MCR 1
	#include "mountain_car_rand.h"
#endif


MountainCarRand::MountainCarRand(double ActVar, double PosVar){
   
  bool t;
  startState(CurrentState, t);
  reward=-1;
  ActionNoiseVar=ActVar;
  PositionNoiseVar=PosVar;
}



void MountainCarRand::transition(const Action& action, State& s_new, double& r_new, bool& terminal){
  
  State s_last=CurrentState;
  double temp;
  double ActionValue;
		
		
  //check action applicability
  if (applicable(CurrentState, action)==false)
    {	
      cout << "Error (env_mc): unapplicable action performed in state: " << CurrentState << endl;
      exit(EXIT_FAILURE);
    }
  //calculate new velocity
  if (action.value == 0) ActionValue=0;	
  else ActionValue=sqrt(ActionNoiseVar)*gasdev(&idum)+action.value;
		      
  temp=s_last.x[1]+0.001*ActionValue-0.0025*cos(3*s_last.x[0]);
  if (temp<-0.07) 
    CurrentState.x[1]=-0.07;
  else if (temp<=0.07) 
    CurrentState.x[1]=temp;
  else CurrentState.x[1]=0.07;
		
  //calculate new position
  reward=-1.0; terminal=false;	//until goal is reached
		
  temp=sqrt(PositionNoiseVar)*gasdev(&idum)+(s_last.x[0]+CurrentState.x[1]);
  if (temp<-1.2) 
    {CurrentState.x[0]=-1.2; CurrentState.x[1]=0.0;}
  else if (temp<0.5) 
    CurrentState.x[0]=temp;
  else {CurrentState.x[0]=0.5;terminal=true;reward=0.0;}
		
#ifdef DEBUG
  cout << "Current State: " << endl;
  getState();
  cout << "Action performed: " << action.description << endl;
  cout << "Reward: " << reward << endl;
#endif

  CurrentAction=action;	//remember last action
  s_new=CurrentState;	//return new state
  r_new=reward;	//return new reward;
  Stages++;


#ifdef DEBUG
  cout << "New State: " << endl;
  getState();
  if (terminal==true)
    cout << "Goal reached after " << Stages << " steps!" << endl;
#endif

}



MountainCarRand::~MountainCarRand(){}



