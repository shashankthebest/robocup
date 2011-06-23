#include <stdlib.h>
#include <string.h>
#include<stdio.h>
#include<math.h>
#include<sys/types.h>		



#include "GoalLineUp.h"

#define TARGET_THETA 0


#include "Infrastructure/Jobs/JobList.h"
#include "Infrastructure/NUSensorsData/NUSensorsData.h"
#include "Infrastructure/NUActionatorsData/NUActionatorsData.h"
#include "Infrastructure/FieldObjects/FieldObjects.h"
#include "Behaviour/BehaviourPotentials.h"
#include "Infrastructure/NUBlackboard.h"
#include "Infrastructure/Jobs/MotionJobs/WalkJob.h"
#include "Infrastructure/Jobs/MotionJobs/HeadJob.h"
#include "Infrastructure/Jobs/MotionJobs/KickJob.h"

#include "Tools/Math/General.h"
#include "Infrastructure/Jobs/JobList.h"
#include "Infrastructure/NUSensorsData/NUSensorsData.h"
#include "Infrastructure/NUActionatorsData/NUActionatorsData.h"
#include "Infrastructure/FieldObjects/FieldObjects.h"
#include "Behaviour/BehaviourPotentials.h"

#include "Infrastructure/Jobs/MotionJobs/WalkJob.h"
#include "Infrastructure/Jobs/MotionJobs/HeadJob.h"
#include "Infrastructure/Jobs/MotionJobs/KickJob.h"


//#include 


//	IMPLEMENTATION OF Environment class for mountain car task

GoalLineUp::GoalLineUp():Environment()
{
			/*	Default constructor.
				Initializes state according to start state distribution
			*/
		bool t;
		CurrentState.x = new double[3];
		startState(CurrentState, t);
		reward=-1;
}

void GoalLineUp::uniformStateSample(State& s)
{
  //s.x[0]=-1.2+(double)rand()/((double)RAND_MAX)*1.7;	//random number in [-1.2, 0.5] - car's position
  //s.x[1]=-0.07+(double)rand()/((double)RAND_MAX)*0.14;//random number in [-0.07,0.07]- car's velocity
}

void GoalLineUp::startState(State& start, bool& terminal){
			/*	Selects start state
			*/
	//CurrentState.x[0]=-1.2+(double)rand()/((double)RAND_MAX)*1.7;	//random number in [-1.2, 0.5] - car's position
	//CurrentState.x[1]=-0.07+(double)rand()/((double)RAND_MAX)*0.14;//random number in [-0.07,0.07]- car's velocity
	CurrentState.x[0] = 0;
	CurrentState.x[1] = 0;
	CurrentState.x[2] = 0;
	//CurrentState.x[3] = 0;
	start=CurrentState;
	terminal = false;
	Stages=1;
}

void GoalLineUp::setState(const State& s, bool& terminal){
	CurrentState=s;
	if ((CurrentState.x[0]<0)||(CurrentState.x[0]>50))
	{
		cout << "State set to invalid value" << endl;
		cout << s << endl;
		exit(EXIT_FAILURE);
	}

	if ((CurrentState.x[1] < -25)||(CurrentState.x[1] > 25)){
		cout << "State set to invalid value" << endl;
		cout << s << endl;
		exit(EXIT_FAILURE);
	}
	
	terminal = checkTerminal();
	Stages=1;
}

bool GoalLineUp::checkTerminal()
{
	//m_current_position = m_field_objects->self.wmState();
	vector <float> currPos(3,0.0f);
	float compass = 0;
	Blackboard->Sensors->getCompass(compass );
	currPos[2] = compass;
	
	bool retVal = false;
	
	MobileObject& ball = Blackboard->Objects->mobileFieldObjects[FieldObjects::FO_BALL];
	
	float measureddistance = Blackboard->Objects->mobileFieldObjects[FieldObjects::FO_BALL].measuredDistance();
	float balldistance = measureddistance * cos(Blackboard->Objects->mobileFieldObjects[FieldObjects::FO_BALL].measuredElevation());
	float ballbearing = Blackboard->Objects->mobileFieldObjects[FieldObjects::FO_BALL].measuredBearing();
	
	vector<float> kickPosition(2,0);
	vector<float> targetPosition(2,0);
	kickPosition[0] = ball.estimatedDistance() * cos(ball.estimatedBearing());
	kickPosition[1] = ball.estimatedDistance() * sin(ball.estimatedBearing());
	targetPosition[0] = kickPosition[0] + 1000.0f;
	targetPosition[1] = kickPosition[1];
	
	vector<float>currVel(3,0.0f);
	Blackboard->Sensors->get(NUSensorsData::MotionWalkSpeed,currVel);
	float mixedSpeed = currVel[0]*currVel[1];
	
	
	if ((fabs(currPos[0] - targetPosition[0]) <=20 )  &&  fabs(currPos[2] -TARGET_THETA) <= 0.034  
	     && mixedSpeed<0.5 && currVel[2]<0.02)
	{
		retVal = true;
	}
	
	
	
}


void GoalLineUp::transition(const Action& action, State& s_new, double& r_new, bool& terminal){
			/*	Implements a transtion in responce to the action 
				performed by the agent. Updates its internal variables 
				and delivers values to the agent.
				Parameters:
					action: action performed by the agent
					s_new : return value - new state
					r_new : return value - new reward
					terminal: indication of whether s_new is a terminal state
			*/

		//new state and reward:

		State s_last=CurrentState;
		double temp;
		
		
		//check action applicability
		if (applicable(CurrentState, action)==false)
		  {	cout << "Error (env_mc): unapplicable action performed in state: " << CurrentState << endl;
			
			exit(EXIT_FAILURE);
		}
		
	
		
		if (action.value == 1 ) // Increase Translation velocity
		{
			if (CurrentState.x[0] <1)
				CurrentState.x[0] += 0.1;
				
		}
		else if (action.value == 2)  // Decrease Translation velocity
		{
			if (CurrentState.x[0] >0)
				CurrentState.x[0] -= 0.1;			
		}
		else if (action.value == 3)  // Increase Angle
		{
			if (CurrentState.x[1] < mathGeneral::deg2rad(180))
				CurrentState.x[1] += mathGeneral::deg2rad(2);						
		}
		else if (action.value == 4)  // Decrease Angle
		{
			if (CurrentState.x[1] > mathGeneral::deg2rad(-180))
				CurrentState.x[1] -= mathGeneral::deg2rad(2);									
		}
		else if (action.value == 5)	 // Increase Rotational Velocity
		{
			if (CurrentState.x[2] < mathGeneral::deg2rad(180))
				CurrentState.x[2] += mathGeneral::deg2rad(2);				
		}
		else if (action.value == 6)	 // Decrease Rotational Velocity
		{
			if (CurrentState.x[2] > mathGeneral::deg2rad(-180))
				CurrentState.x[2] -= mathGeneral::deg2rad(2);				
		}
		
		
		
		
		
		
		
		
		//calculate new velocity
		temp=s_last.x[1]+0.001*action.value-0.0025*cos(3*s_last.x[0]);
		if (temp<-0.07) 
			CurrentState.x[1]=-0.07;
		else if (temp<=0.07) 
				CurrentState.x[1]=temp;
			else CurrentState.x[1]=0.07;
		
		//calculate new position
		reward=-1.0; terminal=false;	//until goal is reached
		
		temp=s_last.x[0]+CurrentState.x[1];
		if (temp<-1.2) 
			{CurrentState.x[0]=-1.2; CurrentState.x[1]=0.0;}
		else if (temp<0.5) 
				CurrentState.x[0]=temp;
			else {CurrentState.x[0]=0.5;terminal=true;reward=0.0;}
		

	CurrentAction=action;	//remember last action
	s_new=CurrentState;	//return new state
	r_new=reward;	//return new reward;
	Stages++;
	

}


bool GoalLineUp::applicable(const State& s, const Action& a)
		/*	Checks if action a is applicable in state s.
		*/
{
		int aVal = (int)a.value;
		if ((aVal==0 ) || (aVal==1 ) || (aVal==2 ) || (aVal==3 )  || (aVal==4 ) || (aVal==5 )  )
			return true;
		else 
			return false;
}

void GoalLineUp::bound(int i, bool& bounded, double& left, double& right)
		/*	Gives bounds on state variables' values
			This implementation is for the case where all variables are unbounded.
			This method should be redefined by derived classes in case of bounded variables
			Parameters:
				i : index of state variable
				bounded: indicates if i^th variable is bounded
				left : left bound
				right: right bound
			*/
{	
	if ((i<0) || (i>= State::dimensionality)) 
	{	cout << "Error (env_mc:bound()): variable index out of limit" << endl;
		exit(EXIT_FAILURE);
	}
	
	bounded=true;
	if (i==0) 
	{	
		left = 0;
		right= 50;
		return;
	}
	if (i==1)
	{
		left = -180;
		right = 180;
	}
	if (i==2)
	{
		left = 0;
		right = 1;
	}
		
}

GoalLineUp::~GoalLineUp(){}
