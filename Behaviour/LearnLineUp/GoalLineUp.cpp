#include <stdlib.h>
#include <string.h>
#include<stdio.h>
#include<math.h>
#include<sys/types.h>		



#include "GoalLineUp.h"




#include "Infrastructure/Jobs/JobList.h"
#include "Infrastructure/NUSensorsData/NUSensorsData.h"
#include "Infrastructure/NUActionatorsData/NUActionatorsData.h"
#include "Infrastructure/FieldObjects/FieldObjects.h"
#include "Behaviour/BehaviourPotentials.h"
#include "Infrastructure/NUBlackboard.h"
#include "Infrastructure/Jobs/MotionJobs/WalkJob.h"
#include "Infrastructure/Jobs/MotionJobs/HeadJob.h"
#include "Infrastructure/Jobs/MotionJobs/KickJob.h"
//#include 


//	IMPLEMENTATION OF Environment class for mountain car task

GoalLineUp::GoalLineUp()
{
			/*	Default constructor.
				Initializes state according to start state distribution
			*/
		bool t;
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
	CurrentState.x[3] = 0;
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
	
	MobileObject& ball = m_field_objects->mobileFieldObjects[FieldObjects::FO_BALL];
	
	float measureddistance = m_field_objects->mobileFieldObjects[FieldObjects::FO_BALL].measuredDistance();
	float balldistance = measureddistance * cos(m_field_objects->mobileFieldObjects[FieldObjects::FO_BALL].measuredElevation());
	float ballbearing = m_field_objects->mobileFieldObjects[FieldObjects::FO_BALL].measuredBearing();
	
	vector<float> kickPosition(2,0);
	vector<float> targetPosition(2,0);
	kickPosition[0] = ball.estimatedDistance() * cos(ball.estimatedBearing());
	kickPosition[1] = ball.estimatedDistance() * sin(ball.estimatedBearing());
	targetPosition[0] = kickPosition[0] + 1000.0f;
	targetPosition[1] = kickPosition[1];
	
	
	if (fabs(currPos[0] - targetPosition ) 
	
	
	
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
	{if ((a.value==-1.0) || (a.value==0.0) || (a.value == 1.0) )
		return true;
	else return false;
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
	if (i==0) {	
		left = -1.2;
		right= 0.5;
		return;
	}
	if (i==1){
		left = -0.07;
		right = 0.07;
	}
		
}

GoalLineUp::~GoalLineUp(){}
