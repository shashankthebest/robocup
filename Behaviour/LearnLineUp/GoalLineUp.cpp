#include <stdlib.h>
#include <string.h>
#include<stdio.h>
#include<math.h>
#include<sys/types.h>		



#include "GoalLineUp.h"

#define TARGET_THETA 0
#define BALLINITIAL_X -50
#define BALLINITIAL_Y 0

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

GoalLineUp::GoalLineUp(	):Environment(),currPos(3,0.0f),kickPosition(2,0),targetPosition(2,0),currVel(3,0.0f), ballPos(2,0.0f), oldBallPos(2,0.0f)
{

			/*	Default constructor.
				Initializes state according to start state distribution
			*/
		bool t;
		CurrentState.x = new double[4];
		startState(CurrentState, t);
		reward=-1;
		transVel = 0;
		dirTheta = 0; 
		rotVel = 0;
		blindFrameCount = 0;
}


void GoalLineUp::uniformStateSample(State& s)
{
  //s.x[0]=-1.2+(double)rand()/((double)RAND_MAX)*1.7;	//random number in [-1.2, 0.5] - car's position
  //s.x[1]=-0.07+(double)rand()/((double)RAND_MAX)*0.14;//random number in [-0.07,0.07]- car's velocity
}

void GoalLineUp::startState(State& start, bool& terminal)
{
			/*	Selects start state
			*/
	//CurrentState.x[0]=-1.2+(double)rand()/((double)RAND_MAX)*1.7;	//random number in [-1.2, 0.5] - car's position
	//CurrentState.x[1]=-0.07+(double)rand()/((double)RAND_MAX)*0.14;//random number in [-0.07,0.07]- car's velocity
	//CurrentState.x[0] = 0;
	//CurrentState.x[1] = 0;
	//CurrentState.x[2] = 0;
	//CurrentState.x[3] = 1;
	double reward;
	makeObservation(reward, true);
	makeObservation(reward, true);
	start = CurrentState;
	start.x[0] = 30;
	start.x[1] = 0;
	start.x[2] = 0;
	start.x[3] = 0;
	start.x[4] = 1;
	terminal = false;
	Stages = 1;
}

void GoalLineUp::setState(const State& s, bool& terminal)
{
	CurrentState = s;
	if ((CurrentState.x[0]<0)||(CurrentState.x[0]>100))
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

void GoalLineUp::makeObservation(double &reward, bool initial)
{
	static bool firstCall = true;
	
	
	
	measureddistance = 0;
	balldistance = 0;
	ballbearing = 0;
	compass = 0;
	mixedSpeed = 0;
	

	
	Blackboard->Sensors->getCompass(compass );
	currPos[2] = compass;
	
	Blackboard->Sensors->getBallGps(ballPos);
	if(firstCall)
	{
		firstCall = false;
		oldBallPos = ballPos;
	}
	//cout<<"\nBall position : ["<<ballPos[0]<<", "<<ballPos[1]<<" ]";
	
	bool retVal = false;
	
	MobileObject& ball = Blackboard->Objects->mobileFieldObjects[FieldObjects::FO_BALL];
	
	measureddistance = Blackboard->Objects->mobileFieldObjects[FieldObjects::FO_BALL].measuredDistance();
	balldistance = measureddistance * cos(Blackboard->Objects->mobileFieldObjects[FieldObjects::FO_BALL].measuredElevation());
	ballbearing = Blackboard->Objects->mobileFieldObjects[FieldObjects::FO_BALL].measuredBearing();
	
	kickPosition[0] = ball.estimatedDistance() * cos(ball.estimatedBearing());
	kickPosition[1] = ball.estimatedDistance() * sin(ball.estimatedBearing());
	targetPosition[0] = kickPosition[0] + 1000.0f;
	targetPosition[1] = kickPosition[1];
	
	
	//Blackboard->Sensors->get(NUSensorsData::MotionWalkSpeed,currVel);
	//mixedSpeed = (transVel + rotVel)/2;
	
	if(ball.estimatedDistance() >= 120)
		CurrentState.x[0] = 120;
	else
		CurrentState.x[0] = ball.estimatedDistance();
		
	double currAngle = mathGeneral::rad2deg(currPos[2]);;
	
	if(currAngle < 0 )
		currAngle = 180 + currAngle;
	else 
		currAngle = currAngle - 180;
	
	
	
	CurrentState.x[1] = currAngle;
	
	CurrentState.x[2] = transVel;
	CurrentState.x[3] = rotVel;
	
	if (Blackboard->Objects->mobileFieldObjects[FieldObjects::FO_BALL].isObjectVisible())
	{
		CurrentState.x[4] = 1;
		if(blindFrameCount>0)
			blindFrameCount--;
	}
	else 
	{
		CurrentState.x[0] = s_last.x[0];  // if ball is not visible, donot decrease its distance from robot
		CurrentState.x[4] = 0;
		cout<<"\nI cannot see the ball!";
	}
	
	
	//cout<<"\nBall position : ["<<ballPos[0]<<", "<<ballPos[1]<<" ]";
	static int moved = 0;
	int movedX = fabs(BALLINITIAL_X - ballPos[0]); 
	int movedY = fabs( BALLINITIAL_Y - ballPos[1]);
	//cout<<"\nBall moved condition : "<<movedX <<", "<<movedY<<"\n";
	

	if(!initial)
	{
		if(checkTerminal() && Stages>2 )
		{
			reward = 100;	
			//terminal = true;
			cout<<"\n\n\nRewarding +ve heavily !\n\n\n\n\n\n";
			Blackboard->Actions->restartCondition = true;
		}
	
		if ( (movedX>2 || movedY>2))
		{
			moved++;
			if (moved>2)
			{
				cout<<"\nBall moved, restarting with negative reward!";
				reward = -10;
			//	Blackboard->Actions->restartCondition = true;
				
			}		
		}
			
		vector<float>roboPose(3,0);
		Blackboard->Sensors->getGps(roboPose);
		Blackboard->Sensors->getBallGps(ballPos);
	
		double balldistanceX = roboPose[0] - ballPos[0];
		double balldistanceY = roboPose[1] - ballPos[1];
	
		
		// cannot see the ball
		if(CurrentState.x[4]==0) // ||  !(Blackboard->Objects->mobileFieldObjects[FieldObjects::FO_BALL].isObjectVisible()))
		{
			reward = -10;
			cout<<"\nReturning heavy -ve reward!\n\n";
		}
/*		else if ( balldistanceX < 25 && (transVel>0.25 || rotVel>0.25) ) //// if reached ball but still walking
		{
			cout<<"\nReturning heavy -ve reward!\n\n";
			reward = -10;
			Blackboard->Actions->restartCondition = true;
		}
 */
		else 
			reward = -1;
	}
	
}


bool GoalLineUp::checkTerminal()
{
	//m_current_position = m_field_objects->self.wmState();
	bool retVal = false;
	//makeObservation();
	
	vector<float>roboPose(3,0);
   	Blackboard->Sensors->getGps(roboPose);
	Blackboard->Sensors->getBallGps(ballPos);
	
	double balldistanceX = roboPose[0] - ballPos[0];
	double balldistanceY = roboPose[1] - ballPos[1];
	
cout<<"\nBall distance = [ "<<balldistanceX<<", "<<balldistanceY<<" ]\n";
	
	if (Blackboard->Objects->mobileFieldObjects[FieldObjects::FO_BALL].isObjectVisible())  
	{
		if (balldistanceX<=25)
		{
			if( fabs(CurrentState.x[1] - TARGET_THETA) <2 )
			{
				if (    ( fabs(transVel) <= 0.25) && (fabs(rotVel) <= 0.25) )
				  {
						retVal = true;
				  }
			}
		}
		else 
		{
			retVal = false;
		}

	}
	else 
	{
		retVal = false;
	}


	return retVal;
	//cout<<"\nCheching terminal";
	
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

		s_last = CurrentState;
		double temp;
		

		//check action applicability
		if (applicable(CurrentState, action)==false)
		  {	cout << "Error (env_mc): unapplicable action performed in state: " << CurrentState << endl;
			
			exit(EXIT_FAILURE);
		}
		//\nPerformaing transition, Action taken
		//cout<<" : "<<action.value;
		
		if (action.value == 1 ) // Increase Translation velocity
		{
				transVel += 0.25;
				
		}
		else if (action.value == 2)  // Decrease Translation velocity
		{
				transVel -= 0.25;			


		}
		else if (action.value == 3)  // Increase Angle
		{
				rotVel -= 0.25;
							cout<<"\n----------------------- Decreasing rotation";
		}
		else if (action.value == 4)  // Decrease Angle
		{			
				rotVel += 0.25;	
				cout<<"\n----------------------- increasing rotation";
			
		}
		
		//cout<<" [ "<<CurrentState.x[0]<<", "<<CurrentState.x[1]<<", "<<CurrentState.x[2]<<" ] ";
	
	
		WalkJob* walk = new WalkJob(transVel, dirTheta, rotVel);
		Blackboard->Jobs->addMotionJob(walk);


	/*	if(checkTerminal())  // Makes observation and checks for terminal state
		{
			reward = 10;
			cout<<"\n\n\nReturned +ve reward\n\n";
		}
		else 
			reward = -1;
		*/
		makeObservation(reward, false); // this will set values for currentState
		
		/*
		
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
		*/

	CurrentAction = action;	//remember last action
	s_new = CurrentState;	//return new state
	r_new = reward;	//return new reward;
	Stages++;
	

}


bool GoalLineUp::applicable(const State& s, const Action& a)
		/*	Checks if action a is applicable in state s.
		*/
{
		bool retVal = false;
		
		int aVal = (int)a.value;
		
		if (aVal==1 && (transVel <1) )
				retVal = true;
		else if (aVal==2  && (transVel >0) )
				retVal = true;
		else if (aVal==3  && (rotVel  > -1 ))
				retVal = true;						
		else if (aVal==4  && (rotVel  < 1 ))
				retVal = true;			
		
		return retVal;			

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
		right = 80;
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
	if (i==3)
	{
		left = -1;
		right = 1;
	}
	if (i==4)
	{
		left = 0;
		right = 1;
	}
	
		
}

GoalLineUp::~GoalLineUp(){}
