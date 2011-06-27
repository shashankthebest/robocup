/*! @file TryLineUpState.h
    @brief A state where the robot tries to line up to ball

    @author Shashank Bhatia

  Copyright (c) 2011 Shashank Bhatia

    This file is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This file is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with NUbot.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef LINEUPWALKTOBALL_H
#define LINEUPWALKTOBALL_H

#include "LineUpProvider.h"
#include "LineUpState.h"

#include "Infrastructure/Jobs/JobList.h"
#include "Infrastructure/NUSensorsData/NUSensorsData.h"
#include "Infrastructure/NUActionatorsData/NUActionatorsData.h"
#include "Infrastructure/FieldObjects/FieldObjects.h"
#include "Behaviour/BehaviourPotentials.h"

#include "Infrastructure/Jobs/MotionJobs/WalkJob.h"
#include "Infrastructure/Jobs/MotionJobs/HeadJob.h"
#include "Infrastructure/Jobs/MotionJobs/KickJob.h"

#include <iostream>
#include<time.h>
#include<math.h>
#include<sys/types.h>
#include<stdlib.h>
#include <fstream>
#include <string.h>
#include <stdio.h>

#include "Tools/RL/Misc/globals.h"
#include "Tools/RL/rlIncludes.h"


#include "debug.h"

class LineUpWalkToBallState : public LineUpState
{
private:
	bool canKick;
	bool kicked;
	bool lostBall;

	// Rl related
	
	int i, j, k;
	char* runID;
	char* fileHistory;	
	char** fileAP;
	State* TestStates;
	
	
	double avrTR; //averrage test return (over many trajectories from each test state)
	double avrTRS; //average test return per step (over many trajectories from each test state)
	double sampleTR; //test return (one trajectory form each test state)
	double sampleTRS; //test return per step (one trajectory form each test state)
	int steps;
	double* MaxParameterChanges;
	int* NumberParametersChanged;
	
	
		
	
	/////////////////////////////////// RL Related
 //   GoalLineUp* mdp;          // Environment
//	Environment* env;         // base pointer
	
//	Approximator** cmacSet;   // CMAC approximatorSet
//	double* left ;            // State bounds
 //   double* right;			  // State bounds
    
  //  StateActionFA* safa;      // State-Action FA
    
  //  SarsaAgentRT* sarsa;      // SARSA Agent
  //  Agent* agent;			  // Base pointer
	
//	MainParameters *mainP;     // Main Parameters
	//////////////////////////////////////////////////
	
	ifstream ifile;
	ofstream ofsHistory;
	int runCount;
	int learnCount;
	bool saveFA;
	bool initialised;
	
	
	
public:
    LineUpWalkToBallState(LineUpProvider* parent): LineUpState(parent) //,MainParameters *mainp, StateActionFA* sa_fa, Agent* p_agent, bool savefa=true) : LineUpState(parent)
    {
		
		//mainP = mainp;
		//safa = sa_fa;
		//agent = p_agent;
		//saveFA = savefa;
        learnCount = 1;
        m_parent = parent;
        initialised = false;
		/////////// Kick Related Stuff
		canKick = false;
        kicked = false;
        lostBall = false;
		runCount = 0;
		i = 0;
		
		///////////    RL Related File handing Stuff

		runID = new char[5];
		sprintf(runID, "%d",  m_parent->mainP->run);
		
		fileHistory = new char[100];	//name of the file for learning history data
		strcpy(fileHistory,  m_parent->mainP->dir);
		strcat(fileHistory, "r_");
		strcat(fileHistory, runID);
		strcat(fileHistory ,".hst");
		
		fileAP  = new char*[Action::count];	//name of files for approximator settings
		for (i=0; i<Action::count; i++)
		{
			fileAP[i] = new char[100];
			strcpy(fileAP[i],  m_parent->mainP->dir);
			strcat(fileAP[i], "r_");
			strcat(fileAP[i],runID);
			char temp[5];
			sprintf(temp,".a%d",i);
			strcat(fileAP[i],temp);
			
		}
		
		//load test states
		char c;
		if ( m_parent->mainP->TestStatesFile==NULL)
		{
			cout << "No name specified for the file containing test states \n\n" << endl;
			exit(EXIT_FAILURE);
		}
		ifile.open( m_parent->mainP->TestStatesFile);
		if (ifile.fail())
		{
			cout << "Cannot open file to load test set\n\n" << cout;
			exit(EXIT_FAILURE);
		}
		
		 TestStates=new State[ m_parent->mainP->TestStatesNumber];
		double random;
		
		for (i=0; i< m_parent->mainP->TestStatesNumber; i++)
		{
			
			ifile >> random;
			if (ifile.fail())
			{
				cout << "Erro loading test set " << cout;
				exit(EXIT_FAILURE);
			}
			
			TestStates[i].x[0]=random;
			ifile.get(c);
			ifile >> random;
			if (ifile.fail()){
				cout << "can't open file\n\n" << cout;
				exit(EXIT_FAILURE);
			}
			TestStates[i].x[1]=random;
		}
		
		ifile.close();
		
		//cout<<"\n\nFile NAme : "<<fileHistory<<"\n\n";
		
		ofsHistory.open(fileHistory);	//file stream for saving learning history data
		if (ofsHistory.fail()){
			cout << "Error: can not open file to save history\n\n" << endl;
			exit(EXIT_FAILURE);
		}
		
		
		
		/////////////////////// Rl Main Computational variables setp
		
		
		double avrTR; //averrage test return (over many trajectories from each test state)
		double avrTRS; //average test return per step (over many trajectories from each test state)
		double sampleTR; //test return (one trajectory form each test state)
		double sampleTRS; //test return per step (one trajectory form each test state)
		int steps;
		 MaxParameterChanges = new double[Action::count];
		 NumberParametersChanged = new int[Action::count];
		 
		 cout<<"\nFinished setting up files for RL engine";
		
    };
    
	
	virtual ~LineUpWalkToBallState() {};
    
	
	
	void performInitialTest()
	{
		//test policy before learning
		avrTR=0;
		avrTRS=0;
		for(k=0; k< m_parent->mainP->TestSamples; k++)
		{	
			sampleTR=0;
			sampleTRS=0;
			for(j=0; j< m_parent->mainP->TestStatesNumber; j++)
			{
				
				steps = m_parent->agent->initTrial( m_parent->mainP->Steps, false, false, &( TestStates[j]), NULL,false);
				
				sampleTR+= m_parent->agent->getReturn();
				if (steps!=0)
					sampleTRS+=  m_parent->agent->getReturn()/(double)steps;
			}
			sampleTR/=(double)( m_parent->mainP->TestStatesNumber);
			sampleTRS/=(double)( m_parent->mainP->TestStatesNumber);
			avrTR += sampleTR;
			avrTRS += sampleTRS;
		}
		
		avrTR/=(double)(m_parent->mainP->TestSamples);
		avrTRS/=(double)(m_parent->mainP->TestSamples);
		
		 m_parent->safa->getMaxParameterChange(MaxParameterChanges);
		 m_parent->safa->getNumberParametersChanged(NumberParametersChanged);
		
		ofsHistory << 0 << "\t" << avrTR << "\t" << avrTRS;
		for(j=0; j<Action::count; j++){
			ofsHistory << "\t" << MaxParameterChanges[j] << "\t" << NumberParametersChanged[j];
		}
		
		ofsHistory << endl;
		if (ofsHistory.fail()){
			cout << "Error writing history after " << i << " learning trials" << endl;
			exit(EXIT_FAILURE);
		}
	}
	
	
	void learnPolicy()
	{
		
		if (i<= m_parent->mainP->Trials)
		{
			cout << "\n\nLearning trial no : "<<i<<"\n\n";;
			i++;                   // Steps in trial, LearnOrNot,  SaveTrajectory?, startingState, fileName4Trajectory, BellmanError? 

			m_parent->agent->stepTrial(true,true,false);

			
			
			//			steps =  m_parent->agent->initTrial( 1,       true,       true,          NULL,          "trajectory.dat", false); //learning trial
			/*
			if ((i% m_parent->mainP->TestFrequency)==0)                  // after few episodes, the agent is tested. This happnes at specified frequency
			{ //testing current policy
				avrTR=0;
				avrTRS=0;
				for(k=0; k< m_parent->mainP->TestSamples; k++)
				{	
					sampleTR=0;
					sampleTRS=0;
					for(j=0; j< m_parent->mainP->TestStatesNumber; j++)
					{
						steps = m_parent->agent->initTrial(m_parent->mainP->Steps, false, false, &(TestStates[j]), NULL,false);
						sampleTR += m_parent->agent->getReturn();
						if (steps!=0)
							sampleTRS += m_parent->agent->getReturn()/(double)steps;
					}
					sampleTR/=(double)(m_parent->mainP->TestStatesNumber);
					sampleTRS/=(double)(m_parent->mainP->TestStatesNumber);
					avrTR += sampleTR;
					avrTRS += sampleTRS;
				}
				
				avrTR/=(double)(m_parent->mainP->TestSamples);
				avrTRS/=(double)(m_parent->mainP->TestSamples);
				
				m_parent->safa->getMaxParameterChange(MaxParameterChanges);
				m_parent->safa->getNumberParametersChanged(NumberParametersChanged);
				
				ofsHistory << i << "\t" << avrTR << "\t" << avrTRS;
				for(j=0; j<Action::count; j++)
				{
					ofsHistory << "\t" << MaxParameterChanges[j] << "\t" << NumberParametersChanged[j];
				}
				
				ofsHistory << endl;
				if (ofsHistory.fail())
				{
					cout << "Error writing history after " << i << " learning trials" << endl;
					exit(EXIT_FAILURE);
				}
			} //end of testing
			*/
			
		}	//end of the learning loop
		
		
		if (saveFA==true && i%200==0) 
			m_parent->safa->saveAllArchitectureParameters(fileAP);
		
		
	}
	
	
	
	virtual BehaviourState* nextState()
    {
        //if(lostBall)
          //  return m_parent->m_localiseBall;
        //if(kicked)
          //  return m_parent->m_generate;
        //else
            return this;
    };
    virtual void doState()
    {
	
        if (m_parent->stateChanged())
        {
                    canKick = false;
                    kicked = false;
                    lostBall = false;
                    start();
                    cout<<"\nReached line up state\n\n";
                  //  vector<float> ballPosition(3,0);
                   //  cout<<"\n\n\n\nI was here\n\n\n\n";
                    //ballPosition = m_parent->m_globalMap->findObservation(6); // ball id = 6
                    // cout<<"\n\n\n\n and here\n\n\n\n";
                   // //m_parent->m_globalMap->clearObservation(6);
                    //float dist = sqrt( ballPosition[0]*ballPosition[0] +  ballPosition[1]*ballPosition[1] );
                    //float elevation = atan2(  dist,55 );

                    //m_jobs->addMotionJob(new HeadTrackJob(elevation ,ballPosition[2],0,0));
                    //m_jobs->addMotionJob(new HeadPanJob(HeadPanJob::Localisation, dist , dist , ballPosition[2] , ballPosition[2] ));
                    //HeadTrackJob* head = new HeadTrackJob(m_field_objects->mobileFieldObjects[FieldObjects::FO_BALL]);
                    //m_jobs->addMotionJob(head);

					// set all initial values, get observations from environment
			
					// Test policy before learning
					//performInitialTest();
					//cout<<"\nI am here  : "<<__FILE__<<"   at "<<__LINE__<<"  \n\n";
			
			if(!initialised)
			{
				cout<<"\n\n\nInitialising trial\n\n";				
				m_parent->agent->initStepWiseTrial( 1,       true,       true,          NULL,          "trajectory.dat", false); //learning trial
				initialised = true;
			}

			
        }

		tick();
		
		Self& self  = m_field_objects->self;
		MobileObject& ball = m_field_objects->mobileFieldObjects[FieldObjects::FO_BALL];
		
		
		if (m_field_objects->mobileFieldObjects[FieldObjects::FO_BALL].isObjectVisible())
			{
			   // cout<<"\n\nBall Visible, going towards it\n\n";
			   // cout<<"\nCurrentTime : "<<m_current_time<<"\n";
				float headyaw, headpitch;
				m_data->getPosition(NUSensorsData::HeadPitch,headpitch);
				m_data->getPosition(NUSensorsData::HeadYaw, headyaw);
				float measureddistance = m_field_objects->mobileFieldObjects[FieldObjects::FO_BALL].measuredDistance();
				float balldistance = measureddistance * cos(m_field_objects->mobileFieldObjects[FieldObjects::FO_BALL].measuredElevation());
				float ballbearing = m_field_objects->mobileFieldObjects[FieldObjects::FO_BALL].measuredBearing();

				float trans_speed = 1;
				float trans_direction = ballbearing;
				float yaw = ballbearing/2;
				
				
				
				m_jobs->addMotionJob(new HeadTrackJob(ball));
//				
//				
//				vector<float> kickPosition(2,0);
//				vector<float> targetPosition(2,0);
//				kickPosition[0] = ball.estimatedDistance() * cos(ball.estimatedBearing());
//				kickPosition[1] = ball.estimatedDistance() * sin(ball.estimatedBearing());
//				targetPosition[0] = kickPosition[0] + 1000.0f;
//				targetPosition[1] = kickPosition[1];
				
				// collect all info, and keep ready for rl algo

			}
			else  if (ball.TimeSinceLastSeen() > 250)
			{

				m_jobs->addMotionJob(new HeadPanJob(ball));

			}

	
			if (m_time_in_state > 2000)
			{
				if (m_current_time - m_field_objects->mobileFieldObjects[FieldObjects::FO_BALL].TimeLastSeen() > 2000)
					lostBall = true;
			}
			
			if( (runCount >5))
			{
				if (learnCount%5==0 )
				{
					//cout<<"\nLearnCount = "<<learnCount;
					learnPolicy();
				}
				learnCount++;
				
				if(learnCount > 32000)
					learnCount = 1;
					
					
			}
			else 
			{
				runCount++;
				
				//m_jobs->addMotionJob(new WalkJob(0.01,0,0));
									 
			}
			

		}
};

#endif

