/*	This is the implementation of the main() function for
	the program that implements randomized Mountain-Car task, 
	SARSA(lambda) agent with replacing eligibility traces and
	CMAC architectures to represent action value functions.

  Author: Bohdana Ratitch
  Version: February 2001
  file: mcrand_sarsart_cmac.cpp

 */

#include <iostream.h>
#include<time.h>
#include<math.h>
#include<sys/types.h>
#include<stdlib.h>
#include <fstream.h>

//#define DISPLAY_OUTPUT

#ifndef INTERFACE_CLASSES	
#define INTERFACE_CLASSES 1
#include "interface_classes.h"
#endif

#ifndef mc
#define mc
#include "mountain_car_rand.h"
#endif

#ifndef FA_CMAC	
#define FA_CMAC 1
#include "cmac.h"
#endif

#ifndef SARSA_AGENT	
#define SARSA_AGENT 1
#include "sarsart_agent.h"
#endif

#include "main_init.h"



void main(int argc, char* argv[])
{

  int i, d, j;
  char c, *buffer;
	
  int p=0;
  int steps=0;
/////////////////// Setting noise parameters
  MainParameters mainP;
  if (strncmp(argv[1],"?",1)==0){
    cout << "Usage:" << endl;
    cout << "executable_name parameter1=value1 parameter2=value2 ..." << endl;
    CMAC::helpLearningParameters();
    SarsaAgentRT::helpLearningParameters();
    cout << "Mountain-Car parameters:" << endl;
    cout << "AV=value : action variance" << endl;
    cout << "PV=value : position variance" << endl;
    p+=mainP.process(argc,argv);
    exit(EXIT_SUCCESS);
  }

  p+=mainP.process(argc,argv);
  double ActVar=0, PosVar=0;
  for (i=0; i<argc; i++){
    
    if (strncmp("AV=", argv[i], 3)==0){
      ActVar=atof(&(argv[i][3]));
    }

    if (strncmp("PV=", argv[i], 3)==0){
      PosVar=atof(&(argv[i][3]));
    }
  }
 ///////////////////// Setting up random number generator
  //seed randon number generator
  time_t stime;
  time(&stime);
  struct tm* currentTime;
  currentTime=gmtime(&stime);
  unsigned seed;
  seed=(currentTime->tm_hour+1)*currentTime->tm_min*currentTime->tm_sec;
  srand(seed);
 
 /////////////////////////// Setting up states and actions
 
  //state space and actions for the mountain car task
	
  State::dimensionality=2;

  ActionSet as(3);
  Action a1("forward", 1.0);
  as.addAction(a1);
  Action a2("reverse", -1.0);
  as.addAction(a2);
  Action a3("zero", 0.0);
  as.addAction(a3);


////////////////       Defining the environment
  MountainCarRand* mdp = new MountainCarRand(ActVar, PosVar);
  Environment* env = mdp;

	
	
///////////////////      Setting up Function Approximator	
  //Set parameters for CMAC and 
  //create approximators for each action with appropriate parameters
	
  
  if (mainP.strFile==NULL){
    cout << "Please provide name of file containing approximator structure or press e to exit" << endl;
    buffer = new char[20];
    cin >> buffer;
    if (strcmp("e", buffer)==0)
      exit(EXIT_SUCCESS);
    else {
      mainP.strFile = new char[strlen(buffer)+1];
      strcpy(mainP.strFile, buffer);
    }
  }

  Approximator** cmacSet = new Approximator*[Action::count];
		
  double* left = new double[State::dimensionality];
  double* right = new double[State::dimensionality];
  env->getStateSpaceBounds(left, right);
  CMAC::setInputBounds(left,right);
	
  for(i=0; i<Action::count; i++){
    cmacSet[i] = new CMAC(mainP.strFile);
  }
	
  StateActionFA* safa = new StateActionFA(Action::count, cmacSet);
  for(i=0; i<Action::count; i++){
    cmacSet[i]->setLearningParameters(argc, argv);
  }


////////////////////// Setting up SARSA Agent
  //create a learning agent object
  SarsaAgentRT* sarsa = new SarsaAgentRT(1, as, safa, env);
  Agent* agent = sarsa;
  agent->setLearningParameters(argc, argv);

/////////////////// Starting the experiment	
  run(mainP, safa, agent, true);
	
  delete sarsa;
  delete safa;
  delete mdp;
}
