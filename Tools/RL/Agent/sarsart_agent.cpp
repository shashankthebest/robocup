#include <stdlib.h>
#include <string.h>
#include<stdio.h>
#include<math.h>
#include<sys/types.h>		



#include "sarsart_agent.h"

//Implementation of the SarsaAgent class derived from Agent base class


//SarsaAgentRT::SarsaAgentRT(double g, const ActionSet& a_s,  StateActionFA* const f,  Environment* const e )
SarsaAgentRT::SarsaAgentRT(double g, ActionSet a_s,  StateActionFA* const f,  Environment* const e )
  //initialization list
  : Agent(g, a_s, f, e)
{	
  lambda=0;
  epsilon=0;
	startedLearning = false;
}

void SarsaAgentRT::helpLearningParameters(){
  cout << "SARSA learning parameters:" << endl;
  cout << "lambda=value : bootstraping parameter" << endl;
  cout << "epsilon=value : exploration parameter" << endl;
}

void SarsaAgentRT::setLearningParameters(int argc, char *argv[])
{
  int i=0;
  int  decimal;   
  char *decay[1];   
  decay[0]= new char[15];

  for (i=0; i<argc; i++){
    if (strncmp("?", argv[i],1)==0){
      cout << "Sarsa parameters:" << endl;
      cout << "lambda=value : bootstraping parameter" << endl;
      cout << "epsilon=value : exploration parameter" << endl;
      return;
    }

    if (strncmp("lambda=", argv[i], 7)==0)
    {
      lambda=atof(&(argv[i][7]));
      if ((lambda<0) || (lambda>1))
      {
		cout << "Error (agent): lambda must be in [0,1]" << endl;
		exit(EXIT_FAILURE);
      }
			 
      decimal=sprintf(decay[0],"decay=%f", lambda*gamma);
      fa->setAllLearningParameters(1, decay);
    }
    if (strncmp("alpha=", argv[i], 6)==0)
      fa->setAllLearningParameters(1,&(argv[i]));
		
    if (strncmp("epsilon=", argv[i], 8)==0)
    {
      epsilon=atof(&(argv[i][8]));
      if ((epsilon<0) || (epsilon>1))
      {
		cout << "Error (agent): epsilon must be in [0,1]" << endl;
		exit(EXIT_FAILURE);
      }
    }

  }
  delete decay[0];
  
}

void SarsaAgentRT::startLearning(int N, bool SaveTrajectory)
{
	discount=1.0;
	i=0;
	steps=0;
	
	Qv = new double[actions.size];
	
	chooseAction(CurrentState, CurrentAction, true);
	
}


int SarsaAgentRT::stepActAndLearn(int N, bool SaveTrajectory)
{
	if(!startedLearning)
	{
		startLearning(N, SaveTrajectory);
		startedLearning = true;
	}

	
	
	fa->decayTraces(lambda*gamma);
    fa->clearTraces(CurrentAction, CurrentState, 0);
    fa->replaceTraces(CurrentAction, CurrentState, 1.0);
    if (SaveTrajectory==true)
    {
		for (j=0; j<actions.size; j++)
			fa->predict(actions.action[j],CurrentState,Qv[j]);        //// Predict value                                S
    }
    
	env->transition(CurrentAction, NewState, CurrentReward, terminal);  //// Transit to next state                  A,R
    steps++;
    
    chooseAction(NewState, NewAction, true);         ////  With new state, select an action                               S
    
    fa->predict(NewAction, NewState, Qvalue);  //// New reward                                                      A (SARSA)
    
    if (SaveTrajectory==true)
    {
		fa->predict(CurrentAction, CurrentState, Qcheck);
		TDerror = Qcheck - (CurrentReward+gamma*Qvalue);
    }
    
    //cout << "cs:" << CurrentState << " ca:" << CurrentAction.id << " ns:" << NewState << " cr=" << CurrentReward << " td=" << TDerror << endl;
    fa->learn(CurrentAction, CurrentState, CurrentReward+gamma*Qvalue);
	
    if (SaveTrajectory==true)
    {
		trajectory->stage[i].state  = CurrentState;
		trajectory->stage[i].action = CurrentAction;
		trajectory->stage[i].reward = CurrentReward;
		for (j=0; j<actions.size; j++)
		{
			trajectory->stage[i].Qvalue[j]=Qv[j];
		}
		trajectory->stage[i].TDerror = TDerror;
		trajectory->length=i+1;
		
    }
	
    CurrentState  = NewState;
    CurrentAction = NewAction;
    Return = Return + discount*CurrentReward;
    discount = discount*gamma;
    i++;


	if (SaveTrajectory==true)
	{
		trajectory->stage[i].state = CurrentState;	//trajectory ends with state
		trajectory->length = i+1;
	}
	return steps;

	
}				


int SarsaAgentRT::actAndLearn(int N, bool SaveTrajectory)
  
{
	
  State NewState;
  Action NewAction;
  double Qvalue, Qcheck;
  double discount=1.0;
  int i=0;
  int steps=0;
  int j;
  double TDerror;
  
  
  double* Qv = new double[actions.size];

  chooseAction(CurrentState, CurrentAction, true);

  while (i<N-1 && terminal==false)
  {
    fa->decayTraces(lambda*gamma);
    fa->clearTraces(CurrentAction, CurrentState, 0);
    fa->replaceTraces(CurrentAction, CurrentState, 1.0);
    if (SaveTrajectory==true)
    {
      for (j=0; j<actions.size; j++)
		fa->predict(actions.action[j],CurrentState,Qv[j]);        //// Predict value                                S
    }
    env->transition(CurrentAction, NewState, CurrentReward, terminal);  //// Transit to next state                  A,R
    steps++;
    
    chooseAction(NewState, NewAction, true);         ////  With new state, select an action                               S
    
    fa->predict(NewAction, NewState, Qvalue);  //// New reward                                                      A (SARSA)
    
    if (SaveTrajectory==true)
    {
      fa->predict(CurrentAction, CurrentState, Qcheck);
      TDerror = Qcheck - (CurrentReward+gamma*Qvalue);
    }
    
    //cout << "cs:" << CurrentState << " ca:" << CurrentAction.id << " ns:" << NewState << " cr=" << CurrentReward << " td=" << TDerror << endl;
    fa->learn(CurrentAction, CurrentState, CurrentReward+gamma*Qvalue);

    if (SaveTrajectory==true)
    {
      trajectory->stage[i].state=CurrentState;
      trajectory->stage[i].action=CurrentAction;
      trajectory->stage[i].reward=CurrentReward;
      for (j=0; j<actions.size; j++)
      {
		trajectory->stage[i].Qvalue[j]=Qv[j];
      }
      trajectory->stage[i].TDerror=TDerror;
      trajectory->length=i+1;
			
    }
			
    CurrentState=NewState;
    CurrentAction=NewAction;
    Return=Return+discount*CurrentReward;
    discount=discount*gamma;
    i++;
  }
  
  if (SaveTrajectory==true)
  {
    trajectory->stage[i].state=CurrentState;	//trajectory ends with state
    trajectory->length=i+1;
  }
  delete [] Qv;
  return steps;
}

int SarsaAgentRT::act(int N, bool SaveTrajectory, bool ComputeBellmanError)
  
{
  State NewState;
  Action NewAction;
  double discount=1;
  int i=0;
  double rememberEpsilon=epsilon;
  int steps=0;
  double Qcurrent, Qnext;

  if (ComputeBellmanError==true) BellmanError=0;
	
  epsilon=0; 
  chooseAction(CurrentState, CurrentAction, false);

  while (i<N-1 && terminal==false){

    env->transition(CurrentAction, NewState, CurrentReward, terminal);
    chooseAction(NewState, NewAction, false);
    steps++;
    Return=Return+discount*CurrentReward;
    discount=discount*gamma;

    if (ComputeBellmanError==true)
	{
      fa->predict(CurrentAction, CurrentState, Qcurrent);
      fa->predict(NewAction, NewState, Qnext);
      BellmanError+=(Qcurrent-(CurrentReward+gamma*Qnext))*(Qcurrent-(CurrentReward+gamma*Qnext));
    }

    if (SaveTrajectory==true){
      trajectory->stage[i].state=CurrentState;
      trajectory->stage[i].action=CurrentAction;
      trajectory->stage[i].reward=CurrentReward;
      trajectory->length=i+1;
    }
			
    CurrentState=NewState;
    CurrentAction=NewAction;
    i++;
  }

  if ((ComputeBellmanError==true) && (steps!=0))  BellmanError/=(double)steps;
	
  if (SaveTrajectory==true){
    trajectory->stage[i].state=CurrentState;
    trajectory->length=i+1;
  }

  epsilon=rememberEpsilon;
  return steps;
}





int SarsaAgentRT::stepAct(int N, bool SaveTrajectory, bool ComputeBellmanError)

{
	static bool startedActing = false;
	if (!startedActing) 
	{
		State NewState;
		Action NewAction;
		double discount = 1;
		int i = 0;
		rememberEpsilon = epsilon;
		int steps = 0;
		Qcurrent = 0;
		Qnext = 0;
		
		if (ComputeBellmanError==true) BellmanError=0;

		chooseAction(CurrentState, CurrentAction, false);
		startedActing = true;
	}
	
	epsilon = 0; 

			
	env->transition(CurrentAction, NewState, CurrentReward, terminal);             // (S) According to previous action, make a transition
	chooseAction(NewState, NewAction, false);											   // (A) choose action
	steps++;
	Return = Return+discount*CurrentReward; 	
	discount = discount*gamma;
		
	if (ComputeBellmanError==true)
	{
		fa->predict(CurrentAction, CurrentState, Qcurrent);
		fa->predict(NewAction, NewState, Qnext);
		BellmanError+=(Qcurrent-(CurrentReward+gamma*Qnext))*(Qcurrent-(CurrentReward+gamma*Qnext));
	}
		
	if (SaveTrajectory==true)
	{
		trajectory->stage[i].state=CurrentState;
		trajectory->stage[i].action=CurrentAction;
		trajectory->stage[i].reward=CurrentReward;
		trajectory->length=i+1;
	}
		
	CurrentState = NewState;
	CurrentAction = NewAction;

	
	if ((ComputeBellmanError==true) && (steps!=0))  BellmanError/=(double)steps;
	
	if (SaveTrajectory==true){
		trajectory->stage[i].state=CurrentState;
		trajectory->length=i+1;
	}
	
	epsilon=rememberEpsilon;
	return steps;
}








void SarsaAgentRT::chooseAction(const State& s, Action& a, bool islearning)
  /* Implements an epsilon-greedy policy
   */
{
	
  int NumberAA=0;		//number of applicable actions
  int i;
  int id=0;	//selected action id
	//cout<<"\n\nTotal Actions  : "<<actions.size;
  for (i=0;i<=actions.size;i++)
  {
	 // cout<<"\t  , "<<actions.action[i] ;
    if (env->applicable(s,actions.action[i])==true)
      {
		  
		  	ApplicableActions[NumberAA]=i;
			NumberAA++;
		 // cout<<"\nValue of applicable action = "<<actions.action[i].id;
			
      }
  }
 // cout<<"\nApplicable actions = "<<NumberAA;
  if (NumberAA==0) 
    {	
		cout << "\n\nNo action can be taken in the current state " << endl<<endl;
		exit(EXIT_FAILURE);
    }

  if  (    (  (double)rand()/(double)RAND_MAX <= epsilon)   && !islearning )
    //take any action uniformenly
    {	
		
      id = ApplicableActions[rand()%NumberAA];
      a = actions.action[id];
		cout<<"\n-------------- taken a random action ----------\n";
    }
  else //select greedy ection
    {	
		
      double* Values, BestValue;
      Action b;
      int NumberGreedyActions, gr;
      int* GreedyActions;

      Values = new double[NumberAA];
      GreedyActions = new int[NumberAA];

      for (i=0; i<NumberAA; i++)
      {
		id = ApplicableActions[i];
		a  = actions.action[id];
		fa->predict(a, s, Values[i]);
      }
      
      BestValue = Values[0];
      NumberGreedyActions = 1;
      GreedyActions[0] = ApplicableActions[0];

		
      for (i=1; i<NumberAA; i++)
      {
		if (Values[i] > BestValue)
		{
			BestValue = Values[i];
			NumberGreedyActions = 1;
			GreedyActions[0] = ApplicableActions[i];
		}
		if (Values[i]==BestValue)
		{
			NumberGreedyActions++;
			GreedyActions[NumberGreedyActions-1] = ApplicableActions[i];
		}
      }
	  if(NumberGreedyActions > 1)
	  {
		  gr = rand()%NumberGreedyActions;
		  a = actions.action[GreedyActions[gr]];
		  cout<<"\n-------------- taken one of a greedy actions ---------- : "<< BestValue<<"\n ";

	  }
	  else 
	  {
		  a = actions.action[GreedyActions[0]];
		  cout<<"\n-------------- taken a greedy action ---------- : "<< BestValue<<"\n ";

	  }

	
	  
      delete [] Values;
      delete [] GreedyActions;
		
    }
}

SarsaAgentRT::~SarsaAgentRT(){
	
}
