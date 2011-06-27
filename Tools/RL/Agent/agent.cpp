//#include<stdlib.h>
#include<math.h>
#include<time.h>

#include "Tools/RL/rlIncludes.h"
#include "agent.h"


//Agent::Agent(double g, const ActionSet& a_s, StateActionFA* const f, Environment* const e)
//  : actions(a_s), fa(f), env(e), gamma(g), trajectory(NULL), CurrentAction(), CurrentState()
Agent::Agent(double g,  ActionSet a_s, StateActionFA* const f, Environment* const e)
  : actions(a_s), fa(f), env(e), gamma(g), trajectory(NULL), CurrentAction(), CurrentState()
{
  ApplicableActions = new int[Action::count];
  cout<<"\n\nAction count seen = "<<Action::count<<"\n\n";
  cout<<"\n\nTotal actions in base = "<<actions.size<<"\n\n";
  
  cout<<"\nAction Size  : "<<__FILE__<<"   at "<<__LINE__<<"  is "<<actions.size;
  
  terminal=false;
  BellmanError=-1.0;
}

///

int Agent::initTrial(int N, bool learning, bool SaveTrajectory, const State* s = NULL, char* fileName = NULL, bool ComputeBellmanError = false) 
{	
	
	int i, j;
	int steps=0;
	
	Return=0.0;
	if (SaveTrajectory==true)
		trajectory = new Trajectory(N+1);
	
	if (s==NULL)
		env->startState(CurrentState, terminal);
	else
	{
		CurrentState=*s;
		env->setState(*s, terminal);
	
	}
	
	if (learning==true)
	{
		steps = actAndLearn(N, SaveTrajectory);

	}
	else
	{
		steps=act(N, SaveTrajectory, ComputeBellmanError);
	}
	
	
	//IMPLEMENT HERE SAVING TRAJECTORY TO A FILE
	if (SaveTrajectory==true)
	{
		ofstream file(fileName, ios::app);
		if (file.fail())
		{
			cout << "Error (agent): cannot open file to save trajectory" << fileName << endl;
			exit(EXIT_FAILURE);
		}
		file << "New trial" << endl;
		for (i=0; i<trajectory->length-1; i++)
		{
						
			file << "Stage " << i << ": ";
			file << "\t" << trajectory->stage[i].state;
			if (file.fail())
			{
				cout << "Error (agent): saving trajectory" << endl;
				exit(EXIT_FAILURE);
			} 
 
			if (learning)
			{
				file << "\t(";
				for (j=0; j<actions.size; j++)
					file <<trajectory->stage[i].Qvalue[j] << ",";
				file << ")";
			}
			file << "\t" << trajectory->stage[i].action;
			if (file.fail())
			{
				cout << "Error (agent): saving trajectory" << endl;
				exit(EXIT_FAILURE);
			}
			
			file << "\t" << trajectory->stage[i].reward;
			if (file.fail())
			{
				cout << "Error (agent): saving trajectory" << endl;
				exit(EXIT_FAILURE);
			}
			file << "\t" << trajectory->stage[i].TDerror << endl;
			
		}
		file << "Stage " << trajectory->length-1 << ": ";
		
		file << "\t" << trajectory->stage[trajectory->length-1].state << endl;
		if (file.fail())
		{
			cout << "Error (agent):  saving trajectory" << endl;
			exit(EXIT_FAILURE);
		}
		
		
		file << "Return=" << Return << endl;
		delete trajectory;
	}
	
	return steps;
}

int Agent::initStepWiseTrial(int N, bool learning, bool SaveTrajectory, const State* s , char* fileName , bool ComputeBellmanError )
{
	
	stepsInTrial = 0;
		
	Return=0.0;
	if (SaveTrajectory==true)
		trajectory = new Trajectory(1000);
		
	if (s==NULL)
	{
		env->startState(CurrentState, terminal);
	}
	else
	{
		CurrentState=*s;
		env->setState(*s, terminal);
		
	}
	//trajectoryFile = new String(fileName);
		
	file.open(fileName, ios::app);
		
    if (file.fail())
	{ 
		cout << "Error (agent): cannot open file to save trajectory" << fileName << endl;
		exit(EXIT_FAILURE);
	}
	
	file << "New trial" << endl;

	
}


int Agent::stepTrial(bool learning, bool SaveTrajectory, bool ComputeBellmanError )
{
	
	int i, j;
	int steps=0;
	
	
	if (learning==true)
	{
		steps = stepActAndLearn(1, SaveTrajectory);
	}
	else
	{
		steps=act(1, SaveTrajectory, ComputeBellmanError);
	}
	
	
	//IMPLEMENT HERE SAVING TRAJECTORY TO A FILE
	if (SaveTrajectory==true && stepsInTrial>1)
	{
//		for (i=0; i<trajectory->length-1; i++)
		{
			cout<< "Stage " << stepsInTrial << ": ";
			cout << "\t" << trajectory->stage[stepsInTrial-1].state;

		//	cout << "\t(";
		//	for (j=0; j<actions.size; j++)
		//		cout <<trajectory->stage[stepsInTrial-1].Qvalue[j] << ",";
		//	cout << ")";
			/*
			
			file << "Stage " << stepsInTrial << ": ";
			file << "\t" << trajectory->stage[stepsInTrial-1].state;
			if (file.fail())
			{
				cout << "Error (agent): saving trajectory" << endl;
				exit(EXIT_FAILURE);
			} 
			
			if (learning)
			{
				file << "\t(";
				for (j=0; j<actions.size; j++)
					file <<trajectory->stage[stepsInTrial-1].Qvalue[j] << ",";
				file << ")";
			}
			file << "\t" << trajectory->stage[stepsInTrial-1].action;
			if (file.fail())
			{
				cout << "Error (agent): saving trajectory" << endl;
				exit(EXIT_FAILURE);
			}
			
			file << "\t" << trajectory->stage[stepsInTrial-1].reward;
			if (file.fail())
			{
				cout << "Error (agent): saving trajectory" << endl;
				exit(EXIT_FAILURE);
			}
			file << "\t" << trajectory->stage[stepsInTrial-1].TDerror << endl;
			*/
		}
	
		/*
		file << "Stage " << trajectory->length-1 << ": ";
		
		file << "\t" << trajectory->stage[trajectory->length-1].state << endl;
		if (file.fail())
		{
			cout << "Error (agent):  saving trajectory" << endl;
			exit(EXIT_FAILURE);
		}
		
		
		file << "Return=" << Return << endl;
		 */
		//delete trajectory;
		 
		 
	}
	stepsInTrial++;
	return stepsInTrial;
	
}


void Agent::endStepTrial()
{
	delete trajectory;
	file.close();
}


////

double Agent::getReturn()
{
  return Return;
}

////

double Agent::getBellmanError()
{
  return BellmanError;
}

////

Agent::~Agent(){
  delete [] ApplicableActions;
}
  
//////////

void Agent::setArchitectureParameters(const Action& a, int argc, char *argv[])
 
{
  fa->setArchitectureParameters(a, argc, argv);
}

void Agent::saveArchitectureParameters(const Action& a, int argc, char *argv[])
  
{
  fa->saveArchitectureParameters(a, argc, argv);
}
