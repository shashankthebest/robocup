#include<stdlib.h>
#include<math.h>
#include<time.h>

#include "Tools/RL/rlIncludes.h"
#include "agent.h"


Agent::Agent(double g, const ActionSet& a_s, StateActionFA* const f, Environment* const e)
  : actions(a_s), fa(f), env(e), gamma(g), trajectory(NULL)
{
  ApplicableActions = new int[Action::count];
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
	
	else{
		CurrentState=*s;
		env->setState(*s, terminal);
	}
	
	if (learning==true)
		steps=actAndLearn(N, SaveTrajectory);
	else
		steps=act(N, SaveTrajectory, ComputeBellmanError);
	
	
	//IMPLEMENT HERE SAVING TRAJECTORY TO A FILE
	if (SaveTrajectory==true){
		ofstream file(fileName, ios::app);
		if (file.fail()){
			cout << "Error (agent): cannot open file to save trajectory" << fileName << endl;
			exit(EXIT_FAILURE);
		}
		file << "New trial" << endl;
		for (i=0; i<trajectory->length-1; i++){
			
			file << "Stage " << i << ": ";
			file << "\t" << trajectory->stage[i].state;
			if (file.fail()){
				cout << "Error (agent): saving trajectory" << endl;
				exit(EXIT_FAILURE);
			}
			if (learning==true){
				file << "\t(";
				for (j=0; j<actions.size; j++)
					file <<trajectory->stage[i].Qvalue[j] << ",";
				file << ")";
			}
			file << "\t" << trajectory->stage[i].action;
			if (file.fail()){
				cout << "Error (agent): saving trajectory" << endl;
				exit(EXIT_FAILURE);
			}
			
			file << "\t" << trajectory->stage[i].reward;
			if (file.fail()){
				cout << "Error (agent): saving trajectory" << endl;
				exit(EXIT_FAILURE);
			}
			file << "\t" << trajectory->stage[i].TDerror << endl;
			
		}
		file << "Stage " << trajectory->length-1 << ": ";
		
		file << "\t" << trajectory->stage[trajectory->length-1].state << endl;
		if (file.fail()){
			cout << "Error (agent):  saving trajectory" << endl;
			exit(EXIT_FAILURE);
		}
		
		
		file << "Return=" << Return << endl;
		delete trajectory;
	}
	
	return steps;
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
