
#include <stdlib.h>
#include <string.h>

#include "stateActionFA.h"

//	IMPLEMENTATION OF StateActionFA

StateActionFA::StateActionFA()
	:A(0), fa(NULL)
{}


StateActionFA::StateActionFA(int n, Approximator** f)
		/*	General constructor.
			Parameters:
				n : number of actions (architectures)
				f : pointer to the array of pointers to approximator objects
		*/
		:A(n),fa(f){}
	
StateActionFA::	~StateActionFA(){
	int i;
	
	for(i=0; i<A; i++){
	
		delete fa[i];
	}
	delete [] fa;
	
}

int StateActionFA::getSize(){
  int i;
  for (i=0;i<A;i++){
    if (fa[i]!=NULL) return fa[i]->getSize();
  }
  return 0;
}

void StateActionFA::getMaxParameterChange(double* changes){
	/* Returns an array of MaxParameterChanges for all component architectures */

  int i;

  for (i=0; i<A; i++){
    if (fa[i]!=NULL)
      changes[i]=fa[i]->getMaxParameterChange();
    else
      changes[i]=0;
  }

}
void StateActionFA::getNumberParametersChanged(int* changes){
	/* Returns an array of the number of changed parameters for each component architecture
	 */
    int i;

  for (i=0; i<A; i++){
    if (fa[i]!=NULL)
      changes[i]=fa[i]->getNumberParametersChanged();
    else
      changes[i]=0;
  }

}

void StateActionFA::predict(const Action& a, const State& s,  double& output)
		/*	Predicts an output value with an approximator 
			corresponding a given action.
			Parameters:
				a : reference to an action
				s : reference to the input (state)
				output : returned value of the predicted output
		*/
{
	
	cout<<"\n\nI am here  : "<<__FILE__<<"   at "<<__LINE__<<" \n\n ";
  if (fa[a.id]==NULL)
  {
    cout << "\n\nError (safa): attempt to use non-existent Approximator object" << endl;
    exit(EXIT_FAILURE);
  }
  fa[a.id]->predict(s, output); 
	  
	  //fa[a.id] is a base pointer to a derived object: dinamic binding
}
void StateActionFA::learn(const Action& a, const State& s, double target)
		/*	Learns an input-output pair with an approximator 
			corresponding a given action.
			Parameters:
				a : reference to an action
				s : reference to the input (state)
				target : target output value
		*/
{
  if (fa[a.id]==NULL){
    cout << "Error (safa): attempt to use non-existent Approximator object" << endl;
    exit(EXIT_FAILURE);
  }
  fa[a.id]->learn(s, target);
}

void StateActionFA::computeGradient(const Action& a, const State& s, double* GradientVector)
/*	Compute the gradient w.r.t. architecture parameters at the current parameters' values and input s
*/
{
  if (fa[a.id]==NULL){
    cout << "Error (safa): attempt to use non-existent Approximator object" << endl;
    exit(EXIT_FAILURE);
  }
  fa[a.id]->computeGradient(s, GradientVector);
}
void StateActionFA::updateParameters(const Action& a, double* delta)
/*	Update parameters by amounts in delta array
*/
{
  if (fa[a.id]==NULL){
    cout << "Error (safa): attempt to use non-existent Approximator object" << endl;
    exit(EXIT_FAILURE);
  }
  fa[a.id]->updateParameters(delta);
}

void StateActionFA::clearTraces(const Action& a, const State& s, double replace)
	/*	Replaces traces for those actions that were not taken in state s.
		Parameters:
			a : action for traces should NOT be replaced
			s : input (state)
			replace : value to which traces should be replaced (usually zero)
	*/
{
	int i;

	for(i=0; i<A; i++)
	  if (fa[i]!=NULL){
	    if (i != a.id)
	      fa[i]->replaceTraces(s, replace);
	  }
}

void StateActionFA::decayTraces(double factor){
	int i;

	for(i=0; i<A; i++){
	  if (fa[i]!=NULL){
		fa[i]->decayTraces(factor);
	  }
	}
}

void StateActionFA::accumulateTraces(const Action& a, const State& s, double amount){
  if (fa[a.id]==NULL){
    cout << "Error (safa): attempt to use non-existent Approximator object" << endl;
    exit(EXIT_FAILURE);
  }
  fa[a.id]->accumulateTraces(s, amount);
}

void StateActionFA::replaceTraces(const Action& a, const State& s, double trace){
  if (fa[a.id]==NULL){
    cout << "Error (safa): attempt to use non-existent Approximator object" << endl;
    exit(EXIT_FAILURE);
  }
  fa[a.id]->replaceTraces(s, trace);
}

void StateActionFA::setArchitectureParameters(const Action& a, int argc, char *argv[])
		/*	Loads parameters of the architecture corresponding 
			to a given action.
			Parameters:
				a : action
				argc : number of supplied arguments
				argv : array of arguments
		*/
{ 
  if (fa[a.id]==NULL){
    cout << "Error (safa): attempt to use non-existent Approximator object" << endl;
    exit(EXIT_FAILURE);
  }
  fa[a.id]->setArchitectureParameters(argc, argv);
}

void StateActionFA::saveAllArchitectureParameters(char** fileNames){

  char** filePar = new char*[1];
  filePar[0]=new char[100];
  int i;

  for (i=0; i<A; i++){
    if (fa[i]!=NULL){
      strcpy(filePar[0],fileNames[i]);
      fa[i]->saveArchitectureParameters(1,filePar);
    }
  }

  delete [] filePar[0];
  delete [] filePar;
}
	
void StateActionFA::saveArchitectureParameters(const Action& a, int argc, char *argv[])
		/*	Saves parameters of the architecture corresponding 
			to a given action.
			Parameters:
				a : action
				argc : number of supplied arguments
				argv : array of arguments
		*/
{  
  if (fa[a.id]==NULL){
    cout << "Error (safa): attempt to use non-existent Approximator object" << endl;
    exit(EXIT_FAILURE);
  }
  fa[a.id]->saveArchitectureParameters(argc, argv);
}
void StateActionFA::setLearningParameters(const Action& a, int argc, char *argv[])
		/*	Sets learning parametersof the architecture corresponding 
			to a given action.
			Parameters:
				a : action
				argc : number of supplied arguments
				argv : array of arguments
		*/
{ 
  if (fa[a.id]==NULL){
    cout << "Error (safa): attempt to use non-existent Approximator object" << endl;
    exit(EXIT_FAILURE);
  }
  fa[a.id]->setLearningParameters(argc, argv);
}

void StateActionFA::setAllLearningParameters(int argc, char* argv[])
{
  for(int i=0; i<Action::count; i++){
    if (fa[i]!=NULL)
      fa[i]->setLearningParameters(argc, argv);
  }
}
