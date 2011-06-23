
#include <stdlib.h>
#include <string.h>
#include <stddef.h>

#include "states.h"
#include "actions.h"
#include "Tools/RL/Environment/environment.h"

//IMPLENENTATION of the State class


int State::dimensionality=0;
bool Environment::seeded = false;
long Environment::idum=0;


State::State(){
/*	Default constructor.
	Creates array of variables.
*/
	x=new double[dimensionality];
}

void State::operator = (const State& s){
	int i;
	delete [] x;
	x=new double[dimensionality];
	for(i=0; i<dimensionality; i++)
		x[i] = s.x[i];
	
}

ostream& operator << (ostream& file, const State& s){
	int i;

	for(i=0; i<State::dimensionality; i++){
		file << "x[" << i << "]=" << s.x[i] << " ";
		if (file.fail()){
			cout << "Error: state output " << endl;
			exit(EXIT_FAILURE);
		}
	}

	return file;
}

ostream& operator << (ostream& file, const Action& a){
	
	file << "value = " << a.value;
	if (file.fail()){
			cout << "Error: action output " << endl;
			exit(EXIT_FAILURE);
		}

	file << " ( " << a.description << " ) ";
	if (file.fail()){
			cout << "Error: action output " << endl;
			exit(EXIT_FAILURE);
		}

	return file;
}

State::State(State& s){
	int i;

	x=new double[dimensionality];
	for(i=0; i<dimensionality; i++)
		x[i] = s.x[i];
}

State::~State(){
	delete [] x;
}

State::State(int n)
/*	Construcs a "super"-state that combines n states with current dimensionality
*/
{
	x = new double[dimensionality*n];
}







////////////////////////////////////

Subset::Subset(){
	left = new double[State::dimensionality];
	right = new double[State::dimensionality];
	
}

Subset::Subset(int N){
  left = new double[State::dimensionality*N];
  right = new double[State::dimensionality*N];
}

Subset::~Subset(){
	delete [] left;
	delete [] right;
}














//IMPLEMENTATION OF THE Action CLASS

int Action::count = 0;

Action::Action(const char* d) {	
/*	Conversion constructor.
	Parameters: 
		d : description of an action
*/
	description= new char[strlen(d)+1];
	strcpy(description, d);
	id=-1;
	count++;
	value=0.0;
}

Action::Action(const char* d, double v){
/* General constructor.
	Parameters:
		d : description of an action
		v : numerical value of an action
*/
	description= new char[strlen(d)+1];
	strcpy(description, d);
	id=-1;
	count++;
	value=v;
}

Action::Action(){
/* Default constructor.
*/
	description = new char[1] ;
	id=-1;
	value=0.0;
}

Action::Action(Action& a){
/*	Copy constructor.
*/
	id = a.id;
	value = a.value;
	description = new char[strlen(a.description)+1];
	strcpy(description, a.description);
}

void Action::operator = (const Action& a){
/*	Assignment operator overloaded to 
	accomodate copying of data in heap memory
*/
	id = a.id;
	value = a.value;
	delete [] description;
	description = new char[strlen(a.description)+1];
	strcpy(description, a.description);
}

Action::~Action(){
	delete [] description;
}
















//IMPLEMENTATION OF THE ActionSet CLASS

ActionSet::ActionSet()
{
	size=0;
	action=NULL;
	added=0;
}

ActionSet::ActionSet(int n){ 
		/*	General constructor.
			Parameters:
				n : size of the action set
		*/
	size=n;
	action=new Action[size];
	added=0;
}

void ActionSet::create(int n){
	size=n;
	action = new Action[size];
	added=0;
}


void ActionSet::addAction(Action& a){ 
		/*	Add action to the action set.
			Parameters:
				a : action to be added.
		*/
		if (added>=size){
			cout << "Error: attempt to add overflow action set" << endl;
			exit(EXIT_FAILURE);
		}
		a.id=added;
		action[added] = a;
		added++;
}

void ActionSet::operator = (const ActionSet& as)
{
	cout<<"\nI am here  : "<<__FILE__<<"   at "<<__LINE__<<"  ";
	size=as.size;
	added = as.added;
	if(action!=NULL)
	delete [] action;
	action = new Action[size];
	int i;
	for (i=0; i<size; i++)
		action[i] = as.action[i];
}

void ActionSet::printActions()
{
	cout<<"\n\n";
	for (int i=0; i<size; i++)
	{
		cout<<"\t "<<action[i].value<<", ";
	}
}


ActionSet::ActionSet(const ActionSet& as)
{
	
	size = as.size;
	added = as.added;
	if(!action)
	 delete [] action;
	cout<<"\nI am here  : "<<__FILE__<<"   at "<<__LINE__<<"  ";
	action = new Action[size];
	int i;
	for (i=0; i<size; i++)
		action[i] = as.action[i];
}

ActionSet::~ActionSet()
{
	delete [] action;
}


