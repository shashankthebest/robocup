#ifndef ACTIONREPRESENTATION_H
#define ACTIONREPRESENTATION_H

#include "Tools/RL/Misc/globals.h"

struct Action{//implementation in sarepr.cpp
	/* For descrete actions
	 */
	
	int id; /* by default is assigned the ordinal number 
			 as the actions are added to some action set.
			 In other words its value coincides with the action's array 
			 index in the action set to which it belongs 
			 (see ActionSet declaration below).
			 */
	
	char* description;//may be given a "name"
	double value;	//numerical value of the action
	static int count; //total number of Action objects created
	
	Action();
	/* Default constructor.
	 */
	
	Action(const char* d);
	/* General constructor.
     Parameters: 
     d : description of an action
	 */
	
	Action(const char* d, double v);
	/* General constructor.
     Parameters:
     d : description of an action
     v : numerical value of an action
	 */
	
	void operator = (const Action& b);
	
	Action(Action& a);
	
	~Action();
};

ostream& operator << (ostream& file, const Action& a);
/* Overloaded operator for action output to a file or cout.
 */

///////////////////////////////////////////////////////////////////////

struct ActionSet{//implementation in sarepr.cpp
	/* Groups together all actions for an RL system.
	 */
	
	int size;//number of actions in the set
	Action* action;//array of actions that belong to this set	
	int added;//indicates how many actions have already been added to the set
	
	ActionSet();
	
	ActionSet(int n);
	/* General constructor.
     n : size of the action set
	 */
	
	void create(int n);//as a constructor, can be called after object is created with default constructor
	
	void addAction(Action& a);
	/* Add action to the action set.
     a : action to be added.
	 */
	
	void operator = (const ActionSet& a);
	
	~ActionSet();
};


#endif
