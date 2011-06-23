#ifndef AGENTBASE_H
#define AGENTBASE_H

#include "Tools/RL/Misc/globals.h"
#include "Tools/RL/Environment/environment.h"

class Agent { //implementation of some non-virtual functions in agent.cpp
	
public:
	
	//Agent(double g, const ActionSet& a_s, StateActionFA* const f, Environment* const e);
	Agent(double g, ActionSet a_s, StateActionFA* const f, Environment* const e);
	/* Constructor.
     g : discount factor
     a_s : action set available to the agent
     f : pointer to the architectures containing either action-value functions 
     or random policy
     e : pointer to the environment
	 */
		int initTrial(int N, bool learning, bool SaveTrajectory, const State* s , char* fileName , bool ComputeBellmanError ); 
    /* Gets the start state from the environment
     and then calls appropriate act function to perform 
     the trial for a maximum of N steps)
     Argument "learning" indicates whether learning should take place.
     If yes, actAndLearn() function is called, otherwise
     act() function is called.
     Computes the return for this trial.
     Function returns the number of steps actually performed during the trial.
	 
     N : maximal number of steps in the trial
     learning : indicates whether learning should take place
     SaveTrajectory : indicates whether the trajectory should be saved
     fileName : name of the file to which the trajectory should be saved.
     ComputeBellmanError : indicates if estimated Bellman Error should be computed. Has a default false value.
	 */
	
	double getReturn();
	/* Gets return collected during the last trial
	 */
	
	double getBellmanError();
	/* Returns BellmanError for the last trajectory traversed without learning   */
	
	virtual void setLearningParameters(int argc, char *argv[])=0;
	/*	Sets parameters of the RL learning algorithm
	 */
	
	void setArchitectureParameters(const Action& a, int argc, char *argv[]);
	/* Sets parameters of the architecture (fa) representing
     value function or a policy distribution
     argc : number of arguments in argv array
     argv : array of arguments
     The two above arguments should be as they would be sent 
     to setArchitectureParameters() function of the derived approximator class.  
	 */
	
	void saveArchitectureParameters(const Action& a, int argc, char *argv[]);
	/* Seves parameters of the architecture (fa) representing
     value function or a policy distribution
     argc : number of arguments in argv array
     argv : array of arguments
     The two above arguments should be as they would be sent 
     to saveArchitectureParameters() function of the derived approximator class.  	
	 */
	
	virtual ~Agent();
	
protected:
	
	//component structures
	struct StageInfo; //fully defined at the end of this class' declaration
	struct Trajectory;//fully defined at the end of this class' declaration
	
	//Data members
	State CurrentState;//current state of the environment	
	Action CurrentAction;//action chosen in the current state
	bool terminal; //indicates if the state is terminal
	double CurrentReward;
	ActionSet actions; //const ActionSet& actions; //action set of the RL system
	StateActionFA* const fa; /* Pointer to an arcitecture representing
							  either policy probability distribution
							  or action-value functions.
							  */
	
	double gamma;	//discount factor
	double Return;//return collected during a trial
	Environment* const env;//pointer to the environment object
	int* ApplicableActions;//array to be used by chooseAction() function
	Trajectory* trajectory;
	double BellmanError;
	
	virtual int act(int N, bool SaveTrajectory, bool ComputeBellmanError)=0;
	
	/* Implements maximum of N successive steps of the trial
     or until a terminal state is entered by the environment
     Communicates with the environment to get current state and reward.
     Computes return collected on this trial.
	 
     N : maximal number of steps in the trial
     SaveTrajectory : indicates whether trajectory should be saved
     ComputeBellmanError : indicates whether (estimated) Bellman error 
     should be computed for the state action pairs on the trajectory.
	 
	 */
	
	virtual int actAndLearn(int N, bool SaveTrajectory)=0;	
	/* Implements maximum of N successive steps of the trial
     or until the terminal state is entered by the environment
     Communicates with the environment to get 
     current state and reward.
     Calls the rlAlgorithm when appropriate. 
     Computes return collected on this trial.
	 
     N : maximal number of steps in the trial;
     SaveTrajectory : indicates whether trajectory should be saved
	 */
	
	
	virtual void chooseAction(const State& s, Action& a) =0 ;
	/* Implements behavior policy
     Uses fa - representation of the random policy or the action value functions 
     s : state in which the action should be performed
     a : chosen action
	 */
	
	// component structures
	
	struct StageInfo{
		State state;
		Action action;
		double reward;
		
		double* Qvalue;
		double TDerror;
		
		StageInfo(){
			Qvalue = new double[Action::count];
		}
		
		~StageInfo(){
			delete [] Qvalue;
		}
		
	};
	
	struct Trajectory
	{
		StageInfo* stage;
		int length;			//actual length of the recorded trajectory 
		
		Trajectory(int n){
			// n is the maximal number of stages in the trjectory
			stage = new StageInfo[n];
			length=0;
		}
		
		~Trajectory(){
			delete [] stage;
		}
		
	};
	
};
#endif
