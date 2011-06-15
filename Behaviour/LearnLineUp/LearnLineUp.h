
#include <iostream>
#include <fstream>
#include <iomanip>
#include "krandom.h"  // link it with Knuth's random number generator
#include <math.h> 
#include <time.h>
#include <vector>

using namespace std;
	
// Defined for Linup	
static double X_RANGE[2] = {0,50}; 	
static double Y_RANGE[2] = {-25,25}; 	
static double T_RANGE[2] = {0,360}; 	
static double S_RANGE[2] = {-10.00,10.00};


// allowable actions
enum ACTION {incTrans, decTrans, incDir, decDir, incRot, decRot};


int run_trial(int trial); // function prototype 

ostream& operator << (ostream& out, ACTION a); 


// define the main class

class LearnLineUp {

	public: 
	
		LearnLineUp();  // constructor function defines a random starting point (robotPos,ballPos,targetTheta)
						// within the allowable ranges
		
		LearnLineUp(double robotPosX, double robotPosY, double robotPosTheta );  // second constructor function with specified  (robotPos,ballPos,targetTheta)
				 
				 
		friend double random_robotPos();  // generate random starting robot position 
		friend double random_targetTheta();
		
		friend double random_ballPos();  // generate random starting ball position
		
		double reward();  // return -1 if not lined up properly with 0 speed
		
		vector<double> curr_robotPos()
		{ 
			vector<double>retVal;
			retVal.push_back(x);
			retVal.push_back(y);
			retVal.push_back(theta);
			
			return(retVal); 
		};  // retrieve current position
		
		vector<double> curr_speed()  // retrieve current velocity
		{
			vector<double>retVal;
			retVal.push_back(transSpeed);
			retVal.push_back(direction);
			retVal.push_back(rotSpeed);
			return retVal;
		};   
		
		void set_curr_pos(vector<double> pos) // set position to pos
		{
			x = pos[0];
			y = pos[1];
			theta = pos[2];
		};  
		
		void set_curr_vel(vector<double> vel) // set velocity to vel
		{
			transSpeed = vel[0];
			direction = vel[1];
			rotSpeed = vel[2];
		};  
		
		ACTION int_to_act(int action);  // return named act 
		
		ACTION choose_random_act();  // choose an action 
		
		void  update_position_velocity(ACTION a); // update position and velocity 
		
		int reached_goal();  // 1 if reached, else 0
		
		friend ostream& operator << (ostream& out, LearnLineUp c); // display position and velocity 
		
		
	private: 
	
		double x,y,theta, s, transSpeed, rotSpeed, direction; // current state = relX, relY, globalTheta, transSpeed, rotSpeed
		double targetX, targetY, targetTheta;
}; 	

