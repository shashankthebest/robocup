#include "LearnLineUp.h"   

// choose a random starting position and velocity for the car

LearnLineUp::LearnLineUp(void)
{
	double trand; // random values for final desired theta (initial theta is always 0) 
	
	trand = choose_random_value();  
	
	x = 50; //relative position from the ball in x
	y = 0;  //relative position from the ball in y
	theta = 0; // always set to zero (the experiment is designed this way)
	s = 0;
	
	targetX = 5;
	targetY = 5;
	targetTheta = 0;
	
	
//	cout << "Starting car at " << p << "  with velocity " << v << endl; 
	
}

// second constructor function where pos and vel are given 

LearnLineUp::LearnLineUp(double robotPosX, double robotPosY, double robotPosTheta)
{
	x = robotPosX;
	y = robotPosY;
	theta = robotPosTheta; 
	transSpeed = 0;
	rotSpeed = 0;
	direction = 0;
	
}

// reward function for mountain car problem 

double LearnLineUp::reward()
{
	 if (reached_goal())
		return(0);  
	else 
		return(-1); 
}

// generate random starting position 

double random_targetTheta()
{
	double  trand = choose_random_value();  
	
	return((T_RANGE[1] - T_RANGE[0]) * trand + T_RANGE[0]); // scale position into legal range 
}


double random_pos()
{
//	double  prand = choose_random_value();  
	
//	return((POS_RANGE[1] - POS_RANGE[0]) * prand + POS_RANGE[0]); // scale position into legal range 
}

// generate random starting velocity 
double random_vel()
{
//	double vrand = choose_random_value();  
	
//	return((VEL_RANGE[1] - VEL_RANGE[0]) * vrand + VEL_RANGE[0]); // scale velocity into legal range
}

// overload the << operator for the enum data type

ostream& operator << (ostream& out, ACTION a)
{
	
	if (a == incTrans)
	 out << " Increase Translation Velocity"; 
	else if (a == decTrans)
	 out << " Decrease Translation Velocity"; 
	else if (a == incRot)
	 out << "Increase Rotational Velocity "; 
	 else if(a == decRot)
		out<< "Decrease Rotational Velocity";
	else if (a== incDir)
		out<< "Increase Direction";
	else if (a== decDir)
		out<< "Decrease Direction";		
	return(out); 
	
}
	
ACTION LearnLineUp::int_to_act(int value)
{
	ACTION act; 
	
	if (value == 0)
		act = incTrans; 
	else if (value == 1)
		act = decTrans; 
	else if (value == 2)
		act = incDir; 	
	else if (value == 3)
		act = decDir; 			
	else if (value == 4)
		act = incRot; 	
	else if (value == 5)
		act = decRot; 			
	
	return(act); 
}

// for now, choose a random action 

ACTION LearnLineUp::choose_random_act()
{
	int rvalue = choose_random_int_value(5); // return a value between 0 and 2
	
	return(int_to_act(rvalue)); 
}


// update velocity and position  -- range is clipped if it is out of bounds 


void LearnLineUp::update_position_velocity(ACTION a)
{
	
	// read current position from NUBlackboard
	/*
	double oldv = v; // preserve old values
	double oldp = p; 
	
	double newv, newp;  // new values of velocity and position
	
	int aval;
	
	if (a == backward)
	 aval = -1; 
	else aval = (int) a;  // coast = 0, forward = +1, backward = -1; 
	
	newv = oldv + (0.001 * aval) + (GRAVITY * cos(3 * oldp)); // update equation for velocity 
	
	if (newv < VEL_RANGE[0])  // clip velocity if necessary to keep it within range
	  {
//	    cout << "clipping velocity to " << VEL_RANGE[0] << endl; 
		newv = VEL_RANGE[0]; 
	  }
	else if (newv > VEL_RANGE[1])
	  {
//	    cout << "clipping velocity to " << VEL_RANGE[1] << endl; 
	    newv = VEL_RANGE[1];

	  }
			
	newp = p + newv;  // update equation for position 
	
	if (newp < POS_RANGE[0])  // clip position and velocity if necessary to keep it within range
		{
			newp = POS_RANGE[0]; 
			newv = 0;  // reduce velocity to 0 if position was out of bounds 
		}
	else if (newv > POS_RANGE[1])
		{
			newp = POS_RANGE[1]; 
			newv = 0;  
		}
		
	p = newp; 
	v = newv;   // update state to new values 
	* 
	* 
	* */
	
}
	


int LearnLineUp::reached_goal(void)
{
	if ((fabs(x-targetX) < 2) && (fabs(y-targetY) < 2) && (fabs(theta-targetTheta) < 0.01) &&
		(fabs(transSpeed)<0.2) && (fabs(rotSpeed)<0.2) )
			return(1); 
	else
			return(0); 
	
}


// overload the << operator for printing the current position and velocity 

ostream& operator << (ostream& out, LearnLineUp lu)
{
//	out << "Position: " << mc.p << "   Velocity:   " << mc.v << "  "; 
	
	return(out); 
	
}

