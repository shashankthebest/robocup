

#include "cmac.h" // definition of CMAC class and functions 

#include "string.h"
#include <unistd.h>

#define RUNS 1

#define MAX_TRIALS 1000

#define VALUE_PLOT_STEP_SIZE 100 // output value function once in N trials

#define Q0 0 // for initializing weights to Q0/TILES 

#define GRID_RES 50

double X_STEP = (X_RANGE[1] - X_RANGE[0])/GRID_RES; 

double Y_STEP = (Y_RANGE[1] - Y_RANGE[0])/GRID_RES; 

double T_STEP = (T_RANGE[1] - T_RANGE[0])/GRID_RES; 

double S_STEP = (S_RANGE[1] - S_RANGE[0])/GRID_RES; 


double weight[X_BINS+1][Y_BINS+1][T_BINS+1][S_BINS+1][TILES][ACTIONS];  // qvalue representation over tiles 

double eligibility[X_BINS+1][Y_BINS+1][T_BINS+1][S_BINS+1][TILES][ACTIONS]; // eligibility of a tile 

static int trial_data[RUNS][MAX_TRIALS]; // keep track of solution time for each trial and run 

// main routine for running trials

void run_trials();  

void output_trial_std_dev_data(); 


// define Q-learning with CMAC as a derived class of cmac

class qlCmac : public cmac 
{   // inherit from cmac and LearnLineUp

public: 
	
		qlCmac(double x, double y , double theta); // constructor initializes state and cmac tiles 
		
		ACTION choose_action(); // choose highest Q-value action, but explore sometimes 
		
		void initialize_weights_eligibilities(); 
		
		void update_eligibilities(ACTION a); // mark all active tiles for a particular action 
		
		double qvalue(ACTION a); //  qvalue  computed as weighted sum over active tiles 
		
		double qvalue(ACTION a, double x, double y, double theta, double speed ); // at any desired point
		
		double best_qvalue(double x, double y, double theta, double speed); 
		
		void generate_qvalue_plot(int run, int trial); 
		
		// Given previously active tiles and newly active tiles, update Q values 
		void update_weights(double reward, double old_qval, double new_qval); 
		
	private:  // data structures for Q-learning
	
		double exploration_rate; 
		
		double GAMMA;  // discount factor 
		double BETA;   // learning rate  
		double LAMBDA; // recency parameter
		
}; 


// constructor function calls cmac and mcar constructor functions 
qlCmac::qlCmac(double x, double y , double theta) : cmac(x,y,theta) 
{
// initialize Q-values  to 0

  for (int xbin=0; xbin <= X_BINS; xbin++)
    for (int ybin=0; ybin <= Y_BINS; ybin++)
		for (int tbin=0; tbin <= T_BINS; tbin++)
			for (int sbin=0; sbin <= S_BINS; sbin++)
				for (int tile=0; tile<TILES; tile++)
					for (int act=0; act<ACTIONS; act++)
					{
						weight[xbin][ybin][tbin][sbin][tile][act] = Q0/TILES; 
						eligibility[xbin][ybin][tbin][sbin][tile][act] = 0.0; 
					} 
									
				
GAMMA = 1.0; // discount factor

BETA = 0.5;  // learning rate 

LAMBDA = 0.9;  // recency parameter 

exploration_rate = 0.05;  // percentage of randomness

}

// initialize weight and eligibilities

void qlCmac::initialize_weights_eligibilities()
{
  for (int xbin=0; xbin <= X_BINS; xbin++)
    for (int ybin=0; ybin <= Y_BINS; ybin++)
		for (int tbin=0; tbin <= T_BINS; tbin++)
			for (int sbin=0; sbin <= S_BINS; sbin++)
				for (int tile=0; tile<TILES; tile++)
					for (int act=0; act<ACTIONS; act++)
					{
						weight[xbin][ybin][tbin][sbin][tile][act] = Q0/TILES; 
						eligibility[xbin][ybin][tbin][sbin][tile][act] = 0.0; 
					}
}		

// compute Q value of current state as weighted sum over active tiles 
double qlCmac::qvalue(ACTION a)
{
	double value = 0; 
	
	for (int tile=0; tile<TILES; tile++)
		value += weight[atiles[tile].x_bin][atiles[tile].y_bin][atiles[tile].t_bin][atiles[tile].s_bin][tile][a]; 
		
	return(value);
 	
}

// compute qvalue at any desired state
double qlCmac::qvalue(ACTION a,  double x, double y, double theta, double speed)
{
	double value =0; 
	
	// compute active tiles 
  for (int tile =0; tile<TILES; tile++)
    {
      int x_tile = (x - X_RANGE[0] + offset[0][tile])/x_interval; // add offset!!
      int y_tile = (y - Y_RANGE[0] + offset[1][tile])/y_interval; 
      int t_tile = (y - T_RANGE[0] + offset[2][tile])/t_interval;
      int s_tile = (y - S_RANGE[0] + offset[3][tile])/s_interval;
      
      value += weight[x_tile][y_tile][t_tile][s_tile][tile][a];
    }
    
    return(value); 
    
}

double qlCmac::best_qvalue(double x, double y, double theta, double speed)
{

	double bvalue = qvalue(incTrans, x,  y,  theta,  speed); 
	double tempVal;
	for (int a=0; a<ACTIONS; a++)
	{
	  tempVal = qvalue(int_to_act(a), x,  y,  theta,  speed);
      if ( tempVal > bvalue)
		bvalue = tempVal; 
	}
	return(bvalue);
}

// given a state bin, pick the highest Q-value action with high probability 

ACTION qlCmac::choose_action()
{
  double rvalue; 
  int bact = choose_random_int_value(2); 
  
  rvalue = choose_random_value(); 
	
  if (rvalue < exploration_rate)  // do a random action 
    return(choose_random_act()); 
  else 
    for (int a=0; a<ACTIONS; a++)
      if (qvalue(int_to_act(a)) > qvalue(int_to_act(bact)))
		bact = a; 

  return(int_to_act(bact)); 
}

// update eligibilities 
void qlCmac::update_eligibilities(ACTION a)
{
  for (int xbin=0; xbin <= X_BINS; xbin++)
    for (int ybin=0; ybin <= Y_BINS; ybin++)
		for (int tbin=0; tbin <= T_BINS; tbin++)
			for (int sbin=0; sbin <= S_BINS; sbin++)
				for (int tile=0; tile<TILES; tile++)
					for (int act=0; act<ACTIONS; act++)
					eligibility[xbin][ybin][tbin][sbin][tile][act] *= LAMBDA; // decay eligibilites
						
					
	for (int tile=0; tile<TILES; tile++)
		for (int act = 0; act<ACTIONS; act++)
			if (act == a)
				eligibility[atiles[tile].x_bin][atiles[tile].y_bin][atiles[tile].t_bin][atiles[tile].s_bin][tile][act] = 1;
			else 
				eligibility[atiles[tile].x_bin][atiles[tile].y_bin][atiles[tile].t_bin][atiles[tile].s_bin][tile][act] = 0; 
		
}

// update weights 

void qlCmac::update_weights(double reward, double old_qval, double new_qval)
{

// TD(lambda) sarsa update rule 

  for (int xbin=0; xbin <= X_BINS; xbin++)
    for (int ybin=0; ybin <= Y_BINS; ybin++)
		for (int tbin=0; tbin <= T_BINS; tbin++)
			for (int sbin=0; sbin <= S_BINS; sbin++)
				for (int tile=0; tile<TILES; tile++)
					for (int act=0; act<ACTIONS; act++)
					weight[xbin][ybin][tbin][sbin][tile][act] +=  
		 			 (BETA/TILES)*(reward + new_qval - old_qval) * eligibility[xbin][ybin][tbin][sbin][tile][act]; 
}


// collect statistics on each trial over runs 

void output_trial_std_dev_data()
{

  ofstream out("mcar-sarsa-soln-data");  

  double sum,sum_sq,std_dev,mean;
  
  for (int j=0; j<MAX_TRIALS; j++)
      {
   	   sum=0.0;
   	   sum_sq=0.0;

	   for (int i=0; i<RUNS; i++)
	   {
	     sum += trial_data[i][j];
	     sum_sq += trial_data[i][j]*trial_data[i][j];
	    }

	   mean = sum/RUNS;

//	   std_dev = sqrt((sum_sq - (sum*sum/RUNS))/(RUNS-1));
	   
	   out <<  j <<  " " << mean << endl; 

//	     <<  " " << mean - std_dev << " " << mean + std_dev << endl; 
     }

  out.close(); 
} 


void qlCmac::generate_qvalue_plot(int run, int trial)
{
  char name[20];
	
  sprintf(name,"qvalue-%d-%d",run, trial);
	
  ofstream output(name); 
	
  for (int relX=0; relX<= GRID_RES; relX++)
		{
		  double xvalue = X_RANGE[0] + X_STEP*relX; 
//  + POS_STEP/2; // midpoint of bin
			
		  output << endl; 
		
		  for (int relY=0; relY <= GRID_RES; relY++)
		    {
		      double yvalue = Y_RANGE[0] + Y_STEP*relY;  
// + VEL_STEP/2; // midpoint of bin
			for (int t=0; t <= GRID_RES; t++)
		    {
		      double tvalue = T_RANGE[0] + T_STEP*t;  
			
				for (int s=0; s <= GRID_RES; s++)
					{
					double svalue = S_RANGE[0] + S_STEP*s;			
					
					double value = best_qvalue(xvalue,yvalue,tvalue,svalue); 
							
					if (value < 0) value = -value; 
					     output << value << "\t"; 
					}
						
				}
			}
		}
  output.close(); 
	
}	


void run_trials(double theta)  // theta is to be provided by supervisor, this is target theta 
{
  int count = 0; 
  char data[30]; // data string to print out
  
  vector<double> Startpos;
  Startpos.push_back(50);
  Startpos.push_back(0);
  Startpos.push_back(theta);
  vector<double> Startvel(3,0);
  
  qlCmac mcq(50,0,theta); // initialize state to starting position

  for (int run = 0; run<RUNS; run++)
    {

      int best_changed = 1; 
      int best_so_far=10000;  // shortest distance to goal so far 

      mcq.initialize_weights_eligibilities(); 
      // initialize weights and eligibilities to 0. 



     if (run > 0)  // on subsequent runs, keep the same CMAC tiling 
	 {
		
	   mcq.set_curr_pos(Startpos);  
	   
	   mcq.set_curr_vel(Startvel); 

	 }
	
      for (int trial=0; trial< MAX_TRIALS; trial++)
	{

	  if ((trial+1)%VALUE_PLOT_STEP_SIZE==0)
	    mcq.generate_qvalue_plot(run, trial+1); 
       
	  best_changed = 1; 

	  double old_qvalue, new_qvalue;  // for TD(lambda) sarsa update rule 
     
	  int done  = 0, i = 0; 
	  double r; 
	  ACTION a, na; 

	  mcq.active_tiles();  // recompute set of active CMAC tiles 
		
	  a = mcq.choose_action(); // pick highest Q-value action with high probability 
	  
	  old_qvalue = mcq.qvalue(a); // compute q value as weighted sum over active tiles 
	
	  while (!done) // not yet reached goal
		{
		
		  i++; 

		  sprintf(data, "Run: %d Trial: %d Shortest solution: %d", run, trial, best_so_far); 

		  if (best_changed)
		    {
		     
		      best_changed=0;
		    }
				 
		  mcq.update_eligibilities(a); // decay and update eligibilities 
		 
		  mcq.update_position_velocity(a);  // move the car 
	 	 
		  if (!mcq.reached_goal())
		    {
	 	 
		      mcq.active_tiles();  // recompute set of active tiles 
	 		 
		      na = mcq.choose_action();  // choose highest Q-value action in new state
		 	 
		      new_qvalue = mcq.qvalue(na);
		 	 
		      r = mcq.reward();  // -1 always 

		      mcq.update_weights(r,old_qvalue, new_qvalue);  // TD(0) sarsa update 
	  
// RECOMPUTE QVALUE AFTER WEIGHTS CHANGED! (rich's correction)
		      old_qvalue = mcq.qvalue(na); 

		      a = na; 
		    }
		  else 
		    {
		      // TD(lambda) sarsa update at goal
		      mcq.update_weights(-1,old_qvalue, 0);  

		      if (i<best_so_far) 
			{
			  best_so_far = i; 
			  best_changed = 1; 
			}
		      trial_data[run][trial] = i; // record number of steps 
		      done=1;
		    }
		}
		
	   mcq.set_curr_pos(Startpos);  
	   
	   mcq.set_curr_vel(Startvel); 
	
	}
    }
	
}


// do a bunch of learning runs

int main(void)  
{

  int rinit = -time(NULL)%100; 
  int rseed = -1000 + rinit*rinit*rinit; 

 // WINDOW = start_display(); 

  //WINDOW2 = start_display2(); 

  initialize_random_number_generator(rseed); // set up large negative number as random seed; 
	
  cout << "Learning run started at " << __TIME__ << " with seed " << rseed << endl; 

  run_trials(); 

  output_trial_std_dev_data(); 

}
			
		
		
				

