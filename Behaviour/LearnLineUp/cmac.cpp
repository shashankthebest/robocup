#include "cmac.h"



// create the tiles for the CMAC
cmac::cmac(double x, double y , double theta) : LearnLineUp( x,  y ,  theta)
{
  // for each grid, compute an offset from center
	 
  cout << "setting up CMAC tiles..." << endl; 
	 
  x_interval = (X_RANGE[1] - X_RANGE[0])/X_BINS; 
  y_interval = (Y_RANGE[1] - Y_RANGE[0])/Y_BINS; 
  t_interval = (T_RANGE[1] - T_RANGE[0])/T_BINS; 
  s_interval = (S_RANGE[1] - S_RANGE[0])/S_BINS; 
	
  cout << "X interval is \t" << x_interval << endl; 
  cout << "Y interval is \t" << y_interval << endl; 
  cout << "Theta interval is \t" << t_interval << endl; 
  cout << "Speed interval is \t" << s_interval << endl; 
  
	
  for (int tile = 0; tile<TILES; tile++)
    {
      // generate random offset as a fraction of an interval 
			
//      offset[0][tile]  = (choose_random_value() * 2  * pos_interval) - pos_interval; 
//      offset[1][tile]  = (choose_random_value() * 2 * vel_interval) - vel_interval; 

// CMAC offsets are positive fractions of an interval (Rich's correction!)
      offset[0][tile]  = choose_random_value() * x_interval;  // offsets = pos fraction of int
      offset[1][tile]  = choose_random_value() * y_interval;
      offset[2][tile]  = choose_random_value() * t_interval;
      offset[3][tile]  = choose_random_value() * s_interval;
    }
		
  active_tiles();  // set up active tiles 
}

// display the offsets 

void cmac::display_cmac(void)
{

  for (int tile = 0; tile < TILES; tile++)
    {
      cout << endl; 
			
      cout << "Tile Number " << tile << " X offset " << offset[0][tile] << endl; 
      
      cout << "X intervals: " ;
      
      for (int pos=0; pos<=X_BINS; pos++)
      	cout << X_RANGE[0] - offset[0][tile] + pos*(X_RANGE[1] - X_RANGE[0])/X_BINS << "  "; 
      
      cout << endl; 
      	
			
      cout << "Tile Number " << tile << " Y offset " << offset[1][tile] << endl; 
      
      cout << "Y intervals: " ;
      
      for (int iy=0;iy <= Y_BINS ; iy++)
      	cout << Y_RANGE[0] - offset[1][tile] + iy*(Y_RANGE[1] - Y_RANGE[0])/Y_BINS << "  "; 
      	
      	
      cout << "Tile Number " << tile << " Theta offset " << offset[2][tile] << endl; 
      
      cout << "Theta intervals: " ;
      
      for (int it=0;it <= T_BINS ; it++)
      	cout << T_RANGE[0] - offset[2][tile] + it*(T_RANGE[1] - T_RANGE[0])/T_BINS << "  "; 
      	
      
      cout << "Tile Number " << tile << " Speed offset " << offset[3][tile] << endl; 
      
      cout << "Speed intervals: " ;
      
      for (int is=0;is <= S_BINS ; is++)
      	cout << S_RANGE[0] - offset[3][tile] + is*(S_RANGE[1] - S_RANGE[0])/S_BINS << "  "; 
      
            
      
      cout << endl; 
    }
		
}

// given a state of the mountain car, return the set of CMAC tiles corresponding to it

void cmac::active_tiles()
{

  vector<double> curPos = curr_robotPos();
  vector<double> curSpeed = curr_speed();
  
  double speed = curSpeed[0]*curSpeed[2]*10;
  
  
//  cout << "Active tiles in pos " << pos << " and vel " << vel << endl; 
	
  for (int tile =0; tile<TILES; tile++)
    {
      int x_tile = (curPos[0] - X_RANGE[0] + offset[0][tile])/x_interval; // add offset!!
      int y_tile  = (curPos[1] - Y_RANGE[0] + offset[1][tile])/y_interval; 
      int t_tile  = (curPos[2] - T_RANGE[0] + offset[2][tile])/t_interval;
      int s_tile  = (speed - S_RANGE[0] + offset[3][tile])/s_interval;
      
   //   cout << "(" << pos_tile << "," << vel_tile << ")" << " "; 
			
      atiles[tile].x_bin = x_tile; 
      atiles[tile].y_bin = y_tile; 
      atiles[tile].t_bin = t_tile; 
      atiles[tile].s_bin = s_tile; 
    }
//    cout << endl; 
}

// show which tiles are active 	
void cmac::display_active_tiles()
{

  for (int tile = 0; tile < TILES; tile++)
    {
     // cout << "Tile " << tile << " "; 
      cout << "(" << atiles[tile].x_bin << "," << atiles[tile].y_bin << "," << atiles[tile].t_bin <<
       "," << atiles[tile].s_bin<<")  ";
  //    cout << endl; 
    }
    
    cout << endl << endl; 
}


