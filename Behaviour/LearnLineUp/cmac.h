// CMAC representation for mountain car problem 

#include "LearnLineUp.h"   // definition of mountain car problem 

// define a tiling for the mcar problem 

#define X_BINS 10
#define Y_BINS 10
#define T_BINS 2
#define S_BINS 2
#define TILES 10



#define ACTIONS 6  // incTrans, decTrans, incDir, decDir, incRot, decRot

struct cell 
{
	int x_bin; 
	int y_bin;
	int t_bin;
	int s_bin;
}; 

// make a class for a CMAC

class cmac : public LearnLineUp 
{
	public: 
	
		cmac(void);   // create the tiles for a new position
		
		cmac(double x, double y , double theta); // call base constructor and set up tiles
		
		void display_cmac(void);  // print out the offsets 
		
		void active_tiles();  // map the state of the robot into the tiles 
		
		void display_active_tiles(); // show which tiles are active currently
		
	protected: 
	
		cell atiles[TILES];  // keep list of active tiles 
		
		double offset[4][TILES];  // 0 is for x, 1 is for y, 2 for theta, 3 for speed
		
		double x_interval, y_interval, t_interval, s_interval; // bin size for each dimension 
	
	private:
			
};
