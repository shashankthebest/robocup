
#include <stdlib.h>
#include <string.h>
#include<stdio.h>
#include<math.h>
#include<sys/types.h>		


#include "Tools/RL/Misc/globals.h"
#include "approximator.h"


// constants that represent the schedule for learning step change
#define CONSTANT 1
#define	DECREASE 2
#define	VISITATION 3

class CMAC : public Approximator {

 public:

  /////////////////////////////////////////////////////
  struct Tile{
    double w;	//weight
    double e;	//eligibility trace (used with TD algorithms)
    double alpha; //learning step for this parameter
    bool changed;//indicate whether the parameter has been changed since the beginning of learning

    Tile();
  };
  ///////////////////////////////////////////////////

  class Tiling{
	
    int N;		//total number of tiles
    double* h;	//size of tiles along each dimension
    int* n;		//number of tiles along each dimension
    Tile* tiles; //array of tiles
    double* origin;	//coordinates of the lower left corner of the tiling
    static double* LeftBounds; //left bounds of input variables
    static double* RightBounds;//right bound of input variables
    static bool BoundsSet;	//indicates if bounds have been set
    int* IndCoef;	//coefficients used to calculate index of a tile in the one-dimensional array of tiles
    static double decay; //trace decay coefficient
    double ALPHA_VISITATION_FACTOR; //constant "C" used in visitation schedule for the rule: C/(C+number of updates of this parameter)
		
    void createTiles();
    /*	Creates tiling as a collection of tiles.
	Offsets this tiling by a random amount
    */

  public:
    Tiling();
    /* Default constructor
     */

    Tiling(const int* nn);
    /*	General constructor.
	Parameters:
	nn : pointer to an array contatining number of tiles 
	along each input dimension
    */

    void operator = (Tiling& t);
    /*	Overloaded assignemnt operator
     */

    int getSize();
    /*	Returns number of tiles (parameters) in this tiling
     */

    static void setBounds(const double* left, const double* right);
    /*	Sets bounds for input variables 
     */
	
    static void deleteBounds();
    /*	Deletes bounds for input variables 
     */
	
    static void setTraceDecay(double d);
    /*	Sets trace decay factor to a particular value
     */

    void setAlpha(double alpha);
    /*	Sets learning step to the same value for all tiles
     */
		
    void getActiveParameter(const State& s, double& weight, int& index);
    /*	Calcuates the weight cooresponding to the active tile on
	this tiling.
	Parameters:
	s : input state
	weight : parameter to return
	index : index of the active tile in the 1-dimensional array of tiles
    */

    void updateParameters(double* delta, double& MaxParameterChange, int& NumberParametersChanged);
    /*	Increment parameters by the amounts in array delta, 
	multiplied by the appropriate learning step of each tile.
	Updates arguments MaxParameterChange and NumberParametersChanged 
	acoordingly.
    */

			
    void updateParameters(const State& s, double w_update, double& MaxParameterChange, int& NumberParametersChanged);
    /*	Updates all parameters of the tiling proportionally to 
	the amount w_update and corresponding eligibility traces of
	each tile. Update eligibility traces as well.
	Updates arguments MaxParameterChange and NumberParametersChanged 
	acoordingly.
    */
		
    void decreaseAlpha(double factor);
    /*	Decrease learning rate of all tiles by "factor".
	(Used with a decreasing schedule).
    */

    void decreaseAllAlpha();
    /*	Update learning rate of all tiles 
	(used with decreasing schedule conversly proportional 
	 number of seen examples).
    */


    void decreaseAlpha(const State& s);
    /*	Update learning rate of the tile activated by input state "s".
	(used with decreasing schedule where learning rate 
	is conversly proportional to  number of visitations to the tile)
    */

    void setVisitationFactor(double factor);
	
    void replaceTrace(const State& s, double replace);
    /*	Replace traces of parameters, 
	activated by input state "s" to value "replace".
    */

    void decayTraces(double factor);
    /* Decay traces of all tiles by "factor"
     */

    void accumulateTraces(const State& s, double amount);
    /* Increase the trace of the tile activated by input state "s" by "amount".
    */

    void getMinMax(double& min, double& max, int& minInd, int& maxInd);
    /*	Get values and tile indeces of the minimum and maximum tile weights
     */

    void saveParameters(ofstream& file);
    /*	Save parameters of the tiling to the file.
	Parameters:
	file : output file stream obeject 
    */

    void setParameters(ifstream& file);
    /*	Read parameters of the tiling from a text file
	Parameters:
	file : input file stream object
    */
		
    ~Tiling();
  };

  struct ParameterIndex{
	
    int tilingIndex;
    int tileIndex;

    ParameterIndex(){
      tilingIndex=0;
      tileIndex=0;
    }
  };

 private: //members of CAMC class
  int T;  //number of tilings
  int NumberOfParameters;	//number of parameters in all tilings
  Tiling* tilings;	//array of tilings
  int ALPHA_SCHEDULE; //can be CONSTANT, DECREASE or VISITATION
  int ALPHA_DECREASE_FREQUENCY;	//frequency of learning step decrease in DECREASE schedule, given in terms of the number of seen examples
  double ALPHA_DECREASE_FACTOR;	//constant "C" in the rule for the learning step change; C/(C+number of seen examples)
  int ExamplesNumber; //keeps track of the number of processed training examples
  bool SaveParameterHistory; //indicates whether min and max values of parameters have to be saved to a file
	
 public:

  CMAC(char* fileName);
  /* General constructor. fileName is the name of the text file containing
     architectural parameters, such as number of Tilings and tiles along 
     each dimension for each tiling.
  */
	

  int getSize();
  /* Returns total number of parameters in this camc architecture
   */

  static void setInputBounds(const double* left, const double* right);
  /*	Sets bounds on input variables
   */

  static void deleteInputBounds();
  /* Deletes bounds on input variables
   */
	
  void predict(const State& s, double& output);
  /* Predicts an output value for a given input.
     Parameters:
     s : reference to the input (state)
     output : returned value of the predicted output
  */
	
  void learn(const State& s, const double target);
  /* Learns an input-output pair by gradient descent on 
     Mean Squared Error criteria.
     Parameters:
     s : input (state)
     target: target output value.
  */

  void computeGradient(const State& s, double* GradientVector);
  /* Compute the gradient vector w.r.t. architecture parameters at input s and
     current parameter values.
  */

  void updateParameters(double* delta);
  /* Increase parameters by amounts in delta array multiplied by appropriate learning step for each parameter.
  */

  void replaceTraces(const State& s, double replace);
  /* Set traces of parameters, activated by input state "s", to value "replace".
   */

  void decayTraces(double factor);
  /* Decay all traces by "factor".
   */

  void accumulateTraces(const State& s, double amount);
  /* Increase traces of the parameters, activated by input state "s", by "amount.
   */

  void setArchitectureParameters(int argc, char *argv[]);
  /* Loads parameters of the architecture from a text file.
     Parameters:
     argc : number of supplied arguments in array argv.
     argv : array of parameters.
     In this case 1 argument is expected (argc=1) and 
     argv[0] is the name of the file from which parameters are to be read.
  */
	
  void saveArchitectureParameters(int argc, char *argv[]);
  /* Saves parameters of the architecture.
     Parameters:
     argc : number of supplied arguments
     argv : array of parameters
	In this case 1 argument is expected (argc=1) and 
	argv[0] is the name of the file to which parameters 
	are to be saved.
  */
	
  void setLearningParameters(int argc, char *argv[]);
  /*	Sets learning parameters.
	Parameters:
	argc : number of supplied arguments
	argv : array of parameters
	Format and meaning of the parameters passed in "argv":
	schedule=value :  type of the schedule for decreasing learning steps 
	                  (possible values: constant, decrease, visitation);
	alpha=value : learning step (constant if using constant schedule, 
	              or initial value if using other schedules);
	f=value : decrease frequency, if using decrease schedule
	d=value : decrease factor, if using decrease schedule
	v=value : constant used in the visitation schedule rule to decrease 
	          learning steps: v/(v+number of visits to the tile). 
	Should be v>1; the larger v, the slower learning steps decrease.
	  
  */

  
  static void helpLearningParameters();
  /* Print out the format for command line specification of the learning parameters
   */

  ~CMAC();
  /* Destructor.
   */

 protected:

  void createCMAC(int t, int** n);
  /* Parameters:
     t : number of tilings
     n : array that indicates number of tiles along each dimension (second index) for each tiling (first index).
  */

  void setCmacStructure(char* fileName, int& TilingsNumber, int*** TilesNumber);
  /* This function reads description of the CMAC architectural parameters 
     from a text file and records them in the last two arguments
  */
	
};





	
