/*	Reinforcement Learning
	Implementation of CMAC funcltion approximation 
	In this implementation, function CMAC::learn(...) implements
	learning using eligibility traces. There are functions 
	implemented to replace, clear and accumulate traces.
	There are two ways to use this implementation as in learning without traces. 
	One: first decay all traces to 0 and then use replace trace for active tiles to 1. 
	Two: use functions computeGradient and updateParameters.

*/

#include <stdlib.h>
#include <string.h>
#include<stdio.h>
#include<math.h>
#include<sys/types.h>		
#include "Tools/RL/Misc/globals.h"
#include "cmac.h"


double* CMAC::Tiling::LeftBounds = NULL;
double* CMAC::Tiling::RightBounds = NULL;
double CMAC::Tiling::decay=0; 
bool CMAC::Tiling::BoundsSet=false;

CMAC::Tile::Tile(){
  //w=(double)rand()/(double)RAND_MAX;	//initialize to random values in [0,1]
  w=0;
  e=1;
  alpha=1;
  changed=false;
}


void CMAC::Tiling::createTiles()
/*	Creates tiling as a collection of tiles.
	Offsets this tiling by a random amount
*/
{
  int i;
  double offset;

  N=1;
  IndCoef[State::dimensionality-1]=1;
		
  for(i=State::dimensionality-1; i>=0; i--)			// Start from last state, travel to first
  { 
    /*	offset the tiling along each dimension 
	by a random amount bounded by the width 
	of tiles along this dimension 
    */
    offset = (double)rand()/(double)RAND_MAX*h[i];   // get a random value
    n[i] = n[i] + 1;								// Since one tile is added, increment the # tiles for this dim
    N = N * n[i];									// Total no. of tiles for all dims
    origin[i] = LeftBounds[i] - offset;				// Caclulate the base

	if ( i< (State::dimensionality-1) )				// If not last state
		IndCoef[i] = IndCoef[i+1] * n[i+1];			//
    
  }

  tiles = new Tile[N];								// Create total tiles, corresponding to this tiling for all dims

#ifdef DEBUG
  cout << "Total number of tiles: " << N << endl;
  cout << "Left bounds \t Origin \t IndCoef " << endl;
  for (i=0; i<State::dimensionality; i++)
    cout << LeftBounds[i] << "\t" << origin[i] << "\t" << IndCoef[i] << endl;
#endif

}

CMAC::Tiling::Tiling()
{
  n = new int[State::dimensionality];  // Stores number of tiles accross each dimension, therefore its size should be same as dim
  h = new double[State::dimensionality]; // Stores size of tiles accross each dimension, therefore its size should be same as dim
  origin = new double[State::dimensionality]; //coordinates of the lower left corner of the tiling, for each dimension
  IndCoef = new int[State::dimensionality];  //coefficients used to calculate index of a tile in the one-dimensional array of tiles
  N = 0;  //total number of tiles
  ALPHA_VISITATION_FACTOR = 1;
  tiles = NULL;
}

CMAC::Tiling::Tiling(const int* nn){
  /*	General constructor.
	Parameters:
	nn : pointer to an array contatining number of tiles 
	along each input dimension
  */

  if (BoundsSet==false)
    {	cout << "Error (tiling): variables' bounds are not set" << endl;
    exit(EXIT_FAILURE);
    }

  int i;
  n=new int[State::dimensionality];
  h=new double[State::dimensionality];
  origin = new double[State::dimensionality];
  IndCoef = new int[State::dimensionality];

  for(i=0; i<State::dimensionality; i++){
    n[i]=nn[i];
    h[i]=(RightBounds[i]-LeftBounds[i])/n[i];
  }

  createTiles();
  ALPHA_VISITATION_FACTOR=1;

#ifdef DEBUG
  cout << "Number of tiles \t Tile width" << endl;
  for(i=0; i<State::dimensionality; i++)
    cout << n[i] << "   " << h[i] << endl;
#endif
}

void CMAC::Tiling::operator = (Tiling& t){
  int i;

  N=t.N;
  for(i=0; i<State::dimensionality; i++){
    n[i]=t.n[i];
    h[i]=t.h[i];
    origin[i]=t.origin[i];
    IndCoef[i]=t.IndCoef[i];
  }
  tiles=new Tile[N];
  for(i=0; i<N; i++)
    tiles[i]=t.tiles[i];
}

int CMAC::Tiling::getSize(){
  /*	Returns number of tiles (parameters) in this tiling
   */
  return N;
}

void CMAC::Tiling::setBounds(const double* left, const double* right){
  /*	Sets bounds for input variables 
   */			
  int i;
  LeftBounds = new double[State::dimensionality];
  RightBounds = new double[State::dimensionality];

  for(i=0; i<State::dimensionality; i++){
    LeftBounds[i]=left[i];
    RightBounds[i]=right[i];
  }
  BoundsSet=true;

#ifdef DEBUG
  cout << "Bounds are set to: " << endl;
  cout << "Left bounds \t Right bounds" << endl;
  for(i=0; i<State::dimensionality; i++)
    cout << LeftBounds[i] << "\t" << RightBounds[i] << endl;
#endif

}

void CMAC::Tiling::deleteBounds(){
  /*	Deletes bounds for input variables 
   */
  delete [] LeftBounds;
  delete [] RightBounds;
  BoundsSet=false;
}

void CMAC::Tiling::setTraceDecay(double d){
  /*	Sets trace decay factor to a particular value
   */
  if ((d<0) || (d>1)){
    cout << "Error (tiling): trace decay must be in [0,1]" << endl;
    exit(EXIT_FAILURE);
  }

  decay = d;
}

void CMAC::Tiling::setAlpha(double alpha){
  /*	Sets learning step to the same value for all tiles
   */
  int i;

  for(i=0; i<N; i++)
    tiles[i].alpha=alpha;
}


void CMAC::Tiling::getActiveParameter(const State& s, double& weight, int& index){
  /*	Calcuates the weight cooresponding to the active tile on
	this tiling.
	Parameters:
	s : input state
	weight : parameter to return
	index : index of the active tile in the 1-dimensional array of tiles
  */

  int i, ind;


  index=0;
  for(i=0; i<State::dimensionality; i++)
  {
    ind=(int)(float)((s.x[i]-origin[i])/h[i]);
    

    index=index+IndCoef[i]*ind;
  }
		


  if ((index<0) || (index>=N)) 
  {
    cout << "Error (tiling): tile index out of limits" << endl;
    cout << "state: " << s << " index=" << index << " N=" << N << endl;
    index=0;
    for(i=0; i<State::dimensionality; i++){
      ind=(int)(float)((s.x[i]-origin[i])/h[i]);
      cout << "ind[" << i << "]=" << ind << " IndCoef[" << i << "]=" << IndCoef[i];

      index=index+IndCoef[i]*ind;
      cout << " index=" << index << endl;
    }
    exit(EXIT_FAILURE);
  }

  weight = tiles[index].w;


}

void CMAC::Tiling::updateParameters(double* delta, double& MaxParameterChange, int& NumberParametersChanged)
  /*	Increment parameters by the amounts 
	in array delta, multiplied by the appropriate
	learning step of each tile
*/
{
  int i;
  double change;

  for (i=0; i<N; i++){
    change =  tiles[i].alpha * delta[i];
    tiles[i].w=tiles[i].w + change;
    if (fabs(change)>MaxParameterChange) MaxParameterChange=fabs(change);
    if (tiles[i].changed==false){
      if (change!=0){
	NumberParametersChanged++;
	tiles[i].changed=true;
      }
    }
  }
}


void CMAC::Tiling::updateParameters(const State& s, double w_update, double& MaxParameterChange, int& NumberParametersChanged){
  /*	Updates all parameters of the tiling proportionally to 
	the amount w_update and corresponding eligibility traces of
	each tile. Update eligibility traces as well.
	Parameters:
	s : input which determines the active tile
	w_update : the ammount proportionally to which parameters are updated
  */
	
  int i;
  double change;

  for(i=0; i<N; i++){
    change = tiles[i].alpha*w_update * tiles[i].e;
    tiles[i].w = tiles[i].w + change;
    if (fabs(change)>MaxParameterChange) MaxParameterChange=fabs(change);
    if (tiles[i].changed==false){
      if (change!=0){
	NumberParametersChanged++;
	tiles[i].changed=true;
      }
    }
  }	

}

void CMAC::Tiling::decreaseAlpha(double factor){
  /*	Decrease learning rate of all tiles by factor
	(Used with a decreasing schedule)
  */
  int i;

  for(i=0; i<N; i++)
    tiles[i].alpha=tiles[i].alpha/factor;
}

void CMAC::Tiling::decreaseAllAlpha(){
  /*	Update learning rate of active tile
	(used with decreasing schedule where learning rate 
	is eqaul to 1/#of visitations to the tile)
  */
  int i;
  double newAlpha;
  newAlpha=ALPHA_VISITATION_FACTOR/(floor(ALPHA_VISITATION_FACTOR/tiles[0].alpha)+1);
  for(i=0; i<N; i++)
    tiles[i].alpha=newAlpha;
}

void CMAC::Tiling::decreaseAlpha(const State& s){
  /*	Update learning rate of active tile
	(used with decreasing schedule where learning rate 
	is eqaul to 1/#of visitations to the tile)
  */
  int ActiveIndex;
  double temp;

  getActiveParameter(s, temp, ActiveIndex);
	
  tiles[ActiveIndex].alpha=ALPHA_VISITATION_FACTOR/(ALPHA_VISITATION_FACTOR/tiles[ActiveIndex].alpha+1);
	
	
}

void CMAC::Tiling::setVisitationFactor(double factor){
  ALPHA_VISITATION_FACTOR=factor;
}

void CMAC::Tiling::replaceTrace(const State& s, double replace){
  /*	Replace traces of parameters, 
	activated by input state s to value replace
  */
  int ActiveIndex;
  double temp;

  getActiveParameter(s, temp, ActiveIndex);
  tiles[ActiveIndex].e=replace;
}

void CMAC::Tiling::decayTraces(double factor){
  int i;

  for(i=0; i<N; i++)
    tiles[i].e=tiles[i].e*factor;
}

void CMAC::Tiling::accumulateTraces(const State& s, double amount){
	
  int ActiveIndex;
  double temp;

  getActiveParameter(s, temp, ActiveIndex);
  tiles[ActiveIndex].e+=amount;
}

void CMAC::Tiling::getMinMax(double& min, double& max, int& minInd, int& maxInd){
  /*	Get values and tile indeces of the minimum and maximum tile weights
   */
  int i;
  min=fabs(tiles[0].w);
  minInd=0;
  max=fabs(tiles[0].w);
  maxInd=0;

  for(i=1; i<N; i++)
    if (fabs(tiles[i].w)<min){
      min=fabs(tiles[i].w);
      minInd=i;
    }
    else{
      if (fabs(tiles[i].w)>max){
	max=fabs(tiles[i].w);
	maxInd=i;
      }
    }
}

void CMAC::Tiling::saveParameters(ofstream& file){
  /*	Save parameters of the tiling to the file.
	Parameters:
	file : output file obeject 
  */
  int i;

  file << "Number of tiles: " << N << endl;
  if (file.fail()){
    cout << "Error (tiling): disk is full " << endl;
    exit(EXIT_FAILURE);
  }

  file << "Number of tiles along each dimension: " << endl;
  if (file.fail()){
    cout << "Error (tiling): disk is full " << endl;
    exit(EXIT_FAILURE);
  }

  for(i=0; i<State::dimensionality; i++){
    file << n[i] << "    ";
    if (file.fail()){
      cout << "Error (tiling): disk is full " << endl;
      exit(EXIT_FAILURE);
    }
  }
  file << endl;

  file << "Tiles' width along each dimension: " << endl;
  if (file.fail()){
    cout << "Error (tiling): disk is full " << endl;
    exit(EXIT_FAILURE);
  }

  for(i=0; i<State::dimensionality; i++){
    file << h[i] << "    ";
    if (file.fail()){
      cout << "Error (tiling): disk is full " << endl;
      exit(EXIT_FAILURE);
    }
  }
  file << endl;

  file << "Origin of the tiling: " << endl;
  if (file.fail()){
    cout << "Error (tiling): disk is full " << endl;
    exit(EXIT_FAILURE);
  }

  for(i=0; i<State::dimensionality; i++){
    file << origin[i] << "    ";
    if (file.fail()){
      cout << "Error (tiling): disk is full " << endl;
      exit(EXIT_FAILURE);
    }
  }
  file << endl;
	
  file << "Tiles' parameters: " << endl;
  if (file.fail()){
    cout << "Error (tiling): disk is full " << endl;
    exit(EXIT_FAILURE);
  }
  for(i=0; i<N; i++){
    file << tiles[i].w << endl;
    if (file.fail()){
      cout << "Error (tiling): disk is full " << endl;
      exit(EXIT_FAILURE);
    }
  }
}

void CMAC::Tiling::setParameters(ifstream& file){
  /*	Read parameters of the tiling from a text file
	Parameters:
	file : input file object
  */
  int i;
  char buffer[40];
  char c;
	
  file.get(c);
  file.get(buffer, strlen("Number of tiles: ")+1);

  if (file.fail()){
    cout << "Error (tiling): input failed (1)" << endl;
    exit(EXIT_FAILURE);
  }
	
  int t; 
  file >> t;
	
  if (file.fail()){
    cout << "Error (tiling): input failed (2)" << endl;
    exit(EXIT_FAILURE);
  }

  file.get(c);

  file.get(buffer, strlen("Number of tiles along each dimension: ")+1);
  if (file.fail()){
    cout << "Error (tiling): input failed (3) " << endl;
    exit(EXIT_FAILURE);
  }

  for(i=0; i<State::dimensionality; i++){
    file >> n[i];
    if (file.fail()){
      cout << "Error (tiling): input failed (4) " << i << endl;
      exit(EXIT_FAILURE);
    }
  }
  file.get(buffer, 5);
	
  file.get(c);
  file.get(buffer, strlen("Tiles' width along each dimension: ")+1);
  if (file.fail()){
    cout << "Error (tiling): input failed (5) " << endl;
    exit(EXIT_FAILURE);
  }

  for(i=0; i<State::dimensionality; i++){
    file >> h[i];
    if (file.fail())
	{
      
		cout << "Error (tiling): input failed (6) " << i << endl << __FILE__<<"  "<<__LINE__;
      exit(EXIT_FAILURE);
    }
  }
  file.get(buffer, 5);

  file.get(c);
  file.get(buffer, strlen("Origin of the tiling: ")+1);
  if (file.fail()){
    cout << "Error (tiling): input failed (7) " << endl;
    exit(EXIT_FAILURE);
  }

  for(i=0; i<State::dimensionality; i++){
    file >> origin[i];
    if (file.fail()){
      cout << "Error (tiling): input failed (8) " << i << endl;
      exit(EXIT_FAILURE);
    }
  }
  file.get(buffer, 5);

  if (t!=N){
    delete [] tiles;
    N=t;
    tiles = new Tile[N];

    IndCoef[State::dimensionality-1]=1;
		
    for(i=State::dimensionality-2; i>=0; i--)
      IndCoef[i]=IndCoef[i+1]*n[i];
  }
	
  file.get(c);
  file.get(buffer, strlen("Tiles' parameters: ")+1);
  if (file.fail()){
    cout << "Error (tiling): input failed (9) " << endl;
    exit(EXIT_FAILURE);
  }
  for(i=0; i<N; i++){
    file >> tiles[i].w;
    if (file.fail()){
      cout << "Error (tiling): input failed (10) " << i << endl;
      exit(EXIT_FAILURE);
    }
    tiles[i].e=0;
  }
}

CMAC::Tiling::~Tiling(){
  
  delete [] origin;
  delete [] IndCoef;
  delete [] tiles;
  delete [] n;
  delete [] h;
	
}

CMAC::CMAC(char* fileName){
  int TilingsNumber = 0;
  int** TilesNumber=NULL;

  setCmacStructure(fileName, TilingsNumber, &TilesNumber);
  
//  cout<<"\nAt file "<<__FILE__<<" on line "<<__LINE__<<"  TilingsNumber  = "<<TilingsNumber<<"\n\n\n";
	
  if (TilesNumber==NULL){
    cout << "Something is wrong with reading CMAC structure from file" << fileName << endl;
    exit(EXIT_FAILURE);
  }

  createCMAC(TilingsNumber, TilesNumber);

  int i;
  for (i=0; i<TilingsNumber; i++)
    delete [] TilesNumber[i];
  delete [] TilesNumber;

}


void CMAC::createCMAC(int t, int** n){
  /*	General constructor.
	Parameters:
	t : number of tiles
	n : array that indicates number of tiles along each dimension for each tile
  */
  int i;
  
  tilings = new Tiling[t];
  T=t;
  ALPHA_SCHEDULE=VISITATION;
  ALPHA_DECREASE_FREQUENCY=100;
  ALPHA_DECREASE_FACTOR=2.0;
  ExamplesNumber=0;
  NumberOfParameters=0;
  MaxParameterChange=0;
  NumberParametersChanged=0;

  for(i=0; i<t; i++)
  {
    Tiling* til = new Tiling(n[i]);
    tilings[i] = (*til);
    NumberOfParameters = NumberOfParameters+tilings[i].getSize();
  }
		
  CMAC::Tiling::setTraceDecay(0.0);
}

int CMAC::getSize()
{
  /*	Returns total number of parameters in this camc architecture
   */
  return NumberOfParameters;
}

void CMAC::setInputBounds(const double* left, const double* right)
{
  /*	Sets bounds on input variables
   */
  CMAC::Tiling::setBounds(left, right);
}

void CMAC::deleteInputBounds(){
  CMAC::Tiling::deleteBounds();
}


void CMAC::predict(const State& s, double& output)
  /*	Predicts an output value for a given input.
	Parameters:
		s : reference to the input (state)
		output : returned value of the predicted output
*/
{	int i, ind;
 double w;

 output=0;
 for(i=0; i<T; i++)
 {
   tilings[i].getActiveParameter(s, w, ind);
   output = output+w;
 }

#ifdef DEBUG
 cout << "Input: " << endl;
 for(i=0; i<State::dimensionality; i++)
   cout << s.x[i] << "  ";
 cout << endl;
 cout << "Output: " << output << endl;
#endif
}



void CMAC::learn(const State& s, const double target)
  /*	Learns an input-output pair. 
	Used when learning according to MSE criteria
	Parameters:
		s : input (state)
		target: target output value
*/
{
  int i;
  double w_update, PredictedValue;

  ExamplesNumber++;
		
  predict(s, PredictedValue);
  w_update=(target-PredictedValue);

  for(i=0; i<T; i++)
    tilings[i].updateParameters(s, w_update, MaxParameterChange, NumberParametersChanged); //this is a function that uses eleigibility traces. It is assumed that the user of this class sets eligibility traces using appropriate functions.

  if (ALPHA_SCHEDULE==DECREASE){
    if ((ExamplesNumber%ALPHA_DECREASE_FREQUENCY)==0)
      for(i=0; i<T; i++)
	tilings[i].decreaseAlpha(ALPHA_DECREASE_FACTOR);
  }
  else
    if (ALPHA_SCHEDULE==VISITATION)
      for(i=0; i<T; i++)
	tilings[i].decreaseAlpha(s);
}

void CMAC::computeGradient(const State& s, double* GradientVector)
  /*	Compute the gradient w.r.t. architecture parameters 
	at input s
*/
{
  int i, j, ind, N, I;
  double par;
	
  I=0;
  for (i=0; i<T; i++){
    N=tilings[i].getSize();
    tilings[i].getActiveParameter(s, par, ind);
    for (j=0; j<N; j++){
      if (j==ind)
	GradientVector[I+j]=1;
      else
	GradientVector[I+j]=0;
    }
    I=I+N;
  }
}

void CMAC::updateParameters(double* delta)
  /*	Increase parameters by amounts in delta array
	multiplied by appropriate learning step for each parameter
*/
{
  int i, N, I;

  ExamplesNumber++;

  I=0;
  for (i=0; i<T; i++){
    N=tilings[i].getSize();
    //cout << "Tiling " << i << endl;
    tilings[i].updateParameters(&(delta[I]), MaxParameterChange, NumberParametersChanged);
    I=I+N;
  }

	
		
  if (ALPHA_SCHEDULE==DECREASE){
    if ((ExamplesNumber%ALPHA_DECREASE_FREQUENCY)==0)
      for(i=0; i<T; i++)
	tilings[i].decreaseAlpha(ALPHA_DECREASE_FACTOR);
  }
  else
    if (ALPHA_SCHEDULE==VISITATION)
      for(i=0; i<T; i++)
	tilings[i].decreaseAllAlpha();
}


void CMAC::replaceTraces(const State& s, double replace)
  /*	Replace traces of parameters, activated by input state s to value replace
*/
{
  int i;

  for(i=0; i<T; i++)
    tilings[i].replaceTrace(s, replace);
}

void CMAC::decayTraces(double factor){
  int i;

  for(i=0; i<T; i++)
    tilings[i].decayTraces(factor);
}

void CMAC::accumulateTraces(const State& s, double amount){
  int i;

  for(i=0; i<T; i++)
    tilings[i].accumulateTraces(s, amount);
}

void CMAC::setArchitectureParameters(int argc, char *argv[])
  /*	Loads parameters of the architecture.
	Parameters:
		argc : number of supplied arguments
		argv : array of arguments
	In this case 1 argument is expected (argc=1) and 
	argv[0] is the name of the file from which parameters are 
	to be read
*/
{	int i;

 if (argc==0){
   cout << "No file name to read data." << endl;
   exit(EXIT_SUCCESS);
 }
	
 ifstream file(argv[0]);
 if (file.fail()){
   cout << "Error (cmac): cannot open file " << argv[0] << endl;
   exit(EXIT_FAILURE);
 }
	
 char buffer[40];
	
 file.get(buffer, strlen("Number of tilings: ")+1);
 if (file.fail()){
   cout << "Error (cmac): input failed (1)" << endl;
   exit(EXIT_FAILURE);
 }
 int t;
	
 file >> t;
 if (file.fail()){
   cout << "Error (cmac): input failed (2)" << endl;
   exit(EXIT_FAILURE);
 }

 if (t!=T){
   delete [] tilings;
   T=t;
   tilings = new Tiling[T];
 }

 for(i=0; i<T; i++){
   int j;
   char c;
   file.get(c);
   if (file.fail()){
     cout << "Error (cmac): input failed (2-3) " << endl;
     exit(EXIT_FAILURE);
   }

   file.get(buffer, strlen("Tiling ")+1);
   if (file.fail()){
     cout << "Error (cmac): input failed (3) " << endl;
     exit(EXIT_FAILURE);
   }

   file >> j;
   if (file.fail()){
     cout << "Error (cmac): input failed (4) " << endl;
     exit(EXIT_FAILURE);
   }
   tilings[i].setParameters(file);
 }

 file.close();
}

void CMAC::saveArchitectureParameters(int argc, char *argv[])
  /*	Saves parameters of the architecture.
	Parameters:
		argc : number of supplied arguments
		argv : array of arguments
	In this case 1 argument is expected (argc=1) and 
	argv[0] is the name of the file to which parameters 
	are to be saved
*/
{	int i;
	
 if (argc==0){
   cout << "No file name to save data." << endl;
   exit(EXIT_SUCCESS);
 }
	
 ofstream file(argv[0]);
 if (file.fail()){
   cout << "Error (cmac): cannot open file " << argv[0] << endl;
   exit(EXIT_FAILURE);
 }
	
 file << "Number of tilings: " << T << endl;
 if (file.fail()){
   cout << "Error (cmac): disk is full " << endl;
   exit(EXIT_FAILURE);
 }

 for(i=0; i<T; i++){
   file << "Tiling " << i << endl;
   if (file.fail()){
     cout << "Error (cmac): disk is full " << endl;
     exit(EXIT_FAILURE);
   }
   tilings[i].saveParameters(file);
 }

 file.close();
}
void CMAC::setLearningParameters(int argc, char *argv[])
  /*	Sets learning parameters.
	Parameters:
		argc : number of supplied arguments
		argv : array of arguments
*/
{
  int i, j;
  double alpha=0;
  double decay=0;
  double v=0;
  bool alphaSet=false;

  	
  for (i=0; i<argc; i++){

    if (strncmp(argv[i],"?",1)==0){
      cout << "CMAC learning parammeters:" << endl;
      cout << "schedule= (possible values: constant, decrease, visitation)" << endl;
      cout << "alpha= (constant if using constant schedule, or initial value if using other schedules)" << endl;
      cout << "f= (decrease frequency, if using decrease schedule)" << endl;
      cout << "d= (decrease factor, if using decrease schedule)" << endl;
      cout << "v= (visitation factor for alpha)" << endl;
	
      return;
    }

    if (strncmp("schedule=", argv[i], 9)==0)
    {
			
      if (strncmp("schedule=constant", argv[i], 17)==0)
	ALPHA_SCHEDULE=CONSTANT;
      if (strncmp("schedule=decrease", argv[i], 17)==0)
	ALPHA_SCHEDULE=DECREASE;
      if (strncmp("schedule=visitation", argv[i], 19)==0)
	ALPHA_SCHEDULE=VISITATION;
    }
		
    if (strncmp("alpha=", argv[i], 6)==0)
    {
      alpha=atof(&(argv[i][6]));
      if ((alpha<=0) || (alpha>1))
      {
			cout << "Error (cmac): learning step must be in (0,1]" << endl;
			exit(EXIT_FAILURE);
      }
      alphaSet=true;
    }

    if (strncmp("f=", argv[i], 2)==0)
      ALPHA_DECREASE_FREQUENCY=atoi(&(argv[i][2]));
			
		
    if (strncmp("ph=", argv[i], 3)==0)
    {
      SaveParameterHistory=atoi(&(argv[i][3]));
      if ((SaveParameterHistory!=true)&&(SaveParameterHistory!=false))
      {
			cout << "Error (cmac): invalid entry for save parameter history option" << endl;
			exit(EXIT_FAILURE);
      }
    }


    if (strncmp("d=", argv[i], 2)==0){
      ALPHA_DECREASE_FACTOR=atof(&(argv[i][2]));
      if (ALPHA_DECREASE_FACTOR<=1)
      {
			cout << "Error (camc): invalid setting of alpha decrease factor" << endl;
			exit(EXIT_FAILURE);
      }
    }

    if (strncmp("v=", argv[i], 2)==0)
    {
      v=atof(&(argv[i][2]));
      if (v<1)
      {
			cout << "Error (camc): invalid setting of alpha visitation factor" << endl;
			exit(EXIT_FAILURE);
      }
      

		for(j=0; j<T; j++)
			tilings[j].setVisitationFactor(v);
    }

    if (strncmp("decay=", argv[i], 6)==0)
    {
      decay=atof(&(argv[i][6]));
      if ((decay<0) || (decay>1))
      {
			cout << "Error (cmac): trace decay must be in [0,1]" << endl;
			exit(EXIT_FAILURE);
      }
      CMAC::Tiling::setTraceDecay(decay);
    }
  }

	
  if (alphaSet==true)
  {
    if (alpha==0)
    {
      cout << "Please specify learning step from (0,1]" << endl;
      cin >> alpha;
      if (alpha<=0) 
      {
		cout << "Invalid entry" << endl;
		exit(EXIT_SUCCESS);
      }
    }

    for(j=0; j<T; j++)
      tilings[j].setAlpha(alpha);
  }
  
}

CMAC::~CMAC(){
  delete [] tilings;
	
}

void CMAC::helpLearningParameters(){

  cout << "CMAC learning parammeters:" << endl;
  cout << "schedule=value : type of the schedule for decreasing learning steps (possible values: constant, decrease, visitation)" << endl;
  cout << "alpha=value : learning step (constant if using constant schedule, or initial value if using other schedules)" << endl;
  cout << "f=value : decrease frequency, if using decrease schedule" << endl;
  cout << "d=value : decrease factor, if using decrease schedule" << endl;
  cout << "v=value : constant used in the visitation schedule rule to decrease learning steps: v/(v+number of visits to the tile)." << endl;
  cout << "decay=value : trace decay factor" << endl;
  return;

} 



void CMAC::setCmacStructure(char* fileName, int& TilingsNumber, int*** TileNumbers){
/*This function reads configuration of the cmac architecture from a file
*/
	char buffer[40], c;
	int i,j;
	

	ifstream cmacFile(fileName);
	if (cmacFile.fail()){
		cout << "Error: can not open file to read cmac configuration" << endl;
		exit(EXIT_FAILURE); 
	}

	cmacFile.get(buffer, strlen("TilingsNumber =")+1);
	if (cmacFile.fail()){
		cout << "Error: input failed (1)" << endl;
		exit(EXIT_FAILURE);
	}

	cmacFile >> TilingsNumber;
	
	if (cmacFile.fail()){
		cout << "Error: input failed (2)" << endl;
		exit(EXIT_FAILURE);
	}
	
	*TileNumbers = new int*[TilingsNumber];
	
	cmacFile.get(c);
	if (cmacFile.fail()){
		cout << "Error: input failed (3)" << endl;
		exit(EXIT_FAILURE);
	}

	cmacFile.get(buffer, strlen("Number of tiles along each dimension:")+1);
	if (cmacFile.fail()){
		cout << "Error: input failed (4)" << endl;
		exit(EXIT_FAILURE);
	}


	for(i=0; i<TilingsNumber; i++){
		(*TileNumbers)[i] = new int[State::dimensionality];
		
		cmacFile.get(c);
		if (cmacFile.fail()){
			cout << "Error: input failed (4-5)" << endl;
			exit(EXIT_FAILURE);
		}

		cmacFile.get(buffer, strlen("Tiling ")+1);
		if (cmacFile.fail()){
			cout << "Error: input failed (5)" << endl;
			exit(EXIT_FAILURE);
		}

		cmacFile >> j;
		if (cmacFile.fail()){
			cout << "Error: input failed (6) tiling" << i << endl;
			exit(EXIT_FAILURE);
		}

		for (j=0; j<State::dimensionality; j++){
			cmacFile >> (*TileNumbers)[i][j];
			if (cmacFile.fail()){
				cout << "Error: input failed (7)" << endl;
				exit(EXIT_FAILURE);
			}
		}
	}

	cmacFile.close();
}


















