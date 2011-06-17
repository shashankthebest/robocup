#include "Experiment.h"



void run(MainParameters& mainP,  StateActionFA* safa, Agent* agent, bool saveFA=true){

  /* The "run"  function implements a learning run in an RL system.
    It expects as arguments processed command line parameters stored
    in the data structure of the type MainParmeters, pointer to the 
    initialized (learning parameters set, such as learning step, etc.)
    function approximators representing action-value functions
    or randomized policy for all actions and pointer to 
    the initialized (learning parameters set) Agent object.
    The last optional argument indicates whether the 
     function approximators' parameters have to be saved to text files at 
     the end of the run. 

     The file named r_n.hst is produced that contains learning history,
     where n is the number of the run, as specified in mainP.run data member.
     It is stored in the directory, specified in mainP.dir data member.
     Data members of the mainP argument can be assigned values by running 
     mainP.process(...) function on the command-line arguments by the 
     caller function. 
     
     The meaning of the columns in r_n.hst file is the following:
     1 - Trial# 
     2 - Return 
     3 - Average return per step 
     4 - Maximum parameter change (since previous recorded trial) 
         in approximator for action 0
     5 - Number of parameters affected by learning (since the beginning 
         of learning) for action 0
     ...

        the last two colums repeat for each action.

	At the end of the run, the parameters of the function approximators 
	are saved to text files, if saveFA==true. 
   */
 
  int i, j, k;

  char* runID = new char[5];
  sprintf(runID, "%d", mainP.run);

  char* fileHistory = new char[100];	//name of the file for learning history data
  strcpy(fileHistory, mainP.dir);
  strcat(fileHistory, "r_");
  strcat(fileHistory, runID);
  strcat(fileHistory ,".hst");
  
  char** fileAP  = new char*[Action::count];	//name of files for approximator settings
  for (i=0; i<Action::count; i++){
    fileAP[i] = new char[100];
    strcpy(fileAP[i], mainP.dir);
    strcat(fileAP[i], "r_");
    strcat(fileAP[i],runID);
    char temp[5];
    sprintf(temp,".a%d",i);
    strcat(fileAP[i],temp);
  }

  //load test states
  char c;
  if (mainP.TestStatesFile==NULL){
    cout << "No name specified for the file containing test states " << endl;
    exit(EXIT_FAILURE);
  }
  ifstream ifile(mainP.TestStatesFile);
  if (ifile.fail()){
    cout << "Cannot open file to load test set" << cout;
    exit(EXIT_FAILURE);
  }
	
  State* TestStates=new State[mainP.TestStatesNumber];
  double random;

  for (i=0; i<mainP.TestStatesNumber; i++){
    ifile >> random;
    if (ifile.fail()){
      cout << "Erro loading test set " << cout;
      exit(EXIT_FAILURE);
    }
		
    TestStates[i].x[0]=random;
    ifile.get(c);
    ifile >> random;
    if (ifile.fail()){
      cout << "can't open file" << cout;
      exit(EXIT_FAILURE);
    }
    TestStates[i].x[1]=random;
  }
  
  ifile.close();

  ofstream ofsHistory(fileHistory);	//file stream for saving learning history data
  if (ofsHistory.fail()){
    cout << "Error: can not open file to save history" << endl;
    exit(EXIT_FAILURE);
  }
	
// main  computational part

  double avrTR; //averrage test return (over many trajectories from each test state)
  double avrTRS; //average test return per step (over many trajectories from each test state)
  double sampleTR; //test return (one trajectory form each test state)
  double sampleTRS; //test return per step (one trajectory form each test state)
  int steps;
  double* MaxParameterChanges = new double[Action::count];
  int* NumberParametersChanged = new int[Action::count];


  //test policy before learning
  avrTR=0;
  avrTRS=0;
  for(k=0; k<mainP.TestSamples; k++){	
    sampleTR=0;
    sampleTRS=0;
    for(j=0; j<mainP.TestStatesNumber; j++){
      steps=agent->initTrial(mainP.Steps, false, false, &(TestStates[j]), NULL);
      sampleTR+=agent->getReturn();
      if (steps!=0)
	sampleTRS+=agent->getReturn()/(double)steps;
    }
    sampleTR/=(double)(mainP.TestStatesNumber);
    sampleTRS/=(double)(mainP.TestStatesNumber);
    avrTR+=sampleTR;
    avrTRS+=sampleTRS;
  }

  avrTR/=(double)(mainP.TestSamples);
  avrTRS/=(double)(mainP.TestSamples);
      
  safa->getMaxParameterChange(MaxParameterChanges);
  safa->getNumberParametersChanged(NumberParametersChanged);

  ofsHistory << 0 << "\t" << avrTR << "\t" << avrTRS;
  for(j=0; j<Action::count; j++){
    ofsHistory << "\t" << MaxParameterChanges[j] << "\t" << NumberParametersChanged[j];
  }

  ofsHistory << endl;
  if (ofsHistory.fail()){
    cout << "Error writing history after " << i << " learning trials" << endl;
    exit(EXIT_FAILURE);
  }

  //start learning

  for(i=1; i<= mainP.Trials; i++){
    
    steps=agent->initTrial(mainP.Steps, true, false, NULL, NULL); //learning trial

    if ((i%mainP.TestFrequency)==0) { //testing current policy
      avrTR=0;
      avrTRS=0;
      for(k=0; k<mainP.TestSamples; k++){	
	sampleTR=0;
	sampleTRS=0;
	for(j=0; j<mainP.TestStatesNumber; j++){
	  steps=agent->initTrial(mainP.Steps, false, false, &(TestStates[j]), NULL);
	  sampleTR+=agent->getReturn();
	  if (steps!=0)
	    sampleTRS+=agent->getReturn()/(double)steps;
	}
	sampleTR/=(double)(mainP.TestStatesNumber);
	sampleTRS/=(double)(mainP.TestStatesNumber);
	avrTR+=sampleTR;
	avrTRS+=sampleTRS;
      }

      avrTR/=(double)(mainP.TestSamples);
      avrTRS/=(double)(mainP.TestSamples);
      
      safa->getMaxParameterChange(MaxParameterChanges);
      safa->getNumberParametersChanged(NumberParametersChanged);

      ofsHistory << i << "\t" << avrTR << "\t" << avrTRS;
      for(j=0; j<Action::count; j++){
	ofsHistory << "\t" << MaxParameterChanges[j] << "\t" << NumberParametersChanged[j];
      }

      ofsHistory << endl;
      if (ofsHistory.fail()){
	cout << "Error writing history after " << i << " learning trials" << endl;
	exit(EXIT_FAILURE);
      }
    } //end of testing
  }	//end of the learning loop
  

  if (saveFA==true) safa->saveAllArchitectureParameters(fileAP);
    
  delete [] MaxParameterChanges;
  delete [] NumberParametersChanged;
  delete [] TestStates;
  delete [] runID;
  delete [] fileHistory;
  
  for (i=0; i<Action::count; i++){
      delete [] fileAP[i];
  }
  delete [] fileAP;

}
