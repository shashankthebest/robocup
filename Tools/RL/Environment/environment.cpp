
#include<stdlib.h>
#include<math.h>
#include<time.h>
#include "Tools/RL/rlIncludes.h"
#include "environment.h"



TransitionSamples::TransitionSamples(){
  Transitions=0;
  nextState=NULL;
  reward=NULL;
  binIndexNS = NULL;
  prob = NULL;
}

void TransitionSamples::setTransitionNumber(int T){
  Transitions=T;
  nextState  =  new State[Transitions];
  reward =  new double[Transitions];
  binIndexNS = new int[Transitions];
}

void TransitionSamples::computeTransitionProbabilities(int b){

  B=b;
  int i;

  prob = new double[B];
  if (prob==NULL){
    cout << "Error allocating array prob" << endl;
    exit(EXIT_FAILURE);
  }

  for(i=0; i<B; i++)
    prob[i]=0;

  for(i=0; i<Transitions; i++)
    prob[binIndexNS[i]]++;

  for(i=0; i<B; i++)
    prob[i]=prob[i]/(double)Transitions;

}

TransitionSamples::~TransitionSamples(){
  delete [] nextState;
  delete [] reward;
  delete [] binIndexNS;
  delete [] prob;
}

////////////////////////////////////////////////////////////////////

Attributes::Attributes(int N, double c){
  Entropy=new double[N];
  n=N;
  Controllability=-1;
  RiskFactor=-1;
  RewardVariance=-1;
  TransitionDistance=-1;
  TransitionVariability=-1;
  RFconst=c;
}

Attributes::Attributes(){
  Entropy=NULL;

  n=0;
  Controllability=-1;
  RiskFactor=-1;
  RewardVariance=-1;
  TransitionDistance=-1;
  TransitionVariability=-1;
  RFconst=0; 
}

void Attributes::setParameters(int N, double c){

  Entropy=new double[N];
  n=N;
  Controllability=-1;
  RiskFactor=-1;
  RewardVariance=-1;
  TransitionDistance=-1;
  TransitionVariability=-1;
  RFconst=c;
}

Attributes::~Attributes(){

  delete [] Entropy;
}

////////////////////////////////////////////////////////////////////

Environment::Environment()
{
  //initiate random number generator
  if (seeded==false){
    //cout << "New seed" << endl;
    time_t stime;
    time(&stime);
    struct tm* currentTime;
    currentTime=gmtime(&stime);

    idum=-(currentTime->tm_hour+1)*currentTime->tm_min*currentTime->tm_sec;
    ran1(&idum);
    seeded=true;
  }
}

void Environment::getStateSpaceBounds(double* left, double* right){
  int i;
  bool bounded;

  for (i=0; i<State::dimensionality; i++){
    bound(i, bounded, left[i], right[i]);
    if (bounded==false){
      cout << "Error (environment): " << i << "th variable is not bounded" << endl;
      exit(EXIT_FAILURE);
    }
  }
}



/////////////

void Environment::computeAttributes(Attributes& att, const State& startState, int Steps, int Transitions, const int* n, const ActionSet& as, StateActionFA* fa=NULL){
  

  //discretize state space into some number of bins
  int B=1;  //total number of bins
  double* h = new double[State::dimensionality];
  double* left = new double[State::dimensionality];
  double* right = new double[State::dimensionality];
  int* IndCoef = new int[State::dimensionality];
  int i, j, k, d;
  bool bounded;
  int SampleSize=0; //true sample size - length of the trijectory
  IndCoef[State::dimensionality-1]=1;
  for (i=State::dimensionality-1; i>=0; i--){
    bound(i, bounded, left[i], right[i]);
    h[i]=(right[i]-left[i])/n[i];
    B=B*n[i];
    if (i<(State::dimensionality-1))
      IndCoef[i]=IndCoef[i+1]*n[i+1];
  }
		
  //Collect a sample of transitions
  TransitionSamples** sample = new TransitionSamples*[Steps];
  for (i=0; i<Steps; i++)
    sample[i] = new TransitionSamples[as.size];
  int* ActionsOnTraj = new int[Steps];
			
  bool terminal;
  State s;
  int index, ind;
  int diag;
  State CurrentStateOnTraj;
  Action CurrentActionOnTraj;
  double reward;

  CurrentStateOnTraj=startState;
		


  for(i=0; i<Steps; i++){
	   	
			
    for (j=0; j<as.size; j++){
      sample[i][j].currentState=CurrentStateOnTraj;
      sample[i][j].setTransitionNumber(Transitions);
      sample[i][j].a=as.action[j];

      for(k=0; k<Transitions; k++){
			    
	setState(sample[i][j].currentState, terminal);
	transition(sample[i][j].a, sample[i][j].nextState[k], sample[i][j].reward[k], terminal);
					
					
	//compute index of the bin for the next state
	index=0;
	for(d=0; d<State::dimensionality; d++){
			      
	  if (sample[i][j].nextState[k].x[d]==right[d]){ ind=n[d]-1; diag=0;}
	  else{
	    ind=(int)(float)((sample[i][j].nextState[k].x[d]-left[d])/h[d]);
	    diag=1;
	    if (ind == n[d]) ind=n[d]-1;
				
	  }
	  index=index+IndCoef[d]*ind;
	}
		
	if ((index<0) || (index>=B)) {
	  cout << "Error1 (Environment-attributes): bin index out of limits" << endl;
	  cout << "diagnostic=" << diag << endl;
	  cout << "State = " << sample[i][j].nextState[k] << endl;
	  cout << "index=" << index << " B=" << B << endl;
	  exit(EXIT_FAILURE);
	}

	sample[i][j].binIndexNS[k]=index;

      }
    }
			
    chooseAction(0,fa,as,CurrentStateOnTraj,CurrentActionOnTraj);
    ActionsOnTraj[i]=CurrentActionOnTraj.id;
    setState(CurrentStateOnTraj, terminal);
    transition(CurrentActionOnTraj, CurrentStateOnTraj, reward, terminal);
    SampleSize++;
    if (terminal==true) break;
  }

  //cout << "samples gathered" << endl;

  //Compute attributes based on that sample (everything except multi step entropy and size)

  for (i=0; i<SampleSize; i++){
    for(j=0; j<as.size; j++){
      sample[i][j].computeTransitionProbabilities(B);

    }
  }

  //Compute Entropy
  att.Entropy[0]=0;
  double saEntropy;
  for (i=0; i<SampleSize; i++){
    j=ActionsOnTraj[i];
    saEntropy=0;
    for(k=0; k<B; k++){
      if (sample[i][j].prob[k] !=0 )
	saEntropy-= sample[i][j].prob[k]*log10(sample[i][j].prob[k])/(double)log10(2.0);
    }
    att.Entropy[0]+=saEntropy;
			
  }
		
  att.Entropy[0]=att.Entropy[0]/(double)(SampleSize);

  //Compute controllability
  att.Controllability=0;
  double condEntropyS;
  double uncondEntropyS;
  double prob;

  for(i=0; i<SampleSize; i++){
    uncondEntropyS=0;
    condEntropyS=0;

    for(k=0; k<B; k++){
      prob=0;
      for(j=0; j<as.size; j++){
	prob+=sample[i][j].prob[k];
	if (sample[i][j].prob[k]!=0)
	  condEntropyS-=sample[i][j].prob[k]*log10(sample[i][j].prob[k])/(double)log10(2.0);
      }
      prob=prob/(double)(as.size);
      if (prob!=0)
	uncondEntropyS-=prob*log10(prob)/(double)log10(2.0);
    }
    condEntropyS=condEntropyS/(double)(as.size);
    if (uncondEntropyS!=0)
      att.Controllability+=(uncondEntropyS-condEntropyS)/uncondEntropyS;
    else att.Controllability+=1;

  }
  att.Controllability=att.Controllability/(double)(SampleSize);


		
  //compute risk factor
  double avrRewardS;
  int lowRewardS;
  att.RiskFactor=0;
		
  for(i=0; i<SampleSize; i++){
    avrRewardS=0;
    for(j=0; j<as.size; j++){
      for(k=0; k<Transitions; k++){
	avrRewardS+=sample[i][j].reward[k];
      }
    }
    avrRewardS=avrRewardS/(double)(as.size*Transitions);
    lowRewardS=0;
    for(j=0; j<as.size; j++){
      for(k=0; k<Transitions; k++){
	if (sample[i][j].reward[k]<att.RFconst*avrRewardS) lowRewardS++;
      }
    }
    att.RiskFactor+=lowRewardS/(double)(as.size*Transitions);
  }
  att.RiskFactor=att.RiskFactor/(double)SampleSize;

  //Compute RewardVariance

  double* avrBinReward = new double[B];
  if (avrBinReward == NULL){
    cout << "Error allocating array avrBinReward" << endl;
    exit(EXIT_FAILURE);
  }

  double* ssBinReward = new double[B];
  if (ssBinReward == NULL){
    cout << "Error allocating array ssBinReward" << endl;
    exit(EXIT_FAILURE);
  }

  double* dfBinReward = new double[B];
  if (dfBinReward == NULL){
    cout << "Error allocating array dfBinReward" << endl;
    exit(EXIT_FAILURE);
  }

  int b;
  double temp;
  double avrVarianceSA;
  att.RewardVariance=0;
		
  for(i=0; i<SampleSize; i++){
    j=ActionsOnTraj[i];

    avrVarianceSA=0;
    for (b=0; b<B; b++){
      avrBinReward[b]=0;
      ssBinReward[b]=0;
      dfBinReward[b]=0;
    }

    for(k=0; k<Transitions; k++){
      avrBinReward[sample[i][j].binIndexNS[k]]+=sample[i][j].reward[k];
      dfBinReward[sample[i][j].binIndexNS[k]]++;
    }

    for(k=0; k<Transitions; k++){
      if (dfBinReward[sample[i][j].binIndexNS[k]]!=0){
	temp=sample[i][j].reward[k]-
	  avrBinReward[sample[i][j].binIndexNS[k]]/(double)(dfBinReward[sample[i][j].binIndexNS[k]]);
	ssBinReward[sample[i][j].binIndexNS[k]]+=temp*temp;
      }
    }

    for(b=0; b<B; b++){
      if (dfBinReward[b]!=0)
	avrVarianceSA+=ssBinReward[b]/(double)(dfBinReward[b]);
    }
    avrVarianceSA=avrVarianceSA/(double)B;
    att.RewardVariance+=avrVarianceSA;
			
  }
  att.RewardVariance=att.RewardVariance/(double)(SampleSize);
		
  delete [] avrBinReward;
  delete [] ssBinReward;
  delete [] dfBinReward;


  //compute TransitionDistance
  double avrDistanceSA;
  double distSS, dist;
  att.TransitionDistance=0;
  for(i=0; i<SampleSize; i++){
    j=ActionsOnTraj[i];
    avrDistanceSA=0;
    for(k=0; k<Transitions; k++){
      distSS=0;
      for(d=0; d<State::dimensionality; d++){
	dist=sample[i][j].currentState.x[d]-sample[i][j].nextState[k].x[d];
	distSS+=dist*dist;
      }
      avrDistanceSA+=sqrt(distSS);
    }
    avrDistanceSA=avrDistanceSA/(double)Transitions;
    att.TransitionDistance+=avrDistanceSA;
			
  }
  att.TransitionDistance=att.TransitionDistance/(double)(SampleSize);

  //Compute transition variability (average largest distance between possible next states)

  double VariabilitySA;
  int t;
  att.TransitionVariability=0;
	
  for(i=0; i<SampleSize; i++){
    j=ActionsOnTraj[i];
    VariabilitySA=0;
    for(k=0; k<Transitions; k++){
      t=rand()%Transitions;
      distSS=0;
      for(d=0; d<State::dimensionality; d++){
	dist=sample[i][j].nextState[k].x[d]-sample[i][j].nextState[t].x[d];
	distSS+=dist;
      }
      distSS=sqrt(distSS);
      if (distSS > VariabilitySA)
	VariabilitySA=distSS;

    }
    att.TransitionVariability+=VariabilitySA;
			
  }
  att.TransitionVariability=att.TransitionVariability/(double)(SampleSize);

  /*	//Compute Multistep Entropy
	if (att.n>1){
	for(i=2; i<=att.n; i++){
	att.Entropy[i-1]=multiStepEntropy(i,SampleSize, Transitions, n, as);
	}
	}

  */
  
  //delete sample

  delete [] h;
  delete [] left;
  delete [] right;
  delete [] IndCoef;
  for(i=0; i<SampleSize; i++)
    delete [] sample[i];
  delete [] sample;
  delete [] ActionsOnTraj;
		
}
/////////////////////
void Environment::chooseAction(double epsilon, StateActionFA* fa, const ActionSet& actions, const State& s, Action& a)
  /* Implements an epsilon-greedy policy
	*/
{
  int NumberAA=0;		//number of applicable actions
  int i;
  int id;	//selected action id
  int* ApplicableActions = new int[actions.size];
	
  for (i=0;i<actions.size;i++)
    if (applicable(s,actions.action[i])==true)
      {	ApplicableActions[NumberAA]=i;
      NumberAA++;
      }

  if (NumberAA==0) 
    {	cout << "No action can be taken in the current state " << endl;
    exit(EXIT_FAILURE);
    }

  if (fa==NULL) epsilon=1.0; //make sure random action will be selected

  if ((double)rand()/(double)RAND_MAX <= epsilon) 
    //take any action uniformenly
    {	
      id=ApplicableActions[rand()%NumberAA];
      a=actions.action[id];
    }
  else //select greedy ection
    {	
      double* Values, BestValue;
      Action b;
      int NumberGreedyActions, gr;
      int* GreedyActions;

      Values = new double[NumberAA];
      GreedyActions = new int[NumberAA];

      for (i=0; i<NumberAA; i++){
	id=ApplicableActions[i];
	a=actions.action[id];
	fa->predict(a, s, Values[i]);
      }
      BestValue=Values[0];
      NumberGreedyActions=1;
      GreedyActions[0]=ApplicableActions[0];

      for (i=1; i<NumberAA; i++){
	if (Values[i]>BestValue){
	  BestValue=Values[i];
	  NumberGreedyActions=1;
	  GreedyActions[0]=ApplicableActions[i];
	}
	if (Values[i]==BestValue){
	  NumberGreedyActions++;
	  GreedyActions[NumberGreedyActions-1]=ApplicableActions[i];
	}
      }
		
      gr=rand()%NumberGreedyActions;
      a=actions.action[GreedyActions[gr]];

      delete [] Values;
      delete [] GreedyActions;
      delete [] ApplicableActions;
		
    }
}


////

void Environment::computeAttributes(Attributes& att, int SampleSize, int Transitions, const int* n, const ActionSet& as){

  //discretize state space into some number of bins
  int B=1;  //total number of bins
  double* h = new double[State::dimensionality];
  double* left = new double[State::dimensionality];
  double* right = new double[State::dimensionality];
  int* IndCoef = new int[State::dimensionality];
  int i, j, k, d;
  bool bounded;
		
  IndCoef[State::dimensionality-1]=1;
  for (i=State::dimensionality-1; i>=0; i--){
    bound(i, bounded, left[i], right[i]);
    h[i]=(right[i]-left[i])/n[i];
    B=B*n[i];
    if (i<(State::dimensionality-1))
      IndCoef[i]=IndCoef[i+1]*n[i+1];
  }
		
  //Collect a sample of transitions
  TransitionSamples** sample = new TransitionSamples*[SampleSize];
  for (i=0; i<SampleSize; i++)
    sample[i] = new TransitionSamples[as.size];
			
  bool terminal;
  State s;
  int index, ind;
  int diag;

  for(i=0; i<SampleSize; i++){
			
    uniformStateSample(s);
					
    for (j=0; j<as.size; j++){
      sample[i][j].currentState=s;
      sample[i][j].setTransitionNumber(Transitions);
      sample[i][j].a=as.action[j];

      for(k=0; k<Transitions; k++){
	setState(sample[i][j].currentState, terminal);
	transition(sample[i][j].a, sample[i][j].nextState[k], sample[i][j].reward[k], terminal);
					
					
	//compute index of the bin for the next state
	index=0;
				

	for(d=0; d<State::dimensionality; d++){
					  
	  if (sample[i][j].nextState[k].x[d]==right[d]){ ind=n[d]-1; diag=0;}
	  else{
	    ind=(int)(float)((sample[i][j].nextState[k].x[d]-left[d])/h[d]);
	    diag=1;
	    if (ind == n[d]) ind=n[d]-1;
							
	  }
	  index=index+IndCoef[d]*ind;
	}
		
	if ((index<0) || (index>=B)) {
	  cout << "Error1 (Environment-attributes): bin index out of limits" << endl;
	  cout << "diagnostic=" << diag << endl;
	  cout << "State = " << sample[i][j].nextState[k] << endl;
	  cout << "index=" << index << " B=" << B << endl;
	  exit(EXIT_FAILURE);
	}

	sample[i][j].binIndexNS[k]=index;

      }
    }
  }

  //Compute attributes based on that sample (everything except multi step entropy and size)

  for (i=0; i<SampleSize; i++){
    for(j=0; j<as.size; j++){
      sample[i][j].computeTransitionProbabilities(B);

    }
  }

  //Compute Entropy
  att.Entropy[0]=0;
  double saEntropy;
  for (i=0; i<SampleSize; i++){
    for(j=0; j<as.size; j++){
      saEntropy=0;
      for(k=0; k<B; k++){
	if (sample[i][j].prob[k] !=0 )
	  saEntropy-= sample[i][j].prob[k]*log10(sample[i][j].prob[k])/(double)log10(2.0);
      }
      att.Entropy[0]+=saEntropy;
    }
  }
		
  att.Entropy[0]=att.Entropy[0]/(double)(SampleSize*as.size);

  //Compute controllability
  att.Controllability=0;
  double condEntropyS;
  double uncondEntropyS;
  double prob;

  for(i=0; i<SampleSize; i++){
    uncondEntropyS=0;
    condEntropyS=0;

    for(k=0; k<B; k++){
      prob=0;
      for(j=0; j<as.size; j++){
	prob+=sample[i][j].prob[k];
	if (sample[i][j].prob[k]!=0)
	  condEntropyS-=sample[i][j].prob[k]*log10(sample[i][j].prob[k])/(double)log10(2.0);
      }
      prob=prob/(double)(as.size);
      if (prob!=0)
	uncondEntropyS-=prob*log10(prob)/(double)log10(2.0);
    }
    condEntropyS=condEntropyS/(double)(as.size);
    if (uncondEntropyS!=0)
      att.Controllability+=(uncondEntropyS-condEntropyS)/uncondEntropyS;
    else att.Controllability+=1;

  }
  att.Controllability=att.Controllability/(double)SampleSize;


		
  //compute risk factor
  double avrRewardS;
  int lowRewardS;
  att.RiskFactor=0;
		
  for(i=0; i<SampleSize; i++){
    avrRewardS=0;
    for(j=0; j<as.size; j++){
      for(k=0; k<Transitions; k++){
	avrRewardS+=sample[i][j].reward[k];
      }
    }
    avrRewardS=avrRewardS/(double)(as.size*Transitions);
    lowRewardS=0;
    for(j=0; j<as.size; j++){
      for(k=0; k<Transitions; k++){
	if (sample[i][j].reward[k]<att.RFconst*avrRewardS) lowRewardS++;
      }
    }
    att.RiskFactor+=lowRewardS/(double)(as.size*Transitions);
  }
  att.RiskFactor=att.RiskFactor/(double)SampleSize;

  //Compute RewardVariance

  double* avrBinReward = new double[B];
  if (avrBinReward == NULL){
    cout << "Error allocating array avrBinReward" << endl;
    exit(EXIT_FAILURE);
  }

  double* ssBinReward = new double[B];
  if (ssBinReward == NULL){
    cout << "Error allocating array ssBinReward" << endl;
    exit(EXIT_FAILURE);
  }

  double* dfBinReward = new double[B];
  if (dfBinReward == NULL){
    cout << "Error allocating array dfBinReward" << endl;
    exit(EXIT_FAILURE);
  }

  int b;
  double temp;
  double avrVarianceSA;
  att.RewardVariance=0;
		
  for(i=0; i<SampleSize; i++){
    for(j=0; j<as.size; j++){

      avrVarianceSA=0;
      for (b=0; b<B; b++){
	avrBinReward[b]=0;
	ssBinReward[b]=0;
	dfBinReward[b]=0;
      }

      for(k=0; k<Transitions; k++){
	avrBinReward[sample[i][j].binIndexNS[k]]+=sample[i][j].reward[k];
	dfBinReward[sample[i][j].binIndexNS[k]]++;
      }

      for(k=0; k<Transitions; k++){
	if (dfBinReward[sample[i][j].binIndexNS[k]]!=0){
	  temp=sample[i][j].reward[k]-
	    avrBinReward[sample[i][j].binIndexNS[k]]/(double)(dfBinReward[sample[i][j].binIndexNS[k]]);
	  ssBinReward[sample[i][j].binIndexNS[k]]+=temp*temp;
	}
      }

      for(b=0; b<B; b++){
	if (dfBinReward[b]!=0)
	  avrVarianceSA+=ssBinReward[b]/(double)(dfBinReward[b]);
      }
      avrVarianceSA=avrVarianceSA/(double)B;
      att.RewardVariance+=avrVarianceSA;
    }
  }
  att.RewardVariance=att.RewardVariance/(double)(SampleSize*as.size);
		
  delete [] avrBinReward;
  delete [] ssBinReward;
  delete [] dfBinReward;


  //compute TransitionDistance
  double avrDistanceSA;
  double distSS, dist;
  att.TransitionDistance=0;
  for(i=0; i<SampleSize; i++){
    for(j=0; j<as.size; j++){
      avrDistanceSA=0;
      for(k=0; k<Transitions; k++){
	distSS=0;
	for(d=0; d<State::dimensionality; d++){
	  dist=sample[i][j].currentState.x[d]-sample[i][j].nextState[k].x[d];
	  distSS+=dist*dist;
	}
	avrDistanceSA+=sqrt(distSS);
      }
      avrDistanceSA=avrDistanceSA/(double)Transitions;
      att.TransitionDistance+=avrDistanceSA;
    }
  }
  att.TransitionDistance=att.TransitionDistance/(double)(SampleSize*as.size);

  //Compute transition variability (average largest distance between possible next states)

  double VariabilitySA;
  int t;
  att.TransitionVariability=0;
	
  for(i=0; i<SampleSize; i++){
    for(j=0; j<as.size; j++){
      VariabilitySA=0;
      for(k=0; k<Transitions; k++){
	t=rand()%Transitions;
	distSS=0;
	for(d=0; d<State::dimensionality; d++){
	  dist=sample[i][j].nextState[k].x[d]-sample[i][j].nextState[t].x[d];
	  distSS+=dist;
	}
	distSS=sqrt(distSS);
	if (distSS > VariabilitySA)
	  VariabilitySA=distSS;

      }
      att.TransitionVariability+=VariabilitySA;
    }
  }
  att.TransitionVariability=att.TransitionVariability/(double)(SampleSize*as.size);

  //Compute Multistep Entropy
  if (att.n>1){
    for(i=2; i<=att.n; i++){
      att.Entropy[i-1]=multiStepEntropy(i,SampleSize, Transitions, n, as);
    }
  }

  //delete sample

  delete [] h;
  delete [] left;
  delete [] right;
  delete [] IndCoef;
  for(i=0; i<SampleSize; i++)
    delete [] sample[i];
  delete [] sample;
		
}

////


double Environment::multiStepEntropy(int N, int sampleSize, int Transitions, const int* n, const ActionSet& as)
{
		
  if (N==0) return 0;

  int i, j, k, s, q;
  int B;
  int A=(int)(pow(as.size, N));
  int* seq = new int[N];
  double* h = new double[State::dimensionality];
  double* left = new double[State::dimensionality];
  double* right = new double[State::dimensionality];
  int* bin;
  int* IndCoef = new int[State::dimensionality];
  double Entropy=0;
  bool bounded;


  //discretize state space into some number of bins
  B=1;
  IndCoef[State::dimensionality-1]=1;
  for (i=State::dimensionality-1; i>=0; i--){
    bound(i, bounded, left[i], right[i]);
    h[i]=(right[i]-left[i])/n[i];
    B=B*n[i];
    if (i<(State::dimensionality-1))
      IndCoef[i]=IndCoef[i+1]*n[i+1];
  }
  bin=new int[B];
  if (bin==NULL){
    cout << "Multi-step entropy: Allocation of bin array was not successfull" << endl;
    return -1;
  }

  //select a sample of states and compute entropy

  State CurState, NewState;
  bool terminal;
  double reward;
  int ind, index;
  double saEntropy;
  double prob;
  int diag;

  for (i=0; i<sampleSize; i++){
    for (j=0; j<State::dimensionality; j++)
      CurState.x[j]=ran1(&idum);
	
    for (k=0; k<A; k++){
      actionSequence(k, N, as.size, seq);
      for (j=0; j<B; j++)
	bin[j]=0;
      saEntropy=0;

      for (s=0; s<Transitions; s++){

	setState(CurState, terminal);	
	for (q=0; q<N; q++){
	  transition(as.action[seq[q]], NewState, reward, terminal);
	  if (terminal==true) break;
	}
				
	index=0;
	for(j=0; j<State::dimensionality; j++){
	  if (NewState.x[j]==right[j]){ ind=n[j]-1; diag=0;}
	  else{
	    ind=(int)(float)((NewState.x[j]-left[j])/h[j]);
	    diag=1;
	    if (ind == n[j]) ind=n[j]-1;
	  }
	  index=index+IndCoef[j]*ind;
	}
		
	if ((index<0) || (index>=B)) {
	  cout << "Error (RMDP-multi-step-entropy): bin index out of limits" << endl;
	  cout << "diagnostic=" << diag << endl;
	  cout << "State = " << NewState << endl;
	  cout << "index=" << index << " B=" << B << endl;
	  exit(EXIT_FAILURE);
	}
	bin[index]++;
      }

      for (j=0; j<B; j++){
	prob=((double)bin[j])/((double)Transitions);
	if (prob!=0){
	  saEntropy-=prob*log10(prob)/((double)log10(2.0));
	}
      }
	
      Entropy+=saEntropy;
    }
  }

  Entropy=Entropy/((double)(sampleSize*A));

  delete [] h;
  delete [] left;
  delete [] right;
  delete [] bin;
  delete [] IndCoef;
  delete [] seq;

  return Entropy;
}

//////

void Environment::actionSequence(int num, int n, int as_size, int* seq)
{
  int i;
  int digit;

  i=n-1;
  while( (num>=as_size)&& (i>=0)){
    digit=num%as_size;
    num=num/as_size;
    seq[i]=digit;
    i--;
  }
  if (i>=0) seq[i]=num;
  i--;
  while (i>=0){
    seq[i]=0;
    i--;
  }
}


