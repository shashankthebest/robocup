/*! @file ActiveLocalisationProvider.cpp
    @brief Implementation of ActiveLocalisation behaviour class

    @author Shashank Bhatia

 Copyright (c) 2011 Shashank Bhatia

 This file is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 This file is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with NUbot.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "LineUpProvider.h"


#include "LineUpSetupState.h"
#include "LineUpEvaluateState.h"
#include "LineUpPauseState.h"
#include "LineUpWalkToBall.h"


#include "Tools/Optimisation/EHCLSOptimiser.h"
#include "Tools/Optimisation/PGRLOptimiser.h"
#include "Tools/Optimisation/PSOOptimiser.h"
#include "Tools/Math/StlVector.h"

#include "Infrastructure/Jobs/JobList.h"
#include "Infrastructure/NUSensorsData/NUSensorsData.h"
#include "Infrastructure/NUActionatorsData/NUActionatorsData.h"
#include "Infrastructure/FieldObjects/FieldObjects.h"
#include "NUPlatform/NUPlatform.h"
#include "Infrastructure/NUBlackboard.h"
#include "Infrastructure/GameInformation/GameInformation.h"

#include "Infrastructure/Jobs/MotionJobs/HeadJob.h"
#include "Infrastructure/Jobs/MotionJobs/HeadPanJob.h"
#include "Infrastructure/Jobs/MotionJobs/HeadNodJob.h"
#include "Infrastructure/Jobs/MotionJobs/WalkJob.h"
#include "Infrastructure/Jobs/MotionJobs/WalkParametersJob.h"

#include "debug.h"
#include "debugverbositybehaviour.h"
#include "nubotconfig.h"
#include "nubotdataconfig.h"
#include "targetconfig.h"

#include <boost/random.hpp>
#include <sys/types.h>
#include <unistd.h>
#include <signal.h>


LineUpProvider::LineUpProvider(Behaviour* manager) : BehaviourFSMProvider(manager)
{
    #if DEBUG_BEHAVIOUR_VERBOSITY > 1
        debug << "LearnLineUpProvider::LearnLineUpProvider" << endl;
    #endif
    initOptimiser();

 //   if (not m_optimiser)
 //       m_log.open((DATA_DIR + "/Optimisation/" + m_parameters.getName() + ".log").c_str(), fstream::out);
 //   else
  //      m_log.open((DATA_DIR + "/Optimisation/" + m_id + "Log.log").c_str(), fstream::out | fstream::app);

    //m_generate     = new LineUpSetupState(this);
   	
   	//m_evaluate     = new LineUpEvaluateState(this);
    
    m_pause        = new LineUpPauseState(this);

    //m_localiseSelf = new LocaliseSelfState(this);
    
    //m_lineUp       = new TryLineUpState(this);
    //m_kick         = new TryKickState(this);
    m_globalMap    = new OccupancyGridMap(-300,300,-200,200,5,2);  // Map of robocup dimensions

    locAcc.open("localisationAccuracy.log");

 
    m_globalMap->initializeMap();
    m_globalMap->insertObservation(300,0,1, 0.99); // yellow goal center
    m_globalMap->insertObservation(300,-70,2, 0.99); // yellow goal post right
    m_globalMap->insertObservation(300,70,2, 0.99); // yellow goal post left
    m_globalMap->insertObservation(-300,70,3, 0.99); // blue goal post left
    m_globalMap->insertObservation(-300,-70,4, 0.99); // blue goal post right
    m_globalMap->insertObservation(-300,0,5, 0.99);  // blue goal center

    m_state = m_pause;




    fixedBallDistance = 100;
    fixedDuration = 5;
    landMarkId = 0;

    m_iteration_count = 0;
	setupRlEngine();
}

void LineUpProvider::setupRlEngine()
{

/////////////  Parameters for RL agent
  mainP = new MainParameters();
  char* params[] = {"Trials=10000","steps=1000","str=cmac.unx","tf=5000","tsn=1","tsf=teststates.unx","ts=1","dir=learnData.hst"};
  mainP->process(8,params);
	
	
 ///////////////////// Setting up random number generator
  //seed randon number generator
  
  time_t stime;
  time(&stime);
  struct tm* currentTime;
  currentTime=gmtime(&stime);
  unsigned seed;
  seed=(currentTime->tm_hour+1)*currentTime->tm_min*currentTime->tm_sec;
  srand(seed);
 
 /////////////////////////// Setting up states and actions
 
  //state space and actions for the mountain car task
	
  State::dimensionality=4;

  ActionSet actionSet(6); // incTrans, decTrans, incDir, decDir, incRot, decRot
  Action a1("incTrans", 1.0);
  actionSet.addAction(a1);
  Action a2("decTrans", 2 );
  actionSet.addAction(a2);
  Action a3("incDir", 3);
  actionSet.addAction(a3);
  Action a4("decDir", 4);
  actionSet.addAction(a3);
  Action a5("incRot", 5);
  actionSet.addAction(a5);
  Action a6("decRot", 6);
  actionSet.addAction(a6);


////////////////       Defining the environment
  mdp = new GoalLineUp();
  env = mdp;
///////////////   Set up function approximator
  cmacSet = new Approximator*[Action::count];
  
//////////// Set up state space bounds
  left = new double[State::dimensionality];
  right = new double[State::dimensionality];
  env->getStateSpaceBounds(left, right);  // get bounds from environment
  CMAC::setInputBounds(left,right);  // set bounds on tiles

////////////// Get tiling architecture from file

  for(int i=0; i<Action::count; i++)
  {
    cmacSet[i] = new CMAC("cmac.dat");
  }
  
/////////////// Finalising State-Action FA
  safa = new StateActionFA(Action::count, cmacSet);
  char *d[] = {"schedule=constant","alpha=0.5"};
  for(int i=0; i<Action::count; i++)
  {
    cmacSet[i]->setLearningParameters(2, d);
  }  
  
//////////////  Setup Agent
                          //discount factor ,  Possible actions   Approximator		Environment
  sarsa = new SarsaAgentRT(    1,				   actionSet,			safa,			env);
  agent = sarsa;
  agent->setLearningParameters(2, d);
  
  m_localiseBall = new LineUpWalkToBallState(this,mainP, safa, agent, true);

}


LineUpProvider::~LineUpProvider()
{
    delete m_generate;
    delete m_evaluate;
    delete m_pause;
    
    delete m_localiseBall;
    delete m_globalMap;
    
    locAcc.close();
}
 
/*! @brief Handles state transitions common to all states
 * 				- In webots this includes the timely termination of the simulation, and the automatix transition from paused to generate
 * 				- On a real robot this handles the automatic transition from paused to generate
 */
BehaviourState* LineUpProvider::nextStateCommons()
{
    while (Blackboard->GameInfo->getCurrentState() != GameInformation::PlayingState)
        m_game_info->doManualStateChange();

    #ifdef TARGET_IS_NAOWEBOTS
		if (not mdp and Platform->getTime() > 20*60*1e3)
		{
			kill(getppid(), SIGKILL);
			kill(getpid(), SIGKILL);
		}
		else if (m_iteration_count  > 2000)
		{
			kill(getppid(), SIGKILL);
			kill(getpid(), SIGKILL);
		}

		if (m_state == m_pause and Platform->getTime() > 50)
			{
			    cout<<"\nreturned generate\n";
			    return m_generate;
			}
		else
			return m_state;
	#else
        if (m_state == m_paused and Platform->getTime() > 5000)
            return m_generate;
        else
            return m_state;
    #endif
}

/*! @brief Does behaviour common to all states */
void LineUpProvider::doBehaviourCommons()
{;
    //updateTime();

 /*   if (m_previous_state == m_paused and m_state == m_generate)
        m_jobs->addMotionJob(new WalkParametersJob(m_parameters));
	#ifndef USE_VISION		// if there is no vision then just fix the head in (0,0) position
		m_jobs->addMotionJob(new HeadJob(0,vector<float>(2,0)));
	#endif*/
}

    /*
void ActiveLocalisationProvider::tickOptimiser()
{


    #if DEBUG_BEHAVIOUR_VERBOSITY > 1
        debug << "WalkOptimisationProvider::tickOptimiser" << endl;
    #endif
    // update the optimiser and give the next set of parameters to the walk engine
	#ifdef USE_MO
        vector<float> fitness = calculateFitnesses();
	#else
        float fitness = calculateFitness();
	#endif
    if (m_optimiser)
    {
		m_optimiser->setParametersResult(fitness);
        vector<float> nextparameters = m_optimiser->getNextParameters();
        m_parameters.set(nextparameters);
    }
    m_jobs->addMotionJob(new WalkParametersJob(m_parameters));

    #if DEBUG_BEHAVIOUR_VERBOSITY > 1
			debug << "WalkOptimisationProvider::tickOptimiser() new parameters: " << m_parameters.getAsVector() << endl;
    #endif

    // save the state of the optimiser, and the walk parameters in case of hardware failure.
    if (m_optimiser)
        m_optimiser->save();
    m_iteration_count++;


}




vector<float> ActiveLocalisationProvider::calculateFitnesses()
{

    vector<float> fitness(2,0);
    float speed = 0;
    float cost = 0;
    bool unstable = true;

    if (not m_generate->success())
    {
		#if DEBUG_BEHAVIOUR_VERBOSITY > 2
			debug << "WalkOptimisationProvider::calculateFitness() Generate state was not successful" << endl;
		#endif
		float distance = max(10.0f, m_generate->distance());
		float duration = max(300.0f, m_generate->duration());
		float energy = max(20.0f, m_generate->energy());
		speed = 1000*distance/duration;
		cost = 100*energy/(9.81*4.6*distance);

		// penalise for falling
		speed *= 0.5*distance/333.0;
		cost /= 0.5*distance/333.0;
    }
    else if (not m_evaluate->success())
    {
		#if DEBUG_BEHAVIOUR_VERBOSITY > 2
			debug << "WalkOptimisationProvider::calculateFitness() Evaluate state was not successful" << endl;
		#endif
		float distance = max(10.0f, m_generate->distance() + m_evaluate->distance());
		float duration = max(300.0f, m_generate->duration() + m_evaluate->duration());
		float energy = max(20.0f, m_generate->energy() + m_evaluate->energy());
		speed = 1000*distance/duration;
		cost = 100*energy/(9.81*4.6*distance);

		// penalise for falling
		speed *= 0.5*distance/333.0;
		cost /= 0.5*distance/333.0;
    }
    else
    {
		#if DEBUG_BEHAVIOUR_VERBOSITY > 2
			debug << "WalkOptimisationProvider::calculateFitness() Evaluate state was successful" << endl;
		#endif
    	speed = 1000*m_evaluate->distance()/m_evaluate->duration();
    	cost = 100*m_evaluate->energy()/(9.81*4.6*m_evaluate->distance());
    	unstable = false;
    }

	#if DEBUG_BEHAVIOUR_VERBOSITY > 2
		debug << "WalkOptimisationProvider::calculateFitness() speed: " << speed << " " << cost << endl;
	#endif

    #ifdef TARGET_IS_NAOWEBOTS
        //speed *= normalDistribution(1, 0.105);      // 0.045, 0.055, 0.065
        //cost *= normalDistribution(1, 0.105);
    #endif

    fitness[0] = speed;                      	// speed--based fitness
    fitness[1] = 180/(4+cost);                  // cost--based fitness
    //fitness = 20*pow(speed,2)/(9.81*m_parameters.getAsVector()[18]);      // froude--based fitness
    m_log << m_iteration_count << ", " << fitness[1] << ", " << speed << ", " << cost << ", " << unstable << ", " << m_parameters.getAsVector() << endl << flush;

    // update fall count
    if (unstable)
        m_fall_count++;

	#if DEBUG_BEHAVIOUR_VERBOSITY > 1
        debug << "WalkOptimisationProvider::calculateFitness() " << fitness << " for " << m_parameters.getAsVector() << endl;
    #endif

    return fitness;

}

float ActiveLocalisationProvider::calculateFitness()
{

	#if defined(USE_SPEED)
		return calculateFitnesses()[0];
	#elif defined(USE_COST)
		return calculateFitnesses()[1];
	#endif

}

//! @brief Returns the distance required to stop for the current walk parameters
float ActiveLocalisationProvider::stoppingDistance()
{
    vector<float>& speeds = m_parameters.getMaxSpeeds();
    vector<float>& accels = m_parameters.getMaxAccelerations();
    float xd = pow(1.1*speeds[0],2)/(2*accels[0]);          // s = u^2/2a with a 10% margin for error
    float yd = pow(1.1*speeds[1],2)/(2*accels[1]);
    return sqrt(xd*xd + yd*yd);
}


*/

void LineUpProvider::initOptimiser()
{
    /*
	vector<Parameter> parameters = m_parameters.getAsParameters();
	#if not defined(USE_STIFFNESS)
		parameters.resize(parameters.size() - 6);           // remove the stiffnesses from the parameter set
	#endif

	#if defined(USE_EHCLS)
		m_optimiser = new EHCLSOptimiser(m_id + "EHCLS", parameters);
	#elif defined(USE_PGRL)
		m_optimiser = new PGRLOptimiser(m_id + "PGRL", parameters);
	#elif defined(USE_PSO)
		m_optimiser = new PSOOptimiser(m_id + "PSO", parameters);
	#else
		m_optimiser = NULL;
	#endif

	if (m_optimiser)
		m_parameters.set(m_optimiser->getNextParameters());
*/
}


/*! @brief Returns a normal random variable from the normal distribution with mean and sigma
 */
float LineUpProvider::normalDistribution(float mean, float sigma)
{
    static unsigned int seed = 1e6*Platform->getRealTime()*Platform->getRealTime()*Platform->getRealTime();          // I am hoping that at least one of the three calls is different for each process
    static boost::mt19937 generator(seed);                       // you need to seed it here with an unsigned int!
    static boost::normal_distribution<float> distribution(0,1);
    static boost::variate_generator<boost::mt19937, boost::normal_distribution<float> > standardnorm(generator, distribution);

    float z = standardnorm();       // take a random variable from the standard normal distribution
    float x = mean + z*sigma;       // then scale it to belong to the specified normal distribution

    return x;
}

