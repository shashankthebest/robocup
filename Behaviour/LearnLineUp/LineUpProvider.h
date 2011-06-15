/*! @file ActiveLocalisationProvider.h
    @brief Declaration of ActiveLocalisation provider to learn efficient lineup

    @class ActiveLocalisationProvider
    @brief An RL based Learn to Lineup behaviour provider

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

#ifndef LINEUPPROVIDER_H
#define LINEUPPROVIDER_H

#include "Behaviour/BehaviourFSMProvider.h"
#include "Tools/Optimisation/Parameter.h"
#include "Infrastructure/map/occupancyGridMap.h"
#include "Infrastructure/FieldObjects/FieldObjects.h"



class LineUpState;
class Optimiser;


#include <vector>
#include <string>
#include <fstream>
using namespace std;

class LineUpProvider : public BehaviourFSMProvider
{
public:
    LineUpProvider(Behaviour* manager);
    ~LineUpProvider();
    void tickRL();


protected:
    BehaviourState* nextStateCommons();
    void doBehaviourCommons();
private:
    void initOptimiser();
    float normalDistribution(float mean, float sigma);
public:
    LineUpState* m_generate;        //!< New parameters for episode evaluation are generated
    LineUpState* m_evaluate;        //!< After execution of episode, evaluation is performed
    LineUpState* m_pause;           //!< Episode is paused
    LineUpState* m_localiseSelf;    //!< Robot looks for landmarks in order to localise
    LineUpState* m_localiseBall;    //!< Robot looks for ball
    LineUpState* m_lineUp;          //!< Approach the ball and perform a lineup
    LineUpState* m_kick;            //!< Kick the ball into the goal


    float fixedBallDistance;
    float fixedDuration;
    float landMarkId;
    float ballCertainty;
    OccupancyGridMap *m_globalMap;
    ofstream locAcc;

private:


    string m_id;								//!< the name of RL algo
    Optimiser* m_optimiser;                     //!< the RL algorithm



    float calculateFitness();					//!< calculates the fitness of the current episode
    vector<float> calculateFitnesses();			//!< calculates all of the fitnesses of the episode
    int m_iteration_count;						//!< the number of times the rl optimiser has been ticked
    int m_fail_count;                           //!< the number of times the rl optimiser has failed

    ofstream m_log;
};


#endif

