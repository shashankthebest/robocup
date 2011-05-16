/*! @file LocaliseSelfState.h
    @brief A state where the localisation of robot is performed

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

#ifndef LOCALISESELFSTATE_H
#define LOCALISESELFSTATE_H

#include "ActiveLocalisationProvider.h"
#include "ActiveLocalisationState.h"

#include "Infrastructure/Jobs/JobList.h"
#include "Infrastructure/NUSensorsData/NUSensorsData.h"
#include "Infrastructure/NUActionatorsData/NUActionatorsData.h"
#include "Infrastructure/FieldObjects/FieldObjects.h"

#include "Infrastructure/Jobs/MotionJobs/WalkJob.h"
#include "Infrastructure/Jobs/MotionJobs/HeadJob.h"
#include "Infrastructure/Jobs/MotionJobs/HeadTrackJob.h"
#include "Tools/Math/General.h"

#include "debug.h"
using namespace mathGeneral;

class LocaliseSelfState : public ActiveLocalisationState
{
public:
    LocaliseSelfState(ActiveLocalisationProvider* parent) : ActiveLocalisationState(parent) {};
    virtual ~LocaliseSelfState() {};

    virtual BehaviourState* nextState()
    {

        tick();
        if(m_time_in_state - m_parent->fixedDuration*1000 > 0)
            return m_parent->m_lineUp;
        else
            return this;
    };


    virtual void doState()
    {
        #if DEBUG_BEHAVIOUR_VERBOSITY > 3
            debug << "LocaliseSelfState::doState" << endl;
        #endif

        vector<float> goalPosition;
        goalPosition = m_parent->m_globalMap->findObservation(1);
        float elevation = 0;
        if (m_parent->stateChanged())
        {
            cout<<"\nReached localise self state\n\n";
            tick();
            ///TODO: Change this to nearest landmark, or the landmark learned by rl



            cout<<"\nCurrent Position    [ "<<m_current_position[0]<<" , "<<m_current_position[1]<<" , "<<rad2deg(normaliseAngle(m_current_position[2]))<<" ] ";
            cout<<"\nFound Goal position [ "<<goalPosition[0]<<" , "<<goalPosition[1]<<" , "<<rad2deg(normaliseAngle(goalPosition[2]))<<" ] \n\n";
            //lookAtGoals();
            elevation = atan2( sqrt( goalPosition[0]*goalPosition[0] +  goalPosition[1]*goalPosition[1] ) , 55);
            cout<<"\nCalculated elevation : "<<elevation<<"\n";
            m_jobs->addMotionJob(new WalkJob(0,0,0));
             m_jobs->addMotionJob(new HeadTrackJob(elevation ,goalPosition[2],0,0));
        }



          //  vector<float> zero(m_actions->getSize(NUActionatorsData::Head), 0);
           // m_jobs->addMotionJob(new HeadJob(m_actions->CurrentTime + 500, zero));
    };
};

#endif

