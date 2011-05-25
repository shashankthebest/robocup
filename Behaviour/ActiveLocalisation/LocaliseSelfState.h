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


        float elevation = 0, dist = 0;
        if (m_parent->stateChanged())
        {
            start();
            tick();
            vector<float> landmarkPosition;
            landmarkPosition = m_parent->m_globalMap->findBestObservation(1,5);

            cout<<"\nReached localise self state\n\n";

            ///TODO: Change this to the landmark learned by rl



            cout<<"\nCurrent Position    [ "<<m_current_position[0]<<" , "<<m_current_position[1]<<" , "<<rad2deg(normaliseAngle(m_current_position[2]))<<" ] ";
            cout<<"\nFound Landmark position [ "<<landmarkPosition[0]<<" , "<<landmarkPosition[1]<<" , "<<rad2deg(normaliseAngle(landmarkPosition[2]))<<" ] \n\n";
            //lookAtGoals();
            dist = sqrt( landmarkPosition[0]*landmarkPosition[0] +  landmarkPosition[1]*landmarkPosition[1] );
            elevation = atan2(  dist , 55);
            cout<<"\nCalculated elevation : "<<elevation<<"\n";
            m_jobs->addMotionJob(new WalkJob(0,0,0));
            m_jobs->addMotionJob(new HeadTrackJob(elevation ,landmarkPosition[2],0,0));
            m_jobs->addMotionJob(new HeadPanJob(HeadPanJob::Localisation, dist - 10, dist + 10, landmarkPosition[2] , landmarkPosition[2]  ));
        }




          //  vector<float> zero(m_actions->getSize(NUActionatorsData::Head), 0);
           // m_jobs->addMotionJob(new HeadJob(m_actions->CurrentTime + 500, zero));
    };
};

#endif

