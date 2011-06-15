/*! @file TryKickState.h
    @brief A state where the robot tries to kicks ball into goal

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

#ifndef TRYKICKSTATE_H
#define TRYKICKSTATE_H

#include "ActiveLocalisationProvider.h"
#include "ActiveLocalisationState.h"

#include "Infrastructure/Jobs/JobList.h"
#include "Infrastructure/NUSensorsData/NUSensorsData.h"
#include "Infrastructure/NUActionatorsData/NUActionatorsData.h"
#include "Infrastructure/FieldObjects/FieldObjects.h"

#include "Infrastructure/Jobs/MotionJobs/WalkJob.h"
#include "Infrastructure/Jobs/MotionJobs/HeadJob.h"
#include "Infrastructure/Jobs/MotionJobs/KickJob.h"

#include "debug.h"

class TryKickState : public ActiveLocalisationState
{
public:
    TryKickState(ActiveLocalisationProvider* parent) : ActiveLocalisationState(parent) {};
    virtual ~TryKickState() {};
    virtual BehaviourState* nextState() {return this;};
    virtual void doState()
    {
	#if DEBUG_BEHAVIOUR_VERBOSITY > 3
        debug << "LocaliseSelfState::doState" << endl;
    #endif
        if (m_parent->stateChanged())
        {
                  cout<<"\nReached kick state\n\n";
          Self& self  = m_field_objects->self;
            MobileObject& ball = m_field_objects->mobileFieldObjects[FieldObjects::FO_BALL];
            vector<float> kickPosition(2,0);
            vector<float> targetPosition(2,0);
            kickPosition[0] = ball.estimatedDistance() * cos(ball.estimatedBearing());
            kickPosition[1] = ball.estimatedDistance() * sin(ball.estimatedBearing());
            targetPosition[0] = kickPosition[0] + 1000.0f;
            targetPosition[1] = kickPosition[1];
            KickJob* kjob = new KickJob(0,kickPosition, targetPosition);
            m_jobs->addMotionJob(kjob);

        }
        tick();

    };
};

#endif

