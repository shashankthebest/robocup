/*! @file TryLineUpState.h
    @brief A state where the robot tries to line up to ball

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

#ifndef LINEUPWALKTOBALL_H
#define LINEUPWALKTOBALL_H

#include "LineUpProvider.h"
#include "LineUpState.h"

#include "Infrastructure/Jobs/JobList.h"
#include "Infrastructure/NUSensorsData/NUSensorsData.h"
#include "Infrastructure/NUActionatorsData/NUActionatorsData.h"
#include "Infrastructure/FieldObjects/FieldObjects.h"
#include "Behaviour/BehaviourPotentials.h"

#include "Infrastructure/Jobs/MotionJobs/WalkJob.h"
#include "Infrastructure/Jobs/MotionJobs/HeadJob.h"
#include "Infrastructure/Jobs/MotionJobs/KickJob.h"

#include "debug.h"

class LineUpWalkToBallState : public LineUpState
{
private:
bool canKick;
bool kicked;
bool lostBall;
public:
    LineUpWalkToBallState(LineUpProvider* parent) : LineUpState(parent)
    {
        canKick = false;
        kicked = false;
        lostBall = false;
    };
    virtual ~LineUpWalkToBallState() {};
    virtual BehaviourState* nextState()
    {
        if(lostBall)
            return m_parent->m_localiseBall;
        if(kicked)
            return m_parent->m_generate;
        else
            return this;
    };
    virtual void doState()
    {
	
        if (m_parent->stateChanged())
        {
                    canKick = false;
                    kicked = false;
                    lostBall = false;
                    start();
                    cout<<"\nReached line up state\n\n";
                  //  vector<float> ballPosition(3,0);
                   //  cout<<"\n\n\n\nI was here\n\n\n\n";
                    //ballPosition = m_parent->m_globalMap->findObservation(6); // ball id = 6
                    // cout<<"\n\n\n\n and here\n\n\n\n";
                   // //m_parent->m_globalMap->clearObservation(6);
                    //float dist = sqrt( ballPosition[0]*ballPosition[0] +  ballPosition[1]*ballPosition[1] );
                    //float elevation = atan2(  dist,55 );

                    //m_jobs->addMotionJob(new HeadTrackJob(elevation ,ballPosition[2],0,0));
                    //m_jobs->addMotionJob(new HeadPanJob(HeadPanJob::Localisation, dist , dist , ballPosition[2] , ballPosition[2] ));
                    //HeadTrackJob* head = new HeadTrackJob(m_field_objects->mobileFieldObjects[FieldObjects::FO_BALL]);
                    //m_jobs->addMotionJob(head);
        }

		tick();

		if (m_field_objects->mobileFieldObjects[FieldObjects::FO_BALL].isObjectVisible())
			{
			   // cout<<"\n\nBall Visible, going towards it\n\n";
			   // cout<<"\nCurrentTime : "<<m_current_time<<"\n";
				float headyaw, headpitch;
				m_data->getPosition(NUSensorsData::HeadPitch,headpitch);
				m_data->getPosition(NUSensorsData::HeadYaw, headyaw);
				float measureddistance = m_field_objects->mobileFieldObjects[FieldObjects::FO_BALL].measuredDistance();
				float balldistance = measureddistance * cos(m_field_objects->mobileFieldObjects[FieldObjects::FO_BALL].measuredElevation());
				float ballbearing = m_field_objects->mobileFieldObjects[FieldObjects::FO_BALL].measuredBearing();

				float trans_speed = 1;
				float trans_direction = ballbearing;
				float yaw = ballbearing/2;
				m_jobs->addMotionJob(new HeadTrackJob(ball));
				
				Self& self  = m_field_objects->self;
				MobileObject& ball = m_field_objects->mobileFieldObjects[FieldObjects::FO_BALL];
				vector<float> kickPosition(2,0);
				vector<float> targetPosition(2,0);
				kickPosition[0] = ball.estimatedDistance() * cos(ball.estimatedBearing());
				kickPosition[1] = ball.estimatedDistance() * sin(ball.estimatedBearing());
				targetPosition[0] = kickPosition[0] + 1000.0f;
				targetPosition[1] = kickPosition[1];
				
				// collect all info, and keep ready for rl algo
				
			}
			else  if (ball.TimeSinceLastSeen() > 250)
			{

				m_jobs->addMotionJob(new HeadPanJob(ball));

			}

	
			if (m_time_in_state > 2000)
			{
				if (m_current_time - m_field_objects->mobileFieldObjects[FieldObjects::FO_BALL].TimeLastSeen() > 2000)
					lostBall = true;
			}

		}
};

#endif

