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

#ifndef TRYLINEUPSTATE_H
#define TRYLINEUPSTATE_H

#include "ActiveLocalisationProvider.h"
#include "ActiveLocalisationState.h"

#include "Infrastructure/Jobs/JobList.h"
#include "Infrastructure/NUSensorsData/NUSensorsData.h"
#include "Infrastructure/NUActionatorsData/NUActionatorsData.h"
#include "Infrastructure/FieldObjects/FieldObjects.h"
#include "Behaviour/BehaviourPotentials.h"

#include "Infrastructure/Jobs/MotionJobs/WalkJob.h"
#include "Infrastructure/Jobs/MotionJobs/HeadJob.h"
#include "Infrastructure/Jobs/MotionJobs/KickJob.h"

#include "debug.h"

class TryLineUpState : public ActiveLocalisationState
{
private:
bool canKick;
bool kicked;
bool lostBall;
public:
    TryLineUpState(ActiveLocalisationProvider* parent) : ActiveLocalisationState(parent)
    {
        canKick = false;
        kicked = false;
        lostBall = false;
    };
    virtual ~TryLineUpState() {};
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
	#if DEBUG_BEHAVIOUR_VERBOSITY > 3
        debug << "TryLineUpState::doState" << endl;
    #endif
        if (m_parent->stateChanged())
        {
                    canKick = false;
                    kicked = false;
                    lostBall = false;
                    start();
                    cout<<"\nReached line up state\n\n";
                    vector<float> ballPosition(3,0);
                     cout<<"\n\n\n\nI was here\n\n\n\n";
                    ballPosition = m_parent->m_globalMap->findObservation(6); // ball id = 6
                     cout<<"\n\n\n\n and here\n\n\n\n";
                    m_parent->m_globalMap->clearObservation(6);
                    float dist = sqrt( ballPosition[0]*ballPosition[0] +  ballPosition[1]*ballPosition[1] );
                    float elevation = atan2(  dist,55 );

                    m_jobs->addMotionJob(new HeadTrackJob(elevation ,ballPosition[2],0,0));
                    m_jobs->addMotionJob(new HeadPanJob(HeadPanJob::Localisation, dist , dist , ballPosition[2] , ballPosition[2] ));
                    HeadTrackJob* head = new HeadTrackJob(m_field_objects->mobileFieldObjects[FieldObjects::FO_BALL]);
                    m_jobs->addMotionJob(head);


        }

        tick();


        if(!canKick)
        {
                #if DEBUG_BEHAVIOUR_VERBOSITY > 1
                    debug << "GoToBall" << endl;
                #endif
                Self& self  = m_field_objects->self;
                MobileObject& ball = m_field_objects->mobileFieldObjects[FieldObjects::FO_BALL];
                if (ball.isObjectVisible())
                    m_jobs->addMotionJob(new HeadTrackJob(ball));
                else if (ball.TimeSinceLastSeen() > 250)
                    m_jobs->addMotionJob(new HeadPanJob(ball));

                bool iskicking;
                m_data->get(NUSensorsData::MotionKickActive, iskicking);
                if(!iskicking)
                {
                    vector<float> speed = BehaviourPotentials::goToBall(ball, self, BehaviourPotentials::getBearingToOpponentGoal(m_field_objects, m_game_info));
                    vector<float> result;
                    // decide whether we need to dodge or not
                    float leftobstacle = 255;
                    float rightobstacle = 255;
                    vector<float> temp;
                    if (m_data->get(NUSensorsData::LDistance, temp) and temp.size() > 0)
                        leftobstacle = temp[0];
                    if (m_data->get(NUSensorsData::RDistance, temp) and temp.size() > 0)
                        rightobstacle = temp[0];

                    // if the ball is too far away to kick and the obstable is closer than the ball we need to dodge!
                    if (ball.estimatedDistance() > 20 and min(leftobstacle, rightobstacle) < ball.estimatedDistance())
                        result = BehaviourPotentials::sensorAvoidObjects(speed, m_data, min(ball.estimatedDistance(), 25.0f), 75);
                    else
                        result = speed;

                    m_jobs->addMotionJob(new WalkJob(result[0], result[1], result[2]));
                }



                if( (ball.estimatedDistance() < 20.0f) && BehaviourPotentials::opponentsGoalLinedUp(m_field_objects, m_game_info))
                {
                    canKick = true;

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

                if (m_time_in_state > 2000)
                {
                    if (m_current_time - m_field_objects->mobileFieldObjects[FieldObjects::FO_BALL].TimeLastSeen() > 2000)
                        lostBall = true;
                }


        }
        else
        {
            bool iskicking;
            m_data->get(NUSensorsData::MotionKickActive, iskicking);
            if(!iskicking)
                kicked = true;


        }





    };
};

#endif

