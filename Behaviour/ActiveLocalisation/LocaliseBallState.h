/*! @file LocaliseBallState.h
    @brief A state where the localisation of ball is performed

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

#ifndef LOCALISEBALLSTATE_H
#define LOCALISEBALLSTATE_H

#include "ActiveLocalisationProvider.h"
#include "ActiveLocalisationState.h"

#include "Infrastructure/Jobs/JobList.h"
#include "Infrastructure/NUSensorsData/NUSensorsData.h"
#include "Infrastructure/NUActionatorsData/NUActionatorsData.h"
#include "Infrastructure/FieldObjects/FieldObjects.h"
#include "Infrastructure/NUBlackboard.h"
#include "Infrastructure/TeamInformation/TeamInformation.h"

#include "Infrastructure/Jobs/MotionJobs/WalkJob.h"
#include "Infrastructure/Jobs/MotionJobs/HeadJob.h"
#include "Infrastructure/Jobs/MotionJobs/HeadJob.h"
#include "Infrastructure/Jobs/MotionJobs/HeadTrackJob.h"
#include "Infrastructure/Jobs/MotionJobs/HeadPanJob.h"
#include "Infrastructure/Jobs/MotionJobs/HeadNodJob.h"

#include "debug.h"

class LocaliseBallState : public ActiveLocalisationState
{
public:
    LocaliseBallState(ActiveLocalisationProvider* parent) : ActiveLocalisationState(parent)
    {
			m_parent->ballCertainty = 0;
			reachedBall = false;
			reachedHalf = false;

	};
    virtual ~LocaliseBallState() {};

    virtual BehaviourState* nextState()
    {
        tick();
        if(reachedHalf)
            return m_parent->m_localiseSelf;
        else if (reachedBall)
            return m_parent->m_kick;
        else
            return this;
	};


    virtual void doState()
    {
	#if DEBUG_BEHAVIOUR_VERBOSITY > 3
        debug << "LocaliseBallState::doState" << endl;
    #endif

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

                if (balldistance > m_parent->fixedBallDistance)
                {
                    vector<float> temp;
                    float leftobstacle = 255;
                    float rightobstacle = 255;
                    if (m_data->get(NUSensorsData::LDistance, temp))
                        leftobstacle = temp[0];
                    if (m_data->get(NUSensorsData::RDistance, temp))
                        rightobstacle = temp[0];

                    if (leftobstacle < 50)
                    {
                        trans_speed = trans_speed + 0.01*(leftobstacle - 50);
                        if (trans_speed > 0)
                            trans_direction = trans_direction - atan2(20, leftobstacle);
                        else
                            trans_direction = trans_direction + atan2(20, leftobstacle);

                        yaw = yaw - 0.015*(leftobstacle - 50);
                    }
                    else if (rightobstacle < 50)
                    {
                        trans_speed = trans_speed + 0.01*(rightobstacle - 50);
                        if (trans_speed > 0)
                            trans_direction = trans_direction + atan2(20, leftobstacle);
                        else
                            trans_direction = trans_direction - atan2(20, leftobstacle);
                        yaw = yaw + 0.015*(rightobstacle - 50);
                    }

                    WalkJob* walk = new WalkJob(trans_speed, trans_direction, yaw);
                    m_jobs->addMotionJob(walk);
                    HeadTrackJob* head = new HeadTrackJob(m_field_objects->mobileFieldObjects[FieldObjects::FO_BALL]);
                    m_jobs->addMotionJob(head);
                }
                else
                {
                    reachedHalf = true;
                    vector<float> myPos;
                    myPos = m_parent->m_globalMap->getSelf();

                    myPos[0] +=  balldistance*cos(ballbearing);
                    myPos[1] +=  balldistance*sin(ballbearing);
                    myPos[2]  =  ballbearing;
                    m_parent->m_globalMap->insertObservation(myPos[0], myPos[0],5,0.99 );

                }

            }

            else if (m_current_time - m_field_objects->mobileFieldObjects[FieldObjects::FO_BALL].TimeLastSeen() > 500)
            {
               // cout<<"\n\nBall not seen since long, spinning with nodding \n";

                float spin = 0;
                if (m_field_objects->mobileFieldObjects[FieldObjects::FO_BALL].measuredBearing() < 0)
                    spin = -0.4;
                else
                    spin = 0.4;
                m_jobs->addMotionJob(new WalkJob(0,0,spin));
                m_jobs->addMotionJob(new HeadNodJob(HeadNodJob::Ball,spin));

                return;
            }
            else if (not Blackboard->TeamInfo->amIClosestToBall())
            {
               // cout<<"\n\nI am not closest, going towards it\n";
                //cout<<"\nCurrentTime : "<<m_current_time<<"\n";
                if (m_field_objects->mobileFieldObjects[FieldObjects::FO_BALL].isObjectVisible())
                {
                    float headyaw, headpitch;
                    m_data->getPosition(NUSensorsData::HeadPitch,headpitch);
                    m_data->getPosition(NUSensorsData::HeadYaw, headyaw);

                    float measureddistance = m_field_objects->mobileFieldObjects[FieldObjects::FO_BALL].measuredDistance();
                    float balldistance = measureddistance * cos(m_field_objects->mobileFieldObjects[FieldObjects::FO_BALL].measuredElevation());
                    //float balldistance;
                    //if (measureddistance < 46)
                    //    balldistance = 1;
                    //else
                    //    balldistance = sqrt(pow(measureddistance,2) - 46*46);
                    float ballbearing = m_field_objects->mobileFieldObjects[FieldObjects::FO_BALL].measuredBearing();


                    if (balldistance <= m_parent->fixedBallDistance)
                    {
                        vector<float> walkVector(3, 0);
                        walkVector[1] = 2*sin(ballbearing);
                        walkVector[2] = ballbearing/2.0;
                        walkVector[0] = 0.5*(balldistance - 100)*cos(ballbearing);


                        vector<float> temp;
                        float leftobstacle = 255;
                        float rightobstacle = 255;

                        if (m_data->get(NUSensorsData::LDistance, temp))
                            leftobstacle = temp[0];
                        if (m_data->get(NUSensorsData::RDistance, temp))
                            rightobstacle = temp[0];

                        if (leftobstacle < 50)
                        {
                            walkVector[0] = walkVector[0] + 0.5*(leftobstacle - 50);
                            walkVector[1] = -2;
                            walkVector[2] = walkVector[2] + 0.015*(leftobstacle - 50);
                        }
                        else if (rightobstacle < 50)
                        {
                            walkVector[0] = walkVector[0] + 0.5*(rightobstacle - 50);
                            walkVector[1] = 2;
                            walkVector[2] = walkVector[2] - 0.015*(rightobstacle - 50);
                        }

                        WalkJob* walk = new WalkJob(0.5*(balldistance - 100), ballbearing, ballbearing/2.0);
                        m_jobs->addMotionJob(walk);

                        HeadTrackJob* head = new HeadTrackJob(m_field_objects->mobileFieldObjects[FieldObjects::FO_BALL]);
                        m_jobs->addMotionJob(head);
                    }
                    else if (balldistance <= 20)
                    {
                        reachedBall= true;

                    }

                }
                return;
            }


/*      if (m_parent->stateChanged())
        {
            m_jobs->addMotionJob(new WalkJob(0,0,0));
            vector<float> zero(m_actions->getSize(NUActionatorsData::Head), 0);
            m_jobs->addMotionJob(new HeadJob(m_actions->CurrentTime + 500, zero));
        }
*/
    };
private:

    bool reachedBall;
    bool reachedHalf;
};







#endif

