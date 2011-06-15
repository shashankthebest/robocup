/*! @file GenerateEpisodeState.h
    @brief A state to generate an episode for active localisation learning

    @class GenerateEpisodeState
    @brief An Active Localisation state to generate parameters to
           test active localisation

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

#ifndef LINEUPSETUPSTATE_H
#define LINEUPSETUPSTATE_H

#include "LineUpProvider.h"
#include "LineUpState.h"
#include "Tools/Optimisation/Optimiser.h"
#include "Tools/Math/General.h"
#include "Tools/Math/StlVector.h"

#include "Infrastructure/NUActionatorsData/NUActionatorsData.h"
#include "Infrastructure/FieldObjects/FieldObjects.h"
#include "Infrastructure/Jobs/JobList.h"

#include "Infrastructure/Jobs/MotionJobs/WalkJob.h"
#include "Infrastructure/Jobs/MotionJobs/HeadTrackJob.h"
#include "Infrastructure/Jobs/MotionJobs/HeadPanJob.h"
#include "Behaviour/BehaviourPotentials.h"

#include "debug.h"
#include "debugverbositybehaviour.h"

#include <vector>
using namespace std;

class LineUpSetupState : public LineUpState
{
public:
    LineUpSetupState(LineUpProvider* parent) : LineUpState(parent)
    {
        #if DEBUG_BEHAVIOUR_VERBOSITY > 3
            debug << "GenerateEpisodeState::GenerateEpisodeState" << endl;
        #endif
        m_getting_up = false;
        m_previously_getting_up = false;
        m_time_not_getting_up = 0;
    }

    ~LineUpSetupState() {};


    BehaviourState* nextState()
    {   // progress to the localiseBall state when we know the parameters of this episode

        bool gettingup = false;
        m_data->get(NUSensorsData::MotionGetupActive, gettingup);

        if (not gettingup)
        {
        	finish();
            return m_parent->m_localiseBall;
        }
        else
            return this;
    }

    void doState()
    {
        //Use the RL optimiser to generate new parameters
        cout<<"\nInside Generate State\n\n\n";
    }


private:
    bool m_previously_getting_up;
    bool m_getting_up;
    float m_time_not_getting_up;
};

#endif

