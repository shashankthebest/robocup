/*! @file EvaluateEpisodeState.h
    @brief A state to evaluate an episode for active localisation learning

    @class EvaluateEpisodeState
    @brief An Active Localisation state to evaluate parameters to
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

#ifndef LINEUPEVALUATESTATE_H
#define LINEUPEVALUATESTATE_H

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

class LineUpEvaluateState : public LineUpState
{
public:
    LineUpEvaluateState(LineUpProvider* parent) : LineUpState(parent)
    {
        #if DEBUG_BEHAVIOUR_VERBOSITY > 3
            debug << "EvaluateEpisodeState::EvaluateEpisodeState" << endl;
        #endif
        m_getting_up = false;
        m_previously_getting_up = false;
        m_time_not_getting_up = 0;
    }

    ~LineUpEvaluateState() {};
    BehaviourState* nextState()
    {   // progress to the evaluation state when we are in position AND lined up AND stopped

        bool gettingup = false;
        m_data->get(NUSensorsData::MotionGetupActive, gettingup);

        if (not gettingup)
        {
        	finish();
            return m_parent->m_evaluate;
        }
        else
            return this;
    }

    void doState()
    {

    }
private:
    bool m_previously_getting_up;
    bool m_getting_up;
    float m_time_not_getting_up;
};

#endif

