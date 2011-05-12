/*! @file ALParameters.cpp
    @brief Implementation of ALParameters class for Active Localisation

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

#include "ALParameters.h"
#include "Tools/MotionFileTools.h"

#include "debug.h"
#include "debugverbositynumotion.h"
#include "nubotdataconfig.h"


/*! @brief Construct an empty active localisation paramtere set
 */
ALParameters::ALParameters()
{
    m_name = "blank";
}


/*! @brief Construct an empty active localisation paramtere set */
ALParameters::ALParameters(const string& name)
{
    m_name = name;
}

ALParameters::ALParameters(const string& name, const vector<float>& ballDistance, const vector<float>& duration, const vector<float>& accuracy);
{
    m_name = name;
    setBallDistance(maxspeeds);
    setBallDistance(ballDistance);
    setDurations(duration);
    setAccuracies(const vector<float>& data);
    setLandmarkIds(const vector<int>& data);

}

/*! @brief Destroy a walk parameter set
 */
ALParameters::~ALParameters()
{
}


vector<Parameter> ALParameters::getAsParameters()
{
    vector<Parameter> data;
    data.reserve(size());

    data.push_back(Parameter("BallDistance", ballDistances[0], ballDistances[1], ballDistances[2]));

    data.push_back(Parameter("Duration", durations[0], durations[1], durations[2]));
    data.push_back(Parameter("Accuracies", accuracies[0], accuracies[1], accuracies[2]));

    return data;
}

/*! @brief Returns the parameter set's name
    @return the walk parameter name
 */
string& ALParameters::getName()
{
    return m_name;
}

void WalkParameters::setName(const string& name)
{
    m_name = name;
}



/*! @brief Sets the walk engine parameters stored here
    @param parameters the walk engine parameters
 */
void WalkParameters::setParameters(const vector<Parameter>& parameters)
{
    m_parameters = parameters;
}


void WalkParameters::summaryTo(ostream& output)
{
    output << m_name << " WalkParameters: ";
    vector<float> temp = getAsVector();
    for (size_t i=0; i<temp.size(); i++)
        output << temp[i] << " ";
    output << endl;
}

/*! @brief Prints a comma separated string of the parameters
 */
void WalkParameters::csvTo(ostream& output)
{
    vector<float> temp = getAsVector();
    for (size_t i=0; i<temp.size(); i++)
        output << temp[i] << ", ";
}

/*! @brief Saves the entire contents of the WalkParameters class in the stream
 */
ostream& operator<< (ostream& output, const WalkParameters& p_walkparameters)
{
    output << p_walkparameters.m_name << endl;
    output << "Max Speeds (x cm/s, y cm/s, yaw rad/s): " << MotionFileTools::fromVector(p_walkparameters.m_max_speeds) << endl;
    output << "Max Accelerations (x cm/s/s, y cm/s/s, yaw rad/s/s): " << MotionFileTools::fromVector(p_walkparameters.m_max_accelerations) << endl;

    for (unsigned int i=0; i<p_walkparameters.m_parameters.size(); i++)
        output << p_walkparameters.m_parameters[i];

    output << "ArmGains (%): " << MotionFileTools::fromMatrix(p_walkparameters.m_arm_gains) << endl;
    output << "TorsoGains (%): " << MotionFileTools::fromMatrix(p_walkparameters.m_torso_gains) << endl;
    output << "LegGains (%): " << MotionFileTools::fromMatrix(p_walkparameters.m_leg_gains) << endl;

    return output;
}

/*! @brief Saves the entire contents of the WalkParameters class in the stream
 */
ostream& operator<< (ostream& output, const WalkParameters* p_walkparameters)
{
    output << (*p_walkparameters);
    return output;
}

/*! @brief Loads the entire contents of the WalkParameters class from the stream
 */
istream& operator>> (istream& input, WalkParameters& p_walkparameters)
{
    getline(input, p_walkparameters.m_name);
    p_walkparameters.m_max_speeds = MotionFileTools::toFloatVector(input);
    p_walkparameters.m_max_accelerations = MotionFileTools::toFloatVector(input);

    input.ignore(10,'\n');

    Parameter p;
    p_walkparameters.m_parameters.clear();

    int beforepeek = input.tellg();
    string label;
    input >> label;
    while (label.find("ArmGains") == string::npos && label.find("TorsoGains") == string::npos && label.find("LegGains") == string::npos)
    {
        input.seekg(beforepeek);
        input >> p;
        p_walkparameters.m_parameters.push_back(p);
        beforepeek = input.tellg();
        if (beforepeek < 0)
            return input;
        input >> label;
    }

    while (!input.eof())
    {
        if (label.find("ArmGains") != string::npos)
        {
            p_walkparameters.m_arm_gains = MotionFileTools::toFloatMatrix(input);
            p_walkparameters.m_num_arm_gains = MotionFileTools::size(p_walkparameters.m_arm_gains);
        }
        else if (label.find("TorsoGains") != string::npos)
        {
            p_walkparameters.m_torso_gains = MotionFileTools::toFloatMatrix(input);
            p_walkparameters.m_num_torso_gains = MotionFileTools::size(p_walkparameters.m_torso_gains);
        }
        else if (label.find("LegGains") != string::npos)
        {
            p_walkparameters.m_leg_gains = MotionFileTools::toFloatMatrix(input);
            p_walkparameters.m_num_leg_gains = MotionFileTools::size(p_walkparameters.m_leg_gains);
        }
        input >> label;
    }

    return input;
}

/*! @brief Loads the entire contents of the WalkParameters class from the stream
 */
istream& operator>> (istream& input, WalkParameters* p_walkparameters)
{
    input >> (*p_walkparameters);
    return input;
}

/*! @brief Saves the walk parameters to a file
 */
void WalkParameters::save()
{
    saveAs(m_name);
}

/*! @brief Saves the walk parameters to a new file given by the name
    @param name the new file will be name.cfg
 */
void WalkParameters::saveAs(const string& name)
{
    ofstream file((CONFIG_DIR + string("Motion/Walks/") + name + ".cfg").c_str());
    if (file.is_open())
    {
        file << this;
        file.close();
    }
    else
        debug << "WalkParameters::save(): Failed to open file " << name + ".cfg" << endl;
    file.close();
}

/*! @brief Loads the walk parameters from a file
    @param name the name of the file to load from
 */
void WalkParameters::load(const string& name)
{
    string filepath = CONFIG_DIR + string("Motion/Walks/") + name + ".cfg";
    ifstream file(filepath.c_str());
    if (file.is_open())
    {
        file >> this;
        file.close();
    }
    else
        debug << "WalkParameters::load(): Failed to load file " << filepath << endl;
    file.close();
}

/*! @brief Returns the size of the WalkParameters, this is the number of interesting walk parameters
 */
size_t WalkParameters::size() const
{
    return m_max_speeds.size() + m_max_accelerations.size() + m_parameters.size() + m_num_leg_gains;
}
