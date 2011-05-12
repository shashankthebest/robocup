/*! @file ALParameters.h
    @brief Declaration of Active Localisation Parameters class

    @class ALParameters
    @brief A module to store Active Localisation parameters

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

#ifndef ALPARAMETERS_H
#define ALPARAMETERS_H

#include "Tools/Optimisation/Parameter.h"

#include <vector>
#include <string>
#include <iostream>
#include <sstream>
using namespace std;

class ALParameters
{
public:
    ALParameters();
    ALParameters(const string& name);
    ALParameters(const string& name, const vector<float>& ballDistance, const vector<float>& duration, const vector<float>& accuracy);
    ~ALParameters();

    // get methods
    vector<float> getAsVector();
    vector<Parameter> getAsParameters();
    string& getName();

    // set methods
    void set(const vector<float>& data);
    void setName(const string& name);
    void setParameters(const vector<Parameter>& parameters);
    void setBallDistance(const vector<float>& data);
    void setDurations(const vector<float>& data);
    void setAccuracies(const vector<float>& data);
    void setLandmarkIds(const vector<int>& data);

    // display methods
    void summaryTo(ostream& output);
    void csvTo(ostream& output);

    // serialisation
    friend ostream& operator<< (ostream& output, const WalkParameters& p_walkparameters);
    friend ostream& operator<< (ostream& output, const WalkParameters* p_walkparameters);
    friend istream& operator>> (istream& input, WalkParameters& p_walkparameters);
    friend istream& operator>> (istream& input, WalkParameters* p_walkparameters);
    void save();
    void saveAs(const string& name);
    void load(const string& name);

    size_t size() const;


public:
private:
    vector<float>ballDistances;
    vector<float>durations;
    vector<float>accuracies;
    vector<int>landmarkIds;

    vector<Parameter> m_parameters;        //!< stores the parameters for the walk engine

};

#endif

