#ifndef LEARNEXPERIMENT_H
#define LEARNEXPERIMENT_H

#include <iostream.h>
#include<time.h>
#include<math.h>
#include<sys/types.h>
#include<stdlib.h>
#include <fstream.h>
#include <string.h>
#include <stdio.h>

#include "Tools/RL/Misc/globals.h"



void run(MainParameters& mainP,  StateActionFA* safa, Agent* agent, bool saveFA=true);

#endif
