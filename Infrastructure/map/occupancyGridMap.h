#ifndef OCCUPANCY_GRID_MAP
#define OCCUPANCY_GRID_MAP

#include "Infrastructure/FieldObjects/FieldObjects.h"
#include<iostream>
#include<math.h>
#include<vector>




using namespace std;

struct mapElement
{
  int x;
  int y;
  unsigned char type;
  unsigned char val;
};

class occupancyGridMap{

private:

    /// internal information data
	int xLength, yLength;
	int transX , transY;

	float myX, myY;
	float myTheta;

	int resolution;

	unsigned char f2b(float inp);

	float b2f(unsigned char inp);

    void setValue(int xWorldPos, int yWorldPos,unsigned char type, float val);

public:
int decayRate;

    void updateCurrentBelief(int,int,float);

    /// Insert initial 0 belief everywhere
	void initializeMap();

	/// max dimensions of the map in 10 cms
	int minX, minY, maxX, maxY;

	/// Array to hold map data in the form (prob, x,y,volatile)
	vector<mapElement> map;

	void copyFromStream(char*);

	unsigned int getSize();

	/// Constructor
	occupancyGridMap(float min_x, float max_x, float min_y, float max_y,int res, int dr);

    /// Clear everything, in the map, retain its dimensions
	void resetMap();

	void timeUpdate();

	void positionUpdate(float, float,float);

    vector<float> getRelativeDistance(int,int);

	void initializeRobocupMap();

    void serializeMap(char* );

	void insertObservation(int xWorldPos, int yWorldPos,unsigned char type, float val);


    mapElement getValue(int xWorldPos, int yWorldPos);

    vector<float> findObservation(unsigned int type);


	void printMap();

	/// Destructor
	~occupancyGridMap();

};

#endif
