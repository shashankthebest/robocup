#include "occupancyGridMap.h"
#include <iostream>
#include<stdio.h>
#include <algorithm>
#include "Tools/Math/General.h"
#include <vector>
using namespace std;

OccupancyGridMap::OccupancyGridMap(float min_x, float max_x, float min_y, float max_y,int res, int dr)
	{
	    resolution = res;
		minX = min_x/resolution;
		minY = min_y/resolution;
		maxX = max_x/resolution;
		maxY = max_y/resolution;
		decayRate = dr;
        cout<<"\nMinX = "<<minX;
        cout<<"\nMinY = "<<minY;
        cout<<"\nMaxX = "<<maxX;
        cout<<"\nMaxY = "<<maxY;

        myX = 0;
        myY = 0;
        myTheta = 0;

	}

void OccupancyGridMap::positionUpdate(float x,float y ,float theta)
{
    myX = x/resolution;
    myY = y/resolution;
    myTheta = theta;

}

vector<float> OccupancyGridMap::getSelf()
{
    vector <float>retVal(3,0);
    retVal[0] = myX;
    retVal[1] = myY;
    retVal[2] = myTheta;
    return retVal;
}

unsigned char OccupancyGridMap::f2b(float inp)
{
	unsigned char retVal = 0;
	if ( inp >= 1 )
	{
		retVal = 100;
	}
	else if (inp == 0)
	{
		retVal = 0;
	}
	else
	{
		//retVal  = (char) ((int) 100*inp);
		retVal = inp*100;
	}

	return retVal;
}


float OccupancyGridMap::b2f(unsigned char inp)
{
	float retVal = 0;
	retVal = (int)inp;

	if ( retVal == 100)
	{
		retVal = 1;
	}
	else if (retVal == 0)
	{
		retVal = 0;
	}
	else
	{
		retVal = (float(retVal))/100;
	}
	return retVal;
}


void OccupancyGridMap::initializeMap()
{
    mapElement newEl;
    int counter =0 ;
	//for (int i = minY ; i <= maxY ; i++)
	for (int i = maxX ; i >= minX ; i--)
	{
		//for ( int j = minX ; j<= maxX ; j++)
		for ( int j = maxY ; j>= minY ; j--)
		{

            newEl.x = i;
            newEl.y = j;
            newEl.type = 0;
            newEl.val = 0;

			map.push_back(newEl);
			//cout<<"\n X = "<<i<<" , Y = "<<j;
			//getchar();
            counter++;
		}
	}


	//cout<<"\n Total Values inserted: "<<counter;

	//cout<<"\nSize of vector : "<<map.size();
}

void OccupancyGridMap:: initializeRobocupMap()
{

    //  All cells are unknown
	initializeMap();

	//  Insert outer boundary


	for(int i=(minX*resolution); i<=(maxX*resolution) ; i++)
	{
	    // TODO: Change 255 to a constant from field objects
        insertObservation(i,minY*resolution,255,1);
	    insertObservation(i,maxY*resolution,255,1);
	}

	for(int i=(minY*resolution); i<=(maxY*resolution) ; i++)
    {

	    insertObservation(minX*resolution,i,255,1);
	    insertObservation(maxX*resolution,i,255,1);
	}




}


void OccupancyGridMap::insertObservation(int xWorldPos, int yWorldPos,unsigned char type, float val)
{
    int xMapPos = xWorldPos/resolution;
    int yMapPos = yWorldPos/resolution;

    if (val>1)
        val = 0.99;

    setValue(xMapPos,yMapPos,type,f2b(val));
}




mapElement OccupancyGridMap::getValue(int xWorldPos, int yWorldPos)
{
    mapElement retVal;

    for(unsigned int i=0 ; i<map.size() ; i++)
    {

        if ((map[i].x == xWorldPos) && (map[i].y == yWorldPos))
        {
            retVal = map[i];
        }
    }
	return retVal;
}





void OccupancyGridMap::setValue(int xWorldPos, int yWorldPos, unsigned char type, float val)
{
    //cout<<"\n\n\n\n\n\n\n\n\n VAlue to set : "<<val;

    for(unsigned int i=0 ; i<map.size() ; i++)
    {

        mapElement temp = map[i];
        if ((temp.x == xWorldPos) && (temp.y == yWorldPos))
        {
            map[i].val = val;
            map[i].type = type;

        }

    }
}


void OccupancyGridMap::timeUpdate()
{
    float val = 0, oldVal = 0, newVal = 0;

    for(unsigned int i=0 ; i<map.size() ; i++)
    {
        if(map[i].type>3)  // if the observation is not one of the goal posts
        {
            oldVal = b2f(map[i].val);

            if (oldVal >0 && oldVal<1)
            {
                newVal = 0;
                val =  (oldVal/decayRate);

                newVal = 1/ ( 1 + (1-val)*(1-oldVal)/(oldVal*val) );
               //newVal = 1 / ( 1 + ( (1 - currVal)*(1 - oldVal)/(oldVal*currVal)   )  );

               cout<<"\n Old Val = "<<oldVal<<"  currVal = "<<val <<"  NewVal = "<<newVal <<"   type = "<<((int)map[i].type);;

               map[i].val = f2b(newVal);

            }
        }

    }

}



void OccupancyGridMap::printMap()
{

    cout<<"\nSize of vector : "<<map.size()<<"\n\n";
    cout<<"\nMy position on map : "<<myX<<", "<<myY<<", "<<myTheta*180/3.14;

 	float mapVal = 0 ;

    int counter =0;

 //   for (int j=0;j<map.size();j++)
 //       cout<<"\nx = "<<(map[j].x)<<",  y ="<<(map[j].y)<<",  val = "<<b2f(map[j].val);
/*
cout<<"\n____________________________________________\n\n";
    for (int j=minY;j<=maxY;j++)
    {

        for(int i=minX;i<=maxX;i++)
        {
            cout<<b2f(map[counter].val)<<" ";
            counter--;
        }
        cout<<"\n";
    }

*/

    //counter = map.size()-1;
cout<<"\n____________________________________________\n\n";

    //for (int j=minY;j<=maxY;j++)
    for (int i=maxX;i>=minX;i--)
    {

        //for(int i=minX;i<=maxX;i++)
        for(int j=maxY;j>=minY;j--)
        {

			mapVal = b2f(map[counter].val);

           // cout<<"\n X = "<<i<<" , Y = "<<j;
            //getchar();
			if( (i ==(int) myX) && (j==(int)myY))
			{
                cout<<"+";
                //counter--;
                counter++;
			}
			else
			{
                //counter--;
                counter++;
                if(  (mapVal >= 0)  && (mapVal <0.3))
                    cout<<"_";
                else if((mapVal >= 0.3) && (mapVal <= 0.5))
                    cout<<"1";
                else if((mapVal >0.5) && (mapVal <1))
                    cout<<"2";
                else if(mapVal >= 1)
                    cout<<"X";
			}


        }
        cout<<"\n";
    }
  //  counter= map.size()-1;
   // cout<<"\n\n Value at last element  "<<b2f(map[counter].val);
}

void OccupancyGridMap::updateCurrentBelief(int x,int y,float theta)
{
    myX = x;
    myY = y;
    myTheta = theta;
}

vector<float> OccupancyGridMap::getRelativeDistance(int x, int y)
{
    vector<float> retValue;
    float relX,relY,relTheta;
/*
    cout<<"\nMap x = "<<x<<" ,Map y = "<<y;
    cout<<"\nWith Reso x = "<<x*resolution<<" , with reso Map y = "<<y*resolution;
    cout<<"\nMyX = "<<myX<<" ,MyY = "<<myY <<", myT "<<myTheta;
    cout<<"\nWith Reso myX = "<<myX*resolution<<" , with reso MyY = "<<myY*resolution;
*/
    relX = (x*resolution - myX*resolution);
    relY = (y*resolution - myY*resolution);
//    cout<<"\nRel X = "<<relX<<" , Rel Y = "<<relY;
    relTheta = atan2(relY,relX) - myTheta;
    retValue.push_back(relX);
    retValue.push_back(relY);
    retValue.push_back(relTheta);
    // TODO: Calculate relative angle from robot's curent orientation
    return retValue;
}


vector<float> OccupancyGridMap::findObservation(unsigned int type)
{

    vector<float> retValue;
    vector<float> temp;
    float relX,relY,relTheta;

    for(unsigned int i=0; i<map.size();i++)
    {
        if(map[i].type == type)
        {
            temp = getRelativeDistance(map[i].x, map[i].y);
            relX = temp[0];
            relY = temp[1];

            relTheta = temp[2];

            retValue.push_back(relX);
            retValue.push_back(relY);
            retValue.push_back(relTheta);

        }
    }
    return retValue;

}


void OccupancyGridMap::clearObservation(unsigned int type)
{
    for(unsigned int i=0; i<map.size();i++)
    {
        if(map[i].type == type)
        {
            map[i].x = 0;
            map[i].y = 0;
            map[i].type = 0;
            map[i].val = 0;

        }
    }


}

vector<float> OccupancyGridMap::findBestObservation(unsigned int typeMin, unsigned int typeMax)
{
    vector<float> bestObservation(3,0);
//    vector<float> allObservations(3,0);
    vector<float> temp;
    float  mintheta;
    int idx = 0;
    bool nothingFoundYet = true;
  //  float relX,relY,relTheta;

    for(unsigned int i=0; i<map.size();i++)
    {
        if(   (map[i].type >= typeMin)  and (map[i].type <= typeMax) )
        {
            temp = getRelativeDistance(map[i].x, map[i].y);
            cout<<"\nFound type = "<<(int)map[i].type;
            if(nothingFoundYet)
               {
                    nothingFoundYet = false;
                    mintheta = temp[2];
                    bestObservation[0] = temp[0];
                    bestObservation[1] = temp[1];
                    bestObservation[2] = temp[2];
               }
            else if ( fabs(temp[2]) < fabs(mintheta) )
                {
                    mintheta = temp[2];
                    bestObservation[0] = temp[0];
                    bestObservation[1] = temp[1];
                    bestObservation[2] = temp[2];
                    idx = i;

                    cout<<"\nInside replacing min value\n\n";
                }
                cout<<"\nMin Theta = "<<mintheta;

        }
    }

    cout<<"\n\n\n\n\n\n\n\n Found landmark type is  : == "<<(int)map[idx].type<<"\n\n\n\n\n\n\n";

    //idx *= 3; // thetas.begin() - it;

   cout<<"\n\n\n\nIterator Value === \t"<<mathGeneral::rad2deg(mintheta)<<"\t found at index "<<idx<<"\n\n\n\n";


    return bestObservation;

}


vector<float> OccupancyGridMap::findBestObservation(vector<float> allObservations)
{
    vector<float> thetas;
    vector<float> bestObservation;

    for(unsigned int i=0; i<allObservations.size();i+=2)
    {
        thetas.push_back(allObservations[i]);
    }

    vector<float>::const_iterator it;


    it = min_element(thetas.begin(), thetas.end());

    int idx = thetas.begin() - it;

    bestObservation.push_back(allObservations[idx-2]);
    bestObservation.push_back(allObservations[idx-1]);
    bestObservation.push_back(allObservations[idx]);

     return bestObservation;
}



unsigned int OccupancyGridMap::getSize()
{
    return map.size()*2;
}


void OccupancyGridMap::serializeMap(char* serialMap)
{
    int j=0;
    for(unsigned int i=0 ; i<map.size() ; i++)
    {
        serialMap[j] = map[i].type;
        serialMap[j+1] = map[i].val;
        j+=2;
    }


}

void OccupancyGridMap::copyFromStream(char* serialMap)
{
  int j=0;
  for(unsigned int i=0 ; i<map.size() ; i++)
    {
        map[i].type = serialMap[j];
        map[i].val = serialMap[j+1];
        j+=2;
    }
}


OccupancyGridMap::~OccupancyGridMap()
{
}


