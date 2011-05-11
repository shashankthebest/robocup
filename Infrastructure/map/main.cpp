#include "../include/occupancyGridMap.h"
#include<stdio.h>


int main()
{
    // Test for Relative position
    occupancyGridMap myMap(-300,300,-200,200,5,2);

    myMap.initializeRobocupMap();
    myMap.insertObservation(-100,-100,1,0.95);
    myMap.insertObservation(0,0,2,0.95);
    myMap.positionUpdate(-110,-110,45*3.14/180);
    myMap.printMap();
    myMap.timeUpdate();
    vector<float>obs = myMap.findObservation(1);
    cout<<"\nSize of vector : "<<obs.size()<<"\n";
    cout<<"\nFound at : "<<obs[0]<<", "<<obs[1]<<", "<<(obs[2]*180/3.14)<<"\n";


    //
    /*Test for serialisation
    occupancyGridMap myMap(-300,300,-200,200,5,2);
    occupancyGridMap myMap2(-300,300,-200,200,5,2);
    myMap2.initializeMap();
    myMap.initializeRobocupMap();
    myMap.initializeMap();
    myMap.insertObservation(0,0,1,0.95);
   // myMap.insertBallObservation(0,0,0.99);
    int siz = myMap.getSize();
    cout<<"\n\n\nSize found = "<<siz<<"\n\n\n";
    char *c = new char[siz];
    myMap.serializeMap(c);
    myMap2.copyFromStream(c);
    char l;
    while(1)
    {
            myMap2.printMap();
            myMap2.timeUpdate();
            cin>>l;
    }

    */

}








