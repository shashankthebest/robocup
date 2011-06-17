#ifndef RLGLOBALS_H
#define RLGLOBALS_H

#include <iostream>
#include <fstream>
#include <string.h>
#include<stdlib.h>
#include<math.h>
#include<time.h>
using namespace std;

//typedef short bool; //you may have uncomment this typedef, depending on the compiler version

#define DIR_SEP "/"
//#define DIR_SEP "\\" //use this statement if compiling for DOS or Windows 

#define true 1
#define false 0

float gasdev(long *idum); //routive for sampling from Gaussians
float ran1(long *idum);//routine for uniform sampling in [0,1] 

int tokenize(char* sep, char* str, int** tokens);
/* Extracts tokens from string "str", which are separated by separators
   specified in "sep". Allocates array (*tokens) of the appropriate size
   and saves extracted tokens in that array. Returns the number of tokens.
*/



struct MainParameters{
	int Trials;
	int Steps;
	char* strFile;
	int TestFrequency;
	int TestStatesNumber;
	int TestSamples;
	char* TestStatesFile;
	char* dir;
	int run;
	char* faFileName;
	
	MainParameters(){
		Trials=10000;
		Steps=1000;
		strFile=NULL;
		TestFrequency=100;
		TestStatesNumber=50;
		TestSamples=30;
		TestStatesFile=NULL;
		dir=new char[3];
		strcpy(dir,".");
		dir[1]=DIR_SEP[0];
		dir[2]='\0';
		run=0;
		faFileName=NULL;
	}
	
	int process(int argc, char* argv[]){
		int i;
		for (i=0; i<argc; i++){
			if (strncmp(argv[i],"?", 1)==0){
				cout << "Main parammeters:" << endl;
				cout << "Trials=value : number of trials in a learning run" << endl;
				cout << "Steps=value : maximum number of steps in a trial" << endl;
				cout << "str=value : name of the file with structural parameters for function approximator" << endl;
				cout << "tf=value : frequency of policy eveluations, e.g. every 100 trials" << endl;
				cout << "tsn=value : number of test states from which policies are eveluated" << endl;
				cout << "tsf=value :  name of the file with pre-fixed test states" << endl;
				cout << "ts=value : number of sample trajectories from each test state to evaluate policies" << endl;
				cout << "dir=value : directory, where to save learning history data" << endl;
				cout << "fa=value : base (without extension) of the file names with parameters of the FAs representing value functions for each action under some policy (if they have to be loaded)" << endl;
				
				return 1;
			}
			
			if (strncmp("Trials=", argv[i], 7)==0){
				Trials=atoi(&(argv[i][7]));
			}
			
			if (strncmp("Steps=", argv[i], 6)==0){
				Steps=atoi(&(argv[i][6]));
			}
			
			if (strncmp("str=", argv[i], 4)==0){
				strFile = new char[strlen(&(argv[i][4]))+1];
				strcpy(strFile,&(argv[i][4]));
			}
			
			if (strncmp("fa=", argv[i], 3)==0){
				faFileName = new char[strlen(&(argv[i][3]))+1];
				strcpy(faFileName,&(argv[i][3]));
			}
			
			if (strncmp("tf=", argv[i], 3)==0){
				TestFrequency=atoi(&(argv[i][3]));
			}
			
			if (strncmp("tsn=", argv[i], 4)==0){
				TestStatesNumber=atoi(&(argv[i][4]));
			} 
			
			if (strncmp("ts=", argv[i], 3)==0){
				TestSamples=atoi(&(argv[i][3]));
			} 
			
			if (strncmp("tsf=", argv[i], 4)==0){
				TestStatesFile = new char[strlen(&(argv[i][4]))+1];
				strcpy(TestStatesFile,&(argv[i][4]));
			}
			
			if (strncmp("dir=", argv[i], 4)==0){
				delete [] dir;
				dir = new char[strlen(&(argv[i][4]))+2];
				strcpy(dir,&(argv[i][4]));
				if (dir[strlen(dir)-1]!=DIR_SEP[0]){
					int l = strlen(dir);
					dir[l]=DIR_SEP[0];
					dir[l+1]='\0';
				}
				
			}
			
			if (strncmp("run=", argv[i],4)==0){
				run=atoi(&(argv[i][4]));
			}
			
			
			
		}
		
		return 0;
	}
	
	~MainParameters(){
		
		delete [] strFile;
		delete [] faFileName;
		delete [] TestStatesFile;
		delete [] dir;
		
	}
};





#endif
