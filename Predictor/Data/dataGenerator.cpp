/* 
 * Copyright (C): None
 * Authors: Jimmy Baraglia
 * Public License for more details
*/

#include <iostream>
#include <iomanip>
#include <vector>
#include <array>
#include <stdio.h>
#include <stdlib.h>     /* srand, rand */
#include <time.h>
using namespace std;


bool motor = false;
int noise = 0;
int stepsPerAction = 10;
std::array<float, 3> c1, c2;
std::vector<int> relation;

int main(int argc, char * argv[]) {
	srand(time(0));
	
	if(argc <  10){
		cerr<<"Wrong number of arguments \n";
		return -1;
	}
	
	motor = (bool)atoi(argv[1]);
	noise = atoi(argv[2]);
	stepsPerAction = atoi(argv[3]);
	c1 = {(float)atoi(argv[4]), (float)atoi(argv[5]), (float)atoi(argv[6])};
	c2 = {(float)atoi(argv[7]), (float)atoi(argv[8]), (float)atoi(argv[9])};
	
	for(int c = 0; c < argc - 10; c++){
		relation.push_back(atoi(argv[10+c]));
	}
	
	for(int c = 0; c < argc - 10; c++)
	{
		int nbStep = stepsPerAction+(rand()%(stepsPerAction)-stepsPerAction/2);
		
		for(int pause = 0; pause < nbStep; pause++)
		{
			 cout<<"0\t0\t0\t"<<c1[0]<<"\t"<<c1[1]<<"\t"<<c1[2]
			 <<"\t"<<c2[0]<<"\t"<<c2[1]<<"\t"<<c2[2]
			 <<"\t0\t0\t0\t0\n"; //No input
		}
		
		nbStep = stepsPerAction+rand()%(stepsPerAction/2);
		for(int step = 0; step < nbStep; step++)
		{
			 cout<<(float)motor*(0.01*(rand()%(int)100))<<"\t"<<(float)motor*(0.01*(rand()%(int)100))<<"\t"<<(float)motor*(0.01*(rand()%(int)100))
			 <<"\t"<<c1[0]<<"\t"<<c1[1]<<"\t"<<c1[2]
			 <<"\t"<<c2[0]<<"\t"<<c2[1]<<"\t"<<c2[2]<<"\t"; //No input
			 
			 float value[3] = {0};
			 value[relation[c]] = 1 - (rand()%noise)/100.;
			 
			 float dist = 1;
			 if(relation[c] == 0)
				dist = (float)step/(float)nbStep;
			 else if(relation[c] == 1)
				dist = 1-(float)step/(float)nbStep;
			 
			 cout<<value[0]<<"\t"<<value[1]<<"\t"<<value[2]<<"\t"<<dist<<endl;
			 
		}
	  
	}

	return 0;
}

