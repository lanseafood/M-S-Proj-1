//
//  vehicle.c
//  traffic
//
//  Authors: Mingyue Ma, Eisha Nathan, Stefan Henneking
//  01/12/2015
//


#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "intersection.h"
#include "linkedList.h"

//static int idCounter = 0;

struct IntersectionType {
	int zoneID;
	Signal **signals; //0-3: straight (NESW), 4-7: left (NESW)
//	LinkedList lanes; //16 possible lanes at each intersection
	int num_cars;
	//wait times
	double ns_len;
	double ew_len;
};

//creates intersection and its respective signals
Intersection create_intersection(int zoneID) {
	Intersection I = (Intersection) malloc( sizeof(struct IntersectionType));
	I->zoneID = zoneID;
	
	Signal **signals = (Signal **) malloc(8*sizeof(Signal *));
	I->signals = signals;

	for (int i=0; i<8; i++) {
		I->signals[i] = (Signal *) malloc(sizeof(Signal));
	}

//	LinkedList lanes = (LinkedList) malloc(16*sizeof(struct LinkedListType));
//	I->lanes = lanes;

	for (int i = 0; i<8; i++) {
		I->signals[i]->times = (double *) malloc(3*sizeof(double));	
	}
	

	//double zeros[3]; //if signal doesn't exist
	double ns_len=0, ew_len=0;

	for (int j=0; j<8; j++) {
		I->signals[j]->zoneID = zoneID;
		I->signals[j]->init = INV;
		I->signals[j]->light = INV;
	}

	if (zoneID==1) {
		ns_len = 99.73;
		ew_len = 106;

		memcpy(I->signals[0]->times, ((double [3]){34.7, 3.6, 49.3}), 3*sizeof(double)); 
		memcpy(I->signals[1]->times, ((double [3]){30,3.8,55}), 3*sizeof(double)); 
		memcpy(I->signals[2]->times, ((double [3]){34.7,3.6,49.3}), 3*sizeof(double)); 
		memcpy(I->signals[3]->times, ((double [3]){28,3.8,55}), 3*sizeof(double)); 
		memcpy(I->signals[4]->times, ((double [3]){7,3.6,2.2}), 3*sizeof(double)); 
		memcpy(I->signals[5]->times, ((double [3]){8,1.8,1.8}), 3*sizeof(double)); 
		memcpy(I->signals[6]->times, ((double [3]){7,3.6,2.2}), 3*sizeof(double)); 
		memcpy(I->signals[7]->times, ((double [3]){5,3.6,4.2}), 3*sizeof(double)); 
	} else if (zoneID==2) {
		ns_len = 129.88;
		ew_len = 95;	
		memcpy(I->signals[0]->times, ((double [3]){41.5,3.2,55.4}), 3*sizeof(double)); 
		memcpy(I->signals[1]->times, ((double [3]){20.2,3.6,76.1}), 3*sizeof(double)); 
		memcpy(I->signals[2]->times, ((double [3]){41.5,3.2,55.4}), 3*sizeof(double)); 
		memcpy(I->signals[3]->times, ((double [3]){20.3,3.6,76.2}), 3*sizeof(double)); 

		for (int i=4; i<8; i++) {
			memcpy(I->signals[i]->times, ((double [3]){0,0,0}), 3*sizeof(double)); 
		}
	} else if (zoneID==3) {
		ns_len = 73.51;
		ew_len = 102;	

		memcpy(I->signals[0]->times, ((double [3]){60.9,3.2,35.7}), 3*sizeof(double)); 
		memcpy(I->signals[1]->times, ((double [3]){27.3,3.6,69.2}), 3*sizeof(double)); 
		memcpy(I->signals[2]->times, ((double [3]){61.4,3.2,35.7}), 3*sizeof(double)); 
		memcpy(I->signals[3]->times, ((double [3]){27.3,3.6,69.2}), 3*sizeof(double)); 

		for (int i=4; i<8; i++) {
			memcpy(I->signals[i]->times, ((double [3]){0,0,0}), 3*sizeof(double)); 
		}

	} else if (zoneID==4) {
		ns_len = 66.602;
		ew_len = 80;
		for (int i=0; i<8; i++) {
			memcpy(I->signals[i]->times, ((double [3]){0,0,0}), 3*sizeof(double)); 
		}	
	} else if (zoneID==5) {
		ns_len = 121.32;
		ew_len = 82;	

		memcpy(I->signals[0]->times, ((double [3]){34.6,3.2,46.1}), 3*sizeof(double)); 
		memcpy(I->signals[1]->times, ((double [3]){36.9,3.7,60.2}), 3*sizeof(double)); 
		memcpy(I->signals[2]->times, ((double [3]){36.6,3.2,45.3}), 3*sizeof(double)); 
		memcpy(I->signals[3]->times, ((double [3]){22.4,3.7,74}), 3*sizeof(double)); 
		memcpy(I->signals[4]->times, ((double [3]){8.8,3.6,3.6}), 3*sizeof(double)); 
		memcpy(I->signals[5]->times, ((double [3]){9.8,3.6,87}), 3*sizeof(double)); 
		memcpy(I->signals[6]->times, ((double [3]){11.6,3.6,.5}), 3*sizeof(double)); 
		memcpy(I->signals[7]->times, ((double [3]){0,0,0}), 3*sizeof(double)); 

		
	}

	I->ns_len = ns_len;
	I->ew_len = ew_len;

	return I;
}

int get_inter_zoneID(Intersection I) {
	return I->zoneID;
}

int get_signal_zoneID(Signal *S){
	return S->zoneID;
}

int set_init(Signal *S, Color initial_color){
	S->init = initial_color;
	return 0;
}

Color get_init(Signal *S) {
	return S->init;
}

//to change signal color
int set_light(Signal *S, Color light) {
	S->light = light;
	return 0;
}

Color get_light(Signal *S) {
	return S->light;
}

double get_ns_len(Intersection I) {
	return I->ns_len;
}

double get_ew_len(Intersection I) {
	return I->ew_len;
}

int add_car(Intersection I) {
	I->num_cars++;
	return 0;
}

int get_num_cars(Intersection I) {
	return I->num_cars;
}
