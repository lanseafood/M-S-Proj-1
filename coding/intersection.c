//
//  intersection.c
//  traffic
//
//  Authors: Mingyue Ma, Eisha Nathan, Stefan Henneking
//  01/12/2015
//


#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "intersection.h"

// we could either use some function to "proceed" to the next step in the
// in the signal phase modelled for the whole intersection
// OR phases individually for each traffic signal

struct IntersectionType {
	int zoneID;
	Signal **signals; // 0-3: straight (NESW), 4-7: left (NESW)
	
	// Queues
	LinkedList **laneQueues;
	// Lane flags to control the entering phase on each lane
	// 0: no vehicle is currently entering
	// 1; lane is in use, no entering event allowed
	int **laneFlags;
	// Lane counter: Number of cars entering or crossing a lane
	int **laneCounters;
	// Crossing distance: distance in ft to cross intersection
	double **crossingDistances;
	// Number of lanes for each direction
	int numLanes[4];
	// Total number of cars "on" the intersection
	int num_cars;
	
	// Wait times
	// .. //
	
	// Distance traveling in either direction (going straight)
	double ns_len;
	double ew_len;
};

// Allocate and initialize queues, lane flags and lane counter
static void set_up_lanes( Intersection I );

//creates intersection and its respective signals
Intersection create_intersection(int zoneID) {
	Intersection I = (Intersection) malloc( sizeof(struct IntersectionType));
	I->zoneID = zoneID;
	
	Signal **signals = (Signal **) malloc(8*sizeof(Signal *));
	I->signals = signals;

	for (int i=0; i<8; i++) {
		I->signals[i] = (Signal *) malloc(sizeof(Signal));
	}
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
	
	// Allocate lane arrays for each direction ( N-E-S-W )
	I->laneQueues        = (LinkedList **)malloc(4*sizeof(LinkedList *));
	I->laneFlags         = (int        **)malloc(4*sizeof(int        *));
	I->laneCounters      = (int        **)malloc(4*sizeof(int        *));
	I->crossingDistances = (double     **)malloc(4*sizeof(double     *));

	if (zoneID==1) {
		// Distances
		ns_len = 99.73;
		ew_len = 106;
		
		// Allocate and initialize lane arrays
		I->numLanes[NORTH] = 3; I->numLanes[EAST] = 3;
		I->numLanes[SOUTH] = 3; I->numLanes[WEST] = 4;
		set_up_lanes( I );
		
		// Set crossing distances (j: Right-Straight-Left) for each direction
		for( int j = 0; j < 3; j++ ) {
			I->crossingDistances[NORTH][j] = ns_len;
			I->crossingDistances[ EAST][j] = ew_len;
			I->crossingDistances[SOUTH][j] = ns_len;
			I->crossingDistances[ WEST][j] = ew_len;
		}
		
		// Signal timing
		memcpy(I->signals[0]->times, ((double [3]){34.7, 3.6, 49.3}), 3*sizeof(double));
		memcpy(I->signals[1]->times, ((double [3]){30,3.8,55}), 3*sizeof(double));
		memcpy(I->signals[2]->times, ((double [3]){34.7,3.6,49.3}), 3*sizeof(double));
		memcpy(I->signals[3]->times, ((double [3]){28,3.8,55}), 3*sizeof(double));
		memcpy(I->signals[4]->times, ((double [3]){7,3.6,2.2}), 3*sizeof(double));
		memcpy(I->signals[5]->times, ((double [3]){8,1.8,1.8}), 3*sizeof(double));
		memcpy(I->signals[6]->times, ((double [3]){7,3.6,2.2}), 3*sizeof(double));
		memcpy(I->signals[7]->times, ((double [3]){5,3.6,4.2}), 3*sizeof(double));
	}
	else if (zoneID==2) {
		ns_len = 129.88;
		ew_len = 95;	
		memcpy(I->signals[0]->times, ((double [3]){41.5,3.2,55.4}), 3*sizeof(double));
		memcpy(I->signals[1]->times, ((double [3]){20.2,3.6,76.1}), 3*sizeof(double));
		memcpy(I->signals[2]->times, ((double [3]){41.5,3.2,55.4}), 3*sizeof(double));
		memcpy(I->signals[3]->times, ((double [3]){20.3,3.6,76.2}), 3*sizeof(double));

		for (int i=4; i<8; i++) {
			memcpy(I->signals[i]->times, ((double [3]){0,0,0}), 3*sizeof(double)); 
		}
	}
	else if (zoneID==3) {
		ns_len = 73.51;
		ew_len = 102;	

		memcpy(I->signals[0]->times, ((double [3]){60.9,3.2,35.7}), 3*sizeof(double)); 
		memcpy(I->signals[1]->times, ((double [3]){27.3,3.6,69.2}), 3*sizeof(double)); 
		memcpy(I->signals[2]->times, ((double [3]){61.4,3.2,35.7}), 3*sizeof(double)); 
		memcpy(I->signals[3]->times, ((double [3]){27.3,3.6,69.2}), 3*sizeof(double)); 

		for (int i=4; i<8; i++) {
			memcpy(I->signals[i]->times, ((double [3]){0,0,0}), 3*sizeof(double)); 
		}

	}
	else if (zoneID==4) {
		ns_len = 66.602;
		ew_len = 80;
		for (int i=0; i<8; i++) {
			memcpy(I->signals[i]->times, ((double [3]){0,0,0}), 3*sizeof(double)); 
		}	
	}
	else if (zoneID==5) {
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

	} else ;

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
int set_light(Intersection I, Direction D, int laneID, Color light) {
	I->signals[D]->light = light;
	return 0;
}

Color get_light(Intersection I, Direction D, int laneID) {
	return I->signals[D]->light;
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

LinkedList get_lane_queue( Intersection I, Direction D, int laneID ) {
	return I->laneQueues[D][laneID-1];
}
int set_lane_flag( Intersection I, Direction D, int laneID, int flag ) {
	I->laneFlags[D][laneID-1] = flag;
	return 0;
}
int get_lane_flag( Intersection I, Direction D, int laneID ) {
	return I->laneFlags[D][laneID-1];
}
int change_lane_counter( Intersection I, Direction D, int laneID, int val ) {
	I->laneCounters[D][laneID-1] += val;
	return 0;
}
int get_lane_counter( Intersection I, Direction D, int laneID ) {
	return I->laneCounters[D][laneID-1];
}
double get_crossing_distance( Intersection I, Direction D, Route R ) {
	return I->crossingDistances[D][R];
}

static void set_up_lanes( Intersection I ) {
	// Iterate over directions (N-E-S-W)
	for( int i = 0; i < 4; i++ ) {
		if( I->numLanes[i] > 0 ) {
			I->laneQueues       [i] = (LinkedList *) malloc(I->numLanes[i] * sizeof(LinkedList));
			I->laneFlags        [i] = (int        *) malloc(I->numLanes[i] * sizeof(int       ));
			I->laneCounters     [i] = (int        *) malloc(I->numLanes[i] * sizeof(int       ));
			I->crossingDistances[i] = (double     *) malloc(             3 * sizeof(double    ));
		}
		for( int j = 0; j < I->numLanes[i]; j++ ) {
			I->laneQueues  [i][j] = create_list();
			I->laneFlags   [i][j] = 0;
			I->laneCounters[i][j] = 0;
		}
	}
}

/* eof */
