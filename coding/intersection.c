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

struct IntersectionType {
	int zoneID;
	
	// Vehicle queues
	LinkedList **laneQueues;

	// Lane flags to control the entering phase on each lane
	// 0: no vehicle is currently entering
	// 1; lane is in use, no entering event allowed
	int **laneFlags;
	// Lane counter: Number of cars entering or crossing a lane
	int **laneCounters;
	// Crossing distance: distance in ft to cross intersection
	double **crossingDistances;
	// Number of lanes for each direction N-E-S-W
	int numLanes[4];
	
	// Traffic lights
	// direction(4) x numLanes x maxPhase
	int ***signalStatus;
	
	// Indicates protected / permitted intervals for each signal phase
	// protected = 0: permitted interval (watch out for opposing traffic)
	// protected = 1: dedicated left turn lights are green (no opposing traffic)
	int *protected;

	double *phaseLengths;
	double totalPhaseLength;
	int maxPhase;
	int currPhase;
	
	// Wait times
	// .. //
};

// Allocate and initialize queues, lane flags and lane counter
static void set_up_lanes( Intersection I );

// Used equivalent to Color enumeration GREEN-YELLOW-RED
enum { G, Y, R };

//creates intersection and its respective signals
Intersection create_intersection(int zoneID) {
	Intersection I = (Intersection) malloc( sizeof(struct IntersectionType));
	I->zoneID = zoneID;
	
	// Allocate lane arrays for each direction ( N-E-S-W )
	I->laneQueues        = (LinkedList **)malloc(4*sizeof(LinkedList *));
	I->laneFlags         = (int        **)malloc(4*sizeof(int        *));
	I->laneCounters      = (int        **)malloc(4*sizeof(int        *));
	I->crossingDistances = (double     **)malloc(4*sizeof(double     *));
	I->signalStatus      = (int       ***)malloc(4*sizeof(int       **));

	if( zoneID == 1 ) {
		I->numLanes[NORTH] = 3; I->numLanes[EAST] = 3;
		I->numLanes[SOUTH] = 3; I->numLanes[WEST] = 4;
		I->maxPhase = 12;
	} else if( zoneID == 2 ) {
		I->numLanes[NORTH] = 3; I->numLanes[EAST] = 1;
		I->numLanes[SOUTH] = 2; I->numLanes[WEST] = 2;
		I->maxPhase = 9;
	} else if( zoneID == 3 ) {
		I->numLanes[NORTH] = 3; I->numLanes[EAST] = 1;
		I->numLanes[SOUTH] = 3; I->numLanes[WEST] = 2;
		I->maxPhase = 6;
	} else if( zoneID == 4 ) {
		I->numLanes[NORTH] = 2; I->numLanes[EAST] = 1;
		I->numLanes[SOUTH] = 3; I->numLanes[WEST] = 0;
		I->maxPhase = 0;
	} else if( zoneID == 5 ) {
		I->numLanes[NORTH] = 3; I->numLanes[EAST] = 2;
		I->numLanes[SOUTH] = 3; I->numLanes[WEST] = 2;
		I->maxPhase = 5;
	} else { fprintf(stderr,"Error from create_intersection(): invalid zoneID\n"); exit(1); }
	
	set_up_lanes( I );
	const int P = I->maxPhase;
	
	if (zoneID==1) {
		//                                               RIGHT STRAIGHT  LEFT
		memcpy(I->crossingDistances[NORTH],((double[3]){ 99.73,  99.73,  99.73}),3*sizeof(double));
		memcpy(I->crossingDistances[SOUTH],((double[3]){ 99.73,  99.73,  99.73}),3*sizeof(double));
		memcpy(I->crossingDistances[ EAST],((double[3]){106.00, 106.00, 106.00}),3*sizeof(double));
		memcpy(I->crossingDistances[ WEST],((double[3]){106.00, 106.00, 106.00}),3*sizeof(double));

		memcpy(I->protected   ,((int    [12])     {   1,  1,  0,   0,  0,  0,  1,  1,  0,   0,  0,  0 }),P*sizeof(int));
		memcpy(I->phaseLengths,((double [12])     { 7.0,3.6,2.4,34.9,3.8,2.4,8.0,2.4,2.4,30.0,3.2,2.4 }),P*sizeof(double));
		
		memcpy(I->signalStatus[NORTH][0],((int [12]){ G,  Y,  R,   G,  Y,  R,  R,  R,  R,   R,  R,  R }),P*sizeof(int)); // LEFT TURN
		memcpy(I->signalStatus[NORTH][1],((int [12]){ R,  R,  R,   G,  Y,  R,  R,  R,  R,   R,  R,  R }),P*sizeof(int));
		memcpy(I->signalStatus[NORTH][2],((int [12]){ R,  R,  R,   G,  Y,  R,  R,  R,  R,   R,  R,  R }),P*sizeof(int));
		
		memcpy(I->signalStatus[SOUTH][0],((int [12]){ G,  Y,  R,   G,  Y,  R,  R,  R,  R,   R,  R,  R }),P*sizeof(int)); // LEFT TURN
		memcpy(I->signalStatus[SOUTH][1],((int [12]){ R,  R,  R,   G,  Y,  R,  R,  R,  R,   R,  R,  R }),P*sizeof(int));
		memcpy(I->signalStatus[SOUTH][2],((int [12]){ R,  R,  R,   G,  Y,  R,  R,  R,  R,   R,  R,  R }),P*sizeof(int));
		
		memcpy(I->signalStatus[ EAST][0],((int [12]){ R,  R,  R,   R,  R,  R,  G,  Y,  R,   G,  Y,  R }),P*sizeof(int)); // LEFT TURN
		memcpy(I->signalStatus[ EAST][1],((int [12]){ R,  R,  R,   R,  R,  R,  R,  R,  R,   G,  Y,  R }),P*sizeof(int));
		memcpy(I->signalStatus[ EAST][2],((int [12]){ R,  R,  R,   R,  R,  R,  R,  R,  R,   G,  Y,  R }),P*sizeof(int));
		
		memcpy(I->signalStatus[ WEST][0],((int [12]){ R,  R,  R,   R,  R,  R,  G,  Y,  R,   G,  Y,  R }),P*sizeof(int)); // LEFT TURN
		memcpy(I->signalStatus[ WEST][1],((int [12]){ R,  R,  R,   R,  R,  R,  R,  R,  R,   G,  Y,  R }),P*sizeof(int));
		memcpy(I->signalStatus[ WEST][2],((int [12]){ R,  R,  R,   R,  R,  R,  R,  R,  R,   G,  Y,  R }),P*sizeof(int));
		memcpy(I->signalStatus[ WEST][3],((int [12]){ R,  R,  R,   R,  R,  R,  R,  R,  R,   G,  Y,  R }),P*sizeof(int));
		
		
	}
	else if (zoneID==2) {
		//                                               RIGHT STRAIGHT  LEFT
		memcpy(I->crossingDistances[NORTH],((double[3]){129.88, 129.88, 129.88}),3*sizeof(double));
		memcpy(I->crossingDistances[SOUTH],((double[3]){129.88, 129.88, 129.88}),3*sizeof(double));
		memcpy(I->crossingDistances[ EAST],((double[3]){106.00, 106.00, 106.00}),3*sizeof(double));
		memcpy(I->crossingDistances[ WEST],((double[3]){106.00, 106.00, 106.00}),3*sizeof(double));
		
		memcpy(I->protected   ,((int    [9])        {    1,    0,   0,    1,   1,   0,    0,   0,   0 }),P*sizeof(int));
		memcpy(I->phaseLengths,((double [9])        { 18.3, 23.2, 3.2, 15.1, 3.2, 5.0, 20.3, 3.6, 5.0 }),P*sizeof(double));
		
		memcpy(I->signalStatus[NORTH][0],((int [9]) {    G,    G,   Y,    R,   R,   R,    R,   R,   R }),P*sizeof(int));
		memcpy(I->signalStatus[NORTH][1],((int [9]) {    G,    G,   Y,    R,   R,   R,    R,   R,   R }),P*sizeof(int));
		memcpy(I->signalStatus[NORTH][2],((int [9]) {    G,    G,   Y,    R,   R,   R,    R,   R,   R }),P*sizeof(int));
		
		memcpy(I->signalStatus[SOUTH][0],((int [9]) {    R,    G,   G,    G,   Y,   R,    R,   R,   R }),P*sizeof(int));
		memcpy(I->signalStatus[SOUTH][1],((int [9]) {    R,    G,   G,    G,   Y,   R,    R,   R,   R }),P*sizeof(int));
		
		memcpy(I->signalStatus[ EAST][0],((int [9]) {    R,    R,   R,    R,   R,   R,    G,   Y,   R }),P*sizeof(int));
		
		memcpy(I->signalStatus[ WEST][0],((int [9]) {    R,    R,   R,    R,   R,   R,    G,   Y,   R }),P*sizeof(int));
		memcpy(I->signalStatus[ WEST][1],((int [9]) {    R,    R,   R,    R,   R,   R,    G,   Y,   R }),P*sizeof(int));
	}
	else if (zoneID==3) {
		//                                               RIGHT STRAIGHT  LEFT
		memcpy(I->crossingDistances[NORTH],((double[3]){ 73.51,  73.51,  73.51}),3*sizeof(double));
		memcpy(I->crossingDistances[SOUTH],((double[3]){ 73.51,  73.51,  73.51}),3*sizeof(double));
		memcpy(I->crossingDistances[ EAST],((double[3]){102.00, 102.00, 102.00}),3*sizeof(double));
		memcpy(I->crossingDistances[ WEST],((double[3]){102.00, 102.00, 102.00}),3*sizeof(double));
		
		memcpy(I->protected   ,((int    [9])        {    0,   0,   0,    0,   0,   0 }),5*sizeof(int));
		memcpy(I->phaseLengths,((double [6])        {  61.2,    3.2, 2.4,  27.3,    3.6, 2.4 }),5*sizeof(double));
		
		memcpy(I->signalStatus[NORTH][0],((int [6]) { GREEN, YELLOW, RED,   RED,    RED, RED }),P*sizeof(int));
		memcpy(I->signalStatus[NORTH][1],((int [6]) { GREEN, YELLOW, RED,   RED,    RED, RED }),P*sizeof(int));
		memcpy(I->signalStatus[NORTH][2],((int [6]) { GREEN, YELLOW, RED,   RED,    RED, RED }),P*sizeof(int));
		
		memcpy(I->signalStatus[SOUTH][0],((int [6]) { GREEN, YELLOW, RED,   RED,    RED, RED }),P*sizeof(int));
		memcpy(I->signalStatus[SOUTH][1],((int [6]) { GREEN, YELLOW, RED,   RED,    RED, RED }),P*sizeof(int));
		memcpy(I->signalStatus[SOUTH][2],((int [6]) { GREEN, YELLOW, RED,   RED,    RED, RED }),P*sizeof(int));
		
		memcpy(I->signalStatus[ EAST][0],((int [6]) {   RED,    RED, RED, GREEN, YELLOW, RED }),P*sizeof(int));
		
		memcpy(I->signalStatus[ WEST][0],((int [6]) {   RED,    RED, RED, GREEN, YELLOW, RED }),P*sizeof(int));
		memcpy(I->signalStatus[ WEST][1],((int [6]) {   RED,    RED, RED, GREEN, YELLOW, RED }),P*sizeof(int));
		
	}
	else if (zoneID==4) {
		//                                               RIGHT STRAIGHT  LEFT
		memcpy(I->crossingDistances[NORTH],((double[3]){ 66.60,  66.60,  66.60}),3*sizeof(double));
		memcpy(I->crossingDistances[SOUTH],((double[3]){ 66.60,  66.60,  66.60}),3*sizeof(double));
		memcpy(I->crossingDistances[ EAST],((double[3]){ 80.00,   0.00,  80.00}),3*sizeof(double));
	}
	else if (zoneID==5) {
		//                                               RIGHT STRAIGHT  LEFT
		memcpy(I->crossingDistances[NORTH],((double[3]){121.32, 121.32, 121.32}),3*sizeof(double));
		memcpy(I->crossingDistances[SOUTH],((double[3]){121.32, 121.32, 121.32}),3*sizeof(double));
		memcpy(I->crossingDistances[ EAST],((double[3]){ 82.00,  82.00,  82.00}),3*sizeof(double));
		memcpy(I->crossingDistances[ WEST],((double[3]){ 82.00,  82.00,  82.00}),3*sizeof(double));
		
		// TODO: INTERSECTION 5 TIMING
		memcpy(I->protected   ,((int    [9])        {     0,      0,      0,      0,      0}),P*sizeof(int));
		memcpy(I->phaseLengths,((double [5])        {   0.0,    0.0,    0.0,    0.0,    0.0}),P*sizeof(double));
		
		memcpy(I->signalStatus[NORTH][0],((int [5]) {   RED,    RED,    RED,    RED,    RED}),P*sizeof(int));
		memcpy(I->signalStatus[NORTH][1],((int [5]) {   RED,    RED,    RED,    RED,    RED}),P*sizeof(int));
		memcpy(I->signalStatus[NORTH][2],((int [5]) {   RED,    RED,    RED,    RED,    RED}),P*sizeof(int));
		
		memcpy(I->signalStatus[SOUTH][0],((int [5]) {   RED,    RED,    RED,    RED,    RED}),P*sizeof(int));
		memcpy(I->signalStatus[SOUTH][1],((int [5]) {   RED,    RED,    RED,    RED,    RED}),P*sizeof(int));
		memcpy(I->signalStatus[SOUTH][2],((int [5]) {   RED,    RED,    RED,    RED,    RED}),P*sizeof(int));
		
		memcpy(I->signalStatus[ EAST][0],((int [5]) {   RED,    RED,    RED,    RED,    RED}),P*sizeof(int));
		
		memcpy(I->signalStatus[ WEST][0],((int [5]) {   RED,    RED,    RED,    RED,    RED}),P*sizeof(int));
		memcpy(I->signalStatus[ WEST][1],((int [5]) {   RED,    RED,    RED,    RED,    RED}),P*sizeof(int));
		
	} else ;
	
	// Set current phase and compute time of all phases
	I->currPhase = 0;
	I->totalPhaseLength = 0;
	for(int i = 0; i < I->maxPhase; i++) I->totalPhaseLength += I->phaseLengths[i];
	
	return I;
}

int ***get_signalStatus(Intersection I) {
	return I->signalStatus;
}

double *get_phaseLengths(Intersection I) {
	return I->phaseLengths;
}

double get_totalPhaseLength(Intersection I) {
	return I->totalPhaseLength;
}

LinkedList **get_laneQueues(Intersection I) {
	return I->laneQueues;
}

int *get_numLanes(Intersection I) {
	return I->numLanes;
}

int get_maxPhase(Intersection I) {
	return I->maxPhase;
}

int set_currPhase(Intersection I, int currPhase) {
	I->currPhase = currPhase;
	return 0;
}

int get_currPhase(Intersection I) {
	return I->currPhase;
}

int set_next_phase(Intersection I) {
	I->currPhase  = (I->currPhase + 1) % I->maxPhase;
	return 0;
}

int get_inter_zoneID(Intersection I) {
	return I->zoneID;
}

Color get_light(Intersection I, Direction D, int laneID) {
	return I->signalStatus[D][laneID-1][get_currPhase(I)];
}

int get_protected(Intersection I) {
	return I->protected[I->currPhase];
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
			I->signalStatus     [i] = (int       **) malloc(I->numLanes[i] * sizeof(int      *));
		}
		// Iterate over laneIDs
		for( int j = 0; j < I->numLanes[i]; j++ ) {
			I->laneQueues  [i][j] = create_list();
			I->laneFlags   [i][j] = 0;
			I->laneCounters[i][j] = 0;
			I->signalStatus[i][j] = (int *) malloc(I->maxPhase*sizeof(int));
		}
	}
	I->protected    = (int    *) malloc(I->maxPhase*sizeof(int   ));
	I->phaseLengths = (double *) malloc(I->maxPhase*sizeof(double));
}

/* eof */
