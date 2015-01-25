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
		//signals[direction][laneid]
	int phase[7];
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

	int ***signalStatus; //direction (4) x numLanes x maxPhase
	int ***leftSignalStatus; //direction (4) x numLanes x maxPhase
	double *phaseLengths;

	/* EISHA -- naive signal switching methods
	
	*/

	int maxPhase; //make this +1 than nec.?
	int currPhase;

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

	int ***signalStatus = (int ***) malloc(4*sizeof(int **)); //num_dir (NESW-through);
	int ***leftSignalStatus = (int ***) malloc(4*sizeof(int **));

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
	//for (int i=0; i<)

	I->laneFlags         = (int        **)malloc(4*sizeof(int        *));
	I->laneCounters      = (int        **)malloc(4*sizeof(int        *));
	I->crossingDistances = (double     **)malloc(4*sizeof(double     *));

	//EISHA: setting initial phases --> changing these initial ones will determine
	//how the intersection is synced with itself...?
	for (int i=0; i<8; i++) {
		I->phase[i] = 0;
	}

	if (zoneID==1) {
		// Distances
		ns_len = 99.73;
		ew_len = 106;

		// Allocate and initialize lane arrays
		I->numLanes[NORTH] = 3; I->numLanes[EAST] = 3;
		I->numLanes[SOUTH] = 3; I->numLanes[WEST] = 4;
		set_up_lanes( I );

		/*TODO_SIGNALS
		I->maxPhase = maxPhase;
		for (int i=0; i<4; i++) {
			signalStatus[i] = (int **) malloc(I->numLanes[i]*sizeof(int *)); //num-lanes
			for (int j=0; j<I->numLanes[i]; j++) {
				signalStatus[i][j] = (int *) malloc(I->maxPhase*sizeof(int)); //num-phases
			}
		}

		***Set up signalStatus array (after mapping out phases)***

		*/
		
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

		I->maxPhase = 9; //phases 0-4 (5 cycles back to 0)
		I->phaseLengths = (double *) malloc(I->maxPhase*sizeof(double));
		I->phaseLengths[0] = 34.9;
		I->phaseLengths[1] = 3.8;
		I->phaseLengths[2] = 7;		
		I->phaseLengths[3] = 3.6;
		I->phaseLengths[4] = 8;
		I->phaseLengths[5] = 2.4;
		I->phaseLengths[6] = 7.1;
		I->phaseLengths[7] = 30;		
		I->phaseLengths[8] = 3.2;


		for (int i=0; i<4; i++) {
			
			signalStatus[i] = (int **) malloc(I->numLanes[i]*sizeof(int *)); //num-lanes
			leftSignalStatus[i] = (int **) malloc(I->numLanes[i]*sizeof(int *));
			for (int j=0; j<I->numLanes[i]; j++) {
				signalStatus[i][j] = (int *) malloc(I->maxPhase*sizeof(int)); //num-phases
				leftSignalStatus[i][j] = (int *) malloc(I->maxPhase*sizeof(int));
			}
		}

		for (int i=0; i<4; i++) { //direction
			for (int j=0; j<I->numLanes[i]; j++) { //numlanes
				for (int k=0; k<I->maxPhase; k++) { //phases
					if (k==0) { //phase 0
						if (i==NORTH || i==SOUTH) {
							signalStatus[i][j][k] = GREEN;
						} else if (i==EAST || i==WEST) {
							signalStatus[i][j][k] = RED;
						} else {
							signalStatus[i][j][k] = INV;
						}
					//	printf("HELLO\n");
						leftSignalStatus[i][j][k] = RED;
					} else if (k==1) { //phase 1
						if (i==NORTH || i==SOUTH) {
							signalStatus[i][j][k] = YELLOW;
						} else if (i==EAST || i==WEST) {
							signalStatus[i][j][k] = RED;
						} else {
							signalStatus[i][j][k] = INV;
						}
						leftSignalStatus[i][j][k] = RED;						
					} else if (k==2) { //phase 2
						if (i==NORTH || i==SOUTH) {
							signalStatus[i][j][k] = RED;
							leftSignalStatus[i][j][k] = GREEN;

						} else if (i==EAST || i==WEST) {
							signalStatus[i][j][k] = RED;
							leftSignalStatus[i][j][k] = RED;
						} else {
							signalStatus[i][j][k] = INV;
						}						
					} else if (k==3) { //phase 3
						if (i==NORTH || i==SOUTH) {
							signalStatus[i][j][k] = RED;
							leftSignalStatus[i][j][k] = YELLOW;
						} else if (i==EAST || i==WEST) {
							signalStatus[i][j][k] = RED;
							leftSignalStatus[i][j][k] = RED;
						} else {
							signalStatus[i][j][k] = INV;
						}						
					} else if (k==4) { //phase 4
						if (i==NORTH || i==SOUTH) {
							signalStatus[i][j][k] = RED;
							leftSignalStatus[i][j][k] = RED;
						} else if (i==EAST || i==WEST) {
							signalStatus[i][j][k] = RED;
							leftSignalStatus[i][j][k] = GREEN;
						} else {
							signalStatus[i][j][k] = INV;
						}						
					} else if (k==5) { //phase 5
						if (i==NORTH || i==SOUTH) {
							signalStatus[i][j][k] = RED;
							leftSignalStatus[i][j][k] = RED;
						} else if (i==EAST || i==WEST) {
							signalStatus[i][j][k] = RED;
							leftSignalStatus[i][j][k] = YELLOW;
						} else {
							signalStatus[i][j][k] = INV;
						}						
					} else if (k==6) { //phase 6
						if (i==NORTH || i==SOUTH) {
							signalStatus[i][j][k] = RED;
							leftSignalStatus[i][j][k] = RED;
						} else if (i==EAST || i==WEST) {
							signalStatus[i][j][k] = RED;
							leftSignalStatus[i][j][k] = RED;
						} else {
							signalStatus[i][j][k] = INV;
						}						
					} else if (k==7) { //phase 7
						if (i==NORTH || i==SOUTH) {
							signalStatus[i][j][k] = RED;
							leftSignalStatus[i][j][k] = RED;
						} else if (i==EAST || i==WEST) {
							signalStatus[i][j][k] = GREEN;
							leftSignalStatus[i][j][k] = RED;
						} else {
							signalStatus[i][j][k] = INV;
						}						
					} else if (k==8) { //phase 8
						if (i==NORTH || i==SOUTH) {
							signalStatus[i][j][k] = RED;
							leftSignalStatus[i][j][k] = RED;
						} else if (i==EAST || i==WEST) {
							signalStatus[i][j][k] = YELLOW;
							leftSignalStatus[i][j][k] = RED;
						} else {
							signalStatus[i][j][k] = INV;
						}						
					} 													
				}
			}
		}

	}
	else if (zoneID==2) {
		ns_len = 129.88;
		ew_len = 95;	
		memcpy(I->signals[0]->times, ((double [3]){41.5,3.2,55.4}), 3*sizeof(double));
		memcpy(I->signals[1]->times, ((double [3]){20.2,3.6,76.1}), 3*sizeof(double));
		memcpy(I->signals[2]->times, ((double [3]){41.5,3.2,55.4}), 3*sizeof(double));
		memcpy(I->signals[3]->times, ((double [3]){20.3,3.6,76.2}), 3*sizeof(double));

		for (int i=4; i<8; i++) {
			//memcpy(I->signals[i]->times, ((double [3]){0,0,0}), 3*sizeof(double)); 
			I->signals[i] = NULL;
		}

		// Allocate and initialize lane arrays
		I->numLanes[NORTH] = 2; I->numLanes[EAST] = 2;
		I->numLanes[SOUTH] = 3; I->numLanes[WEST] = 1;
		set_up_lanes( I );

		/*TODO_SIGNALS*/
		I->maxPhase = 5; //phases 0-4 (5 cycles back to 0)
		I->phaseLengths = (double *) malloc(I->maxPhase*sizeof(double));
		I->phaseLengths[0] = 41.5;
		I->phaseLengths[1] = 3.2;
		I->phaseLengths[2] = 31.5;		
		I->phaseLengths[3] = 20.3;
		I->phaseLengths[4] = 3.6;



		for (int i=0; i<4; i++) {
			signalStatus[i] = (int **) malloc(I->numLanes[i]*sizeof(int *)); //num-lanes
			for (int j=0; j<I->numLanes[i]; j++) {
				signalStatus[i][j] = (int *) malloc(I->maxPhase*sizeof(int)); //num-phases
			}
		}

		for (int i=0; i<4; i++) { //direction
			for (int j=0; j<I->numLanes[i]; j++) { //numlanes
				for (int k=0; k<I->maxPhase; k++) { //phases
					if (k==0) { //phase 0
						if (i==NORTH || i==SOUTH) {
							signalStatus[i][j][k] = GREEN;
						} else if (i==EAST || i==WEST) {
							signalStatus[i][j][k] = RED;
						} else {
							signalStatus[i][j][k] = INV;
						}
					} else if (k==1) { //phase 1
						if (i==NORTH || i==SOUTH) {
							signalStatus[i][j][k] = YELLOW;
						} else if (i==EAST || i==WEST) {
							signalStatus[i][j][k] = RED;
						} else {
							signalStatus[i][j][k] = INV;
						}						
					} else if (k==2) { //phase 2
						if (i==NORTH || i==SOUTH) {
							signalStatus[i][j][k] = RED;
						} else if (i==EAST || i==WEST) {
							signalStatus[i][j][k] = RED;
						} else {
							signalStatus[i][j][k] = INV;
						}						
					} else if (k==3) { //phase 3
						if (i==NORTH || i==SOUTH) {
							signalStatus[i][j][k] = RED;
						} else if (i==EAST || i==WEST) {
							signalStatus[i][j][k] = GREEN;
						} else {
							signalStatus[i][j][k] = INV;
						}						
					} else if (k==4) { //phase 4
						if (i==NORTH || i==SOUTH) {
							signalStatus[i][j][k] = RED;
						} else if (i==EAST || i==WEST) {
							signalStatus[i][j][k] = YELLOW;
						} else {
							signalStatus[i][j][k] = INV;
						}						
					} 					
				}
			}
		}

		/***Set up signalStatus array (after mapping out phases)***/

		
	}
	else if (zoneID==3) {
		ns_len = 73.51;
		ew_len = 102;	

		memcpy(I->signals[0]->times, ((double [3]){60.9,3.2,35.7}), 3*sizeof(double)); 
		memcpy(I->signals[1]->times, ((double [3]){27.3,3.6,69.2}), 3*sizeof(double)); 
		memcpy(I->signals[2]->times, ((double [3]){61.4,3.2,35.7}), 3*sizeof(double)); 
		memcpy(I->signals[3]->times, ((double [3]){27.3,3.6,69.2}), 3*sizeof(double)); 

		for (int i=4; i<8; i++) {
			//memcpy(I->signals[i]->times, ((double [3]){0,0,0}), 3*sizeof(double)); 
			I->signals[i] = NULL;
		}

		/*TODO_SIGNALS*/
		I->maxPhase = 5; //phases 0-4 (5 cycles back to 0)
		I->phaseLengths = (double *) malloc(I->maxPhase*sizeof(double));
		I->phaseLengths[0] = 61.2;
		I->phaseLengths[1] = 3.2;
		I->phaseLengths[2] = 4.8;		
		I->phaseLengths[3] = 27.3;
		I->phaseLengths[4] = 3.6;

		for (int i=0; i<4; i++) {
			signalStatus[i] = (int **) malloc(I->numLanes[i]*sizeof(int *)); //num-lanes
			for (int j=0; j<I->numLanes[i]; j++) {
				signalStatus[i][j] = (int *) malloc(I->maxPhase*sizeof(int)); //num-phases
			}
		}

		for (int i=0; i<4; i++) { //direction
			for (int j=0; j<I->numLanes[i]; j++) { //numlanes
				for (int k=0; k<I->maxPhase; k++) { //phases
					if (k==0) { //phase 0
						if (i==NORTH || i==SOUTH) {
							signalStatus[i][j][k] = GREEN;
						} else if (i==EAST || i==WEST) {
							signalStatus[i][j][k] = RED;
						} else {
							signalStatus[i][j][k] = INV;
						}
					} else if (k==1) { //phase 1
						if (i==NORTH || i==SOUTH) {
							signalStatus[i][j][k] = YELLOW;
						} else if (i==EAST || i==WEST) {
							signalStatus[i][j][k] = RED;
						} else {
							signalStatus[i][j][k] = INV;
						}						
					} else if (k==2) { //phase 2
						if (i==NORTH || i==SOUTH) {
							signalStatus[i][j][k] = RED;
						} else if (i==EAST || i==WEST) {
							signalStatus[i][j][k] = RED;
						} else {
							signalStatus[i][j][k] = INV;
						}						
					} else if (k==3) { //phase 3
						if (i==NORTH || i==SOUTH) {
							signalStatus[i][j][k] = RED;
						} else if (i==EAST || i==WEST) {
							signalStatus[i][j][k] = GREEN;
						} else {
							signalStatus[i][j][k] = INV;
						}						
					} else if (k==4) { //phase 4
						if (i==NORTH || i==SOUTH) {
							signalStatus[i][j][k] = RED;
						} else if (i==EAST || i==WEST) {
							signalStatus[i][j][k] = YELLOW;
						} else {
							signalStatus[i][j][k] = INV;
						}						
					} 					
				}
			}
		}
	}
	else if (zoneID==4) {
		ns_len = 66.602;
		ew_len = 80;
		for (int i=0; i<8; i++) {
			memcpy(I->signals[i]->times, ((double [3]){0,0,0}), 3*sizeof(double)); 
			I->signals[i] = NULL;
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

		/*TODO_SIGNALS
		I->maxPhase = maxPhase;
		for (int i=0; i<4; i++) {
			signalStatus[i] = (int **) malloc(I->numLanes[i]*sizeof(int *)); //num-lanes
			for (int j=0; j<I->numLanes[i]; j++) {
				signalStatus[i][j] = (int *) malloc(I->maxPhase*sizeof(int)); //num-phases
			}
		}

		***Set up signalStatus array (after mapping out phases)***

		*/

	} else ;


	I->ns_len = ns_len;
	I->ew_len = ew_len;
	I->signalStatus = signalStatus;
	I->leftSignalStatus = leftSignalStatus;
	I->currPhase = I->maxPhase-1;

	return I;
}

//should only be used for left turns
int set_red(Intersection I, int num) {
	I->phase[num] = 2;
	return 0;
}

int get_curr_phase(Intersection I, int curr) {
	return I->phase[curr];
}


int *get_phase(Intersection I) {
	return I->phase;
}

int change_phase(Intersection I, int num) {
	//num is 0-7, determines which dir and which signal (through/left)
	I->phase[num] = I->phase[num] % 3;
	return 0;
}

Signal **get_through_signals(Intersection I) {
	Signal **signals = (Signal **) malloc(4*sizeof(Signal *));
	Signal **i_signal = I->signals;

	for (int i=0; i<4; i++) {
		signals[i] = i_signal[i];
	}
	return signals;
}

Signal **get_left_signals(Intersection I) {
	Signal **signals = (Signal **) malloc(4*sizeof(Signal *));
	Signal **i_signal = I->signals;

	for (int i=0; i<4; i++) {
		signals[i] = i_signal[i+4];
	}
	return signals;
}

int ***get_signalStatus(Intersection I) {
	return I->signalStatus;
}

int ***get_leftSignalStatus(Intersection I) {
	//printf("yo\n");
	return I->leftSignalStatus;
}

double *get_phaseLengths(Intersection I) {
	return I->phaseLengths;
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

int get_currPhase(Intersection I) {
	return I->currPhase;
}

//when setting next phase need to change signals...? nope
int set_next_phase(Intersection I) {
	I->currPhase  = (I->currPhase + 1) % I->maxPhase;
	/*
	depends on intersection id?
	*/
	return 0;
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
	//return I->signals[D]->light;
	//printf("curr phase: %d, laneID: %d, dir: %d\n", get_currPhase(I), laneID, D);
	return I->signalStatus[D][laneID-1][get_currPhase(I)];
	//signal status
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
