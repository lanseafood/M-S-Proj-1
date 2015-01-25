//
//  simApplication.c
//  traffic
//
//  Authors: Mingyue Ma, Eisha Nathan, Stefan Henneking
//  01/12/2015
//

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "event.h"
#include "intersection.h"
#include "linkedList.h"
#include "priorityQueue.h"
#include "section.h"
#include "simApplication.h"

#define IAT  5.0f    // Global Inter Arrival Time (mean value for exponential distribution)
#define VEL 51.0f    // Maximum Vehicle Speed in ft/sec (35mph ~ 51ft/sec)
#define ACC 10.0f    // Vehicle Acceleration in ft/sec^2
#define VHL 15.0f    // Vehicle length
#define SDD 30.0f    // Safety Distance in ft while driving
#define SDT ( ( SDD + VHL ) / VEL ) // Safety Distance in sec while driving
#define SDQ  5.0f    // Safety Distance in ft while queueing up
#define SUD  1.0f    // Start-up delay / lost time (first car: delay when traffic light switches to green)

// Pointer declaration: compare function for priority queue
int (*compare_to)( void*, void* );

// Intersections
static Intersection IS_1, IS_2, IS_3, IS_4, IS_5;

// Road sections between intersections (Peachtree St NE)
static Section S_1, S_2, S_3, S_4, S_5, S_6;

// State variables
static int arrivals;            // Total number of global vehicle arrivals
static int departures;          // Total number of global vehicle departures
static double totalWaitTime;    // Sum of all waiting times at intersections of already departed vehicles
static double totalTravelTime;  // Sum of travel times of already departed vehicles

// Origin and destination zones
static int origins[11] =
{ 101, 102, 103, 106, 112, 113, 114, 115, 121, 122, 123 };
static int destinations[11] =
{ 201, 202, 203, 206, 212, 213, 214, 215, 221, 222, 223 };

// Origin and destination probabilities
static double p_origins[11] =
{ 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
static double p_destinations[11][11] =
{
	{ 0.00, 0.10, 0.00, 0.00, 0.00, 0.00, 0.80, 0.00, 0.00, 0.00, 0.10 }, // origin 101
	{ 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 1.00, 0.00, 0.00, 0.00, 0.00 }, // origin 102
	{ 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 1.00, 0.00, 0.00, 0.00, 0.00 }, // origin 103
	{ 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 1.00, 0.00, 0.00, 0.00, 0.00 }, // origin 106
	{ 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 1.00, 0.00, 0.00, 0.00, 0.00 }, // origin 112
	{ 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 1.00, 0.00, 0.00, 0.00, 0.00 }, // origin 113
	{ 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 1.00, 0.00, 0.00, 0.00, 0.00 }, // origin 114
	{ 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 1.00, 0.00, 0.00, 0.00, 0.00 }, // origin 115
	{ 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 1.00, 0.00, 0.00, 0.00, 0.00 }, // origin 121
	{ 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 1.00, 0.00, 0.00, 0.00, 0.00 }, // origin 122
	{ 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 1.00, 0.00, 0.00, 0.00, 0.00 }  // origin 123
};

// Auxiliary function for computing velocity, distance, time while accelerating
static inline double calc_velocity( double v_cur, double t ) {
	// v = v_cur + a*t
	return fmin(v_cur + ACC * t, VEL);
}

static inline double calc_distance( double v_cur, double t ) {
	// s = v_cur*t + 0.5*a*t^2
	double t_acc = (VEL - v_cur) / ACC;
	double d_acc = v_cur*t_acc + 0.5*ACC*t_acc*t_acc;
	return (t_acc <= t) ? d_acc + (t-t_acc)*VEL : d_acc;
}

static inline double calc_time( double v_cur, double dist ) {
	// t = -v_cur/a + sqrt( v_cur^2 + 2*a*s )/a
	double t_acc = (VEL - v_cur) / ACC;
	double d_acc = v_cur*t_acc + 0.5*ACC*t_acc*t_acc;
	return (d_acc <= dist) ? t_acc + (dist-d_acc)/VEL : -v_cur/ACC + sqrt(v_cur*v_cur+2.0*ACC*dist) / ACC;
}

/* ======================================================================================= */
/* --------------------------------- Function prototypes --------------------------------- */
/* --------------------------------------------------------------------------------------- */

/* Event initialization */
static Event init_event( double timestamp, void *object, TypeOfObject OT, TypeOfEvent ET, fptr cb );

/* Generates a random number from [0,1) */
static double urand();
/* Generates a random number from an exponential distribution with mean = IAT */
static double randexp();

/* Auxiliary functions */
static int pick_origin();
static int pick_destination( int origin );

/* Event handler */
static void IS1_initial(void *P);
static void IS2_initial(void *P);
static void IS3_initial(void *P);
static void IS4_initial(void *P);
static void IS5_initial(void *P);

static void global_arrival( void *P );
static void global_departure( void *P );

static void IS_1_signal( void *P );

static void IS_2_signal( void *P );
/*static void IS_3_signal( void *P );
static void IS_4_signal( void *P );
static void IS_5_signal( void *P );
*/

static void IS_1_N_arrival( void* P );
static void IS_1_E_arrival( void* P );
static void IS_1_S_arrival( void* P );
static void IS_1_W_arrival( void* P );

static void IS_1_N_entering( void* P );
static void IS_1_E_entering( void* P );
static void IS_1_S_entering( void* P );
static void IS_1_W_entering( void* P );

static void IS_1_N_crossing( void* P );
static void IS_1_E_crossing( void* P );
static void IS_1_S_crossing( void* P );
static void IS_1_W_crossing( void* P );

static void IS_1_N_departure( void* P );
static void IS_1_E_departure( void* P );
static void IS_1_S_departure( void* P );
static void IS_1_W_departure( void* P );

/* --------------------------------------------------------------------------------------- */
/* ======================================================================================= */

void create_sim( double simEnd ) {
	
	// Set compare function pointer for priority queue
	compare_to = compare_events;
	
	// Initialize Road Network
	// Create Intersections
	IS_1 = create_intersection( 1 );
	IS_2 = create_intersection( 2 );
	IS_3 = create_intersection( 3 );
	IS_4 = create_intersection( 4 );
	IS_5 = create_intersection( 5 );
	// Create Sections
	S_1 = create_section( 1 );
	S_1 = create_section( 2 );
	S_1 = create_section( 3 );
	S_1 = create_section( 4 );
	S_1 = create_section( 5 );
	S_1 = create_section( 6 );
	
	// Initialize state variables
	arrivals = 0;
	departures = 0;
	totalWaitTime = 0.0;
	totalTravelTime = 0.0;
		
	// Call engine setup
	set_up_sim();

	Event inter1 = init_event(0, IS_1, INTERSECTION, IS_1_SIGNAL, IS_1_signal); 
	
	// Initialize and schedule first event (global arrival)
	Event firstArrival = init_event( -1, NULL, VEHICLE, GLOBAL_ARRIVAL, global_arrival );
	schedule_event( firstArrival );
	
	// Create traffic signal events for each intersection
	// .. //
//	Event signal1 = init_event(0, NULL, INTERSECTION, IS1_INITIAL, IS1_initial);
//	Event signal2 = init_event(0, NULL, INTERSECTION, IS2_INITIAL, IS2_initial);
/*	Event signal3 = init_event(0, NULL, INTERSECTION, IS3_INITIAL, IS3_initial);
	Event signal4 = init_event(0, NULL, INTERSECTION, IS4_INITIAL, IS4_initial);
	Event signal5 = init_event(0, NULL, INTERSECTION, IS5_INITIAL, IS5_initial);
*/
//	schedule_event(signal1);
//	schedule_event(signal2);
/*	schedule_event(signal3);
	schedule_event(signal4);
	schedule_event(signal5);
*/

	// Run simulation
	run_sim( simEnd );
	
	// Calculate output values
	// .. //
	
	// Print stats
	printf( "\nMean Inter Arrival Time: %5.2f ( rate = %6.3f )\n", (double)IAT, (double)(1.0/IAT) );
}

// Initialize new event (without scheduling it)
static Event init_event( double timestamp, void *object, TypeOfObject OT, TypeOfEvent ET, fptr cb ) {
	// Create new event
	Event E = create_event();
	// Set timestamp
	if( timestamp == -1 ) timestamp = get_sim_time() + randexp();
	if( timestamp  <  0 ) { fprintf(stderr,"Error from init_event(): negative timestamp\n"); exit(1); }
	if( set_timestamp( E, timestamp ) != 0 ) { fprintf(stderr,"Error from init_event(): set_timestamp\n"); exit(1); }
	// Set object
	if( object != NULL ) {
		if( set_object( E, object ) != 0 ) { fprintf(stderr,"Error from init_event(): set_object\n"); exit(1); };
	}
	// Set type of object
	if( set_object_type( E, OT ) != 0 ) { fprintf(stderr,"Error from init_event(): set_object_type\n"); exit(1); }
	// Set type of event
	if( set_event_type( E, ET ) != 0 ) { fprintf(stderr,"Error from init_event(): set_event_type\n"); exit(1); }
	// Set callback function
	if( set_callback( E, cb ) != 0 ) { fprintf(stderr,"Error from init_event(): set_callback\n"); exit(1); }
	return E;
}

// Generate random number in [0,1)
static double urand() {
	return ( (double)rand() / (double)RAND_MAX );
}

// Generate random number from exp. distribution with mean IAT
static double randexp() {
	return -IAT*( log( 1.0 - urand() ) );
}


// Pick vehicle origin zone
static int pick_origin() {
	int origin_id = 0;
	double r = urand(), p = 0.0;
	
	for( int i = 0; i < 11; i++ ) {
		p += p_origins[i];
		if( p > r ) {
			origin_id = i;
			break;
		}
	}
	if( origin_id == -1 ) { fprintf(stderr,"Error from pick_origin(): Invalid origin zone\n"); exit(1); }
	return origin_id;
}

// Pick vehicle destination zone
static int pick_destination( int origin_id ) {
	int destination_id = -1;
	double r = urand(), p = 0.0;
	
	for( int i = 0; i < 11; i++ ) {
		p += p_destinations[origin_id][i];
		if( p > r ) {
			destination_id = i;
			break;
		}
	}
	if( destination_id == -1 ) { fprintf(stderr,"Error from pick_destination(): Invalid destination zone\n"); exit(1); }
	return destination_id;
}

/* ======================================================================================== *\
|*                                                                                          *|
|*                               GLOBAL ARRIVAL & DEPARTURE                                 *|
|*                                                                                          *|
\* ---------------------------------------------------------------------------------------- */

static void IS1_initial(void *P) {
	Event E = (Event) P;

	printf("%6.2f, IS 1 init\n",
		   get_sim_time());
	

	Intersection I1 = create_intersection(1);
	
	set_object(E, I1);

	double *phaseLengths = get_phaseLengths(I1);

	//or do we need a 'newEvent' to set up the next signal switch?
	set_timestamp(E, get_sim_time() + phaseLengths[get_currPhase(I1)]);
	set_event_type(E, IS_1_SIGNAL);
	set_callback(E, IS_1_signal);

	schedule_event(E);
}

static void IS2_initial(void *P) {
	Event E = (Event) P;
	
	printf("%6.2f, IS 2 init\n",
		   get_sim_time());

	Intersection I2 = create_intersection(2);
	
	set_object(E, I2);

	double *phaseLengths = get_phaseLengths(I2);

	//or do we need a 'newEvent' to set up the next signal switch?
	set_timestamp(E, get_sim_time() + phaseLengths[get_currPhase(I2)]);
	set_event_type(E, IS_2_SIGNAL);
	set_callback(E, IS_2_signal);

	schedule_event(E);
}

/*
static void IS3_initial(void *P) {
	Event E = (Event) P;
	Intersection I3 = create_intersection(3);
	set_up_lanes(I3);
	
	set_object(E, I3);

	int *phaseLengths = get_phaseLengths(I3);

	//or do we need a 'newEvent' to set up the next signal switch?
	set_timestamp(E, get_sim_time() + phaseLengths[get_currPhase(I3)]);
	set_event_type(E, IS_3_SIGNAL);
	set_callback(E, IS_3_signal);

	schedule_event(E);
}

static void IS4_initial(void *P) {
	Event E = (Event) P;
	Intersection I4 = create_intersection(4);
	set_up_lanes(I4);
	
	set_object(E, I4);

	int *phaseLengths = get_phaseLengths(I4);

	//or do we need a 'newEvent' to set up the next signal switch?
	//set_timestamp(E, get_sim_time() + phaseLengths[get_currPhase(I1)]);
	//set_event_type(E, IS_1_SIGNAL);
	//set_callback(E, IS_1_SIGNAL);

	//schedule_event(E);

	//no lights here..?

}

static void IS5_initial(void *P) {
	Event E = (Event) P;
	Intersection I5 = create_intersection(5);
	set_up_lanes(I5);
	
	set_object(E, I5);

	int *phaseLengths = get_phaseLengths(I5);

	//or do we need a 'newEvent' to set up the next signal switch?
	set_timestamp(E, get_sim_time() + phaseLengths[get_currPhase(I5)]);
	set_event_type(E, IS_5_SIGNAL);
	set_callback(E, IS_5_signal);

	schedule_event(E);
}
*/

// Event handler for a global vehicle arrival
static void global_arrival( void* P ) {
	Event E = (Event) P;
	if( E == NULL ) { fprintf(stderr,"Error from global_arrival(): E is NULL\n"); exit(1); }
	
	// Create new Vehicle and set origin, destination, arrival time
	Vehicle V = create_vehicle();
	int origin_id = pick_origin();
	set_origin( V, origins[origin_id] );
	set_destination( V, destinations[pick_destination( origin_id )] );
	set_velocity( V, VEL );
	set_arrival_time( V, get_sim_time() );
	set_object( E, V );
	
	// Schedule local arrival event depending on origin zone
	// Change type of event and set event handler
	switch( origins[origin_id] ) {
		case 101:
			set_event_type( E, IS_1_S_ARRIVAL );
			set_callback  ( E, IS_1_S_arrival );
			break;
		case 102:
			set_event_type( E, IS_1_E_ARRIVAL );
			set_callback  ( E, IS_1_E_arrival );
			break;
		case 103:
			break;
		case 106:
			break;
		case 112:
			break;
		case 113:
			break;
		case 114:
			break;
		case 115:
			break;
		case 121:
			break;
		case 122:
			break;
		case 123:
			set_event_type( E, IS_1_W_ARRIVAL );
			set_callback  ( E, IS_1_W_arrival );
			break;
		default:
			fprintf(stderr,"Error from global_arrival(): invalid origin zone, E is NULL\n"); exit(1);
	}
	schedule_event( E );
	
	// Print output
	printf("%6.2f, Global Arrival      , Vehicle ID: %3d, Origin Zone: %d, Destination Zone %d\n",
			get_sim_time(), get_id(V), get_origin(V), get_destination(V) );
	
	// Schedule next global vehicle arrival
	Event nextArrival = init_event( -1, NULL, VEHICLE, GLOBAL_ARRIVAL, global_arrival );
	schedule_event( nextArrival );
	
	// Change state variable
	arrivals++;
}

// Event handler for a global vehicle departure
static void global_departure( void* P ) {
	Event E = (Event) P;
	if( E == NULL ) { fprintf(stderr,"Error from global_departure(): E is NULL\n"); exit(1); }
	Vehicle V = get_object( E );
	if( V == NULL ) { fprintf(stderr,"Error from global_departure(): V is NULL\n"); exit(1); }
	
	printf("%6.2f, Global Departure      , Vehicle ID: %3d, Origin Zone: %d, Destination Zone %d\n",
		   get_sim_time(), get_id(V), get_origin(V), get_destination(V) );
	
	// Change state variables
	totalTravelTime += ( get_sim_time() - get_arrival_time( V ) );
	totalWaitTime += get_wait_time( V );
	departures++;
	
	// Clean up
	free_event(E);
}

/* ======================================================================================== *\
|*                                                                                          *|
|*                            INTERSECTIONS : TRAFFIC SIGNALS                               *|
|*                                                                                          *|
\* ---------------------------------------------------------------------------------------- */

/* IGNORE_ME
//intersection 1, north through lights
static void IS_1_signal_NT( void* P ) {
	Event E = (Event) P;

	if( E == NULL ) { fprintf(stderr,"Error from IS_1_signal(): E is NULL\n"); exit(1); }
	Intersection I = get_object( E );
	if( I == NULL ) { fprintf(stderr,"Error from IS_1_signal(): I is NULL\n"); exit(1); }
	
	printf("%6.2f, IS 1 Signal North Through, Intersection Zone ID: %3d\n",
		   get_sim_time(), get_inter_zoneID(I) );
	
	Signal **signals = get_through_signals(I);
	int curr = 0;
	int dir = NORTH;

	Signal *s = signals[curr];

	Event newEvent;
	LinkedList **laneQueues = get_laneQueues(I);
	int *numLanes = get_numLanes(I);
	

	change_phase(I, curr); //change phase of north through signal
	int curr_phase = get_curr_phase(I, curr);

	if (curr_phase == 2) { //if we're at a red light, the left red needs to be set also
		set_red(I, curr+4); //set the corresponding left to be red
	}

	int *phase = get_phase(I); //0-7 array for different phases of each traffic light
	//CHECK phase[curr] should equal curr_phase at this point?

	//schedule the first event in each lane to enter intersection
	for (int i=0; i<numLanes[dir]; i++) { //for all lanes in this direction 
		//how to differentiate between lanes going left etc
		if (phase[curr]==GREEN || phase[curr]==YELLOW) {
			if (get_list_counter(laneQueues[dir][i]) > 0) {
				newEvent = peek_from_list(laneQueues[dir][i]);
				set_timestamp(newEvent, get_sim_time());
				set_event_type(newEvent, IS_1_N_ENTERING);
				set_callback(newEvent, IS_1_N_entering);
			}
			schedule_event(newEvent);
		}
	}
	set_timestamp(E, get_sim_time() + s->times[curr]); //double check this logic
	schedule_event(E);
}
*/

static void IS_1_signal( void* P ) {
	Event E = (Event) P;
	if( E == NULL ) { fprintf(stderr,"Error from IS_1_signal(): E is NULL\n"); exit(1); }
	Intersection I = get_object( E );
	if( I == NULL ) { fprintf(stderr,"Error from IS_1_signal(): I is NULL\n"); exit(1); }
	
	printf("%6.2f, IS 1 Signal, Intersection Zone ID: %3d\n",
		   get_sim_time(), get_inter_zoneID(I) );
	
	int ***signalStatus = get_signalStatus(I);
	int ***leftSignalStatus = get_leftSignalStatus(I);
	double *phaseLengths = get_phaseLengths(I);

	Event newEvent;
	LinkedList **laneQueues = get_laneQueues(I);
	int maxPhase = get_maxPhase(I);
	int *numLanes = get_numLanes(I);
	

	set_next_phase(I); //update to next phase & change appropriate signals
	int curr_phase = get_currPhase(I);

	//left turns..?
	//IMPORTANT: left turn indexing 

	//hmmmmmm

	for (int i=0; i<4; i++) {
		for (int j=0; j<numLanes[i]; j++) {
			if (signalStatus[i][j][curr_phase]==GREEN || signalStatus[i][j][curr_phase]==YELLOW) {
				if (get_list_counter(laneQueues[i][j]) > 0 && get_lane_flag(I, i, j)==0) {
					newEvent = peek_from_list(laneQueues[i][j]);
					set_timestamp(newEvent, get_sim_time());
					if (i==NORTH && (j>0)) {
						set_event_type(newEvent, IS_1_N_ENTERING);
						set_callback(newEvent, IS_1_N_entering);
					} else if (i==EAST && (j>0)) {
						//TODO: left turn should only enter intersection if 
						//west lane 1 cars are not going straight
						set_event_type(newEvent, IS_1_E_ENTERING);
						set_callback(newEvent, IS_1_E_entering);
					} else if (i==SOUTH && (j>0)) {
						//TODO: left turn should only enter if north lane 1/2
						//cars are not going straight
						set_event_type(newEvent, IS_1_S_ENTERING);
						set_callback(newEvent, IS_1_S_entering);
					} else if (i==WEST && (j>0)) {
						//TODO: left turn should only enter if east lane 1 
						//is not going straight
						set_event_type(newEvent, IS_1_W_ENTERING);
						set_callback(newEvent, IS_1_W_entering);
					} else {exit(-1);}
				} else if (leftSignalStatus[i][j][curr_phase]==GREEN || leftSignalStatus[i][j][curr_phase]==YELLOW) {
					newEvent = peek_from_list(laneQueues[i][j]);//j==0
					set_timestamp(newEvent, get_sim_time());
					if (i==NORTH && (j==0)) {
						set_event_type(newEvent, IS_1_N_ENTERING);
						set_callback(newEvent, IS_1_N_entering);
					} else if (i==EAST && (j==0)) {
						//TODO: left turn should only enter intersection if 
						//west lane 1 cars are not going straight
						set_event_type(newEvent, IS_1_E_ENTERING);
						set_callback(newEvent, IS_1_E_entering);
					} else if (i==SOUTH && (j==0)) {
						//TODO: left turn should only enter if north lane 1/2
						//cars are not going straight
						set_event_type(newEvent, IS_1_S_ENTERING);
						set_callback(newEvent, IS_1_S_entering);
					} else if (i==WEST && (j==0)) {
						//TODO: left turn should only enter if east lane 1 
						//is not going straight
						set_event_type(newEvent, IS_1_W_ENTERING);
						set_callback(newEvent, IS_1_W_entering);
					} else {exit(-1);}
					
				}

				schedule_event(newEvent);
			}
		} 
	}
	set_timestamp(E, get_sim_time() + phaseLengths[get_currPhase(I)]);
	schedule_event(E);
}

static void IS_2_signal( void* P ) {
	Event E = (Event) P;
	if( E == NULL ) { fprintf(stderr,"Error from IS_1_signal(): E is NULL\n"); exit(1); }
	Intersection I = get_object( E );
	if( I == NULL ) { fprintf(stderr,"Error from IS_1_signal(): I is NULL\n"); exit(1); }
	
	printf("%6.2f, IS 2 Signal, Intersection Zone ID: %3d\n",
		   get_sim_time(), get_inter_zoneID(I) );
	
	int ***signalStatus = get_signalStatus(I);
	double *phaseLengths = get_phaseLengths(I);

	Event newEvent;
	LinkedList **laneQueues = get_laneQueues(I);
	int maxPhase = get_maxPhase(I);
	int *numLanes = get_numLanes(I);

	set_next_phase(I); //update to next phase & change appropriate signals

	for (int i=0; i<4; i++) {
		for (int j=0; j<numLanes[i]; j++) {
			//for (int k=0; k<maxPhase; k++) {
				if (signalStatus[i][j][get_currPhase(I)]==GREEN || signalStatus[i][j][get_currPhase(I)]==YELLOW) {
					if (get_list_counter(laneQueues[i][j]) > 0) {
						newEvent = peek_from_list(laneQueues[i][j]);
						set_timestamp(newEvent, get_sim_time());
						//set_object(E, v);
						//set_object_type(E, VEHICLE);
						//check left turns and directions?
						if (i==NORTH) {
							set_event_type(newEvent, IS_1_N_ENTERING);
							set_callback(newEvent, IS_1_N_entering);
						} else if (i==EAST) {
							//TODO: left turn should only enter intersection if 
							//west lane 1 cars are not going straight
							set_event_type(newEvent, IS_1_E_ENTERING);
							set_callback(newEvent, IS_1_E_entering);
						} else if (i==SOUTH) {
							//TODO: left turn should only enter if north lane 1/2
							//cars are not going straight
							set_event_type(newEvent, IS_1_S_ENTERING);
							set_callback(newEvent, IS_1_S_entering);
						} else if (i==WEST) {
							//TODO: left turn should only enter if east lane 1 
							//is not going straight
							set_event_type(newEvent, IS_1_W_ENTERING);
							set_callback(newEvent, IS_1_W_entering);
						} else {exit(-1);}
					}
					schedule_event(newEvent);
				}
			//}
		}
	}
	set_timestamp(E, get_sim_time() + phaseLengths[get_currPhase(I)]);
	schedule_event(E);
	// proceed to next IS1 signal status
	// schedule next IS1 signal event
	
	//TODO_EISHA
	//update field
	//timestamp = curr time + len of phase

	// schedule entering event for corresponding lanes
}

/* ======================================================================================== *\
|*                                                                                          *|
|*                            INTERSECTION 1 : VEHICLE EVENTS                               *|
|*                                                                                          *|
\* ---------------------------------------------------------------------------------------- */

static void IS_1_N_arrival( void* P ) {
	Event E = (Event) P;
	if( E == NULL ) { fprintf(stderr,"Error from IS_1_N_arrival(): E is NULL\n"); exit(1); }
	Vehicle V = get_object( E );
	if( V == NULL ) { fprintf(stderr,"Error from IS_1_N_arrival(): V is NULL\n"); exit(1); }
	
	printf("%6.2f, IS 1 North Arrival  , Vehicle ID: %3d, Origin Zone: %d, Destination Zone %d\n",
		   get_sim_time(), get_id(V), get_origin(V), get_destination(V) );
	
	set_event_type( E, IS_1_N_ENTERING );
	set_callback( E, IS_1_N_entering );
	
	// Queue up or enter intersection depending on vehicle destination
	int newLane;
	switch( get_destination( V ) ) {
		case 223: // Right turn
		{
			newLane = 3;
			break;
		}
		case 201: // Straight
		{
			// Choose shorter lane
			newLane = ( get_list_counter( get_lane_queue( IS_1, NORTH, 3 ) ) <
					   get_list_counter( get_lane_queue( IS_1, NORTH, 2 ) )
						  ) ? 3 : 2;
			break;
		}
		case 202: // Left turn
		{
			newLane = 1;
			break;
		}
		default:
			fprintf(stderr,"Error from IS_1_N_arrival(): unexpected destination zone\n"); exit(1);
	}
	set_laneID( V, newLane );
	// Check traffic signal
	if( get_light( IS_1, NORTH, newLane ) == INV ) { fprintf(stderr,"IS_1_N_arrival(): INV traffic signal\n"); }
	if( get_light( IS_1, NORTH, newLane ) != GREEN ) {
		// Put vehicle in queue
		set_velocity( V, 0.0 );
		add_to_list( get_lane_queue( IS_1, NORTH, newLane ), E );
	} else {
		// If queue is empty, and no vehicle is currently entering,
		// schedule entering event directly
		if(   get_list_counter( get_lane_queue( IS_1, NORTH, newLane ) ) == 0
		   && get_lane_flag( IS_1, NORTH, newLane ) == 0 )
		{
			add_to_list( get_lane_queue( IS_1, NORTH, newLane ), E );
			schedule_event( E );
		} // Else: put vehicle in queue
		else {
			set_velocity( V, 0.0 );
			add_to_list( get_lane_queue( IS_1, NORTH, newLane ), E );
		}
	}
	set_wait_time_buf( V, get_sim_time() );
}

static void IS_1_E_arrival( void* P ) {
	Event E = (Event) P;
	if( E == NULL ) { fprintf(stderr,"Error from IS_1_E_arrival(): E is NULL\n"); exit(1); }
	Vehicle V = get_object( E );
	if( V == NULL ) { fprintf(stderr,"Error from IS_1_E_arrival(): V is NULL\n"); exit(1); }
	
	printf("%6.2f, IS 1 East  Arrival  , Vehicle ID: %3d, Origin Zone: %d, Destination Zone %d\n",
		   get_sim_time(), get_id(V), get_origin(V), get_destination(V) );
	
	set_event_type( E, IS_1_E_ENTERING );
	set_callback( E, IS_1_E_entering );
	
	// Queue up or enter intersection, lane depends on vehicle destination
	int newLane;
	switch( get_destination( V ) ) {
		case 223: // Straight
		{
			// Choose shorter lane
			newLane = ( get_list_counter( get_lane_queue( IS_1, EAST, 3 ) ) <
						    get_list_counter( get_lane_queue( IS_1, EAST, 2 ) )
						  ) ? 3 : 2;
			break;
		}
		case 201: // Left turn
		{
			newLane = 1;
			break;
		}
		default:  // Right turn
		{
			newLane = 3;
			break;
		}
	}
	set_laneID( V, newLane );
	// Check traffic signal
	if( get_light( IS_1, EAST, newLane ) == INV ) { fprintf(stderr,"IS_1_E_arrival(): INV traffic signal\n"); }
	if( get_light( IS_1, EAST, newLane ) != GREEN ) {
		// Put vehicle in queue
		set_velocity( V, 0.0 );
		add_to_list( get_lane_queue( IS_1, EAST, newLane ), E );
	} else {
		// If queue is empty, and no vehicle is currently entering,
		// schedule entering event directly
		if(   get_list_counter( get_lane_queue( IS_1, EAST, newLane ) ) == 0
		   && get_lane_flag( IS_1, EAST, newLane ) == 0 )
		{
			add_to_list( get_lane_queue( IS_1, EAST, newLane ), E );
			schedule_event( E );
		} // Else: put vehicle in queue
		else {
			set_velocity( V, 0.0 );
			add_to_list( get_lane_queue( IS_1, EAST, newLane ), E );
		}
	}
	set_wait_time_buf( V, get_sim_time() );
}

static void IS_1_S_arrival( void* P ) {
	Event E = (Event) P;
	if( E == NULL ) { fprintf(stderr,"Error from IS_1_S_arrival(): E is NULL\n"); exit(1); }
	Vehicle V = get_object( E );
	if( V == NULL ) { fprintf(stderr,"Error from IS_1_S_arrival(): V is NULL\n"); exit(1); }
	
	printf("%6.2f, IS 1 South Arrival  , Vehicle ID: %3d, Origin Zone: %d, Destination Zone %d\n",
		   get_sim_time(), get_id(V), get_origin(V), get_destination(V) );
	
	set_event_type( E, IS_1_S_ENTERING );
	set_callback( E, IS_1_S_entering );
	
	// Queue up or enter intersection depending on vehicle destination
	int newLane;
	switch( get_destination( V ) ) {
		case 202: // Right turn
		{
			newLane = 3;
			break;
		}
		case 223: // Left turn
		{
			newLane = 1;
			break;
		}
		default: // Straight
		{
			// Choose shorter lane
			newLane = ( get_list_counter( get_lane_queue( IS_1, SOUTH, 3 ) ) <
					    get_list_counter( get_lane_queue( IS_1, SOUTH, 2 ) )
						  ) ? 3 : 2;
		}
	}
	set_laneID( V, newLane );
	// Check traffic signal
	if( get_light( IS_1, SOUTH, newLane ) == INV ) { fprintf(stderr,"IS_1_W_arrival(): INV traffic signal\n"); }
	if( get_light( IS_1, SOUTH, newLane ) != GREEN ) {
		// Put vehicle in queue
		set_velocity( V, 0.0 );
		add_to_list( get_lane_queue( IS_1, SOUTH, newLane ), E );
	} else {
		// If queue is empty, and no vehicle is currently entering,
		// schedule entering event directly
		if(   get_list_counter( get_lane_queue( IS_1, SOUTH, newLane ) ) == 0
		   && get_lane_flag( IS_1, SOUTH, newLane ) == 0 )
		{
			add_to_list( get_lane_queue( IS_1, SOUTH, newLane ), E );
			schedule_event( E );
		} // Else: put vehicle in queue
		else {
			set_velocity( V, 0.0 );
			add_to_list( get_lane_queue( IS_1, SOUTH, newLane ), E );
		}
	}
	set_wait_time_buf( V, get_sim_time() );
}

static void IS_1_W_arrival( void* P ) {
	Event E = (Event) P;
	if( E == NULL ) { fprintf(stderr,"Error from IS_1_W_arrival(): E is NULL\n"); exit(1); }
	Vehicle V = get_object( E );
	if( V == NULL ) { fprintf(stderr,"Error from IS_1_W_arrival(): V is NULL\n"); exit(1); }
	
	printf("%6.2f, IS 1 West  Arrival  , Vehicle ID: %3d, Origin Zone: %d, Destination Zone %d\n",
		   get_sim_time(), get_id(V), get_origin(V), get_destination(V) );
	
	set_event_type( E, IS_1_W_ENTERING );
	set_callback( E, IS_1_W_entering );
	
	// Queue up or enter intersection depending on vehicle destination
	int newLane;
	switch( get_destination( V ) ) {
		case 201: // Right turn
		{
			newLane = 4;
			break;
		}
		case 202: // Straight
		{
			// Choose shorter lane
			newLane = ( get_list_counter( get_lane_queue( IS_1, WEST, 3 ) ) <
					   get_list_counter( get_lane_queue( IS_1, WEST, 2 ) )
						  ) ? 3 : 2;
			break;
		}
		default: // Left turn
		{
			newLane = 1;
		}
	}
	set_laneID( V, newLane );
	// Check traffic signal
	if( get_light( IS_1, WEST, newLane ) == INV ) { fprintf(stderr,"IS_1_W_arrival(): INV traffic signal\n"); }
	if( get_light( IS_1, WEST, newLane ) != GREEN ) {
		// Put vehicle in queue
		set_velocity( V, 0.0 );
		add_to_list( get_lane_queue( IS_1, WEST, newLane ), E );
	} else {
		// If queue is empty, and no vehicle is currently entering,
		// schedule entering event directly
		if(   get_list_counter( get_lane_queue( IS_1, WEST, newLane ) ) == 0
		   && get_lane_flag( IS_1, WEST, newLane ) == 0 )
		{
			add_to_list( get_lane_queue( IS_1, WEST, newLane ), E );
			schedule_event( E );
		} // Else: put vehicle in queue
		else {
			set_velocity( V, 0.0 );
			add_to_list( get_lane_queue( IS_1, WEST, newLane ), E );
		}
	}
	set_wait_time_buf( V, get_sim_time() );
}

//////////////////////////////////////////////////////////////////////////////////////////////

static void IS_1_N_entering( void* P ) {
	Event E = (Event) P;
	if( E == NULL ) { fprintf(stderr,"Error from IS_1_N_entering(): E is NULL\n"); exit(1); }
	Vehicle V = get_object( E );
	if( V == NULL ) { fprintf(stderr,"Error from IS_1_N_entering(): V is NULL\n"); exit(1); }
	
	printf("%6.2f, IS 1 North Entering , Vehicle ID: %3d, Origin Zone: %d, Destination Zone %d\n",
		   get_sim_time(), get_id(V), get_origin(V), get_destination(V) );
	
	int laneID = get_laneID( V );
	// Check if this event is first in the queue
	if( peek_from_list( get_lane_queue( IS_1, NORTH, laneID ) ) != E ) {
		fprintf(stderr,"Error from IS_1_N_entering(): E is not first in queue\n"); exit(1);
	}
	// Check if no other vehicle is currently entering
	// Check also congestion here
	if( get_lane_flag( IS_1, NORTH, laneID ) == 0 ) {
		set_event_type( E, IS_1_N_CROSSING );
		set_callback( E, IS_1_N_crossing );
		set_timestamp( E, get_sim_time()+SDT );
		if( get_velocity(V) < VEL ) {
			// Pre-calculate temp distance of V such that crossing knows how far V has traveled
			set_temp_distance( V, calc_distance( get_velocity(V), SDT ) );
			set_velocity(V, calc_velocity(get_velocity(V), SDT) );
		}
		set_lane_flag( IS_1, NORTH, laneID, 1 );
		change_lane_counter( IS_1, NORTH, laneID, 1 );
		schedule_event( poll_from_list( get_lane_queue( IS_1, NORTH, laneID ) ) );
		add_wait_time( V, get_sim_time()-get_wait_time_buf( V ) );
		set_wait_time_buf( V, 0.0 );
	}
}

static void IS_1_E_entering( void* P ) {
	Event E = (Event) P;
	if( E == NULL ) { fprintf(stderr,"Error from IS_1_E_entering(): E is NULL\n"); exit(1); }
	Vehicle V = get_object( E );
	if( V == NULL ) { fprintf(stderr,"Error from IS_1_E_entering(): V is NULL\n"); exit(1); }
	
	printf("%6.2f, IS 1 East  Entering , Vehicle ID: %3d, Origin Zone: %d, Destination Zone %d\n",
		   get_sim_time(), get_id(V), get_origin(V), get_destination(V) );
	
	int laneID = get_laneID( V );
	// Check if this event is first in the queue
	if( peek_from_list( get_lane_queue( IS_1, EAST, laneID ) ) != E ) {
		fprintf(stderr,"Error from IS_1_E_entering(): E is not first in queue\n"); exit(1);
	}
	// Check if no other vehicle is currently entering
	// Check also congestion here
	if( get_lane_flag( IS_1, EAST, laneID ) == 0 ) {
		set_event_type( E, IS_1_E_CROSSING );
		set_callback( E, IS_1_E_crossing );
		set_timestamp( E, get_sim_time()+SDT );
		if( get_velocity(V) < VEL ) {
			// Pre-calculate temp distance of V such that crossing knows how far V has traveled
			set_temp_distance( V, calc_distance( get_velocity(V), SDT ) );
			set_velocity(V, calc_velocity(get_velocity(V), SDT) );
		}
		set_lane_flag( IS_1, EAST, laneID, 1 );
		change_lane_counter( IS_1, EAST, laneID, 1 );
		schedule_event( poll_from_list( get_lane_queue( IS_1, EAST, laneID ) ) );
		add_wait_time( V, get_sim_time()-get_wait_time_buf( V ) );
		set_wait_time_buf( V, 0.0 );
	}
}

static void IS_1_S_entering( void* P ) {
	Event E = (Event) P;
	if( E == NULL ) { fprintf(stderr,"Error from IS_1_S_entering(): E is NULL\n"); exit(1); }
	Vehicle V = get_object( E );
	if( V == NULL ) { fprintf(stderr,"Error from IS_1_S_entering(): V is NULL\n"); exit(1); }
	
	printf("%6.2f, IS 1 South Entering , Vehicle ID: %3d, Origin Zone: %d, Destination Zone %d\n",
		   get_sim_time(), get_id(V), get_origin(V), get_destination(V) );
	
	int laneID = get_laneID( V );
	// Check if this event is first in the queue
	if( peek_from_list( get_lane_queue( IS_1, SOUTH, laneID ) ) != E ) {
		fprintf(stderr,"Error from IS_1_S_entering(): E is not first in queue\n"); exit(1);
	}
	// Check if no other vehicle is currently entering
	// Check also congestion here
	if( get_lane_flag( IS_1, SOUTH, laneID ) == 0 ) {
		set_event_type( E, IS_1_S_CROSSING );
		set_callback( E, IS_1_S_crossing );
		set_timestamp( E, get_sim_time()+SDT );
		if( get_velocity(V) < VEL ) {
			// Pre-calculate temp distance of V such that crossing knows how far V has traveled
			set_temp_distance( V, calc_distance( get_velocity(V), SDT ) );
			set_velocity(V, calc_velocity(get_velocity(V), SDT) );
		}
		set_lane_flag( IS_1, SOUTH, laneID, 1 );
		change_lane_counter( IS_1, SOUTH, laneID, 1 );
		schedule_event( poll_from_list( get_lane_queue( IS_1, SOUTH, laneID ) ) );
		add_wait_time( V, get_sim_time()-get_wait_time_buf( V ) );
		set_wait_time_buf( V, 0.0 );
	}
}

static void IS_1_W_entering( void* P ) {
	Event E = (Event) P;
	if( E == NULL ) { fprintf(stderr,"Error from IS_1_W_entering(): E is NULL\n"); exit(1); }
	Vehicle V = get_object( E );
	if( V == NULL ) { fprintf(stderr,"Error from IS_1_W_entering(): V is NULL\n"); exit(1); }
	
	printf("%6.2f, IS 1 West  Entering , Vehicle ID: %3d, Origin Zone: %d, Destination Zone %d\n",
		   get_sim_time(), get_id(V), get_origin(V), get_destination(V) );
	
	int laneID = get_laneID( V );
	// Check if this event is first in the queue
	if( peek_from_list( get_lane_queue( IS_1, WEST, laneID ) ) != E ) {
		fprintf(stderr,"Error from IS_1_W_entering(): E is not first in queue\n"); exit(1);
	}
	// Check if no other vehicle is currently entering
	// Check also congestion here
	if( get_lane_flag( IS_1, WEST, laneID ) == 0 ) {
		set_event_type( E, IS_1_W_CROSSING );
		set_callback( E, IS_1_W_crossing );
		set_timestamp( E, get_sim_time()+SDT );
		if( get_velocity(V) < VEL ) {
			// Pre-calculate temp distance of V such that crossing knows how far V has traveled
			set_temp_distance( V, calc_distance( get_velocity(V), SDT ) );
			set_velocity(V, calc_velocity(get_velocity(V), SDT) );
		}
		set_lane_flag( IS_1, WEST, laneID, 1 );
		change_lane_counter( IS_1, WEST, laneID, 1 );
		schedule_event( poll_from_list( get_lane_queue( IS_1, WEST, laneID ) ) );
		add_wait_time( V, get_sim_time()-get_wait_time_buf( V ) );
		set_wait_time_buf( V, 0.0 );
	}
}

//////////////////////////////////////////////////////////////////////////////////////////////

static void IS_1_N_crossing( void* P ) {
	Event E = (Event) P;
	if( E == NULL ) { fprintf(stderr,"Error from IS_1_N_crossing(): E is NULL\n"); exit(1); }
	Vehicle V = get_object( E );
	if( V == NULL ) { fprintf(stderr,"Error from IS_1_N_crossing(): V is NULL\n"); exit(1); }
	
	printf("%6.2f, IS 1 North Crossing , Vehicle ID: %3d, Origin Zone: %d, Destination Zone %d\n",
		   get_sim_time(), get_id(V), get_origin(V), get_destination(V) );
	
	double crossingDistance = 0;
	switch( get_destination( V ) ) {
		case 223: // Right turn
		{
			crossingDistance = get_crossing_distance( IS_1, NORTH, RIGHT );
			set_event_type( E, IS_1_W_DEPARTURE );
			set_callback( E, IS_1_W_departure );
			break;
		}
		case 201: // Straight
		{
			crossingDistance = get_crossing_distance( IS_1, NORTH, STRAIGHT );
			set_event_type( E, IS_1_S_DEPARTURE );
			set_callback( E, IS_1_S_departure );
			break;
		}
		case 202: // Left turnf
		{
			crossingDistance = get_crossing_distance( IS_1, NORTH, LEFT );
			set_event_type( E, IS_1_E_DEPARTURE );
			set_callback( E, IS_1_E_departure );
		}
		default:
			fprintf(stderr,"Error from IS_1_N_crossing(): Unexpected destination\n"); exit(1);
	}
	// Allow next vehicle to enter this lane
	int laneID = get_laneID( V );
	set_lane_flag( IS_1, NORTH, laneID, 0 );
	
	// Time to departure is computed by using current position
	// (temp distance that was precalculated in entering event handler)
	double distance_to_departure = fmax( 0.0, crossingDistance - get_temp_distance(V) );
	double time_to_departure = calc_time( get_velocity(V), distance_to_departure );
	if( get_velocity(V) < VEL ) {
		set_velocity(V, calc_velocity(get_velocity(V), time_to_departure) );
	}
	set_timestamp( E, get_sim_time() + time_to_departure );
	// Schedule entering event for following vehicle, if signal is still green
	if( get_list_counter( get_lane_queue( IS_1, NORTH, laneID ) ) > 0
	   && get_light( IS_1, NORTH, laneID ) == GREEN )
	{
		Event next = peek_from_list( get_lane_queue( IS_1, NORTH, laneID ) );
		set_timestamp( next, get_sim_time() );
		schedule_event( next );
	}
}

static void IS_1_E_crossing( void* P ) {
	Event E = (Event) P;
	if( E == NULL ) { fprintf(stderr,"Error from IS_1_E_crossing(): E is NULL\n"); exit(1); }
	Vehicle V = get_object( E );
	if( V == NULL ) { fprintf(stderr,"Error from IS_1_E_crossing(): V is NULL\n"); exit(1); }
	
	printf("%6.2f, IS 1 East  Crossing , Vehicle ID: %3d, Origin Zone: %d, Destination Zone %d\n",
		   get_sim_time(), get_id(V), get_origin(V), get_destination(V) );
	
	double crossingDistance = 0;
	switch( get_destination( V ) ) {
		case 223: // Straight
		{
			crossingDistance = get_crossing_distance( IS_1, EAST, STRAIGHT );
			set_event_type( E, IS_1_W_DEPARTURE );
			set_callback( E, IS_1_W_departure );
			break;
		}
		case 201: // Left turn
		{
			crossingDistance = get_crossing_distance( IS_1, EAST, LEFT );
			set_event_type( E, IS_1_S_DEPARTURE );
			set_callback( E, IS_1_S_departure );
			break;
		}
		default:  // Right turn
		{
			crossingDistance = get_crossing_distance( IS_1, EAST, RIGHT );
			set_event_type( E, IS_1_N_DEPARTURE );
			set_callback( E, IS_1_N_departure );
		}
	}
	// Allow next vehicle to enter this lane
	int laneID = get_laneID( V );
	set_lane_flag( IS_1, EAST, laneID, 0 );
	
	// Time to departure is computed by using current position
	// (temp distance that was precalculated in entering event handler)
	double distance_to_departure = fmax( 0.0, crossingDistance - get_temp_distance(V) );
	double time_to_departure = calc_time( get_velocity(V), distance_to_departure );
	if( get_velocity(V) < VEL ) {
		set_velocity(V, calc_velocity(get_velocity(V), time_to_departure) );
	}
	set_timestamp( E, get_sim_time() + time_to_departure );
	// Schedule entering event for following vehicle, if signal is still green
	if( get_list_counter( get_lane_queue( IS_1, EAST, laneID ) ) > 0
	   && get_light( IS_1, EAST, laneID ) == GREEN )
	{
		Event next = peek_from_list( get_lane_queue( IS_1, EAST, laneID ) );
		set_timestamp( next, get_sim_time() );
		schedule_event( next );
	}
}

static void IS_1_S_crossing( void* P ) {
	Event E = (Event) P;
	if( E == NULL ) { fprintf(stderr,"Error from IS_1_S_crossing(): E is NULL\n"); exit(1); }
	Vehicle V = get_object( E );
	if( V == NULL ) { fprintf(stderr,"Error from IS_1_S_crossing(): V is NULL\n"); exit(1); }
	
	printf("%6.2f, IS 1 South Crossing , Vehicle ID: %3d, Origin Zone: %d, Destination Zone %d\n",
		   get_sim_time(), get_id(V), get_origin(V), get_destination(V) );
	
	double crossingDistance = 0;
	switch( get_destination( V ) ) {
		case 202: // Right turn
		{
			crossingDistance = get_crossing_distance( IS_1, SOUTH, RIGHT );
			set_event_type( E, IS_1_E_DEPARTURE );
			set_callback( E, IS_1_E_departure );
			break;
		}
		case 223: // Left turn
		{
			crossingDistance = get_crossing_distance( IS_1, SOUTH, LEFT );
			set_event_type( E, IS_1_W_DEPARTURE );
			set_callback( E, IS_1_W_departure );
			break;
		}
		default:  // Straight
		{
			crossingDistance = get_crossing_distance( IS_1, SOUTH, STRAIGHT );
			set_event_type( E, IS_1_N_DEPARTURE );
			set_callback( E, IS_1_N_departure );
		}
	}
	// Allow next vehicle to enter this lane
	int laneID = get_laneID( V );
	set_lane_flag( IS_1, SOUTH, laneID, 0 );
	
	// Time to departure is computed by using current position
	// (temp distance that was precalculated in entering event handler)
	double distance_to_departure = fmax( 0.0, crossingDistance - get_temp_distance(V) );
	double time_to_departure = calc_time( get_velocity(V), distance_to_departure );
	if( get_velocity(V) < VEL ) {
		set_velocity(V, calc_velocity(get_velocity(V), time_to_departure) );
	}
	set_timestamp( E, get_sim_time() + time_to_departure );
	// Schedule entering event for following vehicle, if signal is still green
	if( get_list_counter( get_lane_queue( IS_1, SOUTH, laneID ) ) > 0
	   && get_light( IS_1, SOUTH, laneID ) == GREEN )
	{
		Event next = peek_from_list( get_lane_queue( IS_1, SOUTH, laneID ) );
		set_timestamp( next, get_sim_time() );
		schedule_event( next );
	}
}

static void IS_1_W_crossing( void* P ) {
	Event E = (Event) P;
	if( E == NULL ) { fprintf(stderr,"Error from IS_1_W_crossing(): E is NULL\n"); exit(1); }
	Vehicle V = get_object( E );
	if( V == NULL ) { fprintf(stderr,"Error from IS_1_W_crossing(): V is NULL\n"); exit(1); }
	
	printf("%6.2f, IS 1 West  Crossing , Vehicle ID: %3d, Origin Zone: %d, Destination Zone %d\n",
		   get_sim_time(), get_id(V), get_origin(V), get_destination(V) );
	
	double crossingDistance = 0;
	switch( get_destination( V ) ) {
		case 201: // Right turn
		{
			crossingDistance = get_crossing_distance( IS_1, WEST, RIGHT );
			set_event_type( E, IS_1_S_DEPARTURE );
			set_callback( E, IS_1_S_departure );
			break;
		}
		case 202: // Straight
		{
			crossingDistance = get_crossing_distance( IS_1, WEST, STRAIGHT );
			set_event_type( E, IS_1_E_DEPARTURE );
			set_callback( E, IS_1_E_departure );
			break;
		}
		default:  // Left turn
		{
			crossingDistance = get_crossing_distance( IS_1, WEST, STRAIGHT );
			set_event_type( E, IS_1_N_DEPARTURE );
			set_callback( E, IS_1_N_departure );
		}
	}
	// Allow next vehicle to enter this lane
	int laneID = get_laneID( V );
	set_lane_flag( IS_1, WEST, laneID, 0 );
	
	// Time to departure is computed by using current position
	// (temp distance that was precalculated in entering event handler)
	double distance_to_departure = fmax( 0.0, crossingDistance - get_temp_distance(V) );
	double time_to_departure = calc_time( get_velocity(V), distance_to_departure );
	if( get_velocity(V) < VEL ) {
		set_velocity(V, calc_velocity(get_velocity(V), time_to_departure) );
	}
	set_timestamp( E, get_sim_time() + time_to_departure );
	// Schedule entering event for following vehicle, if signal is still green
	if( get_list_counter( get_lane_queue( IS_1, WEST, laneID ) ) > 0
	   && get_light( IS_1, WEST, laneID ) == GREEN )
	{
		Event next = peek_from_list( get_lane_queue( IS_1, WEST, laneID ) );
		set_timestamp( next, get_sim_time() );
		schedule_event( next );
	}
}

//////////////////////////////////////////////////////////////////////////////////////////////

static void IS_1_N_departure( void* P ) {
	Event E = (Event) P;
	if( E == NULL ) { fprintf(stderr,"Error from IS_1_N_departure(): E is NULL\n"); exit(1); }
	Vehicle V = get_object( E );
	if( V == NULL ) { fprintf(stderr,"Error from IS_1_N_departure(): V is NULL\n"); exit(1); }
	
	printf("%6.2f, IS 1 North Departure, Vehicle ID: %3d, Origin Zone: %d, Destination Zone %d\n",
		   get_sim_time(), get_id(V), get_origin(V), get_destination(V) );
	
	// Remove vehicle from intersection counter
	Direction from;
	switch(get_origin(V)) {
		case 101: // coming from SOUTH
		{
			from = SOUTH;
			break;
		}
		case 102: // coming from EAST
		{
			from = EAST;
			break;
		}
		case 123: // coming from WEST
		{
			from = WEST;
			break;
		}
		default:
			fprintf(stderr,"Error from IS_1_N_departure(): Unexpected origin\n"); exit(1);
	}
	change_lane_counter( IS_1, from, get_laneID(V), -1 );
	set_temp_distance( V, 0.0 );
	
	/*
	 * Increment Section 2 numVehicles (set congestion flag if numVehicles > capacity)
	 * SCHEDULE local arrival at Intersection 2 - South
	 */
	
	// TEST schedule global departure TEST
	set_event_type( E, GLOBAL_DEPARTURE );
	set_callback( E, global_departure );
	schedule_event( E );
}

static void IS_1_E_departure( void* P ) {
	Event E = (Event) P;
	if( E == NULL ) { fprintf(stderr,"Error from IS_1_E_departure(): E is NULL\n"); exit(1); }
	Vehicle V = get_object( E );
	if( V == NULL ) { fprintf(stderr,"Error from IS_1_E_departure(): V is NULL\n"); exit(1); }
	
	printf("%6.2f, IS 1 East  Departure, Vehicle ID: %3d, Origin Zone: %d, Destination Zone %d\n",
		   get_sim_time(), get_id(V), get_origin(V), get_destination(V) );
	
	// Remove vehicle from intersection counter
	Direction from;
	switch(get_origin(V)) {
		case 101: // coming from SOUTH
		{
			from = SOUTH;
			break;
		}
		case 123: // coming from WEST
		{
			from = WEST;
			break;
		}
		default:  // coming from NORTH
		{
			from = NORTH;
		}
	}
	change_lane_counter( IS_1, from, get_laneID(V), -1 );
	set_temp_distance( V, 0.0 );
	
	// Schedule global departure
	set_event_type( E, GLOBAL_DEPARTURE );
	set_callback( E, global_departure );
	schedule_event( E );
}

static void IS_1_S_departure( void* P ) {
	Event E = (Event) P;
	if( E == NULL ) { fprintf(stderr,"Error from IS_1_S_departure(): E is NULL\n"); exit(1); }
	Vehicle V = get_object( E );
	if( V == NULL ) { fprintf(stderr,"Error from IS_1_S_departure(): V is NULL\n"); exit(1); }
	
	printf("%6.2f, IS 1 South Departure, Vehicle ID: %3d, Origin Zone: %d, Destination Zone %d\n",
		   get_sim_time(), get_id(V), get_origin(V), get_destination(V) );
	
	// Remove vehicle from intersection counter
	Direction from;
	switch(get_origin(V)) {
		case 102: // coming from EAST
		{
			from = EAST;
			break;
		}
		case 123: // coming from WEST
		{
			from = WEST;
			break;
		}
		default:  // coming from NORTH
		{
			from = NORTH;
		}
	}
	change_lane_counter( IS_1, from, get_laneID(V), -1 );
	set_temp_distance( V, 0.0 );
	
	// Schedule global departure
	set_event_type( E, GLOBAL_DEPARTURE );
	set_callback( E, global_departure );
	schedule_event( E );
}

static void IS_1_W_departure( void* P ) {
	Event E = (Event) P;
	if( E == NULL ) { fprintf(stderr,"Error from IS_1_W_departure(): E is NULL\n"); exit(1); }
	Vehicle V = get_object( E );
	if( V == NULL ) { fprintf(stderr,"Error from IS_1_W_departure(): V is NULL\n"); exit(1); }
	
	printf("%6.2f, IS 1 West  Departure, Vehicle ID: %3d, Origin Zone: %d, Destination Zone %d\n",
		   get_sim_time(), get_id(V), get_origin(V), get_destination(V) );
	
	// Remove vehicle from intersection counter
	Direction from;
	switch(get_origin(V)) {
		case 101: // coming from SOUTH
		{
			from = SOUTH;
			break;
		}
		case 102: // coming from EAST
		{
			from = EAST;
			break;
		}
		default:  // coming from NORTH
		{
			from = NORTH;
		}
	}
	change_lane_counter( IS_1, from, get_laneID(V), -1 );
	set_temp_distance( V, 0.0 );
	
	// Schedule global departure
	set_event_type( E, GLOBAL_DEPARTURE );
	set_callback( E, global_departure );
	schedule_event( E );
}

/* ======================================================================================== *\
|*                                                                                          *|
|*                            INTERSECTION 2 : VEHICLE EVENTS                               *|
|*                                                                                          *|
\* ---------------------------------------------------------------------------------------- */


// ..


/* ======================================================================================== *\
|*                                                                                          *|
|*                              SECTIONS : CONGESTION EVENTS                                *|
|*                                                                                          *|
\* ---------------------------------------------------------------------------------------- */


// ..


/* eof */
