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

#define IAT 10.0f    // Global Inter Arrival Time (mean value for exponential distribution)
#define VEL 51.0f    // Maximum Vehicle Speed in ft/sec (35mph ~ 51ft/sec)
#define ACC 10.0f    // Vehicle Acceleration in ft/sec^2
#define VHL 15.0f    // Vehicle length
#define SDD 30.0f    // Safety Distance in ft while driving
#define SDQ  5.0f    // Safety Distance in ft while queueing up
#define SUD  1.0f    // Start-up delay / lost time (first car: delay when traffic light switches to green)

#define SDT ( ( SDD + VHL ) / VEL ) // Safety Distance in sec while driving

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
//-101- -102- -103- -106- -112- -113- -114- -115- -121- -122- -123-
{ 0.50, 0.25, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.25 };
static double p_destinations[11][11] =
{
//    -201- -202- -203- -206- -212- -213- -214- -215- -221- -222- -223-
	{ 0.00, 0.10, 0.00, 0.00, 0.00, 0.00, 0.80, 0.00, 0.00, 0.00, 0.10 }, // origin 101
	{ 0.10, 0.00, 0.00, 0.00, 0.00, 0.00, 0.80, 0.00, 0.00, 0.00, 0.10 }, // origin 102
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

/*
--------------------------------------------------------------------------------------
=================================== NGSIM DATA SET ===================================
--------------------------------------------------------------------------------------
Traffic Peachtree St. NE, 10th-14th: 4.00pm-4.15pm
Original Trajectory Data: 753 Vehicles
Filtered Trajectory Data: 667 Vehicles
  > Origin and Destination Zones restricted to this model (driveways excluded)
  > Origin Zone must be different from Destination Done (no U-Turns)
--------------------------------------------------------------------------------------
[Original Data]: 
Average Inter-Arrival Time: 0.83 sec
Average Vehicle Length    : 16.3 ft
[Filtered Data]:
static double p_origins[11] =
//  -101-   -102-   -103-   -106-   -112-   -113-   -114-   -115-   -121-   -122-   -123-
{ 0.1874, 0.0495, 0.0135, 0,0000, 0.0195, 0.0036, 0.3793, 0.1559, 0.0150, 0.0735, 0.0705 };
static double p_destinations[11][11] =
{
//     -201-   -202-   -203-   -206-   -212-   -213-   -214-   -215-   -221-   -222-   -223-
	{ 0.0000, 0.0960, 0.0000, 0.0160, 0.0080, 0.0160, 0.5360, 0.0640, 0.0400, 0.0240, 0.2000 }, // origin 101
	{ 0.2424, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.5455, 0.0909, 0.0303, 0.0303, 0.0303 }, // origin 102
	{ 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.1111, 0.0000, 0.0000, 0.7778, 0.1111 }, // origin 103
	{ 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000 }, // origin 106
	{ 0.0769, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.6923, 0.1538, 0.0769, 0.0000, 0.0000 }, // origin 112
	{ 0.0417, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.9583, 0.0000, 0.0000, 0.0000, 0.0000 }, // origin 113
	{ 0.2885, 0.0672, 0.0040, 0.0237, 0.0949, 0.2846, 0.0000, 0.1700, 0.0119, 0.0119, 0.0435 }, // origin 114
	{ 0.2692, 0.0385, 0.0192, 0.0192, 0.0577, 0.0096, 0.4423, 0.0000, 0.0288, 0.0288, 0.0865 }, // origin 115
	{ 0.2000, 0.3000, 0.1000, 0.0000, 0.0000, 0.0000, 0.2000, 0.0000, 0.0000, 0.1000, 0.1000 }, // origin 121
	{ 0.2653, 0.4490, 0.0816, 0.0000, 0.0000, 0.0000, 0.0204, 0.0204, 0.0000, 0.0000, 0.1633 }, // origin 122
	{ 0.4894, 0.0000, 0.0426, 0.0000, 0.0000, 0.0213, 0.3617, 0.0426, 0.0000, 0.0426, 0.0000 }  // origin 123
};
--------------------------------------------------------------------------------------
======================================================================================
*/

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
static int pick_signal_phase( Intersection I );
static int pick_origin();
static int pick_destination( int origin );

/* Traffic signal initialization */
static void rand_signal_init();
static void sync_signal_init();

/* Event handler */
static void global_arrival( void *P );
static void global_departure( void *P );

static void IS_1_signal( void *P );
static void IS_2_signal( void *P );
/*
static void IS_3_signal( void *P );
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

static void (*IS_1_ENTER[4])(void *)    = { IS_1_N_entering, IS_1_E_entering, IS_1_S_entering, IS_1_W_entering };
static TypeOfEvent IS_1_ENTER_EVENTS[4] = { IS_1_N_ENTERING, IS_1_E_ENTERING, IS_1_S_ENTERING, IS_1_W_ENTERING };

/* --------------------------------------------------------------------------------------- */
/* ======================================================================================= */

void create_sim( double simEnd ) {
	
	// Set compare function pointer for priority queue
	compare_to = compare_events;
	
	// Create Intersections
	IS_1 = create_intersection( 1 );
	//IS_2 = create_intersection( 2 );
	//IS_3 = create_intersection( 3 );
	//IS_4 = create_intersection( 4 );
	//IS_5 = create_intersection( 5 );
	
	// Create Sections
	S_1 = create_section( 1, VHL, SDQ );
	S_1 = create_section( 2, VHL, SDQ );
	S_1 = create_section( 3, VHL, SDQ );
	S_1 = create_section( 4, VHL, SDQ );
	S_1 = create_section( 5, VHL, SDQ );
	S_1 = create_section( 6, VHL, SDQ );
	
	// Initialize state variables
	arrivals = 0;
	departures = 0;
	totalWaitTime = 0.0;
	totalTravelTime = 0.0;
		
	// Call engine setup
	set_up_sim();

	// Initialize and schedule first event (global arrival)
	Event firstArrival = init_event( -1, NULL, VEHICLE, GLOBAL_ARRIVAL, global_arrival );
	schedule_event( firstArrival );
	
	// Initialize signal events
	printf( "\n---------------------------------------------------------\n");
	rand_signal_init();
	//sync_signal_init();
	printf( "---------------------------------------------------------\n");
	
	// Run simulation
	run_sim( simEnd );
	
	// Print stats
	printf( "\n---------------------------------------------------------\n");
	printf( "Total Simulation Time: %d min, %d sec\n", (int)simEnd/60, (int)simEnd%60 );
	printf( "Mean Inter Arrival Time: %5.2f ( rate = %6.3f )\n", (double)IAT, (double)(1.0/IAT) );
	printf( "Global Arrivals  : %4d Vehicles\n", arrivals );
	printf( "Global Departures: %4d Vehicles\n", departures );
	printf( "Average Travel Time: %6.2f sec\n", totalTravelTime/departures );
	printf( "Average Wait   Time: %6.2f sec\n", totalWaitTime/departures );
	printf( "---------------------------------------------------------\n");
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

// Pick random traffic signal phase
static int pick_signal_phase( Intersection I ) {
	double r = urand() * get_totalPhaseLength( I );
	double *p = get_phaseLengths( I );
	double i = p[0]; int j = 0;
	while( r > i && j < get_maxPhase( I ) ) {
		j++;
		i += p[j];
	}
	return j;
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

static void rand_signal_init() {
	printf("Random Signal Initialization\n");
	// IS 1
	int randPhase = pick_signal_phase( IS_1 );
	set_currPhase( IS_1, randPhase );
	double nextSignalSwitch = urand() * (get_phaseLengths(IS_1))[randPhase];
	printf( "  - Intersection 1; phase: %2d, next signal event: %5.2f\n", randPhase, nextSignalSwitch );
	Event Inter_1 = init_event( nextSignalSwitch, IS_1, INTERSECTION, IS_1_SIGNAL, IS_1_signal );
	schedule_event( Inter_1 );
	// IS 2
	/*
	randPhase = pick_signal_phase( IS_2 );
	set_currPhase( IS_2, randPhase );
	nextSignalSwitch = urand() * (get_phaseLengths(IS_2))[randPhase];
	printf( "  - Intersection 2; phase: %2d, next signal event: %5.2f\n", randPhase, nextSignalSwitch );
	Event Inter_2 = init_event( nextSignalSwitch, IS_2, INTERSECTION, IS_2_SIGNAL, IS_2_signal );
	schedule_event( Inter_2 );
	*/
}

static void sync_signal_init() {
	printf("Synchronized Signal Initialization\n");
	printf( "  - Intersection 1\n" );
	printf( "  - Intersection 2\n" );
	printf( "  - Intersection 3\n" );
	printf( "  - Intersection 4\n" );
	printf( "  - Intersection 5\n" );
	exit(0);
}

/* ======================================================================================== *\
|*                                                                                          *|
|*                               GLOBAL ARRIVAL & DEPARTURE                                 *|
|*                                                                                          *|
\* ---------------------------------------------------------------------------------------- */

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
	
	printf("%6.2f, Global Departure    , Vehicle ID: %3d, Origin Zone: %d, Destination Zone %d\n",
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
|*                         INTERSECTIONS : TRAFFIC SIGNAL EVENTS                            *|
|*                                                                                          *|
\* ---------------------------------------------------------------------------------------- */

static void IS_1_signal( void* P ) {
	Event E = (Event) P;
	if( E == NULL ) { fprintf(stderr,"Error from IS_1_signal(): E is NULL\n"); exit(1); }
	Intersection I = get_object( E );
	if( I == NULL ) { fprintf(stderr,"Error from IS_1_signal(): I is NULL\n"); exit(1); }
	printf("%6.2f, Intersection 1 Signal Event;  ", get_sim_time() );

	// Get intersection fields
	int ***signalStatus = get_signalStatus(I);
	double *phaseLengths = get_phaseLengths(I);
	int maxPhase = get_maxPhase(I);
	int *numLanes = get_numLanes(I);
	LinkedList **laneQueues = get_laneQueues(I);
	
	// Update to next phase & change appropriate signals
	printf("old phase: %d, ", get_currPhase(I));
	set_next_phase(I);
	int curr_phase = get_currPhase(I);
	printf("new phase: %d, phase length: %5.2f\n", curr_phase, phaseLengths[curr_phase]);
	// Schedule next signal event
	set_timestamp(E, get_sim_time() + phaseLengths[curr_phase]);
	schedule_event(E);

	// Schedule entering events for vehicles
	Event newEvent = NULL;
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < numLanes[i]; j++) {
			if ( signalStatus[i][j][curr_phase] == GREEN ) {
				if ( get_list_counter(laneQueues[i][j]) > 0 && get_lane_flag(I, i, j) == 0 ) {
					newEvent = peek_from_list(laneQueues[i][j]);
					set_timestamp(newEvent, get_sim_time());
					
					set_event_type(newEvent, IS_1_ENTER_EVENTS[i]);
					set_callback  (newEvent, IS_1_ENTER       [i]);
				}
			}
		}
	}
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
		printf("get dest\n");
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
			//printf("straight\n");
			// Choose shorter lane
			newLane = ( get_list_counter( get_lane_queue( IS_1, SOUTH, 3 ) ) <
					    get_list_counter( get_lane_queue( IS_1, SOUTH, 2 ) )
						  ) ? 3 : 2;
		}
	}
	set_laneID( V, newLane );
	// Check traffic signal
	if( get_light( IS_1, SOUTH, newLane ) == RED) {
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
	//printf("laneID: %d\n", laneID);
	//printf("num2: %d\n", get_list_counter(get_lane_queue(IS_1, SOUTH, laneID)));
	// Check if this event is first in the queue
	Event test = peek_from_list( get_lane_queue( IS_1, SOUTH, laneID ) );
	//printf("obj: %d")
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
	// Schedule departure
	schedule_event( E );

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
	// Schedule departure
	schedule_event( E );
	
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
	// Schedule departure
	schedule_event( E );
	
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
			crossingDistance = get_crossing_distance( IS_1, WEST, LEFT );
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
	// Schedule departure
	schedule_event( E );
	
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
