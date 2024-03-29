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
#include <sys/time.h>

#include "event.h"
#include "linkedList.h"
#include "section.h"
#include "simApplication.h"

#define IAT  0.83f   // Global Inter Arrival Time (mean value for exponential distribution)
#define VEL 51.0f    // Maximum Vehicle Speed in ft/sec (35mph ~ 51ft/sec)
#define ACC 10.0f    // Vehicle Acceleration in ft/sec^2
#define VHL 16.3f    // Vehicle length
#define SDD 30.0f    // Safety Distance in ft while driving
#define SDQ  5.0f    // Safety Distance in ft while queueing up
#define SUD  1.5f    // Start-up delay / lost time (first car: delay when traffic light switches to green)

#define SDT ( ( SDD + VHL ) / VEL ) // Safety Distance in sec while driving with max. velocity

// Define this preprocessor
#define SYNC_SIGNAL 0

// Write stats to output files
#define OUTPUT_FILES 0

#define MIN(X, Y) (((X) < (Y)) ? (X) : (Y))

// Pointer declaration: compare function for priority queue
int (*compare_to)( void*, void* );

// Intersections
static Intersection IS_1, IS_2, IS_3, IS_4, IS_5;
Intersection IS_pointer[5];

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
//  -101-   -102-   -103-   -106-   -112-   -113-   -114-   -115-   -121-   -122-   -123-
{ 0.1874, 0.0495, 0.0135, 0.0000, 0.0195, 0.0036, 0.3793, 0.1559, 0.0150, 0.0735, 0.0705 };
static double p_destinations[11][11] =
{
//     -201-   -202-   -203-   -206-   -212-   -213-   -214-   -215-   -221-   -222-   -223-
	{ 0.0000, 0.0960, 0.0000, 0.0160, 0.0080, 0.0160, 0.5360, 0.0640, 0.0400, 0.0240, 0.2000 }, // origin 101
	{ 0.2424, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.5455, 0.0909, 0.0303, 0.0303, 0.0303 }, // origin 102
	{ 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.1111, 0.0000, 0.0000, 0.7778, 0.1111 }, // origin 103
	{ 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000 }, // origin 106
	{ 0.0769, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.6923, 0.1538, 0.0769, 0.0000, 0.0000 }, // origin 112
	{ 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 1.0000, 0.0000, 0.0000, 0.0000, 0.0000 }, // origin 113
	{ 0.2885, 0.0672, 0.0040, 0.0237, 0.0949, 0.2846, 0.0000, 0.1700, 0.0119, 0.0119, 0.0435 }, // origin 114
	{ 0.2692, 0.0385, 0.0192, 0.0192, 0.0577, 0.0096, 0.4423, 0.0000, 0.0288, 0.0288, 0.0865 }, // origin 115
	{ 0.2000, 0.3000, 0.1000, 0.0000, 0.0000, 0.0000, 0.2000, 0.0000, 0.0000, 0.1000, 0.1000 }, // origin 121
	{ 0.2653, 0.4490, 0.0816, 0.0000, 0.0000, 0.0000, 0.0204, 0.0204, 0.0000, 0.0000, 0.1633 }, // origin 122
	{ 0.4894, 0.0000, 0.0426, 0.0000, 0.0000, 0.0213, 0.3617, 0.0426, 0.0000, 0.0426, 0.0000 }  // origin 123
};
/*
--------------------------------------------------------------------------------------
=================================== NGSIM DATA SET ===================================
--------------------------------------------------------------------------------------
Traffic Peachtree St. NE, 10th-14th: 4.00pm-4.15pm
Original Trajectory Data: 753 Vehicles
Filtered Trajectory Data: 667 Vehicles
  > Origin and Destination Zones restricted to this model (driveways excluded)
  > Origin Zone must be different from Destination Zone (no U-Turns)
--------------------------------------------------------------------------------------
Average Inter-Arrival Time: 0.83 sec (assuming all vehicles entered during the 15min timespan)
Average Vehicle Length    : 16.3 ft
static double p_origins[11] =
//  -101-   -102-   -103-   -106-   -112-   -113-   -114-   -115-   -121-   -122-   -123-
{ 0.1874, 0.0495, 0.0135, 0.0000, 0.0195, 0.0036, 0.3793, 0.1559, 0.0150, 0.0735, 0.0705 };
static double p_destinations[11][11] =
{
//     -201-   -202-   -203-   -206-   -212-   -213-   -214-   -215-   -221-   -222-   -223-
	{ 0.0000, 0.0960, 0.0000, 0.0160, 0.0080, 0.0160, 0.5360, 0.0640, 0.0400, 0.0240, 0.2000 }, // origin 101
	{ 0.2424, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.5455, 0.0909, 0.0303, 0.0303, 0.0303 }, // origin 102
	{ 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.1111, 0.0000, 0.0000, 0.7778, 0.1111 }, // origin 103
	{ 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000 }, // origin 106
	{ 0.0769, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.6923, 0.1538, 0.0769, 0.0000, 0.0000 }, // origin 112
	{ 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 1.0000, 0.0000, 0.0000, 0.0000, 0.0000 }, // origin 113
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
static double calc_velocity( double v_cur, double t ) {
	// v = v_cur + a*t
	return fmin(v_cur + ACC * t, VEL);
}

static double calc_distance( double v_cur, double t ) {
	// s = v_cur*t + 0.5*a*t^2
	double t_acc = (VEL - v_cur) / ACC;
	double d_acc = v_cur*t_acc + 0.5*ACC*t_acc*t_acc;
	return (t_acc <= t) ? d_acc + (t-t_acc)*VEL : v_cur*t + 0.5*ACC*t*t;
}

static double calc_time( double v_cur, double dist ) {
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

/* Auxiliary functions */
static double urand();
static double randexp();
#if SYNC_SIGNAL == 0
static int pick_signal_phase( Intersection I );
#endif
static int pick_origin();
static int pick_destination( int origin );
static void check_and_print_vehicle_event( Event E );

/* Traffic signal initialization */
#if SYNC_SIGNAL == 0
static void rand_signal_init();
#else
static void sync_signal_init();
#endif

/* Event handler */
static void global_arrival( void *P );
static void global_departure( void *P );

static void IS_signal( void *P );

static void IS_1_arrival  ( void* P );
static void IS_1_entering ( void* P );
static void IS_1_crossing ( void* P );
static void IS_1_departure( void* P );

static void IS_2_arrival  ( void* P );
static void IS_2_entering ( void* P );
static void IS_2_crossing ( void* P );
static void IS_2_departure( void* P );

static void IS_3_arrival  ( void* P );
static void IS_3_entering ( void* P );
static void IS_3_crossing ( void* P );
static void IS_3_departure( void* P );

static void IS_4_arrival  ( void* P );
static void IS_4_entering ( void* P );
static void IS_4_crossing ( void* P );
static void IS_4_departure( void* P );

static void IS_5_arrival  ( void* P );
static void IS_5_entering ( void* P );
static void IS_5_crossing ( void* P );
static void IS_5_departure( void* P );

/* --------------------------------------------------------------------------------------- */
/* ======================================================================================= */

/*
void print_queues( Intersection I ) {
	int *numLanes = get_numLanes( I );
	LinkedList **laneQueues = get_laneQueues( I );
	for( int i = 0; i < 4; i++ ) {
		for( int j = 0; j < numLanes[i]; j++ ) {
			printf("DIR=%d, Lane=%d, Counter: %d\n" , i, j+1, get_list_counter( laneQueues[i][j] ));
		}
	}
}
*/

void create_sim( double simEnd ) {
	
	// Set compare function pointer for priority queue
	compare_to = compare_events;
	
	// Create Intersections
	IS_1 = create_intersection( 1 ); IS_pointer[0] = IS_1;
	IS_2 = create_intersection( 2 ); IS_pointer[1] = IS_2;
	IS_3 = create_intersection( 3 ); IS_pointer[2] = IS_3;
	IS_4 = create_intersection( 4 ); IS_pointer[3] = IS_4;
	IS_5 = create_intersection( 5 ); IS_pointer[4] = IS_5;

	// Create Sections
	S_1 = create_section( 1, VHL, SDQ );
	S_2 = create_section( 2, VHL, SDQ );
	S_3 = create_section( 3, VHL, SDQ );
	S_4 = create_section( 4, VHL, SDQ );
	S_5 = create_section( 5, VHL, SDQ );
	S_6 = create_section( 6, VHL, SDQ );
	
	// Initialize state variables
	arrivals = 0;
	departures = 0;
	totalWaitTime = 0.0;
	totalTravelTime = 0.0;
	
	// Call engine setup
	set_up_sim();
	
	// Initialize signal events
	printf( "\n---------------------------------------------------------\n");
	#if SYNC_SIGNAL == 1
		sync_signal_init();
	#else
		rand_signal_init();
	#endif
	printf( "---------------------------------------------------------\n");
	
	/* Use a new random seed for event simulation (other than for signal initialization) */
	/*
	struct timeval t1;
	gettimeofday(&t1, NULL);
	int seed = (int)(t1.tv_usec * t1.tv_sec);
	srand(seed);
	*/
	//srand(0);
	
	// Initialize and schedule first event (global arrival)
	Event firstArrival = init_event( -1, NULL, VEHICLE, GLOBAL_ARRIVAL, global_arrival );
	schedule_event( firstArrival );

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
	
#if OUTPUT_FILES == 1
	FILE *out1 = fopen( "travel-time.txt", "a" );
	FILE *out2 = fopen( "wait-time.txt", "a" );
	if ( out1 == NULL || out2 == NULL ) { fprintf (stderr, "Error for write\n"); exit(1); }
	fprintf( out1, "%.2f\n", totalTravelTime/departures );
	fprintf( out2, "%.2f\n", totalWaitTime/departures );
	if( fclose(out1) != 0 ) {fprintf(stderr,"Error\n"); exit(1);}
	if( fclose(out2) != 0 ) {fprintf(stderr,"Error\n"); exit(1);}
	FILE *out3 = fopen( "arrivals.txt", "a" );
	FILE *out4 = fopen( "departures.txt", "a" );
	if ( out3 == NULL || out4 == NULL ) { fprintf (stderr, "Error for write\n"); exit(1); }
	fprintf( out3, "%d\n", arrivals );
	fprintf( out4, "%d\n", departures );
	if( fclose(out3) != 0 ) {fprintf(stderr,"Error\n"); exit(1);}
	if( fclose(out4) != 0 ) {fprintf(stderr,"Error\n"); exit(1);}
#endif
	/*
	printf("\nIS1:\n");
	print_queues( IS_1 );
	printf("\nIS2:\n");
	print_queues( IS_2 );
	printf("\nIS3:\n");
	print_queues( IS_3 );
	printf("\nIS4:\n");
	print_queues( IS_4 );
	printf("\nIS5:\n");
	print_queues( IS_5 );
	*/
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

/* ======================================================================================== *\
|*                                                                                          *|
|*                                  AUXILIARY FUNCTIONS                                     *|
|*                                                                                          *|
\* ---------------------------------------------------------------------------------------- */

// Generate random number in [0,1)
static double urand() {
	return ( (double)rand() / (double)RAND_MAX );
}

// Generate random number from exp. distribution with mean IAT
static double randexp() {
	return -IAT*( log( 1.0 - urand() ) );
}

#if SYNC_SIGNAL == 0
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
#endif

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
	if( origin_id == -1 ) origin_id = 10;
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
	if( destination_id == -1 ) destination_id = 10;
	return destination_id;
}

// Check pointer and print event data
static void check_and_print_vehicle_event( Event E ) {
	if( E == NULL ) { fprintf(stderr,"Error from check_and_print_vehicle_event: E is NULL\n"); exit(1); }
	const char *event = TypeOfEventStrings[get_event_type( E )];
	Vehicle V = get_object( E );
	if( V == NULL ) { fprintf(stderr,"Error from %s: V is NULL\n", event); exit(1); }
	#if VERBOSE == 1
		printf("%7.2f, %-20s, Vehicle ID: %3d, Origin Zone: %d, Destination Zone %d\n"
			   , get_sim_time(), event, get_id(V), get_origin(V), get_destination(V) );
	#endif
}

/* ======================================================================================== *\
|*                                                                                          *|
|*                             TRAFFIC SIGNAL INITIALIZATION                                *|
|*                                                                                          *|
\* ---------------------------------------------------------------------------------------- */

#if SYNC_SIGNAL == 0
static void rand_signal_init() {
	printf("Random Signal Initialization\n");
	// IS 1
	int randPhase = pick_signal_phase( IS_1 );
	set_currPhase( IS_1, randPhase );
	double nextSignalSwitch = urand() * (get_phaseLengths(IS_1))[randPhase];
	printf( "  - Intersection 1; phase: %2d, next signal event: %5.2f\n", randPhase, nextSignalSwitch );
	Event Inter_1 = init_event( nextSignalSwitch, IS_1, INTERSECTION, IS_SIGNAL, IS_signal );
	schedule_event( Inter_1 );
	// IS 2
	randPhase = pick_signal_phase( IS_2 );
	set_currPhase( IS_2, randPhase );
	nextSignalSwitch = urand() * (get_phaseLengths(IS_2))[randPhase];
	printf( "  - Intersection 2; phase: %2d, next signal event: %5.2f\n", randPhase, nextSignalSwitch );
	Event Inter_2 = init_event( nextSignalSwitch, IS_2, INTERSECTION, IS_SIGNAL, IS_signal );
	schedule_event( Inter_2 );
	// IS 3
	randPhase = pick_signal_phase( IS_3 );
	set_currPhase( IS_3, randPhase );
	nextSignalSwitch = urand() * (get_phaseLengths(IS_3))[randPhase];
	printf( "  - Intersection 3; phase: %2d, next signal event: %5.2f\n", randPhase, nextSignalSwitch );
	Event Inter_3 = init_event( nextSignalSwitch, IS_3, INTERSECTION, IS_SIGNAL, IS_signal );
	schedule_event( Inter_3 );
	// IS 5
	randPhase = pick_signal_phase( IS_5 );
	set_currPhase( IS_5, randPhase );
	nextSignalSwitch = urand() * (get_phaseLengths(IS_5))[randPhase];
	printf( "  - Intersection 5; phase: %2d, next signal event: %5.2f\n", randPhase, nextSignalSwitch );
	Event Inter_5 = init_event( nextSignalSwitch, IS_5, INTERSECTION, IS_SIGNAL, IS_signal );
	schedule_event( Inter_5 );
}
#else
static void sync_signal_init() {
	printf("Synchronized Signal Initialization\n");
	// IS 1
	int syncPhase = MIN( 9, get_maxPhase(IS_1)-1 );
	set_currPhase( IS_1, syncPhase );
	double nextSignalSwitch = MIN( 5.0, get_phaseLengths(IS_1)[syncPhase] );
	printf( "  - Intersection 1; phase: %2d, next signal event: %5.2f\n", syncPhase, nextSignalSwitch );
	Event Inter_1 = init_event( nextSignalSwitch, IS_1, INTERSECTION, IS_SIGNAL, IS_signal );
	schedule_event( Inter_1 );
	// IS 2
	syncPhase = MIN( 1, get_maxPhase(IS_1)-1 );
	set_currPhase( IS_2, syncPhase );
	nextSignalSwitch = MIN( 12.5, get_phaseLengths(IS_2)[syncPhase] );
	printf( "  - Intersection 2; phase: %2d, next signal event: %5.2f\n", syncPhase, nextSignalSwitch );
	Event Inter_2 = init_event( nextSignalSwitch, IS_2, INTERSECTION, IS_SIGNAL, IS_signal );
	schedule_event( Inter_2 );
	// IS 3
	syncPhase = MIN( 1, get_maxPhase(IS_1)-1 );
	set_currPhase( IS_3, syncPhase );
	nextSignalSwitch = MIN( 3.2, get_phaseLengths(IS_3)[syncPhase] );
	printf( "  - Intersection 3; phase: %2d, next signal event: %5.2f\n", syncPhase, nextSignalSwitch );
	Event Inter_3 = init_event( nextSignalSwitch, IS_3, INTERSECTION, IS_SIGNAL, IS_signal );
	schedule_event( Inter_3 );
	// IS 5
	syncPhase = MIN( 3, get_maxPhase(IS_1)-1 );
	set_currPhase( IS_5, syncPhase );
	nextSignalSwitch = MIN( 34.6, get_phaseLengths(IS_5)[syncPhase] );
	printf( "  - Intersection 5; phase: %2d, next signal event: %5.2f\n", syncPhase, nextSignalSwitch );
	Event Inter_5 = init_event( nextSignalSwitch, IS_5, INTERSECTION, IS_SIGNAL, IS_signal );
	schedule_event( Inter_5 );
}
#endif

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
	
	// Print output
	check_and_print_vehicle_event( E );
	
	// Schedule local arrival event depending on origin zone
	// Change type of event and set event handler
	switch( origins[origin_id] ) {
		case 101:
			set_event_type( E, IS_1_ARRIVAL );
			set_callback  ( E, IS_1_arrival );
			set_dir( V, SOUTH );
			break;
		case 102:
			set_event_type( E, IS_1_ARRIVAL );
			set_callback  ( E, IS_1_arrival );
			set_dir( V, EAST );
			break;
		case 103:
			set_event_type( E, IS_2_ARRIVAL );
			set_callback  ( E, IS_2_arrival );
			set_dir( V, EAST );
			break;
		case 106:
			set_event_type( E, IS_3_ARRIVAL );
			set_callback  ( E, IS_3_arrival );
			set_dir( V, EAST );
			break;
		case 112:
			set_event_type( E, IS_4_ARRIVAL );
			set_callback  ( E, IS_4_arrival );
			set_dir( V, EAST );
			break;
		case 113:
			set_event_type( E, IS_5_ARRIVAL );
			set_callback  ( E, IS_5_arrival );
			set_dir( V, EAST );
			break;
		case 114:
			set_event_type( E, IS_5_ARRIVAL );
			set_callback  ( E, IS_5_arrival );
			set_dir( V, NORTH );
			break;
		case 115:
			set_event_type( E, IS_5_ARRIVAL );
			set_callback  ( E, IS_5_arrival );
			set_dir( V, WEST );
			break;
		case 121:
			set_event_type( E, IS_3_ARRIVAL );
			set_callback  ( E, IS_3_arrival );
			set_dir( V, WEST );
			break;
		case 122:
			set_event_type( E, IS_2_ARRIVAL );
			set_callback  ( E, IS_2_arrival );
			set_dir( V, WEST );
			break;
		case 123:
			set_event_type( E, IS_1_ARRIVAL );
			set_callback  ( E, IS_1_arrival );
			set_dir( V, WEST );
			break;
		default:
			fprintf(stderr,"Error from global_arrival(): invalid origin zone\n"); exit(1);
	}
	schedule_event( E );
	
	// Schedule next global vehicle arrival
	Event nextArrival = init_event( -1, NULL, VEHICLE, GLOBAL_ARRIVAL, global_arrival );
	schedule_event( nextArrival );
	
	// Change state variable
	arrivals++;
}

// Event handler for a global vehicle departure
static void global_departure( void* P ) {
	Event E = (Event) P;
	check_and_print_vehicle_event( E );
	Vehicle V = get_object( E );
	
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

static void IS_signal( void* P ) {
	Event E = (Event) P;
	if( E == NULL ) { fprintf(stderr,"Error from IS_signal(): E is NULL\n"); exit(1); }
	Intersection I = get_object( E );
	if( I == NULL ) { fprintf(stderr,"Error from IS_signal(): I is NULL\n"); exit(1); }
	#if VERBOSE == 1
		printf( "%7.2f, Intersection %d Signal Event; ", get_sim_time(), get_inter_zoneID( I ) );
	#endif
	
	// Get intersection fields
	int ***signalStatus = get_signalStatus( I );
	double *phaseLengths = get_phaseLengths( I );
	int *numLanes = get_numLanes( I );
	LinkedList **laneQueues = get_laneQueues( I );
	
	// Update to next phase & change appropriate signals
	int oldPhase = get_currPhase( I );
	set_next_phase(I);
	int currPhase = get_currPhase( I );
	#if VERBOSE == 1
		printf("old phase: %2d, new phase: %2d, phaseLength: %5.2f\n",
			   oldPhase, currPhase, phaseLengths[currPhase]);
	#endif
	// Schedule next signal event
	set_timestamp( E, get_sim_time() + phaseLengths[currPhase] );
	schedule_event( E );

	// Schedule entering events for vehicles
	for( int i = 0; i < 4; i++ ) {
		for( int j = 0; j < numLanes[i]; j++ ) {
			if( signalStatus[i][j][currPhase] == GREEN && signalStatus[i][j][oldPhase] == RED ) {
				if( get_list_counter( laneQueues[i][j] ) > 0 && get_lane_flag( I, i, j+1 ) == 0 ) {
					Event newEvent = peek_from_list( laneQueues[i][j] );
					if( !(get_scheduled( newEvent )) ) {
						set_timestamp( newEvent, get_sim_time() + SUD );
						schedule_event( newEvent );
					}
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

#include "IS_1_events.c"

/* ======================================================================================== *\
|*                                                                                          *|
|*                            INTERSECTION 2 : VEHICLE EVENTS                               *|
|*                                                                                          *|
\* ---------------------------------------------------------------------------------------- */

#include "IS_2_events.c"

/* ======================================================================================== *\
|*                                                                                          *|
|*                            INTERSECTION 3 : VEHICLE EVENTS                               *|
|*                                                                                          *|
\* ---------------------------------------------------------------------------------------- */

#include "IS_3_events.c"

/* ======================================================================================== *\
 |*                                                                                          *|
 |*                            INTERSECTION 4 : VEHICLE EVENTS                               *|
 |*                                                                                          *|
 \* ---------------------------------------------------------------------------------------- */

#include "IS_4_events.c"

/* ======================================================================================== *\
 |*                                                                                          *|
 |*                            INTERSECTION 5 : VEHICLE EVENTS                               *|
 |*                                                                                          *|
 \* ---------------------------------------------------------------------------------------- */

#include "IS_5_events.c"

/* ======================================================================================== *\
|*                                                                                          *|
|*                              SECTIONS : CONGESTION EVENTS                                *|
|*                                                                                          *|
\* ---------------------------------------------------------------------------------------- */


void section_clear( int ID, Direction D ) {
	Intersection I = IS_pointer[ID-1];
	// Get intersection fields
	
	int *numLanes = get_numLanes( I );
	LinkedList **laneQueues = get_laneQueues( I );
	if( ID == 4 ) {
		// Schedule entering events for vehicles that can now enter as the section is clear again
		// Schedule only North/South for IS 4
		for( int i = 0; i < 4; i+=2 ) {
			for( int j = 0; j < numLanes[i]; j++ ) {
				if( get_list_counter( laneQueues[i][j] ) > 0 && get_lane_flag( I, i, j+1 ) == 0 ) {
					Event newEvent = peek_from_list( laneQueues[i][j] );
					Vehicle V = get_object( newEvent );
					if( get_departDir( V ) == D ) {
						if( !(get_scheduled( newEvent )) ) {
							set_timestamp( newEvent, get_sim_time() );
							schedule_event( newEvent );
						}
					}
				}
			}
		}
	} else {
		int ***signalStatus = get_signalStatus( I );
		int currPhase = get_currPhase( I );
		// Schedule entering events for vehicles that can now enter as the section is clear again
		for( int i = 0; i < 4; i++ ) {
			for( int j = 0; j < numLanes[i]; j++ ) {
				if( signalStatus[i][j][currPhase] != RED ) {
					if( get_list_counter( laneQueues[i][j] ) > 0 && get_lane_flag( I, i, j+1 ) == 0 ) {
						Event newEvent = peek_from_list( laneQueues[i][j] );
						Vehicle V = get_object( newEvent );
						if( get_departDir( V ) == D ) {
							if( !(get_scheduled( newEvent )) ) {
								set_timestamp( newEvent, get_sim_time() );
								schedule_event( newEvent );
							}
						}
					}
				}
			}
		}
	}
}

/* eof */
