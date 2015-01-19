//
//  simApplication.c
//  traffic
//
//  Authors: Mingyue Ma, Eisha Nathan, Stefan Henneking
//  01/12/2015
//

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "event.h"
#include "intersection.h"
#include "linkedList.h"
#include "priorityQueue.h"
#include "section.h"
#include "simApplication.h"

#define IAT  5.0    // Global Inter Arrival Time (mean value for exponential distribution)
#define VEL 51.0    // Maximum Vehicle Speed in ft/sec (35mph ~ 51ft/sec)
#define ACC 10.0    // Vehicle Acceleration in ft/sec^2
#define VHL 15.0    // Vehicle length
#define SDD 30.0    // Safety Distance in ft while driving
#define SDQ  5.0    // Safety Distance in ft while queueing up
#define SUD  1.0    // Start-up delay / lost time (first car: delay when traffic light switches to green)

// Pointer declaration: compare function for priority queue
int (*compare_to)( void*, void* );

// Intersections
static Intersection IS_1;
static Intersection IS_2;
static Intersection IS_3;
static Intersection IS_4;
static Intersection IS_5;

// Road sections between intersections (Peachtree St NE)
static Section S_1;
static Section S_2;
static Section S_3;
static Section S_4;
static Section S_5;
static Section S_6;

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
static void global_arrival( void *P );
static void global_departure( void *P );

static void IS_1_signal( void *P );
static void IS_2_signal( void *P );
static void IS_3_signal( void *P );
static void IS_4_signal( void *P );
static void IS_5_signal( void *P );

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
	
	// Initialize and schedule first event (global arrival)
	Event firstArrival = init_event( -1, NULL, VEHICLE, GLOBAL_ARRIVAL, global_arrival );
	schedule_event( firstArrival );
	
	// Create traffic signal events for each intersection
	// .. //
	
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

// Event handler for a global vehicle arrival
static void global_arrival( void* P ) {
	Event E = (Event) P;
	if( E == NULL ) { fprintf(stderr,"Error from global_arrival(): E is NULL\n"); exit(1); }
	
	// Create new Vehicle and set origin, destination, arrival time
	Vehicle V = create_vehicle();
	int origin_id = pick_origin();
	set_origin( V, origins[origin_id] );
	set_destination( V, destinations[pick_destination( origin_id )] );
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
	if( E == NULL ) { fprintf(stderr,"global_departure(), E is NULL\n"); exit(1); }
	Vehicle V = get_object( E );
	if( V == NULL ) { fprintf(stderr,"global_departure(), V is NULL\n"); exit(1); }
	
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

static void IS_1_signal( void* P ) {
	// proceed to next IS1 signal status
	// schedule next IS1 signal event
	
	// schedule entering events
}

/* ======================================================================================== *\
|*                                                                                          *|
|*                            INTERSECTION 1 : VEHICLE EVENTS                               *|
|*                                                                                          *|
\* ---------------------------------------------------------------------------------------- */

static void IS_1_N_arrival( void* P ) {
	Event E = (Event) P;
	if( E == NULL ) { fprintf(stderr,"IS_1_N_arrival(), E is NULL\n"); exit(1); }
	Vehicle V = get_object( E );
	if( V == NULL ) { fprintf(stderr,"IS_1_N_arrival(), V is NULL\n"); exit(1); }
	
	printf("%6.2f, IS 1 North Arrival  , Vehicle ID: %3d, Origin Zone: %d, Destination Zone %d\n",
		   get_sim_time(), get_id(V), get_origin(V), get_destination(V) );
	
	// Queue up or enter intersection depending on vehicle destination
	switch( get_destination( V ) ) {
		case 223: // Right turn
			// if  : signal green and right lane empty, schedule next event right away
			//       ( enter? / right turn distance? / .. )
			// else: put vehicle in queue
			break;
		case 201: // Straight
			break;
		case 202: // Left turn
			break;
		default:
			fprintf(stderr,"Error from IS_1_N_arrival(): unexpected destination zone, E is NULL\n"); exit(1);
	}
}

static void IS_1_E_arrival( void* P ) {
	Event E = (Event) P;
	if( E == NULL ) { fprintf(stderr,"IS_1_E_arrival(), E is NULL\n"); exit(1); }
	Vehicle V = get_object( E );
	if( V == NULL ) { fprintf(stderr,"IS_1_E_arrival(), V is NULL\n"); exit(1); }
	
	printf("%6.2f, IS 1 East  Arrival  , Vehicle ID: %3d, Origin Zone: %d, Destination Zone %d\n",
		   get_sim_time(), get_id(V), get_origin(V), get_destination(V) );
	
	// Queue up or enter intersection depending on vehicle destination
	switch( get_destination( V ) ) {
		case 223: // Straight
			break;
		case 201: // Left turn
			break;
		default:  // Right turn
			break;
	}
}

static void IS_1_S_arrival( void* P ) {
	Event E = (Event) P;
	if( E == NULL ) { fprintf(stderr,"IS_1_S_arrival(), E is NULL\n"); exit(1); }
	Vehicle V = get_object( E );
	if( V == NULL ) { fprintf(stderr,"IS_1_S_arrival(), V is NULL\n"); exit(1); }
	
	printf("%6.2f, IS 1 South Arrival  , Vehicle ID: %3d, Origin Zone: %d, Destination Zone %d\n",
		   get_sim_time(), get_id(V), get_origin(V), get_destination(V) );
	
	// Queue up or enter intersection depending on vehicle destination
	switch( get_destination( V ) ) {
		case 202: // Right turn
			break;
		case 223: // Left turn
			break;
		default:  // Straight
			break;
	}
}

static void IS_1_W_arrival( void* P ) {
	Event E = (Event) P;
	if( E == NULL ) { fprintf(stderr,"IS_1_W_arrival(), E is NULL\n"); exit(1); }
	Vehicle V = get_object( E );
	if( V == NULL ) { fprintf(stderr,"IS_1_W_arrival(), V is NULL\n"); exit(1); }
	
	printf("%6.2f, IS 1 West  Arrival  , Vehicle ID: %3d, Origin Zone: %d, Destination Zone %d\n",
		   get_sim_time(), get_id(V), get_origin(V), get_destination(V) );
	
	// Queue up or enter intersection depending on vehicle destination
	switch( get_destination( V ) ) {
		case 201: // Right turn
			break;
		case 202: // Straight
			break;
		default:  // Left turn
			break;
	}
}

//////////////////////////////////////////////////////////////////////////////////////////////

static void IS_1_N_entering( void* P ) {
	Event E = (Event) P;
	if( E == NULL ) { fprintf(stderr,"IS_1_N_entering(), E is NULL\n"); exit(1); }
	Vehicle V = get_object( E );
	if( V == NULL ) { fprintf(stderr,"IS_1_N_entering(), V is NULL\n"); exit(1); }
	
	printf("%6.2f, IS 1 North Entering , Vehicle ID: %3d, Origin Zone: %d, Destination Zone %d\n",
		   get_sim_time(), get_id(V), get_origin(V), get_destination(V) );
}

static void IS_1_E_entering( void* P ) {
	Event E = (Event) P;
	if( E == NULL ) { fprintf(stderr,"IS_1_E_entering(), E is NULL\n"); exit(1); }
	Vehicle V = get_object( E );
	if( V == NULL ) { fprintf(stderr,"IS_1_E_entering(), V is NULL\n"); exit(1); }
	
	printf("%6.2f, IS 1 East  Entering , Vehicle ID: %3d, Origin Zone: %d, Destination Zone %d\n",
		   get_sim_time(), get_id(V), get_origin(V), get_destination(V) );
}

static void IS_1_S_entering( void* P ) {
	Event E = (Event) P;
	if( E == NULL ) { fprintf(stderr,"IS_1_S_entering(), E is NULL\n"); exit(1); }
	Vehicle V = get_object( E );
	if( V == NULL ) { fprintf(stderr,"IS_1_S_entering(), V is NULL\n"); exit(1); }
	
	printf("%6.2f, IS 1 South Entering , Vehicle ID: %3d, Origin Zone: %d, Destination Zone %d\n",
		   get_sim_time(), get_id(V), get_origin(V), get_destination(V) );
}

static void IS_1_W_entering( void* P ) {
	Event E = (Event) P;
	if( E == NULL ) { fprintf(stderr,"IS_1_W_entering(), E is NULL\n"); exit(1); }
	Vehicle V = get_object( E );
	if( V == NULL ) { fprintf(stderr,"IS_1_W_entering(), V is NULL\n"); exit(1); }
	
	printf("%6.2f, IS 1 West  Entering , Vehicle ID: %3d, Origin Zone: %d, Destination Zone %d\n",
		   get_sim_time(), get_id(V), get_origin(V), get_destination(V) );
}

//////////////////////////////////////////////////////////////////////////////////////////////

static void IS_1_N_crossing( void* P ) {
	Event E = (Event) P;
	if( E == NULL ) { fprintf(stderr,"IS_1_N_crossing(), E is NULL\n"); exit(1); }
	Vehicle V = get_object( E );
	if( V == NULL ) { fprintf(stderr,"IS_1_N_crossing(), V is NULL\n"); exit(1); }
	
	printf("%6.2f, IS 1 South Crossing , Vehicle ID: %3d, Origin Zone: %d, Destination Zone %d\n",
		   get_sim_time(), get_id(V), get_origin(V), get_destination(V) );
}

static void IS_1_E_crossing( void* P ) {
	Event E = (Event) P;
	if( E == NULL ) { fprintf(stderr,"IS_1_E_crossing(), E is NULL\n"); exit(1); }
	Vehicle V = get_object( E );
	if( V == NULL ) { fprintf(stderr,"IS_1_E_crossing(), V is NULL\n"); exit(1); }
	
	printf("%6.2f, IS 1 South Crossing , Vehicle ID: %3d, Origin Zone: %d, Destination Zone %d\n",
		   get_sim_time(), get_id(V), get_origin(V), get_destination(V) );
}

static void IS_1_S_crossing( void* P ) {
	Event E = (Event) P;
	if( E == NULL ) { fprintf(stderr,"IS_1_S_crossing(), E is NULL\n"); exit(1); }
	Vehicle V = get_object( E );
	if( V == NULL ) { fprintf(stderr,"IS_1_S_crossing(), V is NULL\n"); exit(1); }
	
	printf("%6.2f, IS 1 South Crossing , Vehicle ID: %3d, Origin Zone: %d, Destination Zone %d\n",
		   get_sim_time(), get_id(V), get_origin(V), get_destination(V) );
}

static void IS_1_W_crossing( void* P ) {
	Event E = (Event) P;
	if( E == NULL ) { fprintf(stderr,"IS_1_W_crossing(), E is NULL\n"); exit(1); }
	Vehicle V = get_object( E );
	if( V == NULL ) { fprintf(stderr,"IS_1_W_crossing(), V is NULL\n"); exit(1); }
	
	printf("%6.2f, IS 1 South Crossing , Vehicle ID: %3d, Origin Zone: %d, Destination Zone %d\n",
		   get_sim_time(), get_id(V), get_origin(V), get_destination(V) );
}

//////////////////////////////////////////////////////////////////////////////////////////////

static void IS_1_N_departure( void* P ) {
	Event E = (Event) P;
	if( E == NULL ) { fprintf(stderr,"IS_1_N_departure(), E is NULL\n"); exit(1); }
	Vehicle V = get_object( E );
	if( V == NULL ) { fprintf(stderr,"IS_1_N_departure(), V is NULL\n"); exit(1); }
	
	printf("%6.2f, IS 1 North Departure, Vehicle ID: %3d, Origin Zone: %d, Destination Zone %d\n",
		   get_sim_time(), get_id(V), get_origin(V), get_destination(V) );
	
	// Remove vehicle from intersection counter
	// .. //
	
	// TEST schedule global departure TEST
	set_event_type( E, GLOBAL_DEPARTURE );
	set_callback( E, global_departure );
	schedule_event( E );
}

static void IS_1_E_departure( void* P ) {
	Event E = (Event) P;
	if( E == NULL ) { fprintf(stderr,"IS_1_E_departure(), E is NULL\n"); exit(1); }
	Vehicle V = get_object( E );
	if( V == NULL ) { fprintf(stderr,"IS_1_E_departure(), V is NULL\n"); exit(1); }
	
	printf("%6.2f, IS 1 East  Departure, Vehicle ID: %3d, Origin Zone: %d, Destination Zone %d\n",
		   get_sim_time(), get_id(V), get_origin(V), get_destination(V) );
	
	// Remove vehicle from intersection counter
	// .. //
	
	// Schedule global departure
	set_event_type( E, GLOBAL_DEPARTURE );
	set_callback( E, global_departure );
	schedule_event( E );
}

static void IS_1_S_departure( void* P ) {
	Event E = (Event) P;
	if( E == NULL ) { fprintf(stderr,"IS_1_S_departure(), E is NULL\n"); exit(1); }
	Vehicle V = get_object( E );
	if( V == NULL ) { fprintf(stderr,"IS_1_S_departure(), V is NULL\n"); exit(1); }
	
	printf("%6.2f, IS 1 South Departure, Vehicle ID: %3d, Origin Zone: %d, Destination Zone %d\n",
		   get_sim_time(), get_id(V), get_origin(V), get_destination(V) );
	
	// Remove vehicle from intersection counter
	// .. //
	
	// Schedule global departure
	set_event_type( E, GLOBAL_DEPARTURE );
	set_callback( E, global_departure );
	schedule_event( E );
}

static void IS_1_W_departure( void* P ) {
	Event E = (Event) P;
	if( E == NULL ) { fprintf(stderr,"IS_1_W_departure(), E is NULL\n"); exit(1); }
	Vehicle V = get_object( E );
	if( V == NULL ) { fprintf(stderr,"IS_1_W_departure(), V is NULL\n"); exit(1); }
	
	printf("%6.2f, IS 1 West  Departure, Vehicle ID: %3d, Origin Zone: %d, Destination Zone %d\n",
		   get_sim_time(), get_id(V), get_origin(V), get_destination(V) );
	
	// Remove vehicle from intersection counter
	// .. //
	
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
