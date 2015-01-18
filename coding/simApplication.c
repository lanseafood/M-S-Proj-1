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

#include "linkedList.h"
#include "priorityQueue.h"
#include "simApplication.h"

#define IAT  5.0    // Global Inter Arrival Time
#define VEL 51.0    // Maximum Vehicle Speed in ft/sec (35mph ~ 51ft/sec)
#define ACC 10.0	// Vehicle Acceleration in ft/sec^2
#define VHL 15.0	// Vehicle length
#define SDD 30.0    // Safety Distance in ft while driving
#define SDQ  5.0    // Safety Distance in ft while queueing up
#define SUD  1.0    // Start-up delay / lost time (first car: delay when traffic light switches to green)

// Pointer to compare function for priority queue
int (*compare_to)( void*, void* );

static Intersection IS_1;
static Intersection IS_2;
static Intersection IS_3;
static Intersection IS_4;
static Intersection IS_5;

static Section S_1;
static Section S_2;
static Section S_3;
static Section S_4;
static Section S_5;
static Section S_6;

static int arrivals;            // Total number of global vehicle arrivals
static int departures;          // Total number of global vehicle departures
static double totalWaitTime;    // Sum of all waiting times at intersections of already departed vehicles
static double totalTravelTime;  // Sum of travel times of already departed vehicles
static double totalAvgVelocity; // Sum of average velocities of already departed vehicles

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
	totalAvgVelocity = 0.0;
	
	// Call engine setup
	set_up_sim();
	
	// Schedule first event, one for each direction
	if( schedule_event( init_event( -1, ARRIVAL, NORTH) ) != 0 ) { fprintf(stderr,"create_sim()\n"); exit(1); }
	if( schedule_event( init_event( -1, ARRIVAL, SOUTH) ) != 0 ) { fprintf(stderr,"create_sim()\n"); exit(1); }
	
	// Run simulation
	run_sim( simEnd );
	
	// Calculate avgWaitTime
	avgWaitTimeNorth /= northCounter;
	avgWaitTimeSouth /= southCounter;
	
	// Print stats
	printf( "\nNorth direction, inter arrival time: %5.2f ( rate = %6.3f )\n", (double)NB_IAT, (double)(1.0/NB_IAT) );
	printf( "    -Number of vehicles that have crossed (entered) the bridge    : %u\n", northCounter );
	printf( "    -Number of vehicles that are still waiting to enter the bridge: %u\n", get_count( northQueue ) );
	printf( "    -Average wait time: %7.3f\n", avgWaitTimeNorth );
	printf( "\nSouth direction, inter arrival time: %5.2f ( rate = %6.3f )\n", (double)SB_IAT, (double)(1.0/SB_IAT) );
	printf( "    -Number of vehicles that have crossed (entered) the bridge    : %u\n", southCounter );
	printf( "    -Number of vehicles that are still waiting to enter the bridge: %u\n", get_count( southQueue ) );
	printf( "    -Average wait time: %7.3f\n", avgWaitTimeSouth );
}

Event init_event( double timestamp, TypeOfEvent TE, TypeOfObject TO ) {
	
	Event E = create_event();
	Vehicle V = create_vehicle();
	
	// Set timestamp and type of Event
	if( timestamp == -1 ) timestamp = get_sim_time() + randexp( D );
	if( timestamp  <  0 ) { fprintf(stderr,"init_event(), negative timestamp\n"); exit(1); }
	
	if( set_timestamp( E, timestamp ) != 0 ) { fprintf(stderr,"init_event()\n"); exit(1); }
	if( set_type( E, T )			  != 0 ) { fprintf(stderr,"init_event()\n"); exit(1); }
	
	// Set callback functino of Event
	switch( T ) {
		case ARRIVAL:
			if( set_callback( E, arrival   ) == -1 ) { fprintf(stderr,"init_event()\n"); exit(1); }
			break;
		case ENTERING:
			if( set_callback( E, entering  ) == -1 ) { fprintf(stderr,"init_event()\n"); exit(1); }
			break;
		case CROSSING:
			if( set_callback( E, crossing  ) == -1 ) { fprintf(stderr,"init_event()\n"); exit(1); }
			break;
		case DEPARTURE:
			if( set_callback( E, departure ) == -1 ) { fprintf(stderr,"init_event()\n"); exit(1); }
			break;
		default:
			fprintf(stderr,"init_event(), unexpected TypeOfEvent\n"); exit(1);
	}
	// Set vehicle data
	if( set_direction( V, D )			 != 0 )	{ fprintf(stderr,"init_event()\n"); exit(1); }
	if( set_arrival_time( V, timestamp ) != 0 ) { fprintf(stderr,"init_event()\n"); exit(1); }
	if( set_vehicle( E, V )				 != 0 )	{ fprintf(stderr,"init_event()\n"); exit(1); }
	
	return E;
}

double urand() {
	return ( (double)rand() / (double)RAND_MAX );
}

double randexp( Direction D ) {
	double r = -IAT*( log( 1.0 - urand() ) );
	return r;
}

void arrival( void* P ) {
	
}

void entering( void* P ) {
	
}

void crossing( void* P ) {
	
}

void departure( void* P ) {
	
}

void switch_traffic_signal( void* P ) {
	
}

/* eof */
