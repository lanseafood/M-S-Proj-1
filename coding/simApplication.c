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

#define CROSS 30	// Time to cross the bridge (including ENTER)
#define ENTER 2		// Time to enter the bridge
#define NB_IAT 9.5	// Northbound inter arrival time
#define SB_IAT 12.5	// Southbound inter arrival time
#define GROUP 10	// Maximum #Vehicles allowed in a group

// Pointer to compare function for priority queue
int (*compare_to)( void*, void* );

//
static Direction	 bridgeState;
static PriorityQueue northQueue;
static PriorityQueue southQueue;
static double		 avgWaitTimeNorth;
static double		 avgWaitTimeSouth;
static unsigned int	 northCounter;
static unsigned int	 southCounter;
static unsigned int  groupEnteringCounter;
static unsigned int  groupDepartureCounter;

void create_sim( double simEnd ) {
	// Set compare function pointer for priority queue
	compare_to = compare_events;
	
	// Initialize state variables
	bridgeState		 = EMPTY;
	northQueue		 = create_priority_queue();
	southQueue		 = create_priority_queue();
	avgWaitTimeNorth = 0;
	avgWaitTimeNorth = 0;
	northCounter	 = 0;
	southCounter	 = 0;
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
	// Clean up
	free_priority_queue( northQueue );
	free_priority_queue( southQueue );
}

Event init_event( double timestamp, TypeOfEvent T, Direction D ) {
	
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
	double r;
	if	   ( D == NORTH) r = -NB_IAT*( log( 1.0 - urand() ) );
	else if( D == SOUTH) r = -SB_IAT*( log( 1.0 - urand() ) );
	else { fprintf(stderr,"randexp(), unexpected Direction\n"); exit(1); }
	return r;
}

void arrival( void* P ) {
	Event E = (Event) P;
	if( E == NULL ) { fprintf(stderr,"arrival(), E is NULL \n"); exit(1); }
	// Get Vehicle information
	Vehicle V = get_vehicle( E );
	if( V == NULL ) { fprintf(stderr,"arrival(), V is NULL \n"); exit(1); }
	// Print information to stdout
	printf("Vehicle ID %5d, Arrival   at t = %7.3f", get_id(V), get_sim_time());
	switch( get_direction( V ) ) {
		case NORTH:
			printf(", north direction\n"); break;
		case SOUTH:
			printf(", south direction\n"); break;
		default:
			fprintf(stderr,"arrival(), unexpected Vehicle direction\n"); exit(1);
	}
	// Schedule event or put it in queue, depends on bridgeState
	switch( bridgeState ) {
		case EMPTY:
			if( set_type( E, ENTERING ) != 0 )	   { fprintf(stderr,"arrival()\n"); exit(1); }
			if( set_callback( E, entering ) != 0 ) { fprintf(stderr,"arrival()\n"); exit(1); }
			if( schedule_event( E ) != 0 )		   { fprintf(stderr,"arrival()\n"); exit(1); }
			break;
		default:
			switch( get_direction( V ) ) {
				case NORTH:
					if( add( northQueue, E ) != 0 ) { fprintf(stderr,"arrival()\n"); exit(1); }
					break;
				case SOUTH:
					if( add( southQueue, E ) != 0 ) { fprintf(stderr,"arrival()\n"); exit(1); }
					break;
				default:
					fprintf(stderr,"arrival(), unexpected bridgeState\n"); exit(1);
			}
	}
	
	// Initialize new arrival event with randexp() and schedule it
	Event newE = NULL;
	if( get_direction(V) == NORTH ) newE = init_event( -1, ARRIVAL, NORTH );
	if( get_direction(V) == SOUTH ) newE = init_event( -1, ARRIVAL, SOUTH );
	if( schedule_event( newE ) != 0 ) { fprintf(stderr,"arrival()\n" ); exit(1); }
}

void entering( void* P ) {
	Event E = (Event) P;
	if( E == NULL ) { fprintf(stderr,"entering(), E is NULL\n"); exit(1); }
	// Get Vehicle information
	Vehicle V = get_vehicle( E );
	if( V == NULL ) { fprintf(stderr,"entering(), V is NULL\n"); exit(1); }
	// Print information to stdout
	printf("Vehicle ID %5d, Entering  at t = %7.3f", get_id(V), get_sim_time());
	
	// Add wait time of this Vehicle to average wait time
	switch( get_direction( V ) ) {
		case NORTH:
			printf(", north direction\n");
			if( bridgeState != NORTH ) bridgeState = NORTH;
			avgWaitTimeNorth += ( get_sim_time() - get_arrival_time( V ) );
			northCounter++;
			break;
		case SOUTH:
			printf(", south direction\n");
			if( bridgeState != SOUTH ) bridgeState = SOUTH;
			avgWaitTimeSouth += ( get_sim_time() - get_arrival_time( V ) );
			southCounter++;
			break;
		default:
			fprintf(stderr,"entering(), unexpected Vehicle direction \n"); exit(1);
	}
	groupEnteringCounter++;
	// Schedule crossing for this Vehicle
	if( set_timestamp( E, get_sim_time()+ENTER ) != 0 ) { fprintf(stderr,"entering()\n"); exit(1); }
	if( set_type( E, CROSSING )					 != 0 )	{ fprintf(stderr,"entering()x\n"); exit(1); }
	if( set_callback( E, crossing )				 != 0 )	{ fprintf(stderr,"entering()x\n"); exit(1); }
	if( schedule_event( E )						 != 0 )	{ fprintf(stderr,"entering()x\n"); exit(1); }
}

void crossing( void* P ) {
	Event E = (Event) P;
	if( E == NULL ) { fprintf(stderr,"crossing(), E is NULL\n"); exit(1); }
	// Get Vehicle information
	Vehicle V = get_vehicle( E );
	if( V == NULL ) { fprintf(stderr,"crossing(), V is NULL\n"); exit(1); }
	// Print information to stdout
	printf("Vehicle ID %5d, Crossing  at t = %7.3f", get_id(V), get_sim_time());
	
	// Schedule entering for another waiting vehicle, if number of Vehicles in group is less than defined GROUP
	switch( bridgeState ) {
		case NORTH:
			printf(", north direction\n");
			if( !is_empty( northQueue ) && ( groupEnteringCounter < GROUP ) ) {
				Event nextE = poll( northQueue );
				if( nextE == NULL )								  { fprintf(stderr,"crossing()\n"); exit(1); }
				if( set_timestamp( nextE, get_sim_time() ) != 0 ) { fprintf(stderr,"crossing()\n"); exit(1); }
				if( set_type( nextE, ENTERING )			   != 0 ) { fprintf(stderr,"crossing()\n"); exit(1); }
				if( set_callback( nextE, entering )		   != 0 ) { fprintf(stderr,"crossing()\n"); exit(1); }
				if( schedule_event( nextE )				   != 0 ) { fprintf(stderr,"crossing()\n"); exit(1); }
			}
			break;
		case SOUTH:
			printf(", south direction\n");
			if( !is_empty( southQueue ) && ( groupEnteringCounter < GROUP ) ) {
				Event nextE = poll( southQueue );
				if( nextE == NULL )								  { fprintf(stderr,"crossing()\n"); exit(1); }
				if( set_timestamp( nextE, get_sim_time() ) != 0 ) { fprintf(stderr,"crossing()\n"); exit(1); }
				if( set_type( nextE, ENTERING )			   != 0 ) { fprintf(stderr,"crossing()\n"); exit(1); }
				if( set_callback( nextE, entering )		   != 0 ) { fprintf(stderr,"crossing()\n"); exit(1); }
				if( schedule_event( nextE )				   != 0 ) { fprintf(stderr,"crossing()\n"); exit(1); }
			}
			break;
		default:
			fprintf(stderr,"crossing(), unexpected bridgeState\n"); exit(1);
	}
	
	// Schedule departure for this Vehicle
	if( set_timestamp( E, get_sim_time()+CROSS-ENTER ) != 0 ) { fprintf(stderr,"crossing()\n"); exit(1); }
	if( set_type( E, DEPARTURE )					   != 0 ) { fprintf(stderr,"crossing()\n"); exit(1); }
	if( set_callback( E, departure )				   != 0 ) { fprintf(stderr,"crossing()\n"); exit(1); }
	if( schedule_event( E )							   != 0 ) { fprintf(stderr,"crossing()\n"); exit(1); }
}

void departure( void* P ) {
	Event E = (Event) P;
	if( E == NULL ) { fprintf(stderr,"departure(), E is NULL\n"); exit(1); }
	// Get Vehicle information
	Vehicle V = get_vehicle( E );
	if( V == NULL ) { fprintf(stderr,"departure(), V is NULL\n"); exit(1); }
	// Print information to stdout
	printf("Vehicle ID %5d, Departure at t = %7.3f", get_id(V), get_sim_time());
	switch( get_direction( V ) ) {
		case NORTH:
			printf(", north direction\n"); break;
		case SOUTH:
			printf(", south direction\n"); break;
		default:
			fprintf(stderr,"departure(), unexpected Vehicle direction\n"); exit(1);
	}
	
	// If group has finished crossing: Switch bridgeState and reset counter
	if( groupEnteringCounter == ++groupDepartureCounter ) {
		switch_bridge_state();
		groupEnteringCounter  = 0;
		groupDepartureCounter = 0;
	} else {
		// Clean up this event and return if group hasn't finished crossing
		free_event( E );
		return;
	}
	
	// bridgeState has already switched (see above), schedule next Event
	switch( bridgeState ) {
		case NORTH: {
			Event nextE = poll( northQueue );
			if( nextE == NULL )								  { fprintf(stderr,"departure()\n"); exit(1); }
			if( set_timestamp( nextE, get_sim_time() ) != 0 ) { fprintf(stderr,"departure()\n"); exit(1); }
			if( set_type( nextE, ENTERING )			   != 0 ) { fprintf(stderr,"departure()\n"); exit(1); }
			if( set_callback( nextE, entering )		   != 0 ) { fprintf(stderr,"departure()\n"); exit(1); }
			if( schedule_event( nextE )				   != 0 ) { fprintf(stderr,"departure()\n"); exit(1); }
			break;
		}
		case SOUTH: {
			Event nextE = poll( southQueue );
			if( nextE == NULL )								  { fprintf(stderr,"departure()\n"); exit(1); }
			if( set_timestamp( nextE, get_sim_time() ) != 0 ) { fprintf(stderr,"departure()\n"); exit(1); }
			if( set_type( nextE, ENTERING )			   != 0 ) { fprintf(stderr,"departure()\n"); exit(1); }
			if( set_callback( nextE, entering )		   != 0 ) { fprintf(stderr,"departure()\n"); exit(1); }
			if( schedule_event( nextE )				   != 0 ) { fprintf(stderr,"departure()\n"); exit(1); }
			break;
		}
		default: //nothing to do
			break;
	}
	// Clean up
	free_event( E );
}

void switch_bridge_state() {
	if (is_empty(northQueue) && is_empty(southQueue) ) {
		bridgeState = EMPTY;
		return;
	}
	switch( bridgeState ) {
		case NORTH:
			if( !is_empty(southQueue) ) bridgeState = SOUTH;
			else					    bridgeState = NORTH;
			break;
		case SOUTH:
			if( !is_empty(northQueue) ) bridgeState = NORTH;
			else					    bridgeState = SOUTH;
			break;
		default: //nothing to do
			break;
	}
}

/* eof */
