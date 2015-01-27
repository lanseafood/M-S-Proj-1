//
//  simEngine.c
//  traffic
//
//  Authors: Mingyue Ma, Eisha Nathan, Stefan Henneking
//  01/12/2015
//

#include <stdio.h>
#include <stdlib.h>

#include "simEngine.h"

static double		 simTime;
static PriorityQueue eventQueue;

void set_up_sim() {
	eventQueue = create_priority_queue();
	simTime = 0.0;
}

int schedule_event( queueElement E ) {
	if( add( eventQueue, E ) == -1 ) return -1;
	return 0;
}

void run_sim( double simEnd ) {
	
	queueElement nextElement = NULL;
	cbptr cb = NULL;
	
	while( (simTime < simEnd) && !(is_empty(eventQueue)) ) {
		
		// Get next Event from Queue
		nextElement = poll( eventQueue );
		if( nextElement == NULL ) { fprintf(stderr,"run_sim(), nextElement is NULL\n"); exit(1); }
		
		// Update simulation time
		if( get_timestamp(nextElement) < simTime ) {
			fprintf(stderr,"run_sim(), event timestamp < simTime\n");
			printf( "event timestamp: %f ; simTime: %f\n", get_timestamp(nextElement), simTime );
			exit(1);
		}
		if( get_timestamp(nextElement) < simEnd ) simTime = get_timestamp(nextElement);
		else break;
		
		// Call event handler
		cb = get_callback( nextElement );
		cb( nextElement );
	}
	free_priority_queue( eventQueue );
}

double get_sim_time() {
	return simTime;
}

/* eof */
