//
//  event.c
//  traffic
//
//  Authors: Mingyue Ma, Eisha Nathan, Stefan Henneking
//  01/12/2015
//

#include <stdio.h>
#include <stdlib.h>

#include "event.h"

struct EventType {
	double timestamp;
	Vehicle vehicle;
	TypeOfEvent type;
	fptr callback;
};

Event create_event() {
	Event E = (Event) malloc( sizeof( struct EventType ) );
	if( E == NULL ) {
		fprintf(stderr, "ERROR from create_event(): malloc() failed\n"); exit(1);
	}
	E->vehicle = NULL;
	E->callback = NULL;
	return E;
}

void free_event( Event E ) {
	if( E == NULL ) return;
	if( E->vehicle != NULL ) {
		free(E->vehicle);
		E->vehicle = NULL;
	}
	E->callback = NULL;
	free(E);
	E = NULL;
}

int set_timestamp( Event E, double timestamp ) {
	if( E == NULL	   ) return -1;
	if( timestamp < 0.0) return -1;
	E->timestamp = timestamp;
	return 0;
}

double get_timestamp( void* P ) {
	Event E = (Event) P;
	if( E == NULL ) return -1;
	return E->timestamp;
}

int set_type( Event E, TypeOfEvent T ) {
	if( E == NULL ) return -1;
	if( T != ARRIVAL && T != ENTERING && T != CROSSING && T != DEPARTURE ) return -1;
	E->type = T;
	return 0;
}

TypeOfEvent get_type( Event E ) {
	if( E == NULL ) return -1;
	return E->type;
}

int set_vehicle( Event E, Vehicle V ) {
	if( E == NULL ) return -1;
	if( V == NULL ) return -1;
	E->vehicle = V;
	return 0;

}

Vehicle get_vehicle( Event E ) {
	if( E == NULL ) return NULL;
	return E->vehicle;
}

int set_callback( Event E, fptr cb ) {
	if( E  == NULL ) return -1;
	if( cb == NULL ) return -1;
	E->callback = cb;
	return 0;
}

fptr get_callback( void* P ) {
	Event E = (Event) P;
	if( E == NULL ) return NULL;
	return E->callback;
}

int compare_events( void* P1, void* P2 ) {
	Event E1 = (Event) P1;
	Event E2 = (Event) P2;
	if( E1 == NULL ) {
		fprintf(stderr,"compare_to(), E1 is NULL\n");
		exit(1);
	}
	if( E2 == NULL ) {
		fprintf(stderr,"compare_to(), E2 is NULL\n");
		exit(1);
	}
	return( E1->timestamp < E2->timestamp ) ? 1:0;
}

/* eof */
