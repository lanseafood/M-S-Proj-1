//
//  vehicle.c
//  traffic
//
//  Authors: Mingyue Ma, Eisha Nathan, Stefan Henneking
//  01/12/2015
//

#include <stdio.h>
#include <stdlib.h>

#include "vehicle.h"

static int idCounter = 0;

struct VehicleType {
	int id;
	Direction direction;
	double arrivalTime;
};

Vehicle create_vehicle() {
	Vehicle V = (Vehicle) malloc( sizeof( struct VehicleType ) );
	if( V == NULL ) {
		fprintf(stderr, "ERROR from create_event(): malloc() failed\n"); exit(1);
	}
	V->id = ++idCounter;
	return V;
}

int get_id( Vehicle V ) {
	if( V == NULL ) {
		fprintf(stderr,"get_id(), V is NULL\n");
		exit(1);
	}
	return V->id;
}

int set_direction( Vehicle V, Direction D ) {
	if( V == NULL				 ) return -1;
	if( D != NORTH && D != SOUTH ) return -1;
	V->direction = D;
	return 0;
}

Direction get_direction( Vehicle V ) {
	if( V == NULL ) return -1;
	return V->direction;
}

int set_arrival_time( Vehicle V, double arrivalTime ) {
	if( V == NULL		 ) return -1;
	if( arrivalTime < 0.0) return -1;
	V->arrivalTime = arrivalTime;
	return 0;
}

double get_arrival_time( Vehicle V ) {
	if( V == NULL ) return -1;
	return V->arrivalTime;
}

/* eof */
