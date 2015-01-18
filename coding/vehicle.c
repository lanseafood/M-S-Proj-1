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
	int origin;
	int destination;
	double arrivalTime;
	double waitTime;
};

Vehicle create_vehicle() {
	Vehicle V = (Vehicle) malloc( sizeof( struct VehicleType ) );
	if( V == NULL ) {
		fprintf(stderr, "ERROR from create_event(): malloc() failed\n"); exit(1);
	}
	V->id = ++idCounter;
	V->origin = 0;
	V->destination = 0;
	V->arrivalTime = 0.0;
	V->waitTime = 0.0;
	return V;
}

int get_id( Vehicle V ) {
	if( V == NULL ) {
		fprintf(stderr,"get_id(), V is NULL\n");
		exit(1);
	}
	return V->id;
}

int set_origin( Vehicle V, int origin ) {
	V->origin = origin;
	return 0;
}

int get_origin( Vehicle V ) {
	return V->origin;
}

int set_destination( Vehicle V, int destination ) {
	V->destination = destination;
	return 0;
}

int get_destination( Vehicle V ) {
	return V->destination;
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

int add_wait_time( Vehicle V, double waitTime ) {
	V->waitTime += waitTime;
	return 0;
}

double get_wait_time( Vehicle V ) {
	return V->waitTime;
}

/* eof */
