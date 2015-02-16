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
	int laneID;
	Direction dir;
	Direction departDir;
	Route route;
	double velocity;
	double arrivalTime;
	double waitTime;
	double waitTimeBuf;
	double tempDistance;
};

Vehicle create_vehicle() {
	Vehicle V = (Vehicle) malloc( sizeof( struct VehicleType ) );
	if( V == NULL ) {
		fprintf(stderr, "ERROR from create_event(): malloc() failed\n"); exit(1);
	}
	V->id = ++idCounter;
	V->origin = 0;
	V->destination = 0;
	V->laneID = -1;
	V->velocity = 0.0;
	V->arrivalTime = 0.0;
	V->waitTime = 0.0;
	V->waitTimeBuf = 0.0;
	V->tempDistance = 0.0;
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

int set_laneID( Vehicle V, int laneID ) {
	V->laneID = laneID;
	return 0;
}

int get_laneID( Vehicle V ) {
	return V->laneID;
}

int set_dir( Vehicle V, Direction dir ) {
	V->dir = dir;
	return 0;
}

Direction get_dir( Vehicle V ) {
	return V->dir;
}

int set_departDir( Vehicle V, Direction dir ) {
	V->departDir = dir;
	return 0;
}

Direction get_departDir( Vehicle V ) {
	return V->departDir;
}

int set_route( Vehicle V, Route route ) {
	V->route = route;
	return 0;
}

Route get_route( Vehicle V ) {
	if( V == NULL ) {
		fprintf(stderr,"get_route(), V is NULL\n");
		exit(1);
	}
	return V->route;
}

int set_velocity( Vehicle V, double velocity ) {
	V->velocity = velocity;
	return 0;
}

double get_velocity( Vehicle V ) {
	return V->velocity;
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

int set_wait_time_buf( Vehicle V, double waitTimeBuf ) {
	V->waitTimeBuf = waitTimeBuf;
	return 0;
}

double get_wait_time_buf( Vehicle V ) {
	return V->waitTimeBuf;
}

int set_temp_distance( Vehicle V, double tempDistance ) {
	V->tempDistance = tempDistance;
	return 0;
}

double get_temp_distance( Vehicle V ) {
	return V->tempDistance;
}

/* eof */
