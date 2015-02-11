//
//  vehicle.h
//  traffic
//
//  Authors: Mingyue Ma, Eisha Nathan, Stefan Henneking
//  01/12/2015
//

#ifndef traffic_vehicle_h
#define traffic_vehicle_h

#include "intersection.h"

// Declare abstract type of Vehicle
typedef struct VehicleType *Vehicle;

/**
 * Creates a new Event
 *
 * @return: Pointer to Event on success, exit otherwise
 **/
Vehicle create_vehicle();

/**
 * Gets unique id of Vehicle
 *
 * @params: Pointer to Vehicle
 * @return: Id of Vehicle on success, exits otherwise
 **/
int get_id( Vehicle V );

/**
 * Sets origin zone of Vehicle
 *
 * @return: 0 on success, -1 otherwise
 **/
int set_origin( Vehicle V, int origin );

/**
 * Gets origin zone of Vehicle
 *
 * @params: Pointer to Vehicle
 * @return: Direction of Vehicle on success, -1 otherwise
 **/
int get_origin( Vehicle V );

/**
 * Sets destination zone of Vehicle
 *
 * @return: 0 on success, -1 otherwise
 **/
int set_destination( Vehicle V, int destination );

/**
 * Gets destination zone of Vehicle
 *
 * @params: Pointer to Vehicle
 * @return: Direction of Vehicle on success, -1 otherwise
 **/
int get_destination( Vehicle V );

/**
 * Sets lane ID of Vehicle
 *
 * @return: 0 on success, -1 otherwise
 **/
int set_laneID( Vehicle V, int laneID );

/**
 * Gets lane ID of Vehicle
 *
 * @params: Pointer to Vehicle
 * @return: Lane ID of Vehicle on success, -1 otherwise
 **/
int get_laneID( Vehicle V );

int set_dir( Vehicle V, Direction dir );
Direction get_dir( Vehicle V );

int set_departDir( Vehicle V, Direction dir );
Direction get_departDir( Vehicle V );

int set_route( Vehicle V, Route route );
Route get_route( Vehicle V );

/**
 * 
 * Sets current velocity of Vehicle
 *
 * @params: Pointer to Vehicle
 * @return: 0 on success, -1 otherwise
 **/
int set_velocity( Vehicle V, double velocity );

/**
 * Gets current velocity of Vehicle
 *
 * @params: Pointer to Vehicle
 * @return: Velocity of Vehicle on success, -1 otherwise
 **/
double get_velocity( Vehicle V );

/**
 * Sets arrival time of Vehicle
 *
 * @return: 0 on success, -1 otherwise
 **/
int set_arrival_time( Vehicle V, double arrivalTime );

/**
 * Gets arrival time of Vehicle
 *
 * @params: Pointer to Vehicle
 * @return: Arrival time of Vehicle on success, -1 otherwise
 **/
double get_arrival_time( Vehicle V );

/**
 * Increment wait time of Vehicle
 *
 * @return: 0 on success, -1 otherwise
 **/
int add_wait_time( Vehicle V, double waitTime );

/**
 * Gets wait time of Vehicle
 *
 * @params: Pointer to Vehicle
 * @return: Arrival time of Vehicle on success, -1 otherwise
 **/
double get_wait_time( Vehicle V );

int set_wait_time_buf( Vehicle V, double waitTimeBuf );

double get_wait_time_buf( Vehicle V );

int set_temp_distance( Vehicle V, double tempDistance );

double get_temp_distance( Vehicle V );


#endif

/* eof */
