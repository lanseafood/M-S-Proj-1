//
//  vehicle.h
//  traffic
//
//  Authors: Mingyue Ma, Eisha Nathan, Stefan Henneking
//  01/12/2015
//

#ifndef traffic_vehicle_h
#define traffic_vehicle_h

// Declare abstract type of Vehicle
typedef struct VehicleType *Vehicle;

typedef enum {
	NORTH,
	SOUTH,
	EMPTY
} Direction;

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
 * Sets direction of Vehicle
 *
 * @return: 0 on success, -1 otherwise
 **/
int set_direction( Vehicle V, Direction D );

/**
 * Gets direction of Vehicle
 *
 * @params: Pointer to Vehicle
 * @return: Direction of Vehicle on success, -1 otherwise
 **/
Direction get_direction( Vehicle V );

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

#endif

/* eof */
