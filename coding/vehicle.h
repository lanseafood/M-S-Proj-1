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


#endif

/* eof */
