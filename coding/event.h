//
//  event.h
//  traffic
//
//  Authors: Mingyue Ma, Eisha Nathan, Stefan Henneking
//  01/12/2015
//

#ifndef traffic_event_h
#define traffic_event_h

#include "intersection.h"
#include "vehicle.h"

// Declare abstract type of Event
typedef struct EventType *Event;
typedef void	 (*fptr)(void *);

typedef enum {
	VEHICLE,
	INTERSECTION,
	SECTION
} TypeOfObject;

typedef enum {
	ARRIVAL,
	ENTERING,
	CROSSING,
	DEPARTURE
	// Global Arrival
	// Global Departure
	// 
} TypeOfEvent;

/**
 * Create a new Event
 *
 * @return: Pointer to Event on success, exit otherwise
 **/
Event create_event();

/**
 * Frees all space allocated by an Event
 *
 * @params: Pointer to Event
 **/
void free_event( Event E );

/**
 * Set timestamp of Event
 *
 * @params E		: Pointer to Event
 * @params timestamp: Timestamp
 * @return			: 0 on success, -1 otherwise
 **/
int set_timestamp( Event E, double timestamp );

/**
 * Get timestamp of Event
 *
 * @params P: Pointer to Event
 * @return  : Timestamp of Event on success, -1 otherwise
 **/
double get_timestamp( void *P );

/**
 * Set Object of Event
 *
 * @params E: Pointer to Event
 * @params V: Pointer to Object
 * @return  : 0 on success, NULL otherwise
 **/
int set_object( Event E, void *O );

/**
 * Get Object of Event
 *
 * @params E: Pointer to Event
 * @return  : Pointer to Object of Event on success, -1 otherwise
 **/
Vehicle get_object( Event E );

/**
 * Set type of Object
 *
 * @params E : Pointer to Event
 * @params OT: Type of Object
 * @return   : 0 on success, -1 otherwise
 **/
int set_object_type( Event E, TypeOfObject OT );

/**
 * Get type of Object
 *
 * @params E: Pointer to Event
 * @return  : Type of Object on success, -1 otherwise
 **/
TypeOfEvent get_object_type( Event E );

/**
 * Set type of Event
 *
 * @params E : Pointer to Event
 * @params ET: Type of Event
 * @return   : 0 on success, -1 otherwise
 **/
int set_event_type( Event E, TypeOfEvent ET );

/**
 * Get type of Event
 *
 * @params E: Pointer to Event
 * @return  : Type of Event on success, -1 otherwise
 **/
TypeOfEvent get_event_type( Event E );

/**
 * Set Callback function of Event
 *
 * @params E : Pointer to Event
 * @params cb: Pointer to Vehicle
 * @return	 : 0 on success, -1 otherwise
 **/
int set_callback( Event E, fptr cb );

/**
 * Get Callback function of Event
 *
 * @params P: Pointer to Event
 * @return  : Callback function of Event on success, NULL otherwise
 **/
fptr get_callback( void* P );

/**
 * Provides comparison of two Events for PriorityQueue
 *
 * @params P1: Pointer to Event
 * @params P2: Pointer to Event
 * @return	 : signum(P1->timestamp - P2->timestamp)
 **/
int compare_events( void* P1, void* P2 );


#endif

/* eof */
