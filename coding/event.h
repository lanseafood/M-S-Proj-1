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
typedef void (*fptr)(void *);

typedef enum {
	VEHICLE,
	INTERSECTION,
	SECTION
} TypeOfObject;

typedef enum {
	INV = -1,
	// Global events
	GLOBAL_ARRIVAL, GLOBAL_DEPARTURE,
	// Intersection signal event
	IS_SIGNAL,
	// Intersection vehicle events
	IS_1_ARRIVAL, IS_1_ENTERING, IS_1_CROSSING, IS_1_DEPARTURE,
	IS_2_ARRIVAL, IS_2_ENTERING, IS_2_CROSSING, IS_2_DEPARTURE,
	IS_3_ARRIVAL, IS_3_ENTERING, IS_3_CROSSING, IS_3_DEPARTURE,
	IS_4_ARRIVAL, IS_4_ENTERING, IS_4_CROSSING, IS_4_DEPARTURE,
	IS_5_ARRIVAL, IS_5_ENTERING, IS_5_CROSSING, IS_5_DEPARTURE,
	
	// Section congestion events
	S_2_N_CLEAR, S_2_S_CLEAR,
	S_3_N_CLEAR, S_3_S_CLEAR,
	S_4_N_CLEAR, S_4_S_CLEAR,
	S_5_N_CLEAR, S_5_S_CLEAR,
	// Number of event types
	NUM_EVENT_TYPES
} TypeOfEvent;

static const char TypeOfEventStrings[NUM_EVENT_TYPES][20] = {
	{"GLOBAL_ARRIVAL"},{"GLOBAL_DEPARTURE"},
	{"IS_SIGNAL"},
	
	{"IS_1_ARRIVAL"},{"IS_1_ENTERING"},{"IS_1_CROSSING"},{"IS_1_DEPARTURE"},
	{"IS_2_ARRIVAL"},{"IS_2_ENTERING"},{"IS_2_CROSSING"},{"IS_2_DEPARTURE"},
	{"IS_3_ARRIVAL"},{"IS_3_ENTERING"},{"IS_3_CROSSING"},{"IS_3_DEPARTURE"},
	{"IS_4_ARRIVAL"},{"IS_4_ENTERING"},{"IS_4_CROSSING"},{"IS_4_DEPARTURE"},
	{"IS_5_ARRIVAL"},{"IS_5_ENTERING"},{"IS_5_CROSSING"},{"IS_5_DEPARTURE"},
	
	{"S_2_CLEAR"},{"S_3_CLEAR"},{"S_4_CLEAR"},{"S_5_CLEAR"}
};

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

int set_scheduled( void *P, int value );
int get_scheduled( void *P );

/**
 * Set Object of Event
 *
 * @params E: Pointer to Event
 * @params V: Pointer to Object
 * @return  : 0 on success, -1 otherwise
 **/
int set_object( Event E, void *object );

/**
 * Get Object of Event
 *
 * @params E: Pointer to Event
 * @return  : Pointer to Object of Event on success, -1 otherwise
 **/
void* get_object( Event E );

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
TypeOfObject get_object_type( Event E );

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
 * Provides timestamp comparison of two Events for PriorityQueue
 *
 * @params P1: Pointer to Event
 * @params P2: Pointer to Event
 * @return	 : signum(P1->timestamp - P2->timestamp)
 **/
int compare_events( void* P1, void* P2 );


#endif

/* eof */
