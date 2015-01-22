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
	// Global events
	GLOBAL_ARRIVAL, GLOBAL_DEPARTURE,
	// Intersection signal events
	IS_1_SIGNAL, IS_2_SIGNAL, IS_3_SIGNAL, IS_4_SIGNAL, IS_5_SIGNAL,

	/* EISHA -- naive intersection signal methods
	IS_1_SIGNAL_NT, IS_1_SIGNAL_ET, IS_1_SIGNAL_ST, IS_1_SIGNAL_WT,
	IS_2_SIGNAL_NT, IS_2_SIGNAL_ET, IS_2_SIGNAL_ST, IS_2_SIGNAL_WT,
	IS_3_SIGNAL_NT, IS_3_SIGNAL_ET, IS_3_SIGNAL_ST, IS_3_SIGNAL_WT,
	IS_5_SIGNAL_NT, IS_5_SIGNAL_ET, IS_5_SIGNAL_ST, IS_5_SIGNAL_WT,
	IS_1_SIGNAL_NL, IS_1_SIGNAL_EL, IS_1_SIGNAL_SL, IS_1_SIGNAL_WL,
	IS_5_SIGNAL_NL, IS_5_SIGNAL_EL, IS_5_SIGNAL_SL, IS_5_SIGNAL_WL,
	*/

	// Intersection vehicle events
	IS_1_N_ARRIVAL, IS_1_N_ENTERING, IS_1_N_CROSSING, IS_1_N_DEPARTURE,
	IS_1_E_ARRIVAL, IS_1_E_ENTERING, IS_1_E_CROSSING, IS_1_E_DEPARTURE,
	IS_1_S_ARRIVAL, IS_1_S_ENTERING, IS_1_S_CROSSING, IS_1_S_DEPARTURE,
	IS_1_W_ARRIVAL, IS_1_W_ENTERING, IS_1_W_CROSSING, IS_1_W_DEPARTURE,
	// Section congestion events
	S_2_N_CLEAR, S_2_S_CLEAR,
	S_3_N_CLEAR, S_3_S_CLEAR,
	S_4_N_CLEAR, S_4_S_CLEAR,
	S_5_N_CLEAR, S_5_S_CLEAR
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
