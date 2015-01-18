//
//  simApplication.h
//  traffic
//
//  Authors: Mingyue Ma, Eisha Nathan, Stefan Henneking
//  01/12/2015
//

#ifndef traffic_simApplication_h
#define traffic_simApplication_h

#include "simEngine.h"
#include "event.h"

/**
 * - Initializes a new simulation and its parameters
 * - Schedules the first event
 * - Starts the simulation by calling the simulation engine
 *
 * @params simTime: Overall simulation time
**/
void create_sim( double simTime );

/**
 * Initializes a new event
 *
 * @params timestamp: Timestamp for the new Event, use -1 for random timestamp (exp. distr.)
 * @params T		: Type of Event
 * @params D		: Direction of Vehicle
 * @return			: Pointer to Event
 **/
Event init_event( double timestamp, TypeOfEvent ET, TypeOfObject OT );

/**
 * Calculates random number
 *
 * @return: Random number in [0,1)
 **/
double urand();

/**
 * Calculates a random number for the Event timestamp
 * Depends on mean inter arrival time NB_IAT/ SB_IAT (see simApplication.c)
 *
 * @params D: Direction can be used to specify different inter arrival times for north/south direction
 * @return: Random number r = simulation time + x, where x is exp. distr.
 **/
double randexp();

/**
 * Callback function for arrivalEvents
 *
 * @params P: arrivalEvent to process
 **/
void arrival( void* P );

/**
 * Callback function for enteringEvents
 *
 * @params P: enteringEvent to process
 **/
void entering( void* P );

/**
 * Callback function for crossingEvents
 *
 * @params P: crossingEvent to process
 **/
void crossing( void* P );

/**
 * Callback function for departureEvents
 *
 * @params P: departureEvent to processs
 **/
void departure( void* P );

/**
 *
 **/
void switch_traffic_signal( void *P );

#endif

/* eof */
