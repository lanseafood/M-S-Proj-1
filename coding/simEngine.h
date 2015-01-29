//
//  simEngine.h
//  traffic
//
//  Authors: Mingyue Ma, Eisha Nathan, Stefan Henneking
//  01/12/2015
//

#ifndef traffic_simEngine_h
#define traffic_simEngine_h

#include "priorityQueue.h"

typedef void (*cbptr)(void *);

// Returns timestamp for an element
// Has to be provided by user
extern double get_timestamp( queueElement E );
// Returns callback function of an element
// Has to be provided by user
extern cbptr get_callback( queueElement E );
// Flag that indicates if an element has already been scheduled
// 0: not scheduled yet
// 1: already scheduled
extern int set_scheduled( queueElement E, int value );
extern int get_scheduled( queueElement E );

/**
 * Sets up simulation state variables and data structures
 *
 **/
void set_up_sim();

/**
 * Schedules a new element (adds it to Queue)
 *
 * @params: Pointer to Event to schedule
 * @return: 0 on success, -1 otherwise
 **/
int schedule_event( queueElement E );

/**
 * Starts a new simulation
 *
 * @params: Overall simulation time
 **/
void run_sim( double simEnd );

/**
 * Gets the simulation time
 *
 * @return: Simulation time
 **/
double get_sim_time();

#endif

/* eof */
