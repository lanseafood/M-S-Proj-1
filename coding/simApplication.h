//
//  simApplication.h
//  traffic
//
//  Authors: Mingyue Ma, Eisha Nathan, Stefan Henneking
//  01/12/2015
//

#ifndef traffic_simApplication_h
#define traffic_simApplication_h

#include "intersection.h"
#include "simEngine.h"

// Print events to stdout
#define VERBOSE 1

/**
 * - Initializes a new simulation and its parameters
 * - Schedules the first event
 * - Starts the simulation by calling the simulation engine
 *
 * @params simTime: Overall simulation time
**/
void create_sim( double simTime );

void section_clear( int ID, Direction D );

#endif

/* eof */
