//
//  main.c
//  traffic
//
//  Authors: Mingyue Ma, Eisha Nathan, Stefan Henneking
//  01/12/2015
//

#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>

#include "simApplication.h"

int main(int argc, char **argv) {
    
    // Check input arguments
    if( argc > 2 ) {
        fprintf(stderr,"\nUsage: %s [seed]\n\n", argv[0]);
        return -1;
    }
	
	// Seed value for random number generator
	int seed = 0;
	if( argc == 2 ) {
		seed = atoi(argv[1]);
	} else {
		struct timeval t1;
		gettimeofday(&t1, NULL);
		seed = (int)(t1.tv_usec * t1.tv_sec);
	}
	srand(seed);
	
	// Overall simulation time in seconds
	double hours   = 0;
	double minutes = 1;
	double seconds = 0;
	double simEnd  = hours*3600 + minutes*60 + seconds;
	
	// Simulate road network
	create_sim( simEnd );
	
    return 0;
}

/* eof */
