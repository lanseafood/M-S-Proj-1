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
    if( argc != 1 ) {
        fprintf(stderr,"\nNo arguments allowed! Usage: %s\n\n", argv[0]);
        return -1;
    }
	// Seed value for random number generator
	struct timeval t1;
    gettimeofday(&t1, NULL);
    srand((unsigned int)(t1.tv_usec * t1.tv_sec));
	
	// Overall simulation time in seconds
	double hours   = 4;
	double minutes = 0;
	double seconds = 0;
	double simEnd  = hours*3600 + minutes*60 + seconds;
	
	// Simulate one lane bridge
	create_sim( simEnd );
	
    return 0;
}

/* eof */
