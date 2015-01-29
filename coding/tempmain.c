//
//  tempmain.c
//  
//
//  Created by Lanssie Ma on 1/29/15.
//
//

#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <math.h>
#define IAT  5.0f

// Generate random number in [0,1)
static double urand() {
    return ( (double)rand() / (double)RAND_MAX );
}

// Generate random number from exp. distribution with mean IAT
static double randexp() {
    return -IAT*( log( 1.0 - urand() ) );
}

int main(int argc, char **argv) {
    for (int i = 0; i<10000; i++){
    printf("%.5f\n",randexp());
    }
}

