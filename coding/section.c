//
//  vehicle.c
//  traffic
//
//  Authors: Mingyue Ma, Eisha Nathan, Stefan Henneking
//  01/12/2015
//


#include <stdio.h>
#include <stdlib.h>

#include "section.h"

struct SectionType {
	int zoneID;
	double len; 
	int n_cars; //num cars north
	int s_cars; //num cars south
};



Section create_section(int zoneID) {
	Section S = (Section) malloc(sizeof(struct SectionType));
	S->zoneID = zoneID;

	double len=0;

	if (zoneID==1 || zoneID==6) {
		len = 0;
	} else if (zoneID==2) {
		len = 417.97;	
	} else if (zoneID==3) {
		len = 412.17;	
	} else if (zoneID==4) {
		len = 351.51;	
	} else if (zoneID==5) {
		len = 344.43;	
	}

	S->n_cars = 0;
	S->s_cars = 0;
	S->len = len;
	
	return S;
}

int get_zoneID(Section S) {
	return S->zoneID;
}

double get_len(Section S){
	return S->len;
}

int add_north_cars(Section S){
	S->n_cars++;
	return 0;
}

int add_south_cars(Section S){
	S->s_cars++;
	return 0;
}

int get_north_cars(Section S){
	return S->n_cars;
}

int get_south_cars(Section S){
	return S->s_cars;
}
