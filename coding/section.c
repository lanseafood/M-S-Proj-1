//
//  section.c
//  traffic
//
//  Authors: Mingyue Ma, Eisha Nathan, Stefan Henneking
//  01/12/2015
//

#include <stdio.h>
#include <stdlib.h>

#include "section.h"
#include "simApplication.h"
#include "simEngine.h"

struct SectionType {
	int zoneID;
	double len;
	int n_lanes, s_lanes;
	int n_capacity, s_capacity;
	int n_vehicles, s_vehicles;
	int n_congestion_flag, s_congestion_flag;
};

Section create_section(int zoneID, double vehicle_len, double safety_dist_queue) {
	
	Section S = (Section) malloc(sizeof(struct SectionType));
	S->zoneID = zoneID;

	double len = 0;

	if (zoneID==1 || zoneID==6) {
		len = 0;      S->n_lanes = 0; S->s_lanes = 0;
	} else if (zoneID==2) {
		len = 417.97; S->n_lanes = 2; S->s_lanes = 3;
	} else if (zoneID==3) {
		len = 412.17; S->n_lanes = 2; S->s_lanes = 2;
	} else if (zoneID==4) {
		len = 351.51; S->n_lanes = 2; S->s_lanes = 2;
	} else if (zoneID==5) {
		len = 344.43; S->n_lanes = 3; S->s_lanes = 2;
	} else;

	S->n_capacity = (int) ( S->n_lanes*len / (vehicle_len+safety_dist_queue) );
	S->s_capacity = (int) ( S->s_lanes*len / (vehicle_len+safety_dist_queue) );
	S->n_vehicles = 0;
	S->s_vehicles = 0;
	S->n_congestion_flag = 0;
	S->s_congestion_flag = 0;
	S->len = len;
	
	return S;
}

int get_zoneID(Section S) {
	return S->zoneID;
}

double get_len(Section S){
	return S->len;
}

int change_north_vehicles(Section S, int val){
	S->n_vehicles += val;
	if( !(S->n_vehicles < S->n_capacity) ) {
		if( !S->n_congestion_flag ) {
			S->n_congestion_flag = 1;
			#if VERBOSE == 1
				printf("%7.2f, Congestion in Section %d in north direction\n", get_sim_time(), S->zoneID );
			#endif
		}
	}
	else {
		if( S->n_congestion_flag ) {
			S->n_congestion_flag = 0;
			#if VERBOSE == 1
				printf("%7.2f, Section %d clear in north direction\n", get_sim_time(), S->zoneID );
			#endif
			section_clear( S->zoneID-1, NORTH );
		}
	}
	return 0;
}

int change_south_vehicles(Section S, int val){
	S->s_vehicles += val;
	if( !(S->s_vehicles < S->s_capacity) ) {
		if( !S->s_congestion_flag ) {
			S->s_congestion_flag = 1;
			#if VERBOSE == 1
				printf("%7.2f, Congestion in Section %d in south direction\n", get_sim_time(), S->zoneID );
			#endif
		}
	}
	else {
		if( S->s_congestion_flag ) {
			S->s_congestion_flag = 0;
			#if VERBOSE == 1
				printf("%7.2f, Section %d clear in south direction\n", get_sim_time(), S->zoneID );
			#endif
			section_clear( S->zoneID, SOUTH );
		}
	}
	return 0;
}

int get_north_vehicles(Section S){
	return S->n_vehicles;
}

int get_south_vehicles(Section S){
	return S->s_vehicles;
}

int get_congestion_flag( Section S, Direction D ) {
	if     ( D == NORTH ) return S->n_congestion_flag;
	else if( D == SOUTH ) return S->s_congestion_flag;
	else return -1;
}

/* eof */
