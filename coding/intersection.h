//
//  intersection.h
//  traffic
//
//  Authors: Mingyue Ma, Eisha Nathan, Stefan Henneking
//  01/12/2015
//

#ifndef traffic_intersection_h
#define traffic_intersection_h

#include "linkedList.h"

// Declare abstract type of Section
typedef struct IntersectionType *Intersection;

typedef enum {
	GREEN,
	YELLOW,
	RED,
	INV
} Color;

// Direction
typedef enum {
	NORTH,
	EAST,
	SOUTH,
	WEST
} Direction;

typedef enum {
	RIGHT,
	STRAIGHT,
	LEFT
} Route;

// Traffic signal struct
typedef struct {
	int zoneID;
	double *times;
	Color init;
	Color light;
} Signal;

Intersection create_intersection();

int set_red(Intersection I, int num);

int get_curr_phase(Intersection I, int curr);

int *get_phase(Intersection I);

int change_phase(Intersection I, int num);

Signal **get_through_signals(Intersection I);

Signal **get_left_signals(Intersection I);

int ***get_signalStatus(Intersection I);

int ***get_leftSignalStatus(Intersection I);

double *get_phaseLengths(Intersection I);

LinkedList **get_laneQueues(Intersection I);

int *get_numLanes(Intersection I);

int get_maxPhase(Intersection I);

int get_currPhase(Intersection I);

int set_next_phase(Intersection I);

int get_inter_zoneID(Intersection I);

int get_signal_zoneID(Signal *S);

int set_init(Signal *S, Color initial_color);

Color get_init(Signal *S);

int set_light(Intersection I, Direction D, int laneID, Color light);

Color get_light(Intersection I, Direction D, int laneID);

double get_ns_len(Intersection I);

double get_ew_len(Intersection I);

int add_car(Intersection I);

int get_num_cars(Intersection I);

LinkedList get_lane_queue( Intersection I, Direction D, int laneID);

int set_lane_flag( Intersection I, Direction D, int laneID, int flag);
int get_lane_flag( Intersection I, Direction D, int laneID);

int change_lane_counter( Intersection I, Direction D, int laneID, int val );
int get_lane_counter( Intersection I, Direction D, int laneID);

double get_crossing_distance( Intersection I, Direction D, Route R);

#endif

/* eof */
